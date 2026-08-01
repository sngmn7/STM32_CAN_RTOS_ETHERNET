// SPDX-License-Identifier: GPL-2.0
/*
 * zstatus_drv.c — 존 상태 수신 드라이버 v2 (Front + Rear)
 *
 *   Front H723 --UDP 5102 ('SV' 28B)--> /dev/vehicle_status
 *   Rear  H723 --UDP 5002 ('SR' 32B)--> /dev/rear_status
 *
 * v1(vstatus_drv.c)에서 검증된 패턴(커널 소켓 + kthread + misc 디바이스,
 * 네트워크 장치 등록 없음)을 "존 인스턴스" 배열로 일반화했다.
 * 인스턴스마다 소켓/스레드/버퍼/대기큐가 완전히 독립이라
 * 한 존이 죽어도 다른 존 수신에 영향이 없다.
 *
 * 패킷 검증은 인스턴스별 (크기, magic, version) 3중 체크.
 * 파싱은 하지 않는다 — 해석은 이미 STM32 가 끝냈고,
 * 구조체 해석은 유저스페이스(vlcd)가 zstatus_proto.h 로 한다.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/kthread.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/spinlock.h>
#include <linux/net.h>
#include <linux/in.h>
#include <linux/socket.h>
#include <net/sock.h>

#include "zstatus_proto.h"

#define DRV_NAME "zstatus"

/** 존 하나가 받을 수 있는 최대 패킷 (32B 인 rear 가 최대) */
#define ZST_MAX_PACKET  64U

/* ------------------------------------------------------------------ */
/* 존 인스턴스                                                         */
/* ------------------------------------------------------------------ */

struct zone_inst {
	/* 정적 설정 */
	const char *name;        /* misc 디바이스 이름 = /dev/<name> */
	u16         port;        /* 수신 UDP 포트 */
	u16         magic;       /* 기대 magic (LE 값) */
	u8          version;     /* 기대 version */
	u16         pkt_size;    /* 기대 패킷 크기 */

	/* 런타임 */
	struct socket      *sock;
	struct task_struct *thread;
	struct miscdevice   misc;

	u8   latest[ZST_MAX_PACKET];
	u32  update_count;
	bool have_data;
	spinlock_t        lock;
	wait_queue_head_t waitq;

	u32  rx_count;
	u32  bad_count;
	u32  seq_gap_count;
	u32  last_seq;
	bool seq_valid;
};

static struct zone_inst zones[] = {
	{
		.name     = "vehicle_status",
		.port     = VST_PORT,
		.magic    = VST_MAGIC,
		.version  = VST_VERSION,
		.pkt_size = VST_PACKET_SIZE,
	},
	{
		.name     = "rear_status",
		.port     = RST_PORT,
		.magic    = RST_MAGIC,
		.version  = RST_VERSION,
		.pkt_size = RST_PACKET_SIZE,
	},
};

#define ZONE_COUNT ARRAY_SIZE(zones)

/* 파일 핸들 컨텍스트: 어느 존 + 어디까지 읽었는지 */
struct zst_file_ctx {
	struct zone_inst *z;
	u32 last_seen;
};

/* misc 는 open 시 file->private_data 에 miscdevice* 를 넣어준다.
 * 그걸로 소속 zone_inst 를 역참조한다. */
static struct zone_inst *zone_from_misc(struct file *filp)
{
	struct miscdevice *m = filp->private_data;

	return container_of(m, struct zone_inst, misc);
}

/* ------------------------------------------------------------------ */
/* 수신 스레드 (존마다 하나)                                            */
/* ------------------------------------------------------------------ */

static int zst_rx_thread(void *arg)
{
	struct zone_inst *z = arg;
	u8 buf[ZST_MAX_PACKET];
	struct msghdr msg;
	struct kvec iov;
	int len;
	u16 magic;
	u8 version;
	u32 seq;

	pr_info(DRV_NAME ": [%s] rx thread started (udp %u)\n",
		z->name, z->port);

	while (!kthread_should_stop()) {
		memset(&msg, 0, sizeof(msg));
		iov.iov_base = buf;
		iov.iov_len  = sizeof(buf);

		len = kernel_recvmsg(z->sock, &msg, &iov, 1,
				     sizeof(buf), 0);

		if (len == -EAGAIN || len == -EWOULDBLOCK ||
		    len == -ERESTARTSYS || len == -EINTR)
			continue;

		if (len < 0) {
			pr_err(DRV_NAME ": [%s] recvmsg: %d\n",
			       z->name, len);
			break;
		}

		/* 검증: 크기 / magic / version — 모든 패킷은
		 * offset 0..3 이 magic(2)+version(1)+aux(1) 로 동일 */
		if (len != (int)z->pkt_size) {
			z->bad_count++;
			continue;
		}

		magic   = (u16)buf[0] | ((u16)buf[1] << 8);
		version = buf[2];

		if (magic != z->magic || version != z->version) {
			z->bad_count++;
			continue;
		}

		/* seq 는 두 패킷 모두 offset 4, u32 LE */
		seq = (u32)buf[4] | ((u32)buf[5] << 8) |
		      ((u32)buf[6] << 16) | ((u32)buf[7] << 24);

		if (z->seq_valid && seq != z->last_seq + 1)
			z->seq_gap_count++;
		z->last_seq = seq;
		z->seq_valid = true;

		spin_lock(&z->lock);
		memcpy(z->latest, buf, z->pkt_size);
		z->update_count++;
		z->have_data = true;
		spin_unlock(&z->lock);

		z->rx_count++;
		wake_up_interruptible(&z->waitq);
	}

	pr_info(DRV_NAME ": [%s] rx thread stopped "
		"(rx=%u bad=%u gap=%u)\n",
		z->name, z->rx_count, z->bad_count, z->seq_gap_count);

	return 0;
}

/* ------------------------------------------------------------------ */
/* file_operations (모든 존 공용 — 컨텍스트로 구분)                     */
/* ------------------------------------------------------------------ */

static int zst_open(struct inode *inode, struct file *filp)
{
	struct zone_inst *z = zone_from_misc(filp);
	struct zst_file_ctx *ctx;

	ctx = kzalloc(sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	ctx->z = z;
	filp->private_data = ctx;   /* miscdevice* 를 우리 컨텍스트로 대체 */

	return 0;
}


static int zst_release(struct inode *inode, struct file *filp)
{
	kfree(filp->private_data);
	return 0;
}


static ssize_t zst_read(struct file *filp, char __user *buf,
			size_t count, loff_t *ppos)
{
	struct zst_file_ctx *ctx = filp->private_data;
	struct zone_inst *z = ctx->z;
	u8 snapshot[ZST_MAX_PACKET];
	u32 seen;
	int ret;

	if (count < z->pkt_size)
		return -EINVAL;

	if (filp->f_flags & O_NONBLOCK) {
		spin_lock(&z->lock);
		if (!z->have_data || z->update_count == ctx->last_seen) {
			spin_unlock(&z->lock);
			return -EAGAIN;
		}
		spin_unlock(&z->lock);
	} else {
		ret = wait_event_interruptible(z->waitq,
			z->have_data && z->update_count != ctx->last_seen);
		if (ret)
			return ret;
	}

	spin_lock(&z->lock);
	memcpy(snapshot, z->latest, z->pkt_size);
	seen = z->update_count;
	spin_unlock(&z->lock);

	ctx->last_seen = seen;

	if (copy_to_user(buf, snapshot, z->pkt_size))
		return -EFAULT;

	return z->pkt_size;
}


static __poll_t zst_poll(struct file *filp, poll_table *wait)
{
	struct zst_file_ctx *ctx = filp->private_data;
	struct zone_inst *z = ctx->z;
	__poll_t mask = 0;

	poll_wait(filp, &z->waitq, wait);

	spin_lock(&z->lock);
	if (z->have_data && z->update_count != ctx->last_seen)
		mask |= EPOLLIN | EPOLLRDNORM;
	spin_unlock(&z->lock);

	return mask;
}


static const struct file_operations zst_fops = {
	.owner   = THIS_MODULE,
	.open    = zst_open,
	.release = zst_release,
	.read    = zst_read,
	.poll    = zst_poll,
	.llseek  = no_llseek,
};

/* ------------------------------------------------------------------ */
/* 인스턴스 시작/정지                                                  */
/* ------------------------------------------------------------------ */

static void zone_stop(struct zone_inst *z)
{
	if (z->thread) {
		kthread_stop(z->thread);
		z->thread = NULL;
	}

	if (z->misc.this_device) {
		misc_deregister(&z->misc);
		z->misc.this_device = NULL;
	}

	if (z->sock) {
		sock_release(z->sock);
		z->sock = NULL;
	}
}


static int zone_start(struct zone_inst *z)
{
	struct sockaddr_in local;
	int ret;

	spin_lock_init(&z->lock);
	init_waitqueue_head(&z->waitq);

	ret = sock_create_kern(&init_net, AF_INET, SOCK_DGRAM,
			       IPPROTO_UDP, &z->sock);
	if (ret < 0) {
		pr_err(DRV_NAME ": [%s] sock_create: %d\n", z->name, ret);
		return ret;
	}

	z->sock->sk->sk_rcvtimeo = msecs_to_jiffies(500);

	memset(&local, 0, sizeof(local));
	local.sin_family      = AF_INET;
	local.sin_addr.s_addr = htonl(INADDR_ANY);
	local.sin_port        = htons(z->port);

	ret = kernel_bind(z->sock, (struct sockaddr *)&local,
			  sizeof(local));
	if (ret < 0) {
		pr_err(DRV_NAME ": [%s] bind udp/%u: %d\n",
		       z->name, z->port, ret);
		goto err;
	}

	z->misc.minor = MISC_DYNAMIC_MINOR;
	z->misc.name  = z->name;
	z->misc.fops  = &zst_fops;
	z->misc.mode  = 0444;

	ret = misc_register(&z->misc);
	if (ret) {
		pr_err(DRV_NAME ": [%s] misc_register: %d\n",
		       z->name, ret);
		goto err;
	}

	z->thread = kthread_run(zst_rx_thread, z, "%s-rx", z->name);
	if (IS_ERR(z->thread)) {
		ret = PTR_ERR(z->thread);
		z->thread = NULL;
		pr_err(DRV_NAME ": [%s] kthread: %d\n", z->name, ret);
		goto err;
	}

	pr_info(DRV_NAME ": /dev/%s ready (udp %u, %uB '%c%c')\n",
		z->name, z->port, z->pkt_size,
		(char)(z->magic & 0xFF), (char)(z->magic >> 8));

	return 0;

err:
	zone_stop(z);
	return ret;
}

/* ------------------------------------------------------------------ */
/* init / exit                                                         */
/* ------------------------------------------------------------------ */

static int __init zst_init(void)
{
	size_t i;
	int ret;

	for (i = 0; i < ZONE_COUNT; i++) {
		ret = zone_start(&zones[i]);
		if (ret) {
			while (i-- > 0)
				zone_stop(&zones[i]);
			return ret;
		}
	}

	pr_info(DRV_NAME ": loaded (%zu zones)\n", ZONE_COUNT);

	return 0;
}


static void __exit zst_exit(void)
{
	size_t i;

	for (i = 0; i < ZONE_COUNT; i++)
		zone_stop(&zones[i]);

	pr_info(DRV_NAME ": unloaded\n");
}

module_init(zst_init);
module_exit(zst_exit);

MODULE_AUTHOR("Seungmin");
MODULE_DESCRIPTION("Zone status receiver v2 (front + rear UDP -> char devices)");
MODULE_LICENSE("GPL");
MODULE_VERSION("2.0");
