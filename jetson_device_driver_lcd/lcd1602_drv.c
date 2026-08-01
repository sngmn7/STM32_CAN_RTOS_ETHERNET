// SPDX-License-Identifier: GPL-2.0
/*
 * lcd1602_drv.c
 *
 * HD44780 호환 1602A 캐릭터 LCD 드라이버 (PCF8574 I2C 백팩 경유)
 * 대상: Jetson Orin Nano Super / Linux
 *
 * 계층 구조
 *   유저 앱  --write()/ioctl()-->  /dev/lcd1602-0
 *                                      |
 *                              [ 이 드라이버 ]
 *                     HD44780 커맨드 규칙을 아는 유일한 곳
 *                                      |
 *                              i2c_smbus_write_byte()
 *                                      |
 *                              PCF8574 --> HD44780 --> 화면
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/idr.h>
#include <linux/of.h>
#include <linux/version.h>
#include <linux/bits.h>

#include "lcd1602.h"

#define DRV_NAME     "lcd1602"
#define LCD_MAX_DEVS 4

/* ------------------------------------------------------------------ */
/* PCF8574 백팩 비트 배치                                              */
/* 대부분의 시중 I2C 백팩이 이 배치를 쓴다.                             */
/*   P0=RS  P1=RW  P2=EN  P3=BL  P4~P7 = D4~D7                         */
/* ------------------------------------------------------------------ */
#define PCF_RS BIT(0)
#define PCF_RW BIT(1)
#define PCF_EN BIT(2)
#define PCF_BL BIT(3)

/* ------------------------------------------------------------------ */
/* HD44780 커맨드                                                      */
/* ------------------------------------------------------------------ */
#define HD_CLEAR         0x01
#define HD_HOME          0x02
#define HD_ENTRY_MODE    0x06 /* 커서 오른쪽 이동, 화면 시프트 없음 */
#define HD_DISPLAY_OFF   0x08
#define HD_DISPLAY_ON    0x0C /* 디스플레이 on, 커서 off, 블링크 off */
#define HD_FUNC_4BIT_2L  0x28 /* 4비트, 2줄, 5x8 폰트 */
#define HD_SET_DDRAM     0x80

/* 2줄 LCD의 각 줄 시작 DDRAM 주소 */
static const u8 lcd_row_offset[LCD_ROWS] = { 0x00, 0x40 };

/* ------------------------------------------------------------------ */
/* 디바이스별 상태                                                     */
/* ------------------------------------------------------------------ */
struct lcd1602 {
	struct i2c_client *client;
	struct cdev	   cdev;
	struct device	  *dev;
	dev_t		   devt;
	int		   id;
	struct mutex	   lock;	/* 동시 접근 직렬화 */
	bool		   backlight;
	u8		   col;
	u8		   row;
};

static struct class *lcd_class;
static dev_t	     lcd_devt_base;
static DEFINE_IDA(lcd_ida);

/* ================================================================== */
/* Low level: PCF8574 로 1바이트 밀어넣기                              */
/* ================================================================== */

static int pcf_write(struct lcd1602 *lcd, u8 data)
{
	if (lcd->backlight)
		data |= PCF_BL;

	return i2c_smbus_write_byte(lcd->client, data);
}

/*
 * EN 핀을 올렸다 내려서 HD44780이 데이터 라인을 래치하게 만든다.
 * 데이터시트상 EN 펄스 폭은 최소 450ns, 사이클은 최소 1us.
 */
static int lcd_pulse_enable(struct lcd1602 *lcd, u8 data)
{
	int ret;

	ret = pcf_write(lcd, data | PCF_EN);
	if (ret)
		return ret;
	udelay(2);

	ret = pcf_write(lcd, data & ~PCF_EN);
	if (ret)
		return ret;
	udelay(50); /* 대부분의 명령 실행시간 ~37us */

	return 0;
}

/* 상위 4비트만 전송 (4비트 모드) */
static int lcd_write_nibble(struct lcd1602 *lcd, u8 nibble, bool is_data)
{
	u8 out = (nibble & 0xF0) | (is_data ? PCF_RS : 0);
	int ret;

	ret = pcf_write(lcd, out);
	if (ret)
		return ret;

	return lcd_pulse_enable(lcd, out);
}

/* 1바이트를 상위 니블 → 하위 니블 순으로 전송 */
static int lcd_send(struct lcd1602 *lcd, u8 val, bool is_data)
{
	int ret;

	ret = lcd_write_nibble(lcd, val & 0xF0, is_data);
	if (ret)
		return ret;

	return lcd_write_nibble(lcd, (val << 4) & 0xF0, is_data);
}

static inline int lcd_command(struct lcd1602 *lcd, u8 cmd)
{
	return lcd_send(lcd, cmd, false);
}

static inline int lcd_data(struct lcd1602 *lcd, u8 ch)
{
	return lcd_send(lcd, ch, true);
}

/* ================================================================== */
/* 중간 계층: LCD 동작                                                 */
/* ================================================================== */

static int lcd_set_cursor(struct lcd1602 *lcd, u8 col, u8 row)
{
	if (col >= LCD_COLS || row >= LCD_ROWS)
		return -EINVAL;

	lcd->col = col;
	lcd->row = row;

	return lcd_command(lcd, HD_SET_DDRAM | (lcd_row_offset[row] + col));
}

static int lcd_clear(struct lcd1602 *lcd)
{
	int ret = lcd_command(lcd, HD_CLEAR);

	if (ret)
		return ret;

	usleep_range(2000, 2500); /* clear 는 느리다 (~1.52ms) */
	lcd->col = 0;
	lcd->row = 0;

	return 0;
}

/*
 * HD44780 파워온 초기화 시퀀스.
 * 8비트 모드로 시작해서 4비트 모드로 전환하는 정해진 절차가 있다.
 */
static int lcd_hw_init(struct lcd1602 *lcd)
{
	int ret;

	msleep(50); /* 전원 안정화 대기 (>40ms) */

	/* 백라이트만 켜고 나머지 라인은 0 */
	lcd->backlight = true;
	ret = pcf_write(lcd, 0x00);
	if (ret)
		return ret;
	msleep(20);

	/* 8비트 모드 요청 3회 (데이터시트 규정) */
	ret = lcd_write_nibble(lcd, 0x30, false);
	if (ret)
		return ret;
	usleep_range(4500, 5000);

	ret = lcd_write_nibble(lcd, 0x30, false);
	if (ret)
		return ret;
	usleep_range(150, 200);

	ret = lcd_write_nibble(lcd, 0x30, false);
	if (ret)
		return ret;
	usleep_range(150, 200);

	/* 4비트 모드로 전환 */
	ret = lcd_write_nibble(lcd, 0x20, false);
	if (ret)
		return ret;
	usleep_range(150, 200);

	/* 여기서부터는 정상적인 바이트 단위 커맨드 사용 가능 */
	ret = lcd_command(lcd, HD_FUNC_4BIT_2L);
	if (ret)
		return ret;

	ret = lcd_command(lcd, HD_DISPLAY_OFF);
	if (ret)
		return ret;

	ret = lcd_clear(lcd);
	if (ret)
		return ret;

	ret = lcd_command(lcd, HD_ENTRY_MODE);
	if (ret)
		return ret;

	return lcd_command(lcd, HD_DISPLAY_ON);
}

/*
 * 문자 한 개 출력 + 커서 자동 진행.
 * '\n' 은 다음 줄 맨 앞으로, 줄 끝에 닿으면 자동 개행.
 */
static int lcd_putchar(struct lcd1602 *lcd, char ch)
{
	int ret;

	if (ch == '\n') {
		u8 next = (lcd->row + 1) % LCD_ROWS;

		return lcd_set_cursor(lcd, 0, next);
	}

	if (ch == '\r')
		return lcd_set_cursor(lcd, 0, lcd->row);

	if (lcd->col >= LCD_COLS) {
		u8 next = (lcd->row + 1) % LCD_ROWS;

		ret = lcd_set_cursor(lcd, 0, next);
		if (ret)
			return ret;
	}

	ret = lcd_data(lcd, (u8)ch);
	if (ret)
		return ret;

	lcd->col++;

	return 0;
}

/* ================================================================== */
/* file_operations — 유저스페이스가 보는 인터페이스                    */
/* ================================================================== */

static int lcd_open(struct inode *inode, struct file *filp)
{
	struct lcd1602 *lcd = container_of(inode->i_cdev, struct lcd1602, cdev);

	filp->private_data = lcd;

	return 0;
}

static int lcd_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static ssize_t lcd_write(struct file *filp, const char __user *buf,
			 size_t count, loff_t *ppos)
{
	struct lcd1602 *lcd = filp->private_data;
	char *kbuf;
	size_t i;
	int ret = 0;

	if (count == 0)
		return 0;

	/* 한 번에 받아들일 최대치를 제한 (화면 두 배 정도면 충분) */
	if (count > 256)
		count = 256;

	kbuf = memdup_user(buf, count);
	if (IS_ERR(kbuf))
		return PTR_ERR(kbuf);

	if (mutex_lock_interruptible(&lcd->lock)) {
		kfree(kbuf);
		return -ERESTARTSYS;
	}

	for (i = 0; i < count; i++) {
		ret = lcd_putchar(lcd, kbuf[i]);
		if (ret) {
			dev_err(&lcd->client->dev,
				"write failed at byte %zu: %d\n", i, ret);
			break;
		}
	}

	mutex_unlock(&lcd->lock);
	kfree(kbuf);

	if (i == 0 && ret)
		return ret;

	return i;
}

static long lcd_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct lcd1602 *lcd = filp->private_data;
	struct lcd_cursor cur;
	int val;
	int ret = 0;

	if (_IOC_TYPE(cmd) != LCD_IOC_MAGIC)
		return -ENOTTY;
	if (_IOC_NR(cmd) > LCD_IOC_MAXNR)
		return -ENOTTY;

	if (mutex_lock_interruptible(&lcd->lock))
		return -ERESTARTSYS;

	switch (cmd) {
	case LCD_IOC_CLEAR:
		ret = lcd_clear(lcd);
		break;

	case LCD_IOC_HOME:
		ret = lcd_command(lcd, HD_HOME);
		if (!ret) {
			usleep_range(2000, 2500);
			lcd->col = 0;
			lcd->row = 0;
		}
		break;

	case LCD_IOC_SET_CURSOR:
		if (copy_from_user(&cur, (void __user *)arg, sizeof(cur))) {
			ret = -EFAULT;
			break;
		}
		ret = lcd_set_cursor(lcd, cur.col, cur.row);
		break;

	case LCD_IOC_BACKLIGHT:
		if (get_user(val, (int __user *)arg)) {
			ret = -EFAULT;
			break;
		}
		lcd->backlight = !!val;
		/* 데이터 라인은 건드리지 않고 백라이트 비트만 갱신 */
		ret = pcf_write(lcd, 0x00);
		break;

	case LCD_IOC_DISPLAY:
		if (get_user(val, (int __user *)arg)) {
			ret = -EFAULT;
			break;
		}
		ret = lcd_command(lcd, val ? HD_DISPLAY_ON : HD_DISPLAY_OFF);
		break;

	default:
		ret = -ENOTTY;
		break;
	}

	mutex_unlock(&lcd->lock);

	return ret;
}

static const struct file_operations lcd_fops = {
	.owner		= THIS_MODULE,
	.open		= lcd_open,
	.release	= lcd_release,
	.write		= lcd_write,
	.unlocked_ioctl	= lcd_ioctl,
	.llseek		= no_llseek,
};

/* ================================================================== */
/* i2c_driver — probe / remove                                        */
/* ================================================================== */

static int lcd_probe(struct i2c_client *client)
{
	struct lcd1602 *lcd;
	int ret;

	dev_info(&client->dev, "probing 1602A at addr 0x%02x\n", client->addr);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE)) {
		dev_err(&client->dev, "adapter lacks SMBUS_BYTE support\n");
		return -EIO;
	}

	lcd = devm_kzalloc(&client->dev, sizeof(*lcd), GFP_KERNEL);
	if (!lcd)
		return -ENOMEM;

	lcd->client = client;
	mutex_init(&lcd->lock);
	i2c_set_clientdata(client, lcd);

	ret = lcd_hw_init(lcd);
	if (ret) {
		dev_err(&client->dev, "LCD init failed: %d\n", ret);
		return ret;
	}

	/* 마이너 번호 할당 */
	lcd->id = ida_alloc_max(&lcd_ida, LCD_MAX_DEVS - 1, GFP_KERNEL);
	if (lcd->id < 0)
		return lcd->id;

	lcd->devt = MKDEV(MAJOR(lcd_devt_base), MINOR(lcd_devt_base) + lcd->id);

	cdev_init(&lcd->cdev, &lcd_fops);
	lcd->cdev.owner = THIS_MODULE;

	ret = cdev_add(&lcd->cdev, lcd->devt, 1);
	if (ret) {
		dev_err(&client->dev, "cdev_add failed: %d\n", ret);
		goto err_ida;
	}

	lcd->dev = device_create(lcd_class, &client->dev, lcd->devt, lcd,
				 "lcd1602-%d", lcd->id);
	if (IS_ERR(lcd->dev)) {
		ret = PTR_ERR(lcd->dev);
		dev_err(&client->dev, "device_create failed: %d\n", ret);
		goto err_cdev;
	}

	dev_info(&client->dev, "registered as /dev/lcd1602-%d\n", lcd->id);

	return 0;

err_cdev:
	cdev_del(&lcd->cdev);
err_ida:
	ida_free(&lcd_ida, lcd->id);

	return ret;
}

static void lcd_remove_common(struct i2c_client *client)
{
	struct lcd1602 *lcd = i2c_get_clientdata(client);

	mutex_lock(&lcd->lock);
	lcd_command(lcd, HD_CLEAR);
	usleep_range(2000, 2500);
	lcd->backlight = false;
	pcf_write(lcd, 0x00);
	mutex_unlock(&lcd->lock);

	device_destroy(lcd_class, lcd->devt);
	cdev_del(&lcd->cdev);
	ida_free(&lcd_ida, lcd->id);

	dev_info(&client->dev, "removed\n");
}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 1, 0)
static void lcd_remove(struct i2c_client *client)
{
	lcd_remove_common(client);
}
#else
static int lcd_remove(struct i2c_client *client)
{
	lcd_remove_common(client);
	return 0;
}
#endif

/* 디바이스트리로 붙일 때 매칭되는 문자열 */
static const struct of_device_id lcd_of_match[] = {
	{ .compatible = "seungmin,lcd1602" },
	{ }
};
MODULE_DEVICE_TABLE(of, lcd_of_match);

/* sysfs new_device 로 수동 등록할 때 쓰는 이름 */
static const struct i2c_device_id lcd_i2c_id[] = {
	{ "lcd1602", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lcd_i2c_id);

static struct i2c_driver lcd_i2c_driver = {
	.driver = {
		.name		= DRV_NAME,
		.of_match_table	= lcd_of_match,
	},
#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 3, 0)
	.probe		= lcd_probe,
#else
	.probe_new	= lcd_probe,
#endif
	.remove		= lcd_remove,
	.id_table	= lcd_i2c_id,
};

/* ================================================================== */
/* 모듈 init / exit                                                    */
/* ================================================================== */

static int __init lcd_init(void)
{
	int ret;

	ret = alloc_chrdev_region(&lcd_devt_base, 0, LCD_MAX_DEVS, DRV_NAME);
	if (ret) {
		pr_err(DRV_NAME ": alloc_chrdev_region failed: %d\n", ret);
		return ret;
	}

#if LINUX_VERSION_CODE >= KERNEL_VERSION(6, 4, 0)
	lcd_class = class_create(DRV_NAME);
#else
	lcd_class = class_create(THIS_MODULE, DRV_NAME);
#endif
	if (IS_ERR(lcd_class)) {
		ret = PTR_ERR(lcd_class);
		goto err_chrdev;
	}

	ret = i2c_add_driver(&lcd_i2c_driver);
	if (ret)
		goto err_class;

	pr_info(DRV_NAME ": loaded (major %d)\n", MAJOR(lcd_devt_base));

	return 0;

err_class:
	class_destroy(lcd_class);
err_chrdev:
	unregister_chrdev_region(lcd_devt_base, LCD_MAX_DEVS);

	return ret;
}

static void __exit lcd_exit(void)
{
	i2c_del_driver(&lcd_i2c_driver);
	class_destroy(lcd_class);
	unregister_chrdev_region(lcd_devt_base, LCD_MAX_DEVS);
	ida_destroy(&lcd_ida);

	pr_info(DRV_NAME ": unloaded\n");
}

module_init(lcd_init);
module_exit(lcd_exit);

MODULE_AUTHOR("Seungmin");
MODULE_DESCRIPTION("HD44780 1602A character LCD driver over PCF8574 I2C backpack");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1");
