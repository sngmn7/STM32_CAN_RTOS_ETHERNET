// SPDX-License-Identifier: GPL-2.0
/*
 * vlcd.c — Front + Rear 존 상태 → 1602A LCD (7페이지)
 *
 *   /dev/vehicle_status ('SV' 28B) ─┐
 *                                   ├─ poll() ─→ 이 앱 ─→ /dev/lcd1602-0
 *   /dev/rear_status    ('SR' 32B) ─┘
 *
 * 페이지
 *   [1/9] F-ADC   조향/가속/브레이크/액추에이터
 *   [2/9] F-LAMP  방향지시/스위치/헤드램프/오버라이드
 *   [3/9] F-ENV   온도/습도
 *   [4/9] F-DIAG  seq / 버스오프 / DHT / 액추에이터 허용
 *   [5/9] R-LAMP  Rear 적용 방향지시/브레이크/창문
 *   [6/9] R-DIST  후방 초음파 거리 + 토글 스위치
 *   [7/9] R-IMU   BNO08x 회전벡터 -> yaw/pitch/roll
 *   [8/9] R-LINK  링크 생존 + 명령 출처 (Jetson vs Front 중재)
 *   [9/9] R-DIAG  seq / CAN 헬스 / 센서 수신 카운트
 *
 * 조작: space/n 다음, p 이전, 1~9 직접, a 자동(3초), q 종료
 * 존별 링크 타임아웃 독립 — Front 만 죽으면 Front 페이지만 NO DATA.
 */

#define _GNU_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
#include <math.h>
#include <sys/ioctl.h>

#include "zstatus_proto.h"
#include "lcd1602.h"

#define FRONT_DEV  "/dev/vehicle_status"
#define REAR_DEV   "/dev/rear_status"
#define LCD_DEV    "/dev/lcd1602-0"

#define LCD_REFRESH_MS    200
#define LINK_TIMEOUT_MS  1000
#define TITLE_HOLD_MS     700
#define AUTO_ROTATE_MS   3000

#define PAGE_COUNT 9

/* 페이지 → 존 매핑: 0~3 = front, 4~8 = rear */
#define PAGE_IS_REAR(pg)  ((pg) >= 4)

static const char *page_name[PAGE_COUNT] = {
	"F-ADC", "F-LAMP", "F-ENV", "F-DIAG",
	"R-LAMP", "R-DIST", "R-IMU", "R-LINK", "R-DIAG"
};

static volatile sig_atomic_t running = 1;
static struct termios saved_tio;
static int tio_saved;

static void on_signal(int sig)
{
	(void)sig;
	running = 0;
}

static long long now_ms(void)
{
	struct timespec ts;

	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (long long)ts.tv_sec * 1000LL + ts.tv_nsec / 1000000LL;
}

static void tty_raw(void)
{
	struct termios tio;

	if (!isatty(STDIN_FILENO))
		return;
	if (tcgetattr(STDIN_FILENO, &saved_tio) < 0)
		return;

	tio_saved = 1;
	tio = saved_tio;
	tio.c_lflag &= ~(ICANON | ECHO);
	tio.c_cc[VMIN] = 1;
	tio.c_cc[VTIME] = 0;
	tcsetattr(STDIN_FILENO, TCSANOW, &tio);
}

static void tty_restore(void)
{
	if (tio_saved)
		tcsetattr(STDIN_FILENO, TCSANOW, &saved_tio);
}

/* ------------------------------------------------------------------ */
/* 렌더링                                                              */
/* ------------------------------------------------------------------ */

static const char *front_turn(const struct vstatus_packet *p)
{
	if (p->turn_left && p->turn_right)
		return "<>";
	if (p->turn_left)
		return "<<";
	if (p->turn_right)
		return ">>";
	return "--";
}

static void render_front(const struct vstatus_packet *p, int page,
			 char l1[17], char l2[17])
{
	unsigned f = p->flags;

	switch (page) {
	case 0:
		snprintf(l1, 17, "STR%4u ACC%4u",
			 p->steering_adc > 9999 ? 9999 : p->steering_adc,
			 p->accel_adc    > 9999 ? 9999 : p->accel_adc);
		snprintf(l2, 17, "BRK%4u ACT%4u",
			 p->brake_adc    > 9999 ? 9999 : p->brake_adc,
			 p->actuator_pos > 9999 ? 9999 : p->actuator_pos);
		break;

	case 1:
		snprintf(l1, 17, "TURN %-2s  SW:%c",
			 front_turn(p),
			 (f & VST_FLAG_TURN_SWITCH) ? '1' : '0');
		snprintf(l2, 17, "HL:%-3s OVR:%c%c",
			 p->headlamp ? "ON" : "OFF",
			 (f & VST_FLAG_STEER_OVR)    ? 'S' : '-',
			 (f & VST_FLAG_HEADLAMP_OVR) ? 'H' : '-');
		break;

	case 2:
		if (f & VST_FLAG_DHT_VALID) {
			int t  = p->temp_x10;
			int ti = t / 10;
			int tf = (t < 0 ? -t : t) % 10;
			unsigned hi = p->humidity_x10 / 10U;

			if (ti > 99)
				ti = 99;
			if (ti < -9)
				ti = -9;
			if (hi > 100U)
				hi = 100U;

			snprintf(l1, 17, "TEMP: %3d.%d C", ti, tf);
			snprintf(l2, 17, "HUMI: %3u.%u %%",
				 hi, p->humidity_x10 % 10U);
		} else {
			snprintf(l1, 17, "%-16s", "TEMP:  --.- C");
			snprintf(l2, 17, "%-16s", "HUMI:  --.- %");
		}
		break;

	default:
		snprintf(l1, 17, "SEQ %10u", p->seq);
		snprintf(l2, 17, "BUS:%c DHT:%c AE:%c",
			 (f & VST_FLAG_CAN_BUSOFF)  ? 'Y' : 'N',
			 (f & VST_FLAG_DHT_VALID)   ? 'Y' : 'N',
			 (f & VST_FLAG_ACTUATOR_EN) ? '1' : '0');
		break;
	}
}

static void render_rear(const struct rstatus_packet *p, int page,
			char l1[17], char l2[17])
{
	unsigned s = p->source;
	unsigned h = p->health;
	unsigned sen = p->sensor;
	const char *turn;

	if (p->led_left && p->led_right)
		turn = "<>";
	else if (p->led_left)
		turn = "<<";
	else if (p->led_right)
		turn = ">>";
	else
		turn = "--";

	switch (page) {
	case 4:  /* R-LAMP — 적용 중인 액추에이터 */
		snprintf(l1, 17, "TURN %-2s BRK:%c",
			 turn, p->led_brake ? '1' : '0');
		snprintf(l2, 17, "WIN:%-3s FBRK%4u",
			 p->window ? "ON" : "OFF",
			 p->front_brake_adc > 9999 ?
				9999 : p->front_brake_adc);
		break;

	case 5:  /* R-DIST — 초음파 + 토글 스위치 */
		if (sen & RST_SEN_ULTRA_FRESH) {
			unsigned mm = p->ultrasonic_mm;
			unsigned cm = mm / 10U;

			if (cm > 9999U)
				cm = 9999U;

			snprintf(l1, 17, "DIST %4u cm", cm);
		} else {
			snprintf(l1, 17, "%-16s", "DIST  ---- cm");
		}
		snprintf(l2, 17, "SW:%-3s  RX%5u",
			 (sen & RST_SEN_SWITCH_FRESH) ?
				((sen & RST_SEN_SWITCH) ? "ON" : "OFF") : "?",
			 p->ultra_rx_count % 100000U);
		break;

	case 6:  /* R-IMU — 회전벡터를 yaw/pitch/roll 로 */
		if (sen & RST_SEN_IMU_FRESH) {
			/* BNO08x 회전벡터는 Q14 고정소수점 */
			double qi = p->quat_i    / 16384.0;
			double qj = p->quat_j    / 16384.0;
			double qk = p->quat_k    / 16384.0;
			double qr = p->quat_real / 16384.0;
			double sinr, cosr, sinp, siny, cosy;
			int roll, pitch, yaw;

			sinr = 2.0 * (qr * qi + qj * qk);
			cosr = 1.0 - 2.0 * (qi * qi + qj * qj);
			roll = (int)(atan2(sinr, cosr) * 57.29578);

			sinp = 2.0 * (qr * qj - qk * qi);
			if (sinp > 1.0)
				sinp = 1.0;
			if (sinp < -1.0)
				sinp = -1.0;
			pitch = (int)(asin(sinp) * 57.29578);

			siny = 2.0 * (qr * qk + qi * qj);
			cosy = 1.0 - 2.0 * (qj * qj + qk * qk);
			yaw = (int)(atan2(siny, cosy) * 57.29578);

			snprintf(l1, 17, "YAW%4d PIT%4d", yaw, pitch);
			snprintf(l2, 17, "ROL%4d  RX%4u",
				 roll, p->imu_rx_count % 10000U);
		} else {
			snprintf(l1, 17, "%-16s", "IMU: NO DATA");
			snprintf(l2, 17, "%-16s", "check F466RE");
		}
		break;

	case 7:  /* R-LINK — 링크 + 명령 출처 중재 */
		snprintf(l1, 17, "LNK F:%c J:%c",
			 (s & RST_SRC_FRONT_LINK)  ? 'Y' : 'N',
			 (s & RST_SRC_JETSON_LINK) ? 'Y' : 'N');
		snprintf(l2, 17, "SRC T:%s B:%s",
			 (s & RST_SRC_TURN_JETSON)  ? "JET" : "FRT",
			 (s & RST_SRC_BRAKE_JETSON) ? "JET" : "FRT");
		break;

	default: /* R-DIAG */
		snprintf(l1, 17, "SEQ %10u", p->seq);
		snprintf(l2, 17, "C5:%c C2:%c I%4u",
			 (h & RST_HLT_CAN500_ERR) ? 'E' : 'O',
			 (h & RST_HLT_CAN250_ERR) ? 'E' : 'O',
			 p->imu_rx_count % 10000U);
		break;
	}
}

/* ------------------------------------------------------------------ */

static int lcd_write_lines(int fd, const char *l1, const char *l2)
{
	struct lcd_cursor cur;
	char b1[17], b2[17];

	snprintf(b1, sizeof(b1), "%-16s", l1);
	snprintf(b2, sizeof(b2), "%-16s", l2);

	cur.col = 0;
	cur.row = 0;
	if (ioctl(fd, LCD_IOC_SET_CURSOR, &cur) < 0)
		return -1;
	if (write(fd, b1, 16) < 0)
		return -1;

	cur.col = 0;
	cur.row = 1;
	if (ioctl(fd, LCD_IOC_SET_CURSOR, &cur) < 0)
		return -1;
	if (write(fd, b2, 16) < 0)
		return -1;

	return 0;
}


int main(void)
{
	struct vstatus_packet fpkt;
	struct rstatus_packet rpkt;
	struct pollfd pfd[3];
	char l1[17], l2[17];
	char prev1[17] = "", prev2[17] = "";
	long long last_render = 0;
	long long f_last_rx = 0, r_last_rx = 0;
	long long title_until, last_rotate;
	unsigned long f_total = 0, r_total = 0;
	int page = 0, auto_rotate = 0;
	int fvst, rvst, lcd, have_tty;

	signal(SIGINT, on_signal);
	signal(SIGTERM, on_signal);

	fvst = open(FRONT_DEV, O_RDONLY);
	if (fvst < 0) {
		fprintf(stderr, "open %s: %s\n", FRONT_DEV, strerror(errno));
		fprintf(stderr, "  → sudo insmod zstatus_drv.ko 확인\n");
		return 1;
	}

	rvst = open(REAR_DEV, O_RDONLY);
	if (rvst < 0) {
		fprintf(stderr, "open %s: %s\n", REAR_DEV, strerror(errno));
		close(fvst);
		return 1;
	}

	lcd = open(LCD_DEV, O_WRONLY);
	if (lcd < 0) {
		fprintf(stderr, "open %s: %s\n", LCD_DEV, strerror(errno));
		fprintf(stderr, "  → lcd1602_drv.ko / new_device 확인\n");
		close(fvst);
		close(rvst);
		return 1;
	}

	have_tty = isatty(STDIN_FILENO);
	tty_raw();
	ioctl(lcd, LCD_IOC_CLEAR);

	printf("페이지: space/n 다음, p 이전, 1~9 직접, a 자동, q 종료\n");
	fflush(stdout);

	memset(&fpkt, 0, sizeof(fpkt));
	memset(&rpkt, 0, sizeof(rpkt));
	title_until = now_ms() + TITLE_HOLD_MS;
	last_rotate = now_ms();

	while (running) {
		long long t;
		int nfds = 2, ret, changed = 0, linked;

		pfd[0].fd = fvst;
		pfd[0].events = POLLIN;
		pfd[1].fd = rvst;
		pfd[1].events = POLLIN;

		if (have_tty) {
			pfd[2].fd = STDIN_FILENO;
			pfd[2].events = POLLIN;
			nfds = 3;
		}

		ret = poll(pfd, nfds, 100);

		if (ret < 0) {
			if (errno == EINTR)
				continue;
			fprintf(stderr, "poll: %s\n", strerror(errno));
			break;
		}

		if (pfd[0].revents & POLLIN) {
			ssize_t n = read(fvst, &fpkt, sizeof(fpkt));

			if (n == (ssize_t)sizeof(fpkt)) {
				f_last_rx = now_ms();
				f_total++;
			}
		}

		if (pfd[1].revents & POLLIN) {
			ssize_t n = read(rvst, &rpkt, sizeof(rpkt));

			if (n == (ssize_t)sizeof(rpkt)) {
				r_last_rx = now_ms();
				r_total++;
			}
		}

		if (nfds == 3 && (pfd[2].revents & POLLIN)) {
			char c;

			if (read(STDIN_FILENO, &c, 1) == 1) {
				switch (c) {
				case ' ':
				case 'n':
					page = (page + 1) % PAGE_COUNT;
					changed = 1;
					break;
				case 'p':
					page = (page + PAGE_COUNT - 1)
						% PAGE_COUNT;
					changed = 1;
					break;
				case '1': case '2': case '3': case '4':
				case '5': case '6': case '7':
				case '8': case '9':
					page = c - '1';
					changed = 1;
					break;
				case 'a':
					auto_rotate = !auto_rotate;
					last_rotate = now_ms();
					printf("자동 전환 %s\n",
					       auto_rotate ? "ON" : "OFF");
					fflush(stdout);
					break;
				case 'q':
					running = 0;
					break;
				default:
					break;
				}
			}
		}

		t = now_ms();

		if (auto_rotate && (t - last_rotate) >= AUTO_ROTATE_MS) {
			page = (page + 1) % PAGE_COUNT;
			last_rotate = t;
			changed = 1;
		}

		if (changed) {
			title_until = t + TITLE_HOLD_MS;
			printf("[%d/%d] %s\n", page + 1, PAGE_COUNT,
			       page_name[page]);
			fflush(stdout);
		}

		if (t - last_render < LCD_REFRESH_MS)
			continue;
		last_render = t;

		/* 존별 독립 링크 판정 */
		if (PAGE_IS_REAR(page))
			linked = r_last_rx &&
				 (t - r_last_rx) < LINK_TIMEOUT_MS;
		else
			linked = f_last_rx &&
				 (t - f_last_rx) < LINK_TIMEOUT_MS;

		if (t < title_until) {
			snprintf(l1, sizeof(l1), "[%d/%d] %s",
				 page + 1, PAGE_COUNT, page_name[page]);
			snprintf(l2, sizeof(l2), "%-16s",
				 auto_rotate ? "auto rotate" : "space=next");
		} else if (!linked) {
			snprintf(l1, sizeof(l1), "%-16s",
				 PAGE_IS_REAR(page) ?
					"REAR: NO DATA" : "FRONT: NO DATA");
			snprintf(l2, sizeof(l2), "%-16s", "check STM32...");
		} else if (PAGE_IS_REAR(page)) {
			render_rear(&rpkt, page, l1, l2);
		} else {
			render_front(&fpkt, page, l1, l2);
		}

		if (strcmp(l1, prev1) != 0 || strcmp(l2, prev2) != 0) {
			if (lcd_write_lines(lcd, l1, l2) < 0)
				fprintf(stderr, "LCD write: %s\n",
					strerror(errno));
			strcpy(prev1, l1);
			strcpy(prev2, l2);
		}
	}

	printf("\n종료. Front %lu / Rear %lu 패킷\n", f_total, r_total);

	ioctl(lcd, LCD_IOC_CLEAR);
	close(lcd);
	close(fvst);
	close(rvst);
	tty_restore();

	return 0;
}
