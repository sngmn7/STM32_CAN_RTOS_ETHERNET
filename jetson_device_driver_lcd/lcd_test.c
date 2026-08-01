// SPDX-License-Identifier: GPL-2.0
/*
 * lcd_test.c — 드라이버 동작 확인용 유저스페이스 앱
 *
 * 이 파일이 "앱코드"의 예시다.
 * PCF8574, HD44780, I2C 라는 단어가 한 번도 안 나온다는 점에 주목.
 * 앱은 open/write/ioctl 만 알면 된다.
 *
 * 빌드: gcc -Wall -O2 -I.. -o lcd_test lcd_test.c
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>

#include "lcd1602.h"

#define DEV_PATH "/dev/lcd1602-0"

static int lcd_print_at(int fd, int col, int row, const char *s)
{
	struct lcd_cursor cur = { .col = col, .row = row };

	if (ioctl(fd, LCD_IOC_SET_CURSOR, &cur) < 0) {
		perror("ioctl SET_CURSOR");
		return -1;
	}

	if (write(fd, s, strlen(s)) < 0) {
		perror("write");
		return -1;
	}

	return 0;
}

int main(void)
{
	int fd, on;

	fd = open(DEV_PATH, O_WRONLY);
	if (fd < 0) {
		fprintf(stderr, "open %s: %s\n", DEV_PATH, strerror(errno));
		return 1;
	}

	/* 1. 화면 지우기 */
	if (ioctl(fd, LCD_IOC_CLEAR) < 0)
		perror("ioctl CLEAR");

	/* 2. 두 줄에 각각 출력 */
	lcd_print_at(fd, 0, 0, "Rear Zone Ctrl");
	lcd_print_at(fd, 0, 1, "STM32 UDP  OK");
	sleep(2);

	/* 3. 값 갱신 흉내 — 2번째 줄만 다시 씀 */
	for (int i = 0; i < 5; i++) {
		char line[LCD_COLS + 1];

		snprintf(line, sizeof(line), "SPD:%3d STR:%+3d", i * 12, i * 7 - 14);
		lcd_print_at(fd, 0, 1, "                ");
		lcd_print_at(fd, 0, 1, line);
		sleep(1);
	}

	/* 4. 백라이트 토글 */
	on = 0;
	ioctl(fd, LCD_IOC_BACKLIGHT, &on);
	sleep(1);
	on = 1;
	ioctl(fd, LCD_IOC_BACKLIGHT, &on);

	/* 5. 개행 문자 처리 확인 */
	ioctl(fd, LCD_IOC_CLEAR);
	dprintf(fd, "Line one\nLine two");
	sleep(2);

	close(fd);
	printf("test done\n");

	return 0;
}
