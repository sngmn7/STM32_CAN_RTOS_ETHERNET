/* SPDX-License-Identifier: GPL-2.0 */
/*
 * lcd1602.h — 커널 드라이버와 유저스페이스 앱이 공유하는 인터페이스 정의
 *
 * 이 파일이 곧 "앱코드 ↔ 드라이버" 사이의 계약서(ICD)다.
 * 앱은 이 헤더에 정의된 것만 알면 되고, HD44780/PCF8574 가 뭔지는 몰라도 된다.
 */

#ifndef _LCD1602_H_
#define _LCD1602_H_

#include <linux/ioctl.h>
#include <linux/types.h>

#define LCD_COLS 16
#define LCD_ROWS 2

/* SET_CURSOR 용 인자 구조체 */
struct lcd_cursor {
	__u8 col; /* 0 ~ 15 */
	__u8 row; /* 0 ~ 1  */
};

#define LCD_IOC_MAGIC 'L'

/* 화면 전체 지우고 커서를 (0,0)으로 */
#define LCD_IOC_CLEAR      _IO(LCD_IOC_MAGIC, 0)

/* 커서만 (0,0)으로 (내용은 유지) */
#define LCD_IOC_HOME       _IO(LCD_IOC_MAGIC, 1)

/* 커서 위치 지정 — arg: struct lcd_cursor * */
#define LCD_IOC_SET_CURSOR _IOW(LCD_IOC_MAGIC, 2, struct lcd_cursor)

/* 백라이트 on(1)/off(0) — arg: int * */
#define LCD_IOC_BACKLIGHT  _IOW(LCD_IOC_MAGIC, 3, int)

/* 디스플레이 on(1)/off(0) — arg: int * */
#define LCD_IOC_DISPLAY    _IOW(LCD_IOC_MAGIC, 4, int)

#define LCD_IOC_MAXNR 4

#endif /* _LCD1602_H_ */
