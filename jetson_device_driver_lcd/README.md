# lcd1602-driver

Jetson Orin Nano Super + 1602A(PCF8574 I2C 백팩)용 리눅스 캐릭터 디바이스 드라이버.

## 파일 구조

```
lcd1602-driver/
├── lcd1602.h                   앱 ↔ 드라이버 계약 (ioctl 정의)
├── lcd1602_drv.c               커널 모듈 본체
├── Makefile
├── tegra-lcd1602-overlay.dts   디바이스트리 오버레이
└── test/lcd_test.c             유저스페이스 테스트 앱
```

## 계층

```
lcd_test.c (앱)  --write()/ioctl()-->  /dev/lcd1602-0
                                            |
                                    lcd1602_drv.c
                            HD44780 커맨드 규칙을 아는 유일한 곳
                                            |
                                  i2c_smbus_write_byte()
                                            |
                                PCF8574 → HD44780 → 1602A
```

---

## 진행 순서 (단계별 검증 포함)

### 0. 배선

| LCD 백팩 | Jetson 40핀 |
|---|---|
| VCC | 5V (핀 2 또는 4) |
| GND | GND (핀 6) |
| SDA | 핀 3 |
| SCL | 핀 5 |

> 백팩 로직은 3.3V I2C와 대체로 호환되지만, 통신이 불안정하면 레벨 시프터를 넣는다.

### 1. I2C 버스 / 주소 확인

```bash
sudo apt install -y i2c-tools
ls /dev/i2c-*
sudo i2cdetect -y -r 7
```

**확인 포인트**: `0x27`(PCF8574) 또는 `0x3F`(PCF8574A)가 표에 잡히면 배선 정상.
주소가 `0x3F`면 오버레이의 `lcd1602@27` / `reg = <0x27>`을 `3f` / `<0x3f>`로 바꾼다.

### 2. 빌드

```bash
sudo apt install -y build-essential
make
```

**확인 포인트**: `lcd1602_drv.ko` 생성.

### 3. 디바이스 등록 — 두 가지 방법

#### 방법 A: sysfs 수동 등록 (빠름, 먼저 이걸로 검증 추천)

```bash
sudo insmod lcd1602_drv.ko
echo lcd1602 0x27 | sudo tee /sys/bus/i2c/devices/i2c-7/new_device
dmesg | tail
```

**확인 포인트**: `dmesg`에 `probing 1602A at addr 0x27` → `registered as /dev/lcd1602-0`.
LCD 백라이트가 켜지고 커서가 초기화되면 초기화 시퀀스 성공.

해제:
```bash
echo 0x27 | sudo tee /sys/bus/i2c/devices/i2c-7/delete_device
sudo rmmod lcd1602_drv
```

#### 방법 B: 디바이스트리 오버레이 (부팅 시 자동 등록, "제대로 된" 방법)

먼저 target-path를 실제 값으로 확인:
```bash
ls -l /sys/bus/i2c/devices/i2c-7/of_node
```
출력 경로에서 `/proc/device-tree`를 뗀 나머지를 `.dts`의 `target-path`에 넣는다.

```bash
sudo apt install -y device-tree-compiler
dtc -O dtb -o tegra-lcd1602-overlay.dtbo -@ tegra-lcd1602-overlay.dts
sudo cp tegra-lcd1602-overlay.dtbo /boot/
sudo /opt/nvidia/jetson-io/config-by-hardware.py -l   # 오버레이 목록 확인
```

**확인 포인트**: 재부팅 후 `dmesg | grep lcd1602`에 probe 로그, `/dev/lcd1602-0` 존재.

### 4. 노드 확인

```bash
ls -l /dev/lcd1602-0
```

권한 때문에 매번 sudo가 필요하면 udev 룰 추가:
```bash
echo 'KERNEL=="lcd1602-*", MODE="0666"' | sudo tee /etc/udev/rules.d/99-lcd1602.rules
sudo udevadm control --reload && sudo udevadm trigger
```

### 5. 동작 테스트

```bash
make test
./test/lcd_test
```

**확인 포인트**: 1줄에 `Rear Zone Ctrl`, 2줄에 속도/조향 값이 1초마다 갱신,
백라이트 껐다 켜짐, 마지막에 개행 처리 확인.

셸에서 바로 쏘는 것도 가능:
```bash
echo -n "Hello LCD" > /dev/lcd1602-0
```

### 6. UDP 앱과 연결 (다음 단계)

기존 STM32 수신 코드에서:

```c
int lcd = open("/dev/lcd1602-0", O_WRONLY);

while (1) {
    recvfrom(sock, &status, sizeof(status), 0, NULL, NULL);

    struct lcd_cursor c = { 0, 0 };
    char line[17];

    ioctl(lcd, LCD_IOC_SET_CURSOR, &c);
    snprintf(line, sizeof(line), "SPD:%3d STR:%+3d", status.speed, status.steer);
    write(lcd, line, strlen(line));
}
```

---

## 트러블슈팅

| 증상 | 확인할 것 |
|---|---|
| `i2cdetect`에 아무것도 안 뜸 | 배선, 5V 공급, 버스 번호(7이 아닐 수 있음) |
| 백라이트만 켜지고 글자 없음 | 백팩 뒤 가변저항(콘트라스트) 돌려보기 |
| 글자가 깨져서 나옴 | 백팩 비트 배치가 다를 수 있음 → `PCF_RS/RW/EN/BL` 매핑 확인 |
| probe 로그 없음 | `dmesg \| grep -i i2c`, compatible 문자열 / 주소 일치 확인 |
| `insmod` 시 version magic 에러 | 커널 헤더 버전 불일치 → `uname -r`과 `/lib/modules/*/build` 확인 |
