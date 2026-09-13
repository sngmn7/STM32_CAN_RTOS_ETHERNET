# Front H723 FreeRTOS 이식본 (보관)

2026-08-03 작업. 검증까지 마쳤으나 사용자 요청으로 롤백함.

## 구성
- FreeRTOS (STM32Cube_FW_H7 V1.13.0), heap_4, 힙 32 KB
- lwIP 는 NO_SYS=1 유지 (CubeMX 를 쓰지 않은 이유는 METRICS.md 6-b 절)
- 태스크: CTRL(prio 3, 알림 기반) / NET(prio 1, 배경 폴링)
- SysTick 을 HAL 과 공유 (스케줄러 기동 후에만 커널 틱)

## 측정 결과 (같은 조건 A/B)
슈퍼루프 대비 평균 지표는 전부 동등, 최악 명령지연만 79 -> 759 us.
자세한 수치는 METRICS.md 6-b 절.

## 되살리려면
1. main_rtos.c -> Core/Src/main.c
2. stm32h7xx_it_rtos.c -> Core/Src/stm32h7xx_it.c
3. FreeRTOSConfig.h -> Core/Inc/
4. freertos_hooks.c -> Core/Src/
5. FreeRTOS 소스를 Middlewares/Third_Party/FreeRTOS 에 복사
   (STM32Cube_FW_H7_V1.13.0/Middlewares/Third_Party/FreeRTOS/Source)
6. Debug/ 의 makefile, objects.list, subdir.mk 들에 FreeRTOS 항목 추가
   (tools/rtos_refactor.py 및 세션 기록 참조)
