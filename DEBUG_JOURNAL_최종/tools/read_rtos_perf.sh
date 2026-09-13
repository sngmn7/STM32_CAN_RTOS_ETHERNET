#!/bin/bash
# read_rtos_perf.sh — RTOS 전환 후의 태스크 주기·스택·힙을 읽어 CSV 로 낸다.
#
# perf_stat_t = { last, min, max, avg, count, over_1ms } (u32 x6 = 24 B)
# STM32_Programmer_CLI -r32 의 길이 인자는 워드가 아니라 '바이트'다.
#
#   ./read_rtos_perf.sh > exec_time_rtos.csv

CLI=/c/ST/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304/tools/bin/STM32_Programmer_CLI.exe
NM="C:/ST/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740/tools/bin/arm-none-eabi-nm.exe"

REAR_ELF="C:/Users/PC/STM32CubeIDE/workspace_2.2.0/ZoneController_Rear_H723_real/Release/ZoneController_Rear_H723_real.elf"
FRONT_ELF="C:/stm_workspace/ZoneController_Front_H723/Debug/ZoneController_Front_H723.elf"
REAR_SN=004900343234510237333934
FRONT_SN=003F00343233510739363634

addr() { "$NM" --defined-only "$1" | awk -v n="$2" '$3==n{print "0x"$1}'; }

# perf_stat_t 한 개를 읽어 "min max avg count over1ms" 로 출력
perf() {   # $1=elf $2=sn $3=symbol
  local a; a=$(addr "$1" "$3"); [ -z "$a" ] && { echo ",,,,"; return; }
  "$CLI" -c port=SWD sn=$2 mode=HOTPLUG -r32 "$a" 24 2>&1 \
    | grep -E "^0x" \
    | awk '{for(i=3;i<=NF;i++) printf "%d ", strtonum("0x"$i)}' \
    | awk '{printf "%s,%s,%s,%s,%s", $2, $3, $4, $5, $6}'
}

u32() {    # $1=elf $2=sn $3=symbol
  local a; a=$(addr "$1" "$3"); [ -z "$a" ] && { echo ""; return; }
  "$CLI" -c port=SWD sn=$2 mode=HOTPLUG -r32 "$a" 4 2>&1 \
    | grep -E "^0x" | awk '{print strtonum("0x"$3)}'
}

echo "보드,태스크,심볼,종류,min_us,max_us,avg_us,표본수,1ms초과"
for t in wheel:500k_핑 ctrl:250k_하행지령 net:텔레메트리송신 diag:DTC판정; do
  s=${t%%:*}; d=${t##*:}
  echo "Rear_H723,$s ($d),perf_t_$s,태스크 주기,$(perf "$REAR_ELF" $REAR_SN perf_t_$s)"
done
echo "Rear_H723,브레이크 지령->CAN송신,perf_brake_latency,지연,$(perf "$REAR_ELF" $REAR_SN perf_brake_latency)"

for t in ctrl:하행명령7종 net:텔레메트리송신 diag:DTC판정; do
  s=${t%%:*}; d=${t##*:}
  echo "Front_H723,$s ($d),perf_t_$s,태스크 주기,$(perf "$FRONT_ELF" $FRONT_SN perf_t_$s)"
done
echo "Front_H723,종단 반영 (지령->F407되보고),perf_reflect_latency,지연,$(perf "$FRONT_ELF" $FRONT_SN perf_reflect_latency)"
echo "Front_H723,FDCAN RX ISR,perf_can_isr,실행시간,$(perf "$FRONT_ELF" $FRONT_SN perf_can_isr)"
