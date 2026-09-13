#!/bin/bash
# Rear H723 의 perf_stat_t 하나를 읽어 출력한다.
CLI=/c/ST/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304/tools/bin/STM32_Programmer_CLI.exe
NM=$(ls /c/ST/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32*/tools/bin/arm-none-eabi-nm.exe | head -1)
ELF=/c/Users/PC/STM32CubeIDE/workspace_2.2.0/ZoneController_Rear_H723_real/Debug/ZoneController_Rear_H723_real.elf
SN=004900343234510237333934
SYM=${1:-perf_rear_cmd_latency}
LABEL=${2:-$SYM}

A=$("$NM" --defined-only "$ELF" | awk -v n="$SYM" '$3==n{print "0x"$1}')
if [ -z "$A" ]; then echo "심볼 없음: $SYM"; exit 1; fi

OUT=$("$CLI" -c port=SWD sn=$SN mode=HOTPLUG -r32 $A 24 2>&1 | grep -E "^0x[0-9A-Fa-f]{8} :")
VALS=$(echo "$OUT" | awk '{for(i=3;i<=NF;i++) print strtonum("0x"$i)}')
set -- $VALS
echo "### $LABEL   ($SYM @ $A)"
printf "  최근   %8s us\n" "$1"
printf "  최소   %8s us\n" "$2"
printf "  최대   %8s us\n" "$3"
printf "  평균   %8s us\n" "$4"
printf "  표본   %8s\n"    "$5"
printf "  1ms초과 %7s\n"   "$6"
