#!/bin/bash
# 상행 송신율 확인: 텔레메트리(H723) + 이산상태 CAN(F407)
CLI=/c/ST/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304/tools/bin/STM32_Programmer_CLI.exe
NM=$(ls /c/ST/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32*/tools/bin/arm-none-eabi-nm.exe | head -1)
H_ELF=/c/stm_workspace/ZoneController_Front_H723/Debug/ZoneController_Front_H723.elf
F_ELF=/c/stm_workspace/F407_Front/Debug/F407_Front.elf
H_SN=003F00343233510739363634
F_SN=066EFF575377524867034154
W=${2:-60}

addr() { "$NM" --defined-only "$1" | awk -v n="$2" '$3==n{print "0x"$1}'; }
rd()   { "$CLI" -c port=SWD sn=$1 mode=HOTPLUG -r32 $2 4 2>&1 \
           | grep -E "^0x[0-9A-Fa-f]{8} :" | tail -1 | awk '{print strtonum("0x"$3)}'; }

A_FT=$(addr  $H_ELF ft_tx_count)
A_HL=$(addr  $F_ELF can_headlamp_output_tx_count)
A_TL=$(addr  $F_ELF can_turn_left_tx_count)
A_SW=$(addr  $F_ELF can_turn_switch_tx_count)
A_AD=$(addr  $F_ELF can_steering_tx_count)

echo "### ${1:-측정}  (${W}s, 무부하)"
f0=$(rd $H_SN $A_FT); h0=$(rd $F_SN $A_HL); t0=$(rd $F_SN $A_TL); s0=$(rd $F_SN $A_SW); a0=$(rd $F_SN $A_AD)
sleep $W
f1=$(rd $H_SN $A_FT); h1=$(rd $F_SN $A_HL); t1=$(rd $F_SN $A_TL); s1=$(rd $F_SN $A_SW); a1=$(rd $F_SN $A_AD)

row() { awk -v d=$(( $3 - $2 )) -v w=$W -v n="$1" -v e="$4" \
        'BEGIN{printf "  %-26s %6d 회  %6.1f Hz   (설계 %s)\n", n, d, d/w, e}'; }
row "SV 텔레메트리"        $f0 $f1 "10 Hz"
row "헤드램프 상태 CAN"    $h0 $h1 "20 Hz"
row "방향지시 좌 CAN"      $t0 $t1 "20 Hz"
row "토글스위치 CAN"       $s0 $s1 "20 Hz"
row "조향 ADC CAN(대조군)" $a0 $a1 "50 Hz"
