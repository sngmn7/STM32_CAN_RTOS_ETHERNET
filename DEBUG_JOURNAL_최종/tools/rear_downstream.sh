#!/bin/bash
# Rear 하행 유실률 + 0x323 고아 신호 영향 측정
#   Rear H723 led_cmd_tx_count  <->  F446 led_cmd_rx_count   (짝지어 뺄셈)
#   F446 can250_rx_bad_count 증가율 = 아무도 안 받는 0x323 의 흔적
CLI=/c/ST/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304/tools/bin/STM32_Programmer_CLI.exe
NM=$(ls /c/ST/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32*/tools/bin/arm-none-eabi-nm.exe | head -1)

R_ELF=/c/Users/PC/STM32CubeIDE/workspace_2.2.0/ZoneController_Rear_H723_real/Debug/ZoneController_Rear_H723_real.elf
N_ELF=/c/Users/PC/STM32CubeIDE/workspace_2.2.0/Node_F447_Real/Debug/Node_F447_Real.elf
R_SN=004900343234510237333934
N_SN=0668FF515086874967255335
W=${1:-180}

a(){ "$NM" --defined-only "$1" | awk -v n="$2" '$3==n{print "0x"$1}'; }
rd(){ "$CLI" -c port=SWD sn=$1 mode=HOTPLUG -r32 $2 4 2>&1 \
        | grep -E "^0x[0-9A-Fa-f]{8} :" | tail -1 | awk '{print strtonum("0x"$3)}'; }

R_TX=$(a $R_ELF led_cmd_tx_count)
R_WTX=$(a $R_ELF window_cmd_tx_count)
N_RX=$(a $N_ELF led_cmd_rx_count)
N_ALL=$(a $N_ELF can250_rx_count)
N_BAD=$(a $N_ELF can250_rx_bad_count)
N_OK=$(a $N_ELF can250_rx_ok_count)

echo "### Rear 하행 CAN 유실 + 0x323 영향  (창 ${W}s)"
t0=$(rd $R_SN $R_TX);  w0=$(rd $R_SN $R_WTX)
r0=$(rd $N_SN $N_RX);  A0=$(rd $N_SN $N_ALL)
b0=$(rd $N_SN $N_BAD); o0=$(rd $N_SN $N_OK)
sleep $W
t1=$(rd $R_SN $R_TX);  w1=$(rd $R_SN $R_WTX)
r1=$(rd $N_SN $N_RX);  A1=$(rd $N_SN $N_ALL)
b1=$(rd $N_SN $N_BAD); o1=$(rd $N_SN $N_OK)

DT=$((t1-t0)); DW=$((w1-w0)); DR=$((r1-r0))
DA=$((A1-A0)); DB=$((b1-b0)); DO=$((o1-o0))

echo
echo "  [LED 명령 0x320/0x321/0x322]"
printf "    H723 송신        %6d 프레임  (%5.1f Hz)\n" $DT $(awk -v d=$DT -v w=$W 'BEGIN{print d/w}')
printf "    F446 수신        %6d 프레임  (%5.1f Hz)\n" $DR $(awk -v d=$DR -v w=$W 'BEGIN{print d/w}')
printf "    유실             %6d\n" $((DT-DR))
[ $DT -gt 0 ] && awk -v l=$((DT-DR)) -v t=$DT 'BEGIN{printf "    유실률           %8.3f %%\n", l*100/t}'
echo
echo "  [창문 명령 0x323 - 수신 노드 없음]"
printf "    H723 송신        %6d 프레임  (%5.1f Hz)\n" $DW $(awk -v d=$DW -v w=$W 'BEGIN{print d/w}')
printf "    F446 처리 못한 프레임 %6d  (%5.1f Hz)\n" $DB $(awk -v d=$DB -v w=$W 'BEGIN{print d/w}')
echo
echo "  [F446 전체 수신]"
printf "    총 수신          %6d 프레임  (%5.1f Hz)\n" $DA $(awk -v d=$DA -v w=$W 'BEGIN{print d/w}')
printf "    핑 에코 정상     %6d\n" $DO
