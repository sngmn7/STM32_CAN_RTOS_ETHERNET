#!/bin/bash
# CAN250 하행 유실 정밀 측정 (계측 카운터 사용)
CLI=/c/ST/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304/tools/bin/STM32_Programmer_CLI.exe
NM=$(ls /c/ST/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32*/tools/bin/arm-none-eabi-nm.exe | head -1)

H_ELF=/c/stm_workspace/ZoneController_Front_H723/Debug/ZoneController_Front_H723.elf
F_ELF=/c/stm_workspace/F407_Front/Debug/F407_Front.elf
H_SN=003F00343233510739363634
F_SN=066EFF575377524867034154

LABEL=${1:-측정}
WINDOW=${2:-180}

addr() { "$NM" --defined-only "$1" | awk -v n="$2" '$3==n{print "0x"$1}'; }
rd()   { "$CLI" -c port=SWD sn=$1 mode=HOTPLUG -r32 $2 1 2>&1 \
           | grep -E "^0x[0-9A-Fa-f]{8} :" | tail -1 | awk '{print strtonum("0x"$3)}'; }

A_TX=$(addr   $H_ELF front_command_tx_count)
A_TXE=$(addr  $H_ELF front_command_tx_error_count)
A_TXB=$(addr  $H_ELF front_command_tx_busy_count)
A_RX=$(addr   $F_ELF can_command_rx_count)
A_FOV=$(addr  $F_ELF can_fifo_overrun_count)
A_ERR=$(addr  $F_ELF can_error_callback_count)
A_BLK=$(addr  $F_ELF dht11_block_max_us)
A_BLL=$(addr  $F_ELF dht11_block_last_us)
A_DOK=$(addr  $F_ELF dht11_ok_count)
A_DER=$(addr  $F_ELF dht11_error_count)

snapH() { echo "$(rd $H_SN $A_TX) $(rd $H_SN $A_TXE) $(rd $H_SN $A_TXB)"; }
snapF() { echo "$(rd $F_SN $A_RX) $(rd $F_SN $A_FOV) $(rd $F_SN $A_ERR) $(rd $F_SN $A_DOK) $(rd $F_SN $A_DER)"; }

echo "### $LABEL  (창 ${WINDOW}s)"
read t0 te0 tb0 <<< "$(snapH)"; read r0 f0 e0 d0 x0 <<< "$(snapF)"
sleep $WINDOW
read t1 te1 tb1 <<< "$(snapH)"; read r1 f1 e1 d1 x1 <<< "$(snapF)"

DT=$((t1-t0)); DR=$((r1-r0)); DF=$((f1-f0)); DE=$((e1-e0))
DD=$((d1-d0)); DX=$((x1-x0)); DTE=$((te1-te0)); DTB=$((tb1-tb0))
LOSS=$((DT-DR))

echo "  H723 하행 송신     $DT   (tx_error $DTE / tx_busy $DTB)"
echo "  F407 하행 수신     $DR"
echo "  유실               $LOSS"
if [ $DT -gt 0 ]; then
  awk -v l=$LOSS -v t=$DT 'BEGIN{printf "  유실률             %.3f %%\n", l*100/t}'
fi
echo "  RX FIFO 오버런     $DF"
echo "  CAN 에러 콜백      $DE"
echo "  DHT11              $DD ok / $DX err"
echo "  IRQ 차단 최대      $(rd $F_SN $A_BLK) us   (최근 $(rd $F_SN $A_BLL) us)"
