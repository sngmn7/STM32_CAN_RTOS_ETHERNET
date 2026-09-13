#!/bin/bash
# confirm_ms 트레이드오프 스윕
#
#   각 confirm_ms 값마다 예상 임계(500 + confirm_ms) 주변으로 공백을 넣어
#   실제 오검출 임계가 어디인지 확인한다.
#
#   관측 지표는 TEST_FAILED 다. CONFIRMED 는 ISO 14229 대로 래치되어
#   과거 이력으로 계속 서 있으므로 현재 상태 판정에 쓸 수 없다.
CLI=/c/ST/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304/tools/bin/STM32_Programmer_CLI.exe
PLINK="/c/Program Files/PuTTY/plink.exe"
HK="SHA256:swAzBGACQB0mpaJ5F7ZWP6u2LqHuTyUQ5gjJIzLejhQ"
JET="nongsa@192.168.0.79"
SN=003F00343233510739363634

# front_dtc_defs[1].confirm_ms   (엔트리 8B, index1 = U0300, confirm 오프셋 +2)
ADDR=0x24000016
TIMEOUT=500          # JETSON_COMMAND_TIMEOUT_MS

N=${1:-2}
VALUES="${2:-200 500 1000 1500 2000}"

echo "### confirm_ms 스윕 — 오검출 임계 vs 검출 시간"
echo "    링크 타임아웃 ${TIMEOUT} ms 고정, 각 조건 ${N}회 반복"
echo
printf "  %-11s %-11s   %s\n" "confirm_ms" "예상임계" "공백 G(ms) -> TEST_FAILED 발생/시행"
echo "  ---------------------------------------------------------------------"

for V in $VALUES; do
  HV=$(printf "0x%04X" $V)
  "$CLI" -c port=SWD sn=$SN mode=HOTPLUG -w16 $ADDR $HV >/dev/null 2>&1
  RB=$("$CLI" -c port=SWD sn=$SN mode=HOTPLUG -r32 0x24000014 8 2>&1 \
        | grep -E "^0x2400" | awk '{print strtonum("0x" substr($3,1,4))}')
  if [ "$RB" != "$V" ]; then echo "  !! write 실패 ($V -> $RB), 건너뜀"; continue; fi

  TH=$((TIMEOUT + V))
  G1=$((TH - 400)); G2=$((TH - 150)); G3=$((TH + 150)); G4=$((TH + 400))
  [ $G1 -lt 550 ] && G1=550

  OUT=$("$PLINK" -ssh -P 22 -pw 1 -batch -hostkey "$HK" $JET \
        "python3 gap_probe.py -g $G1,$G2,$G3,$G4 -n $N" 2>/dev/null)

  LINE=""
  for row in $OUT; do
    G=$(echo "$row" | cut -d, -f2)
    F=$(echo "$row" | cut -d, -f4)
    R=$(echo "$row" | cut -d, -f5)
    LINE="$LINE  ${G}:${F}/${R}"
  done
  printf "  %-11s %-11s %s\n" "$V" "${TH} ms" "$LINE"
done

"$CLI" -c port=SWD sn=$SN mode=HOTPLUG -w16 $ADDR 0x05DC >/dev/null 2>&1
echo
echo "  confirm_ms 를 1500 으로 복구했습니다."
