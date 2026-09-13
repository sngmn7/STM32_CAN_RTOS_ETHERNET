#!/usr/bin/env python3
"""Front H723 슈퍼루프 -> FreeRTOS 태스크 2개로 분리."""
import io

P = r"c:\stm_workspace\ZoneController_Front_H723\Core\Src\main.c"
s = io.open(P, encoding="utf-8", newline="").read()
NL = "\r\n" if "\r\n" in s else "\n"

# ------------------------------------------------------------------ #
# 1) 인클루드                                                          #
# ------------------------------------------------------------------ #
anc = '#include "eth_phy_recover.h"'
assert anc in s
s = s.replace(anc, anc + NL + '#include "FreeRTOS.h"' + NL + '#include "task.h"', 1)

# ------------------------------------------------------------------ #
# 2) 기존 루프 본문 잘라내기                                            #
# ------------------------------------------------------------------ #
B = "/* USER CODE BEGIN WHILE */"
E = "/* USER CODE END WHILE */"
i = s.index(B) + len(B)
j = s.index(E)
body = s[i:j]
assert "MX_LWIP_Process" in body

# 본문에서 두 갈래로 나눌 조각을 뽑는다
k_can_start = body.index("      CAN_UpdateDiagnostics();")
k_net_start = body.index("      FrontZoneNetwork_Process(now);")
can_part = body[k_can_start:k_net_start].rstrip()
net_part = body[k_net_start:].rstrip()

# ------------------------------------------------------------------ #
# 3) 스텝 함수 + 태스크 정의를 CAN_BusOffRecover 뒤에 삽입              #
# ------------------------------------------------------------------ #
def reindent(txt, frm="      ", to="    "):
    out = []
    for ln in txt.split(NL):
        out.append(to + ln[len(frm):] if ln.startswith(frm) else ln)
    return NL.join(out)

can_body = reindent(can_part)
net_body = reindent(net_part)

tasks = NL.join([
"",
"/* ================================================================== */",
"/* FreeRTOS 태스크 구성                                                */",
"/*                                                                    */",
"/* 전환 목적은 응답성 개선이 아니다. 슈퍼루프는 이미 38,660 Hz 로 돌고 */",
"/* 최악 주기가 477 us 라 성능상 얻을 것이 없다(METRICS.md 6절).        */",
"/* 목적은 구조다 — 기능별 태스크 분리, 우선순위로 표현되는 중요도,     */",
"/* 기능 추가 시 다른 기능의 주기에 영향을 주지 않는 것.                */",
"/*                                                                    */",
"/* 우선순위                                                           */",
"/*   CTRL(3) > NET(2)  — CAN 은 차량 구동에 직접 관여하고, 네트워크는  */",
"/*   텔레메트리·명령 수신이라 한 틱 늦어도 안전에 영향이 없다.         */",
"/*                                                                    */",
"/* lwIP 는 NO_SYS=1 을 유지한다. 따라서 **NET 태스크만** lwIP 를       */",
"/* 만져야 한다. 텔레메트리·진단 송신도 전부 NET 에 둔 이유다.          */",
"/* ================================================================== */",
"#define TASK_CTRL_PRIO        3U",
"#define TASK_NET_PRIO         2U",
"#define TASK_CTRL_STACK_WORDS 512U    /* 2 KB */",
"#define TASK_NET_STACK_WORDS  2048U   /* 8 KB — lwIP 호출 깊이 대비 */",
"#define TASK_CTRL_PERIOD_MS   1U",
"#define TASK_NET_PERIOD_MS    1U",
"",
"volatile perf_stat_t perf_task_ctrl = { 0U, UINT32_MAX, 0U, 0U, 0U, 0U };",
"volatile perf_stat_t perf_task_net  = { 0U, UINT32_MAX, 0U, 0U, 0U, 0U };",
"",
"static TaskHandle_t h_task_ctrl = NULL;",
"static TaskHandle_t h_task_net  = NULL;",
"",
"/** @brief CAN 제어 — 진단 갱신, 버스오프 복구, 핑, 하행 명령 송신 */",
"static void App_CanStep(uint32_t now)",
"{",
can_body,
"}",
"",
"/** @brief 네트워크 — lwIP 및 모든 UDP 송수신. 이 태스크만 lwIP 를 만진다. */",
"static void App_NetStep(uint32_t now)",
"{",
net_body,
"}",
"",
"static void Task_Ctrl(void *arg)",
"{",
"    TickType_t last = xTaskGetTickCount();",
"    uint32_t prev_cyc = 0U;",
"",
"    (void)arg;",
"    for (;;)",
"    {",
"#if (FRONT_INSTRUMENT_ENABLE)",
"        {",
"            uint32_t cyc = PERF_NOW();",
"            if (prev_cyc != 0U) { Perf_Update(&perf_task_ctrl, cyc - prev_cyc); }",
"            prev_cyc = cyc;",
"        }",
"#endif",
"        App_CanStep(HAL_GetTick());",
"        vTaskDelayUntil(&last, pdMS_TO_TICKS(TASK_CTRL_PERIOD_MS));",
"    }",
"}",
"",
"static void Task_Net(void *arg)",
"{",
"    TickType_t last = xTaskGetTickCount();",
"    uint32_t prev_cyc = 0U;",
"",
"    (void)arg;",
"    for (;;)",
"    {",
"#if (FRONT_INSTRUMENT_ENABLE)",
"        {",
"            uint32_t cyc = PERF_NOW();",
"            if (prev_cyc != 0U) { Perf_Update(&perf_task_net, cyc - prev_cyc); }",
"            prev_cyc = cyc;",
"        }",
"#endif",
"        App_NetStep(HAL_GetTick());",
"        vTaskDelayUntil(&last, pdMS_TO_TICKS(TASK_NET_PERIOD_MS));",
"    }",
"}",
"",
])


# ------------------------------------------------------------------ #
# 4) while(1) 을 스케줄러 기동으로 교체                                 #
# ------------------------------------------------------------------ #
new_main = NL.join([
"",
"  /*",
"   * 여기서부터는 스케줄러가 돈다. 슈퍼루프 본문은 App_CanStep() /",
"   * App_NetStep() 으로 그대로 옮겨졌고, 동작 순서와 주기 판정은",
"   * 바꾸지 않았다. 바뀐 것은 '누가 언제 부르는가' 뿐이다.",
"   */",
"  if (xTaskCreate(Task_Ctrl, \"ctrl\", TASK_CTRL_STACK_WORDS, NULL,",
"                  TASK_CTRL_PRIO, &h_task_ctrl) != pdPASS)",
"  {",
"      Error_Handler();",
"  }",
"  if (xTaskCreate(Task_Net, \"net\", TASK_NET_STACK_WORDS, NULL,",
"                  TASK_NET_PRIO, &h_task_net) != pdPASS)",
"  {",
"      Error_Handler();",
"  }",
"",
"  vTaskStartScheduler();",
"",
"  /* 여기 도달하면 힙이 모자라 스케줄러가 뜨지 못한 것이다. */",
"  Error_Handler();",
"",
"  while (1)",
"  {",
"    ",
])
s = s[:i] + new_main + s[j:]

# 태스크 정의 삽입은 while 치환 이후에 해야 한다.
# (먼저 하면 i/j 인덱스가 밀려 엉뚱한 곳을 자른다 — 실제로 겪음)
marker = "static void CAN_UpdateDiagnostics(void)" + NL + "{"
assert marker in s
s = s.replace(marker, tasks + NL + marker, 1)

io.open(P, "w", encoding="utf-8", newline="").write(s)
print("refactor 완료")
print("  App_CanStep  %d줄" % len(can_body.split(NL)))
print("  App_NetStep  %d줄" % len(net_body.split(NL)))
