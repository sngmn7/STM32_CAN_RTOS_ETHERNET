/**
 * @file    eth_phy_recover.h
 * @brief   LAN8742 PHY 초기화 실패 복구
 *
 * 문제
 *   CubeMX 가 생성한 low_level_init() 은 부팅 시 LAN8742_Init() 을 딱 한 번
 *   호출한다. 이 함수는 MDIO 로 PHY 주소를 0~31 스캔해 응답하는 주소를 찾는데,
 *   리셋 직후 PHY 가 아직 MDIO 에 응답할 준비가 안 됐으면 전부 실패하고
 *   ADDRESS_ERROR 를 반환한다.
 *
 *   그러면 DevAddr 가 무효(32)로 남고 Is_Initialized 도 0 인데,
 *   아무도 재시도하지 않는다. 이후 ethernet_link_check_state() 가
 *   100 ms 마다 불려도 LAN8742_GetLinkState() 가 READ_ERROR(-5) 를 돌려주고,
 *
 *       else if(!netif_is_link_up(netif) && (PHYLinkState > LAN8742_STATUS_LINK_DOWN))
 *
 *   조건에서 -5 > 1 이 거짓이라 복구 분기에 진입조차 못 한다.
 *   결과적으로 이더넷이 영구히 죽고, 리셋만이 유일한 재시도 기회가 된다.
 *   (증상: "리셋을 몇 번 눌러야 가끔 잡힌다")
 *
 * 해결
 *   Is_Initialized 가 0 이면 주기적으로 LAN8742_Init() 을 재시도한다.
 *   성공하면 곧바로 이어지는 Ethernet_Link_Periodic_Handle() 이
 *   정상적으로 링크를 올린다.
 *
 * 배치
 *   ethernet_link_check_state() 안에는 USER CODE 블록이 없어 직접 고치면
 *   CubeMX 재생성 시 사라진다. 그래서 별도 파일로 두고 lwip.c 의
 *   USER CODE BEGIN 4_1 에서 호출한다.
 */

#ifndef ETH_PHY_RECOVER_H
#define ETH_PHY_RECOVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief 기능 스위치
 *
 * 1 = PHY 초기화 실패 시 200 ms 주기로 재시도
 * 0 = 아무것도 하지 않음 (CubeMX 기본 동작과 동일)
 *
 * 재시도는 LAN8742_Init() 을 부르는데, 실패 시 32개 주소를 MDIO 스캔하므로
 * PHY 가 아예 응답하지 않는 상황에서는 비용만 발생하고 복구되지 않는다.
 * 원인 격리가 필요할 때 0 으로 두고 A/B 비교한다.
 */
#define ETH_PHY_RECOVER_ENABLE   0

/**
 * @brief PHY 초기화 실패 시 재시도 (MX_LWIP_Process 앞에서 매번 호출)
 *
 * 내부에서 주기를 제한하므로 매 루프 호출해도 된다.
 * 이미 초기화된 상태면 즉시 반환하여 오버헤드가 없다.
 */
void EthPhy_RecoverPoll(void);

/* 디버거/텔레메트리 관찰용 */
extern volatile uint32_t eth_phy_retry_count;    /**< 재시도 횟수        */
extern volatile uint32_t eth_phy_recovered;      /**< 재시도로 살아난 횟수 */
extern volatile uint8_t  eth_phy_initialized;    /**< 현재 초기화 상태   */

#ifdef __cplusplus
}
#endif

#endif /* ETH_PHY_RECOVER_H */
