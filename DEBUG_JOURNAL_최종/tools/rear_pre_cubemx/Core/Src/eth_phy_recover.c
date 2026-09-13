/**
 * @file    eth_phy_recover.c
 * @brief   LAN8742 PHY 초기화 실패 복구 구현
 */

#include "eth_phy_recover.h"

#include "stm32h7xx_hal.h"
#include "lan8742.h"

/**
 * 재시도 주기.
 *
 * LAN8742_Init() 은 실패 시 32개 주소 x MDIO 읽기를 수행하며,
 * 각 읽기는 HAL 타임아웃까지 기다릴 수 있어 비용이 작지 않다.
 * 부팅 직후 PHY 안정화에는 수십~수백 ms 면 충분하므로
 * 200 ms 간격이면 복구 속도와 부하가 모두 무난하다.
 */
#define ETH_PHY_RETRY_PERIOD_MS   200U

/* ethernetif.c 에서 정의된 PHY 객체 (static 아님) */
extern lan8742_Object_t LAN8742;

volatile uint32_t eth_phy_retry_count = 0U;
volatile uint32_t eth_phy_recovered = 0U;
volatile uint8_t  eth_phy_initialized = 0U;

static uint32_t eth_phy_last_try_ms = 0U;


void EthPhy_RecoverPoll(void)
{
#if (ETH_PHY_RECOVER_ENABLE == 0)
	/* 기능 비활성 — 관찰용 플래그만 갱신한다 */
	eth_phy_initialized = (LAN8742.Is_Initialized != 0U) ? 1U : 0U;
	return;
#else
	uint32_t now;

	if (LAN8742.Is_Initialized != 0U) {
		/* 정상 — 관찰용 플래그만 갱신하고 즉시 반환 */
		eth_phy_initialized = 1U;
		return;
	}

	eth_phy_initialized = 0U;

	now = HAL_GetTick();

	if ((uint32_t)(now - eth_phy_last_try_ms) < ETH_PHY_RETRY_PERIOD_MS) {
		return;
	}

	eth_phy_last_try_ms = now;
	eth_phy_retry_count++;

	/*
	 * 재시도. 성공하면 Is_Initialized 가 1 이 되고 DevAddr 가 유효해지며,
	 * 곧바로 이어 호출되는 Ethernet_Link_Periodic_Handle() 이
	 * ethernet_link_check_state() 를 통해 링크를 올린다.
	 */
	if (LAN8742_Init(&LAN8742) == LAN8742_STATUS_OK) {
		eth_phy_initialized = 1U;
		eth_phy_recovered++;
	}
#endif /* ETH_PHY_RECOVER_ENABLE */
}
