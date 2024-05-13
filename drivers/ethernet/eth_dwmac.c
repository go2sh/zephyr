/*
 * Driver for Synopsys DesignWare MAC
 *
 * Copyright (c) 2021 BayLibre SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT snps_designware_ethernet
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(dwmac_core, CONFIG_ETHERNET_LOG_LEVEL);

#include <sys/types.h>
#include <zephyr/kernel.h>
#include <zephyr/kernel/mm.h>
#include <zephyr/cache.h>
#include <zephyr/device.h>
#include <zephyr/drivers/ethernet/eth_dwc_eth_qos_platform.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/phy.h>
#include <zephyr/sys/barrier.h>
#include <zephyr/sys/util.h>
#include <ethernet/eth_stats.h>

#include "eth_dwmac_priv.h"
#include "eth.h"

/*
 * This driver references network data fragments with a zero-copy approach.
 * Even though the hardware can store received packets with an arbitrary
 * offset in memory, the gap bytes in the first word will be overwritten,
 * and subsequent fragments have to be buswidth-aligned anyway.
 * This means CONFIG_NET_BUF_VARIABLE_DATA_SIZE requires special care due
 * to its refcount byte placement, so we take the easy way out for now.
 */
#ifdef CONFIG_NET_BUF_VARIABLE_DATA_SIZE
BUILD_ASSERT(sizeof(void *) == 4, "Size does not match bus width");
#define RX_FRAG_SIZE NET_ETH_MAX_FRAME_SIZE
#else
/* size of pre-allocated packet fragments */
#define RX_FRAG_SIZE CONFIG_NET_BUF_DATA_SIZE
#endif

/*
 * Grace period to wait for TX descriptor/fragment availability.
 * Worst case estimate is 1514*8 bits at 10 mbps for an existing packet
 * to be sent and freed, therefore 1ms is far more than enough.
 * Beyond that we'll drop the packet.
 */
#define TX_AVAIL_WAIT K_MSEC(1)

/* descriptor index iterators */
#define INC_WRAP(idx, size) ({ idx = (idx + 1) % size; })
#define DEC_WRAP(idx, size) ({ idx = (idx + size - 1) % size; })

/*
 * Descriptor physical location .
 * MMU is special here as we have a separate uncached mapping that is
 * different from the normal RAM virt_to_phys mapping.
 */
#ifdef CONFIG_MMU
#define TXDESC_PHYS_H(ch, idx) hi32(p->tx_ch[ch].descs_phys + (idx) * sizeof(struct dwmac_dma_desc))
#define TXDESC_PHYS_L(ch, idx) lo32(p->tx_ch[ch].descs_phys + (idx) * sizeof(struct dwmac_dma_desc))
#define RXDESC_PHYS_H(ch, idx) hi32(p->tx_ch[ch].descs_phys + (idx) * sizeof(struct dwmac_dma_desc))
#define RXDESC_PHYS_L(ch, idx) lo32(p->tx_ch[ch].descs_phys + (idx) * sizeof(struct dwmac_dma_desc))
#else
#define TXDESC_PHYS_H(ch, idx) phys_hi32(&p->tx_ch[ch].descs[idx])
#define TXDESC_PHYS_L(ch, idx) phys_lo32(&p->tx_ch[ch].descs[idx])
#define RXDESC_PHYS_H(ch, idx) phys_hi32(&p->rx_ch[ch].descs[idx])
#define RXDESC_PHYS_L(ch, idx) phys_lo32(&p->rx_ch[ch].descs[idx])
#endif

static inline uint32_t hi32(uintptr_t val)
{
	/* trickery to avoid compiler warnings on 32-bit build targets */
	if (sizeof(uintptr_t) > 4) {
		uint64_t hi = val;

		return hi >> 32;
	}
	return 0;
}

static inline uint32_t lo32(uintptr_t val)
{
	/* just a typecast return to be symmetric with hi32() */
	return val;
}

static inline uint32_t phys_hi32(void *addr)
{
	/* the default 1:1 mapping is assumed */
	return hi32((uintptr_t)addr);
}

static inline uint32_t phys_lo32(void *addr)
{
	/* the default 1:1 mapping is assumed */
	return lo32((uintptr_t)addr);
}

static struct k_spinlock net_pkt_slist_lock;

void net_pkt_slist_put(sys_slist_t *list, struct net_pkt *pkt)
{
	k_spinlock_key_t key;

	__ASSERT_NO_MSG(list);
	__ASSERT_NO_MSG(pkt);

	key = k_spin_lock(&net_pkt_slist_lock);
	sys_slist_append(list, &pkt->next);
	k_spin_unlock(&net_pkt_slist_lock, key);
}

struct net_pkt *net_pkt_slist_get(sys_slist_t *list)
{
	sys_snode_t *node;
	struct net_pkt *pkt;
	k_spinlock_key_t key;

	__ASSERT_NO_MSG(list);

	key = k_spin_lock(&net_pkt_slist_lock);
	node = sys_slist_get(list);
	k_spin_unlock(&net_pkt_slist_lock, key);

	pkt = SYS_SLIST_CONTAINER(node, pkt, next);

	return pkt;
}

static void dwmac_mtl_txq_init(const struct device *dev, uint8_t queue,
			       const struct dwmac_tx_queue *qcfg)
{
	const struct dwmac_config *cfg = dev->config;

	REG_WRITE(MTL_TXQn_OPERATION_MODE(queue),
		  ((qcfg->size + 255) / 256 << MTL_TXQn_OPERATION_MODE_TQS_POS) |
			  (qcfg->sf ? MTL_TXQn_OPERATION_MODE_TSF
				    : (qcfg->threshold << MTL_TXQn_OPERATION_MODE_TTC_POS)) |
			  MTL_TXQn_OPERATION_MODE_TXQEN);
}

static void dwmac_mtl_rxq_init(const struct device *dev, uint8_t queue,
			       const struct dwmac_rx_queue *qcfg)
{
	const struct dwmac_config *cfg = dev->config;

	REG_WRITE(MTL_RXQn_OPERATION_MODE(queue),
		  (qcfg->size / 256 << MTL_RXQn_OPERATION_MODE_RQS_POS) |
			  (qcfg->sf ? MTL_RXQn_OPERATION_MODE_RSF
				    : (qcfg->threshold << MTL_RXQn_OPERATION_MODE_RTC_POS)));
	REG_WRITE(MAC_RXQ_CTRL0,
		  (REG_READ(MAC_RXQ_CTRL0) & ~(0x3 << MAC_RXQ_CTRL0_RXQnEN_POS(queue))) |
			  ((qcfg->av_queue ? MAC_RXQ_CTRL0_RXQnEN_AV : MAC_RXQ_CTRL0_RXQnEN_GENERIC)
			   << MAC_RXQ_CTRL0_RXQnEN_POS(queue)));
}

static void dwmac_mtl_init(const struct device *dev)
{
	const struct dwmac_config *cfg = dev->config;
	uint32_t queue;

	for (queue = 0; queue < cfg->rx_queues_to_use; queue++) {
		dwmac_mtl_rxq_init(dev, queue, &cfg->rx_queues_cfg[queue]);
	}

	for (queue = 0; queue < cfg->tx_queues_to_use; queue++) {
		dwmac_mtl_txq_init(dev, queue, &cfg->tx_queues_cfg[queue]);
	}

	/* set up MTL */
	REG_WRITE(MTL_OPERATION_MODE,
		  (MTL_OPERATION_MODE_SCHALG_STRICT << MTL_OPERATION_MODE_SCHALG_POS));
}

static void dwmac_dma_rxc_init(const struct device *dev, uint8_t ch)
{
	const struct dwmac_config *cfg = dev->config;
	struct dwmac_priv *p = dev->data;

	REG_WRITE(DMA_CHn_RXDESC_LIST_HADDR(ch), RXDESC_PHYS_H(ch, 0));
	REG_WRITE(DMA_CHn_RXDESC_LIST_ADDR(ch), RXDESC_PHYS_L(ch, 0));
	REG_WRITE(DMA_CHn_RXDESC_RING_LENGTH(ch), p->rx_ch[ch].desc_count - 1);
	REG_WRITE(DMA_CHn_RX_CTRL(ch),
		  FIELD_PREP(DMA_CHn_RX_CTRL_PBL,
			     (cfg->burst_length > 32 ? cfg->burst_length / 8 : cfg->burst_length)) |
			  FIELD_PREP(DMA_CHn_RX_CTRL_RBSZ, RX_FRAG_SIZE));
}

static void dwmac_dma_txc_init(const struct device *dev, uint8_t ch)
{
	const struct dwmac_config *cfg = dev->config;
	struct dwmac_priv *p = dev->data;

	REG_WRITE(DMA_CHn_TXDESC_LIST_HADDR(ch), TXDESC_PHYS_H(ch, 0));
	REG_WRITE(DMA_CHn_TXDESC_LIST_ADDR(ch), TXDESC_PHYS_L(ch, 0));
	REG_WRITE(DMA_CHn_TXDESC_RING_LENGTH(ch), p->tx_ch[ch].desc_count - 1);
	REG_WRITE(DMA_CHn_TX_CTRL(ch),
		  FIELD_PREP(DMA_CHn_TX_CTRL_PBL,
			     (cfg->burst_length > 32 ? cfg->burst_length / 8 : cfg->burst_length))
		  //| FIELD_PREP(DMA_CHn_TX_CTRL_TCW, cfg->tx_queues_cfg[ch].weight)
	);
}

static void dwmac_dma_init(const struct device *dev)
{
	const struct dwmac_config *cfg = dev->config;
	uint8_t ch;
	uint32_t irq_enable[8] = {};

	REG_WRITE(DMA_SYSBUS_MODE,
#ifdef CONFIG_64BIT
		  DMA_SYSBUS_MODE_EAME |
#endif
			  (cfg->aal ? DMA_SYSBUS_MODE_AAL : 0) |
			  (cfg->fb ? DMA_SYSBUS_MODE_FB : 0));

	for (ch = 0; ch < MAX(cfg->rx_channel_to_use, cfg->tx_queues_to_use); ch++) {
		bool txen = ch < cfg->tx_queues_to_use;
		bool rxen = ch < cfg->rx_channel_to_use;

		REG_WRITE(DMA_CHn_IRQ_ENABLE(ch), DMA_CHn_IRQ_ENABLE_NIE | DMA_CHn_IRQ_ENABLE_FBEE |
							  DMA_CHn_IRQ_ENABLE_CDEE |
							  DMA_CHn_IRQ_ENABLE_AIE |
							  (rxen ? DMA_CHn_IRQ_ENABLE_RIE : 0) |
							  (txen ? DMA_CHn_IRQ_ENABLE_TIE : 0));

		if (txen) {
			dwmac_dma_txc_init(dev, ch);
			REG_WRITE(DMA_CHn_TX_CTRL(ch),
				  REG_READ(DMA_CHn_TX_CTRL(ch)) | DMA_CHn_TX_CTRL_ST);
		}
		if (rxen) {
			dwmac_dma_rxc_init(dev, ch);
			REG_WRITE(DMA_CHn_RX_CTRL(ch),
				  REG_READ(DMA_CHn_RX_CTRL(ch)) | DMA_CHn_RX_CTRL_SR);
		}
	}
}

static void dwmac_mac_init(const struct device *dev)
{
	const struct dwmac_config *cfg = dev->config;
	struct dwmac_priv *p = dev->data;

	REG_WRITE(MAC_CONF, REG_READ(MAC_CONF) |
				    (p->feature0 & MAC_HW_FEATURE0_RXCOESEL ? MAC_CONF_IPC : 0) |
				    MAC_CONF_CST | MAC_CONF_ACS);

	REG_WRITE(MAC_PKT_FILTER, REG_READ(MAC_PKT_FILTER) | MAC_PKT_FILTER_PM);

#if CONFIG_NET_VLAN
	REG_WRITE(MAC_VLAN_TAG)
#endif
}

static enum ethernet_hw_caps dwmac_caps(const struct device *dev)
{
	struct dwmac_priv *p = dev->data;
	enum ethernet_hw_caps caps = 0;

	if (p->feature0 & MAC_HW_FEATURE0_GMIISEL) {
		caps |= ETHERNET_LINK_1000BASE_T;
	}

	if (p->feature0 & MAC_HW_FEATURE0_MIISEL) {
		caps |= ETHERNET_LINK_10BASE_T | ETHERNET_LINK_100BASE_T;
	}

	if (p->feature0 & MAC_HW_FEATURE0_RXCOESEL) {
		caps |= ETHERNET_HW_RX_CHKSUM_OFFLOAD;
	}

	if (p->feature0 & MAC_HW_FEATURE0_TSSEL) {
		caps |= ETHERNET_PTP;
	}

	if (p->feature0 & MAC_HW_FEATURE0_HDSEL) {
		caps |= ETHERNET_DUPLEX_SET;
	}

	if ((p->feature2 & MAC_HW_FEATURE2_TXQCNT) != 0 &&
	    (p->feature2 & MAC_HW_FEATURE2_RXQCNT) != 0) {
		caps |= ETHERNET_PRIORITY_QUEUES;
	}

	caps |= ETHERNET_PROMISC_MODE;

	return caps;
}

/* for debug logs */
static inline int net_pkt_get_nbfrags(struct net_pkt *pkt)
{
	struct net_buf *frag;
	int nbfrags = 0;

	for (frag = pkt->buffer; frag; frag = frag->frags) {
		nbfrags++;
	}
	return nbfrags;
}

static int dwmac_send(const struct device *dev, struct net_pkt *pkt)
{
	const struct dwmac_config *cfg = dev->config;
	struct dwmac_priv *p = dev->data;
	struct net_buf *frag;
	unsigned int pkt_len = net_pkt_get_len(pkt);
	unsigned int d_idx;
	struct dwmac_dma_desc *d;
	uint32_t des2_flags, des3_flags;
	uint32_t dma_ch = 0;
	uint32_t frag_count = 0;

	LOG_DBG("pkt len/frags=%d/%d", pkt_len, net_pkt_get_nbfrags(pkt));

	/* initial flag values */
	des2_flags = (CONFIG_NET_PKT_TIMESTAMP && pkt->ptp_pkt ? (TDES2_TTSE) : 0);
	des3_flags = TDES3_FD | TDES3_OWN;

	/* map packet fragments */
	d_idx = p->tx_ch[dma_ch].tail;
	net_pkt_ref(pkt);
	frag = pkt->frags;
	do {
		LOG_DBG("desc sem/tail=%d/%d", k_sem_count_get(&p->tx_ch[dma_ch].desc_used),
			p->tx_ch[dma_ch].tail);

		/* reserve a free descriptor for this fragment */
		if (k_sem_take(&p->tx_ch[dma_ch].desc_used, TX_AVAIL_WAIT) != 0) {
			LOG_ERR("no more free tx descriptors");
			goto abort;
		}

		frag_count++;
		sys_cache_data_flush_range(frag->data, frag->len);

		/* if no more fragments after this one: */
		if (!frag->frags) {
			/* set those flags on the last descriptor */
			des2_flags |= TDES2_IOC;
			des3_flags |= TDES3_LD;
		}

		/* fill the descriptor */
		d = &p->tx_ch[dma_ch].descs[d_idx];
		d->des0 = phys_lo32(frag->data);
		d->des1 = phys_hi32(frag->data);
		d->des2 = frag->len | des2_flags;
		d->des3 = pkt_len | des3_flags;

		/* clear the FD flag on subsequent descriptors */
		des3_flags &= ~TDES3_FD;

		INC_WRAP(d_idx, p->tx_ch[dma_ch].desc_count);
		frag = frag->frags;
	} while (frag);

	/* make sure all the above made it to memory */
	barrier_dmem_fence_full();

	/* update the descriptor index tail */
	p->tx_ch[dma_ch].tail = d_idx;
	/* Store the packet */
	net_pkt_slist_put(&p->tx_ch[dma_ch].pkts, pkt);

	/* lastly notify the hardware */
	REG_WRITE(DMA_CHn_TXDESC_TAIL_PTR(dma_ch), TXDESC_PHYS_L(dma_ch, d_idx));

	return 0;

abort:
	net_pkt_unref(pkt);
	for (; frag_count > 0; frag_count--) {
		k_sem_give(&p->tx_ch[dma_ch].desc_used);
	}

	return -ENOMEM;
}

static void dwmac_tx_release(struct dwmac_priv *p, uint32_t dma_ch)
{
	unsigned int d_idx;
	struct dwmac_dma_desc *d;
	struct net_pkt *pkt;
	uint32_t des3_val;

	for (d_idx = p->tx_ch[dma_ch].head; d_idx != p->tx_ch[dma_ch].tail;
	     INC_WRAP(d_idx, p->tx_ch[dma_ch].desc_count),
	    k_sem_give(&p->tx_ch[dma_ch].desc_used)) {

		LOG_DBG("desc[%d] sem/tail/head=%d/%d/%d", dma_ch,
			k_sem_count_get(&p->tx_ch[dma_ch].desc_used), p->tx_ch[dma_ch].tail,
			p->tx_ch[dma_ch].head);

		d = &p->tx_ch[dma_ch].descs[d_idx];
		des3_val = d->des3;
		LOG_DBG("TDES3[%d] = 0x%08x", d_idx, des3_val);

		/* stop here if hardware still owns it */
		if (des3_val & TDES3_OWN) {
			break;
		}

		/* last packet descriptor: */
		if (des3_val & TDES3_LD) {
			/* log any errors */
			if (des3_val & TDES3_ES) {
				LOG_ERR("tx error (DES3 = 0x%08x)", des3_val);
				eth_stats_update_errors_tx(p->iface);
			}
			pkt = net_pkt_slist_get(&p->tx_ch[dma_ch].pkts);
#if CONFIG_NET_PKT_TIMESTAMP
			if (des3_val & TDES3_TTSS) {
				pkt->timestamp.nanosecond = d->des0;
				pkt->timestamp._sec.low = d->des1;
				net_if_add_tx_timestamp(pkt);
			}
#endif
			/* release packet ref */
			net_pkt_unref(pkt);
		}
	}
	p->tx_ch[dma_ch].head = d_idx;
}

static bool dwmac_rx_has_frag(const struct device *dev, uint32_t dma_ch)
{
	struct dwmac_priv *p = dev->data;
	return p->rx_ch[dma_ch].head != p->rx_ch[dma_ch].tail &&
	       (p->rx_ch[dma_ch].descs[p->rx_ch[dma_ch].head].des3 & RDES3_OWN) == 0;
}

static struct net_pkt *dwmac_receive(struct dwmac_priv *p, uint32_t dma_ch, struct net_pkt *pkt)
{
	struct dwmac_dma_desc *d;
	struct net_buf *frag;
	unsigned int d_idx;
	uint32_t des3_val;
	int ret;

	for (d_idx = p->rx_ch[dma_ch].head; d_idx != p->rx_ch[dma_ch].tail;
	     INC_WRAP(d_idx, p->rx_ch[dma_ch].desc_count),
	    k_sem_give(&p->rx_ch[dma_ch].desc_used)) {

		LOG_DBG("desc[%d] sem/tail/head=%d/%d/%d", dma_ch,
			k_sem_count_get(&p->rx_ch[dma_ch].desc_used), p->rx_ch[dma_ch].tail, d_idx);

		d = &p->rx_ch[dma_ch].descs[d_idx];
		des3_val = d->des3;
		LOG_DBG("RDES3[%d] = 0x%08x", d_idx, des3_val);

		/* stop here if hardware still owns it */
		if (des3_val & RDES3_OWN) {
			break;
		}

		/* a packet's first descriptor: */
		if (des3_val & RDES3_FD) {
			__ASSERT_NO_MSG(pkt == NULL);
			pkt = net_pkt_rx_alloc_on_iface(p->iface, K_NO_WAIT);
			if (!pkt) {
				LOG_ERR("net_pkt_rx_alloc_on_iface() failed");
				eth_stats_update_errors_rx(p->iface);
			}
		}

		/* retrieve current fragment */
		frag = net_buf_slist_get(&p->rx_ch[dma_ch].pkts);

		/* Check for valid packet */
		if (!pkt) {
			net_buf_unref(frag);
			LOG_ERR("no rx_pkt: skipping desc %d", d_idx);
			continue;
		}

		/* Check for descriptor error */
		if ((des3_val & (RDES3_CTXT | RDES3_FD)) == (RDES3_CTXT | RDES3_FD)) {
			net_buf_unref(frag);
			LOG_ERR("error descriptor seen. skipping desc %d", d_idx);
			continue;
		}

		/* Handle buffer fragment */
		if (des3_val & RDES3_CTXT) {
			net_buf_unref(frag);
		} else {
			net_buf_add(frag, FIELD_GET(RDES3_PL, des3_val) - net_pkt_get_len(pkt));
			net_pkt_frag_add(pkt, frag);
		}

#if defined(CONFIG_NET_PKT_TIMESTAMP)
		/* Handle timestamp from context descriptor */
		if (des3_val & RDES3_CTXT) {
			pkt->timestamp.nanosecond = d->des0;
			pkt->timestamp._sec.low = d->des1;
		}
#endif

		/* Packet reception error */
		if ((des3_val & (RDES3_ES | RDES3_LD)) == (RDES3_ES | RDES3_LD)) {
			LOG_ERR("rx error (DES3 = 0x%08x)", des3_val);
			eth_stats_update_errors_rx(p->iface);
			net_pkt_unref(pkt);
			pkt = NULL;
			continue;
		}

		if ((des3_val & RDES3_CTXT) |
		    ((des3_val & RDES3_LD) &&
		     (!CONFIG_NET_PKT_TIMESTAMP || !(d->des1 & RDES1_TSA)))) {
			LOG_DBG("pkt[%d] len/frags=%zd/%d", dma_ch, net_pkt_get_len(pkt),
				net_pkt_get_nbfrags(pkt));
			ret = net_recv_data(p->iface, pkt);
			if ((ret < 0)) {
				LOG_ERR("Failed to receive packet: %d", ret);
				net_pkt_unref(pkt);
			}
			pkt = NULL;
			continue;
		}
	}

	p->rx_ch[dma_ch].head = d_idx;
	return pkt;
}

static void dwmac_fill_rx_desc(const struct device *dev, uint32_t dma_ch)
{
	const struct dwmac_config *cfg = dev->config;
	struct dwmac_priv *p = dev->data;
	struct dwmac_dma_desc *d;
	struct net_buf *frag;
	uint32_t d_idx;
	uint32_t fill;

	for (d_idx = p->rx_ch[dma_ch].tail, fill = k_sem_count_get(&p->rx_ch[dma_ch].desc_used);
	     fill > 0; fill--, INC_WRAP(d_idx, p->rx_ch[dma_ch].desc_count),
	    k_sem_take(&p->rx_ch[dma_ch].desc_used, K_FOREVER)) {
		d = &p->rx_ch[dma_ch].descs[d_idx];

		__ASSERT(!(d->des3 & RDES3_OWN),
			 "desc[%d]=0x%x: still hw owned! (sem/head/tail=%d/%d/%d)", d_idx, d->des3,
			 k_sem_count_get(&p->rx_ch[dma_ch].desc_used), p->rx_ch[dma_ch].head,
			 p->rx_ch[dma_ch].tail);

		frag = net_pkt_get_reserve_rx_data(RX_FRAG_SIZE, K_FOREVER);
		if (!frag) {
			LOG_ERR("net_pkt_get_reserve_rx_data() returned NULL");
			k_sem_give(&p->rx_ch[dma_ch].desc_used);
			break;
		}

		LOG_DBG("new frag[%d] at %p", d_idx, frag->data);
		__ASSERT(frag->size == RX_FRAG_SIZE, "");
		sys_cache_data_invd_range(frag->data, frag->size);
		net_buf_slist_put(&p->rx_ch[dma_ch].pkts, frag);

		/* all is good: initialize the descriptor */
		d->des0 = phys_lo32(frag->data);
		d->des1 = phys_hi32(frag->data);
		d->des2 = 0;
		d->des3 = RDES3_BUF1V | RDES3_IOC | RDES3_OWN;
	}

	/* commit the above to memory */
	barrier_dmem_fence_full();

	/* Update */
	if (d_idx != p->rx_ch[dma_ch].tail) {
		REG_WRITE(DMA_CHn_RXDESC_TAIL_PTR(dma_ch), RXDESC_PHYS_L(dma_ch, d_idx));
		p->rx_ch[dma_ch].tail = d_idx;
	}
}

static void dwmac_thread(void *arg1, void *unused1, void *unused2)
{
	struct dwmac_priv *p = arg1;
	struct dwmac_dma_desc *d;
	struct net_buf *frag;
	unsigned int d_idx;

	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
}

static uint32_t dwmac_dma_irq(const struct device *dev, unsigned int ch)
{
	const struct dwmac_config *cfg = dev->config;
	struct dwmac_priv *p = dev->data;

	uint32_t status;
	uint32_t events = 0;

	status = REG_READ(DMA_CHn_STATUS(ch));
	LOG_DBG("DMA_CHn_STATUS(%d) = 0x%08x", ch, status);
	REG_WRITE(DMA_CHn_STATUS(ch), status);

	if (status & DMA_CHn_STATUS_AIS) {
		LOG_ERR("Abnormal Interrupt Status received (0x%x)", status);
	}

	if (status & DMA_CHn_STATUS_TI) {
		// events |= (1 << ch);
		dwmac_tx_release(dev->data, ch);
	}

	if (status & DMA_CHn_STATUS_RI) {
		// events |= (1 << (ch + 8));
		while (dwmac_rx_has_frag(dev, ch)) {
			dwmac_receive(p, ch, p->rx_ch[ch].packet);
			dwmac_fill_rx_desc(dev, ch);
		}
	}

	return events;
}

static void dwmac_mac_irq(const struct device *dev)
{
	const struct dwmac_config *cfg = dev->config;
	uint32_t status;

	status = REG_READ(MAC_IRQ_STATUS);
	LOG_DBG("MAC_IRQ_STATUS = 0x%08x", status);
	__ASSERT(false, "unimplemented");
}

static void dwmac_mtl_irq(const struct device *dev)
{
	const struct dwmac_config *cfg = dev->config;
	uint32_t status;

	status = REG_READ(MTL_IRQ_STATUS);
	LOG_DBG("MTL_IRQ_STATUS = 0x%08x", status);
	__ASSERT(false, "unimplemented");
}

void dwmac_common_isr(const struct device *dev)
{
	const struct dwmac_config *cfg = dev->config;
	struct dwmac_priv *p = dev->data;
	uint32_t irq_status;
	unsigned int ch;
	uint32_t events = 0;

	irq_status = REG_READ(DMA_IRQ_STATUS);
	LOG_DBG("DMA_IRQ_STATUS = 0x%08x", irq_status);

	while (irq_status & 0xff) {
		ch = find_lsb_set(irq_status & 0xff) - 1;
		irq_status &= ~BIT(ch);
		events |= dwmac_dma_irq(dev, ch);
	}

	if (irq_status & DMA_IRQ_STATUS_MTLIS) {
		dwmac_mtl_irq(dev);
	}

	if (irq_status & DMA_IRQ_STATUS_MACIS) {
		dwmac_mac_irq(dev);
	}

	k_event_post(&p->irq_events, events);
}

static void dwmac_set_mac_addr(const struct device *dev, uint8_t *addr, int n)
{
	const struct dwmac_config *cfg = dev->config;
	uint32_t reg_val;

	reg_val = (addr[5] << 8) | addr[4];
	REG_WRITE(MAC_ADDRESS_HIGH(n), reg_val | MAC_ADDRESS_HIGH_AE);
	reg_val = (addr[3] << 24) | (addr[2] << 16) | (addr[1] << 8) | addr[0];
	REG_WRITE(MAC_ADDRESS_LOW(n), reg_val);
}

static void dwmac_set_link_state(const struct device *dev, struct phy_link_state *link_state)
{
	const struct dwmac_config *cfg = dev->config;
	uint32_t conf = REG_READ(MAC_CONF);

	__ASSERT_NO_MSG((conf & (MAC_CONF_TE | MAC_CONF_RE)) == 0);

	REG_WRITE(MAC_CONF, conf | (link_state->speed < LINK_HALF_1000BASE_T ? MAC_CONF_PS : 0) |
				    (PHY_LINK_IS_FULL_DUPLEX(link_state->speed) ? MAC_CONF_DM : 0) |
				    (PHY_LINK_IS_SPEED_100M(link_state->speed) ? MAC_CONF_FES : 0) |
				    (link_state->is_up ? (MAC_CONF_TE | MAC_CONF_RE) : 0));
}

static void dwmac_link_state_changed(const struct device *phy_dev,
				     struct phy_link_state *link_state, void *user_data)
{
	const struct device *dev = user_data;
	const struct dwmac_config *cfg = dev->config;
	struct dwmac_priv *p = dev->data;

	if (link_state->is_up) {
		if (!WAIT_FOR(REG_READ(MAC_DEBUG) == 0, 1000, k_busy_wait(1))) {
		}
		dwmac_set_link_state(dev, link_state);
		net_if_carrier_on(p->iface);
		LOG_DBG("Link up");
	} else {
		REG_WRITE(MAC_CONF, REG_READ(MAC_CONF) & ~MAC_CONF_TE & ~MAC_CONF_RE);
		net_if_carrier_off(p->iface);
		LOG_DBG("Link down");
	}
}

static int dwmac_set_config(const struct device *dev, enum ethernet_config_type type,
			    const struct ethernet_config *config)
{
	const struct dwmac_config *cfg = dev->config;
	struct dwmac_priv *p = dev->data;
	uint32_t reg_val;
	int ret = 0;

	(void)reg_val; /* silence the "unused variable" warning */

	switch (type) {
	case ETHERNET_CONFIG_TYPE_MAC_ADDRESS:
		memcpy(p->mac_addr, config->mac_address.addr, sizeof(p->mac_addr));
		dwmac_set_mac_addr(dev, p->mac_addr, 0);
		net_if_set_link_addr(p->iface, p->mac_addr, sizeof(p->mac_addr), NET_LINK_ETHERNET);
		break;

#if defined(CONFIG_NET_PROMISCUOUS_MODE)
	case ETHERNET_CONFIG_TYPE_PROMISC_MODE:
		reg_val = REG_READ(MAC_PKT_FILTER);
		if (config->promisc_mode && !(reg_val & MAC_PKT_FILTER_PR)) {
			REG_WRITE(MAC_PKT_FILTER, reg_val | MAC_PKT_FILTER_PR);
		} else if (!config->promisc_mode && (reg_val & MAC_PKT_FILTER_PR)) {
			REG_WRITE(MAC_PKT_FILTER, reg_val & ~MAC_PKT_FILTER_PR);
		} else {
			ret = -EALREADY;
		}
		break;
#endif

	default:
		ret = -ENOTSUP;
		break;
	}

	return ret;
}

static void dwmac_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	const struct dwmac_config *cfg = dev->config;
	struct dwmac_priv *p = dev->data;
	struct phy_link_state link_state;

	__ASSERT(!p->iface, "interface already initialized?");
	p->iface = iface;

	net_if_set_link_addr(iface, p->mac_addr, sizeof(p->mac_addr), NET_LINK_ETHERNET);
	dwmac_set_mac_addr(dev, p->mac_addr, 0);

	ethernet_init(iface);
}

static int dwmac_start(const struct device *dev)
{
	const struct dwmac_config *cfg = dev->config;
	struct dwmac_priv *p = dev->data;
	uint8_t ch;

	if (!device_is_ready(dev)) {
		return -EIO;
	}

	for (ch = 0; ch < cfg->rx_channel_to_use; ch++) {
		dwmac_fill_rx_desc(dev, ch);
	}

	/* start up TX */
	for (ch = 0; ch < cfg->tx_queues_to_use; ch++) {
		REG_WRITE(DMA_CHn_TX_CTRL(ch), REG_READ(DMA_CHn_TX_CTRL(ch)) | DMA_CHn_TX_CTRL_ST);
	}

	/* Hook phy callback here to set carrier, speed, and link status */
	phy_link_callback_set(cfg->phy_dev, &dwmac_link_state_changed, (void *)dev);

	LOG_DBG("%s started", dev->name);

	return 0;
}

static int dwmac_stop(const struct device *dev)
{
	const struct dwmac_config *cfg = dev->config;
	uint8_t ch;

	/* stop TX */
	for (ch = 0; ch < cfg->tx_queues_to_use; ch++) {
		REG_WRITE(DMA_CHn_TX_CTRL(ch), REG_READ(DMA_CHn_TX_CTRL(ch)) & ~DMA_CHn_TX_CTRL_ST);
	}

	/* TODO: Wait for queue to empty */

	REG_WRITE(MAC_CONF, REG_READ(MAC_CONF) & ~MAC_CONF_TE & ~MAC_CONF_RE);

	/* TODO: Recover possibly unsend packets from descriptors */
	phy_link_callback_set(cfg->phy_dev, NULL, NULL);

	LOG_DBG("%s stopped", dev->name);

	return 0;
}

void __attribute((weak)) dwmac_platform_init(const struct device *dev)
{
}

#if CONFIG_PTP_CLOCK
static const struct device *dwmac_get_ptp_clock(const struct device *dev)
{
	const struct dwmac_config *cfg = dev->config;

	return cfg->ptp_clock;
}
#endif

int dwmac_probe(const struct device *dev)
{
	struct dwmac_priv *p = dev->data;
	const struct dwmac_config *cfg = dev->config;
	int ret;
	uint32_t reg_val;

	if (!device_is_ready(cfg->platform)) {
		LOG_ERR("Platform (%p) is not ready, cannot init dwmac", cfg->platform);
		return -EIO;
	}

	if (!device_is_ready(cfg->phy_dev)) {
		LOG_ERR("PHY device (%p) is not ready, cannot init dwmac", cfg->phy_dev);
		return -EFAULT;
	}

	ret = eth_dwc_eth_qos_clock_control_on(cfg->platform, ETH_DWC_ETH_QOS_CLK_PTP);
	if (ret) {
		return ret;
	}

	ret = eth_dwc_eth_qos_platform_reset(cfg->platform);
	if (ret) {
		return ret;
	}

	reg_val = REG_READ(MAC_VERSION);
	LOG_INF("HW version %u.%u0", (reg_val >> 4) & 0xf, reg_val & 0xf);
	__ASSERT(FIELD_GET(MAC_VERSION_SNPSVER, reg_val) >= 0x40,
		 "This driver expects DWC-ETHERNET version >= 4.00");

	/* get configured hardware features */
	p->feature0 = REG_READ(MAC_HW_FEATURE0);
	p->feature1 = REG_READ(MAC_HW_FEATURE1);
	p->feature2 = REG_READ(MAC_HW_FEATURE2);
	p->feature3 = REG_READ(MAC_HW_FEATURE3);
	LOG_DBG("hw_feature: 0x%08x 0x%08x 0x%08x 0x%08x", p->feature0, p->feature1, p->feature2,
		p->feature3);

	/* set up RX buffer refill thread */
	k_thread_create(&p->rx_thread, p->rx_thread_stack,
			K_KERNEL_STACK_SIZEOF(p->rx_thread_stack), dwmac_thread, p, NULL, NULL, 0,
			K_PRIO_PREEMPT(0), K_NO_WAIT);
	k_thread_name_set(&p->rx_thread, "dwmac_rx");

	k_event_init(&p->irq_events);

#if CONFIG_MMU
	uintptr_t tx_descs_phys;
	struct dwmac_dma_desc *tx_descs_uncached;
	/* make sure no valid cache lines map to the descriptor area */
	sys_cache_data_invd_range(cfg->tx_descs, sizeof(struct dwmac_dma_desc) *
							 cfg->tx_queues_to_use * NB_TX_DESCS);

	tx_descs_phys = z_mem_phys_addr(cfg->tx_descs);

	/* remap descriptor rings uncached */
	z_phys_map((uint8_t **)&tx_descs_uncached, tx_descs_phys,
		   sizeof(struct dwmac_dma_desc) * cfg->tx_queues_to_use * NB_TX_DESCS,
		   K_MEM_PERM_RW | K_MEM_CACHE_NONE);

	uintptr_t rx_descs_phys;
	struct dwmac_dma_desc *rx_descs_uncached;
	/* make sure no valid cache lines map to the descriptor area */
	sys_cache_data_invd_range(cfg->rx_descs, sizeof(struct dwmac_dma_desc) *
							 cfg->rx_channel_to_use * NB_RX_DESCS);

	rx_descs_phys = z_mem_phys_addr(cfg->rx_descs);

	/* remap descriptor rings uncached */
	z_phys_map((uint8_t **)&rx_descs_uncached, rx_descs_phys,
		   sizeof(struct dwmac_dma_desc) * cfg->rx_channel_to_use * NB_RX_DESCS,
		   K_MEM_PERM_RW | K_MEM_CACHE_NONE);
#endif

	uint32_t dma_ch;
	for (dma_ch = 0; dma_ch < cfg->tx_queues_to_use; dma_ch++) {
#if CONFIG_MMU
		p->tx_ch[dma_ch].descs_phys = tx_descs_phys + sizeof(struct dwmac_dma_desc) *
								      cfg->tx_queues_to_use *
								      NB_TX_DESCS;
#endif
		p->tx_ch[dma_ch].descs = &cfg->tx_descs[dma_ch * NB_TX_DESCS];
		p->tx_ch[dma_ch].desc_count = NB_TX_DESCS;
		memset(p->tx_ch[dma_ch].descs, 0,
		       p->tx_ch[dma_ch].desc_count * sizeof(struct dwmac_dma_desc));
		k_sem_init(&p->tx_ch[dma_ch].desc_used, p->tx_ch[dma_ch].desc_count - 1,
			   p->tx_ch[dma_ch].desc_count - 1);
		sys_slist_init(&p->tx_ch[dma_ch].pkts);
	}

	for (dma_ch = 0; dma_ch < cfg->rx_channel_to_use; dma_ch++) {
#if CONFIG_MMU
		p->rx_ch[dma_ch].descs_phys = tx_descs_phys + sizeof(struct dwmac_dma_desc) *
								      cfg->rx_channel_to_use *
								      NB_RX_DESCS;
#endif
		p->rx_ch[dma_ch].descs = &cfg->rx_descs[dma_ch * NB_RX_DESCS];
		p->rx_ch[dma_ch].desc_count = NB_RX_DESCS;
		memset(p->rx_ch[dma_ch].descs, 0,
		       p->rx_ch[dma_ch].desc_count * sizeof(struct dwmac_dma_desc));
		k_sem_init(&p->rx_ch[dma_ch].desc_used, p->rx_ch[dma_ch].desc_count - 1,
			   p->rx_ch[dma_ch].desc_count - 1);
		sys_slist_init(&p->rx_ch[dma_ch].pkts);
	}

	cfg->init_config();

	REG_WRITE(MMC_RX_INTERRUPT_MASK, 0xFFFFFFFF);
	REG_WRITE(MMC_TX_INTERRUPT_MASK, 0xFFFFFFFF);
	REG_WRITE(MMC_IPC_RX_INTERRUPT_MASK, 0xFFFFFFFF);
	dwmac_mac_init(dev);
	dwmac_mtl_init(dev);
	dwmac_dma_init(dev);

	return 0;
}

const struct ethernet_api dwmac_api = {
	.iface_api.init = dwmac_iface_init,
	.start = dwmac_start,
	.stop = dwmac_stop,
	.get_capabilities = dwmac_caps,
	.set_config = dwmac_set_config,
	.send = dwmac_send,
#if CONFIG_PTP_CLOCK
	.get_ptp_clock = dwmac_get_ptp_clock,
#endif
};

#define ETH_DWMAC_GETH_INIT(n)                                                                     \
	DWMAC_DEVICE(n)                                                                            \
	static struct dwmac_priv dwmac_instance##n = {};                                           \
	static struct dwmac_config dwmac_config##n = DWMAC_DT_INST_CONFIG(n);                      \
	ETH_NET_DEVICE_DT_INST_DEFINE(n, dwmac_probe, NULL, &dwmac_instance##n, &dwmac_config##n,  \
				      CONFIG_ETH_INIT_PRIORITY, &dwmac_api, NET_ETH_MTU);

DT_INST_FOREACH_STATUS_OKAY(ETH_DWMAC_GETH_INIT)