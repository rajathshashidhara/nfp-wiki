## DPDK Poll Mode Driver

NFP PMD Code is located in the [`drivers/net/nfp`](https://github.com/DPDK/dpdk/blob/master/drivers/net/nfp) folder.

[UIO device drivers](https://www.kernel.org/doc/html/latest/driver-api/uio-howto.html) provide access to address space of the device and interrupts to userspace through `/dev/uioX` interface. To access the registers or RAM of the device, use `mmap()` on `/dev/uioX`. In addition, a blocking `read()` on `/dev/uioX` returns when interrupts are received from the device. 

NFP PMD uses the UIO [`igb_uio`](https://github.com/DPDK/dpdk/tree/master/kernel/linux/igb_uio) Linux Kernel Driver under the hood. The `igb_uio` driver is a very simple driver that maps PCIe BAR registers to kernel address space and provides interrupt control. However, this driver does not support the `mmap()` operation. It also exposes the BAR information through `sysfs` interface as resources.

##### DPDK initialization walkthrough

Here we look at how DPDK identifies the network devices and configures PMD to access that device.

1. `rte_eal_init()` : First DPDK function called by the application.

   Set `iova_mode` based on whether IOMMU is present and physical address accessible

2. `rte_bus_probe()` : Probe all the buses and devices/drivers on them
   Buses are registered during compile time. We're interested in PCI devices and the related [`rte_pci_bus`](https://github.com/DPDK/dpdk/blob/ab59484a4bde238e3e6356bca407ab02d3a6cde7/drivers/bus/pci/pci_common.c#L675) operations.

3. `rte_pci_probe()`: Finds a registered driver that matches the device based on vendor_id. Finally, check if the device supports the `iova_mode`. This is when the device is matched to NFP PMD.

4. NFP PMD [sets](https://github.com/DPDK/dpdk/blob/master/drivers/net/nfp/nfp_net.c#L3753) the `RTE_PCI_DRV_NEED_MAPPING ` flag. Due to this PCIe BAR resources are [mapped to virtual address space](https://github.com/DPDK/dpdk/blob/ab59484a4bde238e3e6356bca407ab02d3a6cde7/drivers/bus/pci/pci_common.c#L189) using `rte_pci_map_device()`. BAR's PA is accessed via `/sys/bus/pci/devices/<pci_id>/resource%d` file. This is then mapped into VA using `mmap()`. Immediately after this, PMD specific probe function is called.

5. [`nfp_pf_pci_probe()`](https://github.com/DPDK/dpdk/blob/master/drivers/net/nfp/nfp_net.c#L3595): This function uses both BAR and CPP interface to configure the ethernet interface. This function creates and initializes a new ethernet device per port found on the NIC. Look into `nfp_net_init()` to understand how CTRL BAR, TX/RX Queues are accessed from the NIC. Surprisingly, for PF operation, it is accessed using `rtsym` instead of PCIe BAR.

```c
/* Is NFP LE/BE ? */
static inline uint32_t
nn_cfg_readl(struct nfp_net_hw *hw, int off)
{
	return rte_le_to_cpu_32(nn_readl(hw->ctrl_bar + off));
}

hw->ctrl_bar = nfp_rtsym_map(hw->sym_tbl, "_pf0_net_bar0",
					     hw->total_ports * 32768,
					     &hw->ctrl_area);

hw->max_rx_queues = nn_cfg_readl(hw, NFP_NET_CFG_MAX_RXRINGS);
hw->max_tx_queues = nn_cfg_readl(hw, NFP_NET_CFG_MAX_TXRINGS);

start_q = nn_cfg_readl(hw, NFP_NET_CFG_START_TXQ);
tx_bar_off = (uint64_t)start_q * NFP_QCP_QUEUE_ADDR_SZ;
start_q = nn_cfg_readl(hw, NFP_NET_CFG_START_RXQ);
rx_bar_off = (uint64_t)start_q * NFP_QCP_QUEUE_ADDR_SZ;

hwport0->hw_queues = nfp_cpp_map_area(hw->cpp, 0, 0,
                                      NFP_PCIE_QUEUE(0),
                                      NFP_QCP_QUEUE_AREA_SZ,
                                      &hw->hwqueues_area);
hw->tx_bar = hwport0->hw_queues + tx_bar_off;
hw->rx_bar = hwport0->hw_queues + rx_bar_off;

hw->ver = nn_cfg_readl(hw, NFP_NET_CFG_VERSION);
hw->cap = nn_cfg_readl(hw, NFP_NET_CFG_CAP);
hw->max_mtu = nn_cfg_readl(hw, NFP_NET_CFG_MAX_MTU);
```

Operations supported by the PMD passed to DPDK as a `struct eth_dev_ops` [here](https://github.com/DPDK/dpdk/blob/master/drivers/net/nfp/nfp_net.c#L2683). This is a good starting point to understand how different operations are implemented.

##### How does PMD map userspace accessible memory for DMA operations ?

- NFP PMD uses RTE [`memzone`](http://doc.dpdk.org/api/rte__memzone_8h.html) allocator to obtain memory regions capable of having both Virtual and IO address mappings. Look into [`rte_eth_dma_zone_reserve()`](https://doc.dpdk.org/api/rte__ethdev__driver_8h.html#af4bddc0f48f8d1dd17b29e74930d0477)
- IO address is copied into the **RX BAR** region in the PCIe configuration space for every rx/tx queue: See [`nfp_net_rx_queue_setup()`](https://github.com/DPDK/dpdk/blob/master/drivers/net/nfp/nfp_net.c#L1507) for more details.

```C
/*
 * nfp_net_rx_queue_setup() 
 */
rxq->qidx = queue_idx;
rxq->fl_qcidx = queue_idx * hw->stride_rx;	/* FL = Free List of packet buffers */
rxq->rx_qcidx = rxq->fl_qcidx + (hw->stride_rx - 1); // NOTE: why use a stride ?
rxq->qcp_fl = hw->rx_bar + NFP_QCP_QUEUE_OFF(rxq->fl_qcidx);
rxq->qcp_rx = hw->rx_bar + NFP_QCP_QUEUE_OFF(rxq->rx_qcidx);

/* Allocate ring buffer of descriptors accessible to both Userspace and IO space */
tz = rte_eth_dma_zone_reserve(dev, "rx_ring", queue_idx,
                              sizeof(struct nfp_net_rx_desc) *
                              NFP_NET_MAX_RX_DESC, NFP_MEMZONE_ALIGN,
                              socket_id);
rxq->dma = (uint64_t)tz->iova;
rxq->rxds = (struct nfp_net_rx_desc *)tz->addr;

/* Tell the H/W io address of rx ring and number of descriptors */
nn_cfg_writeq(hw, NFP_NET_CFG_RXR_ADDR(queue_idx), rxq->dma);
nn_cfg_writeb(hw, NFP_NET_CFG_RXR_SZ(queue_idx), rte_log2_u32(nb_desc));

/*
 * nfp_net_tx_queue_setup() 
 */
txq->qidx = queue_idx;
txq->tx_qcidx = queue_idx * hw->stride_tx;
txq->qcp_q = hw->tx_bar + NFP_QCP_QUEUE_OFF(txq->tx_qcidx);

tz = rte_eth_dma_zone_reserve(dev, "tx_ring", queue_idx,
                              sizeof(struct nfp_net_tx_desc) *
                              NFP_NET_MAX_TX_DESC, NFP_MEMZONE_ALIGN,
                              socket_id);
txq->dma = (uint64_t)tz->iova;
txq->txds = (struct nfp_net_tx_desc *)tz->addr;

/* Tell the H/W io address of tx ring and number of descriptors */
nn_cfg_writeq(hw, NFP_NET_CFG_TXR_ADDR(queue_idx), txq->dma);
nn_cfg_writeb(hw, NFP_NET_CFG_TXR_SZ(queue_idx), rte_log2_u32(nb_desc));
```

##### Packet receive - walkthrough

[`nfp_net_recv_pkts()`](https://github.com/DPDK/dpdk/blob/master/drivers/net/nfp/nfp_net.c#L1989)  implements packet receive function. A single `dd` bit is used to detect if the RX DESC contains a valid packet or not. Host uses a single `rd_p` to maintain the last read location whereas the NIC uses `qcp_fl` and `qcp_rx` pointers to maintain the DMA descriptors list.

```C
nfp_net_recv_pkts(rx_queue, rx_pkts, nb_pkts)
{
    avail = 0;
    nb_hold = 0;
    
    /* Read DMA descriptors */
    while (avail < nb_pkts)
    {
        rxds = &rxq->rxds[rxq->rd_p];	/*> rd_p: Read pointer */
        if ((rxds->rxd.meta_len_dd & PCIE_DESC_RX_DD) == 0)	// If DD bit is not set
            break;
        
        rte_rmb();	/* Read memory barrier to prevent following reads to be reordered */
        
        /* Replace a buffer at the DMA descriptor */
        rx_pkts[avail++] = rxb->mbuf;
        rxb->mbuf = rte_pktmbuf_alloc(rxq->mem_pool);
        nb_hold++;
        
		/* Reset Descriptor */
        rxds->fld.dd = 0;	// 0: For NIC use 1: For Host use
        rxds->fld.dma_addr = rte_cpu_to_le_64(RTE_MBUF_DMA_ADDR_DEFAULT(rxb->mbuf));
        
        /* Increment Read pointer */
        rxq->rd_p++;
        if (rxq->rd_p == rxq->rx_count)
            rxq->rd_p = 0;
    }
    
 	nb_hold += rxq->nb_rx_hold;	/*> Stores the number of free descriptors */
    
    rte_wmb();	/* Ensure all the descriptors are updated before chaning WR pointer */
    if (nb_hold > rxq->rx_free_thresh)	/* Updated WR ptr only if above threshold*/
    {
        nfp_qcp_ptr_add(rxq->qcp_fl, NFP_QCP_WRITE_PTR, nb_hold);
        nb_hold = 0;
    }
    rxq->nb_rx_hold = nb_hold;
}
```



##### Packet send - walkthrough

[`nfp_net_xmit_packets()`](https://github.com/DPDK/dpdk/blob/master/drivers/net/nfp/nfp_net.c#L2215) is the bulk packet transmit implementation in NFP PMD. This function proceeds in a very similar fashion as the receive function.

```C
nfp_net_tx_free_bufs(txq)
{
    qcp_rd_p = nfp_qcp_read(txq->qcp_q, NFP_QCP_READ_PTR);	/*> RD PTR on NIC */

    if (qcp_rd_p == txq->rd_p)	/* No pkts tx'ed by NIC */
        return 0;
    
    /* Compute how many packets can be freed */
    if (qcp_rd_p > txq->rd_p)
		todo = qcp_rd_p - txq->rd_p;
	else
		todo = qcp_rd_p + txq->tx_count - txq->rd_p;
    
    txq->rd_p += todo;	/* todo: Number of outstanding packets transmitted by NIC */
    if (txq->rd_p >= txq->tx_count)	/* Round up */
		txq->rd_p -= txq->tx_count;
}

/* Returns number of free descriptors available for TX */
nfp_free_tx_desc(txq)
{
    if (txq->wr_p >= txq->rd_p)
		return txq->tx_count - (txq->wr_p - txq->rd_p) - 8;
	else
		return txq->rd_p - txq->wr_p - 8;
}

nfp_net_txq_full(txq)
{
	return (nfp_free_tx_desc(txq) < txq->tx_free_thresh);
}

nfp_net_xmit_packets(tx_queue, tx_pkts, nb_pkts)
{
    /* Free up DMA descriptors if below requirement */
    if ((nfp_free_tx_desc(txq) < nb_pkts) || (nfp_net_txq_full(txq)))
		nfp_net_tx_free_bufs(txq);
    
    free_desc = nfp_free_tx_desc(txq);	/* # of available desc */
    
    while (i < nb_pkts && free_desc)
    {
        pkt = tx_pkts[i];
        txds = &txq->txds[txq->wr_p];	/*> wr_p: Write pointer */
        
        lmbuf = txds->mbuf;	/* Mbuf of current descriptor */
        rte_pktmbuf_free_seg(*lmbuf);	/* Free the previous mbuf */
        
        /* Setup the descriptor for transmission */
        txds->dma_len = pkt->data_len;
        txds->dma_addr = rte_mbuf_data_iova(pkt);
        
        free_descs--;
        txq->wr_p++;
        if (unlikely(txq->wr_p == txq->tx_count)) /* wraparound */
				txq->wr_p = 0;

        i++;
    }
    
    /* All writes must complete before letting H/W know */
    rte_wmb();
    nfp_qcp_ptr_add(txq->qcp_q, NFP_QCP_WRITE_PTR, i);

    return i;
}
```

