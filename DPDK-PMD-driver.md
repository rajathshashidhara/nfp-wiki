## DPDK Poll Mode Driver

[UIO device drivers](https://www.kernel.org/doc/html/latest/driver-api/uio-howto.html) provide access to address space of the device and interrupts to userspace through `/dev/uioX` interface. To access the registers or RAM of the device, use `mmap()` on `/dev/uioX`. In addition, a blocking `read()` on `/dev/uioX` returns when interrupts are received from the device. 

NFP PMD uses the UIO [`igb_uio`](https://github.com/DPDK/dpdk/tree/master/kernel/linux/igb_uio) Linux Kernel Driver under the hood. The `igb_uio` driver is a very simple driver that maps PCIe BAR registers to kernel address space and provides interrupt control. However, this driver does not support the `mmap()` operation. It also exposes the BAR information through `sysfs` interface as resources.

##### DPDK initialization walkthrough

Here we look at how DPDK identifies the network devices and configures PMD to access that device.

1. `rte_eal_init()` : First DPDK function called by the application.

   Set `iova_mode` based on whether IOMMU is present and physical address accessible

2. `rte_bus_probe()` : Probe all the buses and devices/drivers on them
   Buses are registered during compile time. We're interested in PCI devices and the related [`rte_pci_bus`](https://github.com/DPDK/dpdk/blob/ab59484a4bde238e3e6356bca407ab02d3a6cde7/drivers/bus/pci/pci_common.c#L675) operations.

3. `rte_pci_probe()`: Finds a registered driver that matches the device based on vendor_id. Finally, check if the device supports the `iova_mode`. This is when the device is matched to NFP PMD.

4. NFP PMD [sets](https://github.com/DPDK/dpdk/blob/master/drivers/net/nfp/nfp_net.c#L3753) the `RTE_PCI_DRV_NEED_MAPPING ` flag. Due to this PCIe BAR resources are [mapped to virtual address space](https://github.com/DPDK/dpdk/blob/ab59484a4bde238e3e6356bca407ab02d3a6cde7/drivers/bus/pci/pci_common.c#L189) using `rte_pci_map_device()`. BAR's PA is accessed via `/sys/bus/pci/devices/<pci_id>/resource%s` file. This is then mapped into VA using `mmap()`. Immediately after this, PMD specific probe function is called.

5. [`nfp_pf_pci_probe()`](https://github.com/DPDK/dpdk/blob/master/drivers/net/nfp/nfp_net.c#L3595): This function uses both BAR and CPP interface to configure the ethernet interface.

NFP PMD Code is located in the [`drivers/net/nfp`](https://github.com/DPDK/dpdk/blob/master/drivers/net/nfp) folder.

Operations supported by the PMD passed to DPDK as a `struct eth_dev_ops` [here](https://github.com/DPDK/dpdk/blob/master/drivers/net/nfp/nfp_net.c#L2683). This is a good starting point to understand how different operations are implemented.

##### How does PMD map userspace accessible memory for DMA operations ?

- NFP PMD uses RTE [`memzone`](http://doc.dpdk.org/api/rte__memzone_8h.html) allocator to obtain memory regions capable of having both Virtual and IO address mappings. Look into [`rte_eth_dma_zone_reserve()`](https://doc.dpdk.org/api/rte__ethdev__driver_8h.html#af4bddc0f48f8d1dd17b29e74930d0477)
- IO address is copied into the **RX BAR** region in the PCIe configuration space for every rx/tx queue: See [`nfp_net_rx_queue_setup()`](https://github.com/DPDK/dpdk/blob/master/drivers/net/nfp/nfp_net.c#L1507) for more details.

##### Packet receive - walkthrough

`TODO`

##### Packet send - walkthrough

`TODO`

#### BAR layout

`TODO`

