#include "i82580.h"
#include "i82580_hw.h"
#include "i82580_regs.h"

char i82580_driver_name[] = "i82580";
static char i82580_driver_string[] = "Intel(R) I340 Network Driver";

static const struct pci_device_id i82580_pci_tbl[] = {
    { PCI_DEVICE(0x8086, 0x10d3) },
    /* required last entry */
    {0, }
};

MODULE_DEVICE_TABLE(pci, i82580_pci_tbl);

/* all custom funcs */


static int i82580_init_module(void);
static void i82580_exit_module(void);
static int i82580_probe(struct pci_dev *pdev, const struct pci_device_id *ent);
static void i82580_remove(struct pci_dev *pdev);
static int i82580_open(struct net_device *ndev);
static int i82580_setup_tx_ring(struct i82580_adapter *adap);
static void i82580_tx_program_regs(struct i82580_adapter *adap, int q);
static void i82580_tx_enable_mac(struct i82580_adapter *adap);
static int i82580_tx_config_queue(struct i82580_adapter *adap, int q);
void i82580_free_tx_ring(struct i82580_adapter *adap);
static int i82580_setup_rx_ring(struct i82580_adapter *adap);
static int i82580_alloc_all_rx_buffers(struct i82580_adapter *adap);
static int i82580_alloc_rx_buffer(struct i82580_adapter *adap, int index);
static void i82580_rx_program_regs(struct i82580_adapter *adap, int q);
static int i82580_rx_config_queue(struct i82580_adapter *adap, int q);
static void i82580_rx_enable_mac(struct i82580_adapter *adap);
static void i82580_free_all_rx_buffers(struct i82580_adapter *adap);
static void i82580_free_rx_ring(struct i82580_adapter *adap);



static struct pci_driver i82580_driver = {
    .name = i82580_driver_name,
    .id_table = i82580_pci_tbl,
    .probe = i82580_probe,
    .remove = i82580_remove
};

MODULE_DESCRIPTION("Intel(R) I340 Network Driver");
MODULE_LICENSE("GPL");


/**
 * i82580_init_module - Driver Registration Routine
 * 
 * i82580_init_module is the first routine called when the driver is
 * loaded. All it does is register with the PCI subsystem.
**/
static int __init i82580_init_module(void)
{
    int ret;
    pr_info("%s\n", i82580_driver_string);

    ret = pci_register_driver(&i82580_driver);

    return ret;
}

module_init(i82580_init_module);

/**
 * i82580_exit_module - Driver Exit Cleanup Routine
 *
 * i82580_exit_module is called just before the driver is removed
 * from memory.
 **/
static void __exit i82580_exit_module(void)
{
    pci_unregister_driver(&i82580_driver);
}

module_exit(i82580_exit_module);


static const struct net_device_ops i82580_netdev_ops = {
    .ndo_open   = i82580_open,
};


static int i82580_probe(struct pci_dev *pdev, const struct pci_device_id *ent)
{
    struct net_device *netdev;
    struct i82580_adapter *adapter = NULL;
    int err, bars;

    pr_info("%s: vendor: 0x%04x, device: 0x%04x\n", i82580_driver_name, pdev->vendor, pdev->device);

    /* Enable PCI DEVICE (MMIO only) */
    bars = pci_select_bars(pdev, IORESOURCE_MEM);

    err = pci_enable_device_mem(pdev);
    if (err)
            return err;

    err = pci_request_selected_regions(pdev, bars, i82580_driver_name);
    if (err)
            goto err_disable_pci;
    
    pci_set_master(pdev);

    /* DMA mask (82580 supports 64-bit DMA) */
    err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
    if (err) {
        dev_warn(&pdev->dev, "%s : falling back to 32-bit DMA\n", i82580_driver_name);
        err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
        if (err)
            goto err_release_regions;
    }

    /* allocate netdev with private adapter */
    netdev = alloc_etherdev(sizeof(*adapter));
    if (!netdev) {
        err = -ENOMEM;
        goto err_release_regions;
    }

    SET_NETDEV_DEV(netdev, &pdev->dev); // nedded to sysfs hierarchy
    pci_set_drvdata(pdev, netdev);

    adapter = netdev_priv(netdev);
    memset(adapter, 0, sizeof(*adapter));

    adapter->netdev = netdev;
    adapter->pdev   = pdev;
    spin_lock_init(&adapter->tx_lock);
    spin_lock_init(&adapter->rx_lock);




    /* map BAR0 MMIO */
    adapter->hw_addr = pci_iomap(pdev, I82580_BAR_MMIO, 0);
    if (!adapter->hw_addr) {
        dev_err(&pdev->dev, i82580_driver_name, " : pci_iomap failed\n");
        err = -ENODEV;
        goto err_free_netdev_napi;
    }

    return 0;
}

static void i82580_remove(struct pci_dev *pdev)
{
    pr_info("%s : success if romoving\n", i82580_driver_name);
}

static int i82580_open(struct net_device *ndev)
{
    struct i82580_adapter *adapter  = netdev_priv(ndev);
    int err;

    if (test_bit(I82580_TESTING, &adapter->flags))
            return -EBUSY;

    netif_carrier_off(ndev);

    adapter->tx_ring = kzalloc(sizeof(*adapter->tx_ring), GFP_KERNEL);
    if (!adapter->tx_ring)
            return -ENOMEM;

    /* allocate transmit descriptors */
    err = i82580_setup_tx_ring(adapter);    // dma_alloc_coherent + init indices
    if (err)
        goto err_tx;
    
    /* 2) set registers of all params for queue 0 */
    i82580_tx_program_regs(adapter, 0);
    
    /* 3) set registers of behaviour for queue 0 */
    i82580_tx_config_queue(adapter, 0);
    
    /* 4) Set on global TX (TCTL + TIPG) */
    i82580_tx_enable_mac(adapter);

    adapter->rx_ring = kzalloc(sizeof(*adapter->rx_ring), GFP_KERNEL);
    if (!adapter->rx_ring)
            return -ENOMEM;
    
    /* RX bring-up */
    err = i82580_setup_rx_ring(adapter);
    if (err) goto err_rx_setup;
    
    err = i82580_alloc_all_rx_buffers(adapter);
    if (err) goto err_rx_bufs;
    
    i82580_rx_program_regs(adapter, 0);
    
    err = i82580_rx_config_queue(adapter, 0);
    if (err) goto err_rx_cfg;

    i82580_rx_enable_mac(adapter);

    // i82580_configure_hw(adapter);

    // err = request_irq(adapter->pdev->irq, mynic_isr, 0,
    //     netdev->name, adapter);

    // if (err)
    //     goto err_irq;
    
    // napi_enable(&adapter->napi);

    // mynic_irq_enable(adapter);

    // netif_start_queue(netdev);

    // mynic_trigger_link_check(adapter);

    return 0;

err_rx_cfg:
    i82580_free_all_rx_buffers(adapter);
err_rx_bufs:
    i82580_free_rx_ring(adapter);
err_rx_setup:
    kfree(adapter->rx_ring);
err_tx:
    i82580_free_tx_ring(adapter);
    kfree(adapter->tx_ring);
    pci_disable_device(adapter->pdev);
    return err;
}


static int i82580_setup_tx_ring(struct i82580_adapter *adap)
{
    struct i82580_ring *tx = adap->tx_ring;
    struct pci_dev *pdev = adap->pdev;

    tx->count = 256;

    tx->size  = tx->count * sizeof(struct i82580_adv_tx_desc);

    tx->desc = dma_alloc_coherent(&pdev->dev,
                                  tx->size,
                                  &tx->dma,
                                  GFP_KERNEL);
    if (!tx->desc)
            return -ENOMEM;

    if (WARN_ON_ONCE(tx->dma & (128 - 1)))
            pr_warn("tx dma not 128B aligned (driver/pool bug)\n");

    tx->next_to_use = 0;
    tx->next_to_clean = 0;

    memset(tx->desc, 0, tx->size);

    return 0;
}


static void i82580_tx_program_regs(struct i82580_adapter *adap, int q)
{
    struct i82580_ring *tx = &adap->tx_ring[q];
    void __iomem *hw = adap->hw_addr;
    u32 lo = lower_32_bits(tx->dma);
    u32 hi = upper_32_bits(tx->dma);

    writel(lo, hw + I82580_TDBAL(q));
    writel(hi, hw + I82580_TDBAH(q));

    writel(tx->size, hw + I82580_TDLEN(q));

    writel(0, hw + I82580_TDH(q));
    writel(0, hw + I82580_TDT(q));
}


static void i82580_tx_enable_mac(struct i82580_adapter *adap)
{
    void __iomem *hw = adap->hw_addr;
    u32 tctl, tipg;

    tctl = readl(hw + I82580_TCTL);
    tctl |= I82580_TCTL_EN | I82580_TCTL_PSP;

    tctl &= ~I82580_TCTL_CT_MASK;
    tctl |= (I82580_TCTL_CT_DEFAULT << I82580_TCTL_CT_SHIFT);

    tctl &= ~I82580_TCTL_BST_MASK;
    tctl |= (I82580_TCTL_BST_DEFAULT << I82580_TCTL_BST_SHIFT);

    writel(tctl, hw + I82580_TCTL);

    tipg = I82580_TIPG_DEFAULT;
    writel(tipg, hw + I82580_TIPG);
}

static int i82580_tx_config_queue(struct i82580_adapter *adap, int q)
{
    void __iomem *hw = adap->hw_addr;
    u32 txdctl = readl(hw + I82580_TXDCTL(q));

    /* clean up old values */
    txdctl &= ~(TXDCTL_PTHRESH_MASK |
        TXDCTL_HTHRESH_MASK |
        TXDCTL_WTHRESH_MASK);

    /* set recommended  preps from datasheet */
    txdctl |= FIELD_PREP(TXDCTL_PTHRESH_MASK, TXDCTL_PTHRESH_DEFAULT);
    txdctl |= FIELD_PREP(TXDCTL_HTHRESH_MASK, TXDCTL_HTHRESH_DEFAULT);
    txdctl |= FIELD_PREP(TXDCTL_WTHRESH_MASK, TXDCTL_WTHRESH_DEFAULT);

    /* enable queue */
    txdctl |= TXDCTL_QUEUE_ENABLE;

    /* write back to the register */
    writel(txdctl, hw + I82580_TXDCTL(q));

    /* wait while bit Enable is not 1 (not more 1ms) */
    for (int i = 0; i < 1000; i++) {
        if (readl(hw + I82580_TXDCTL(q)) & TXDCTL_QUEUE_ENABLE)
            return 0;
        udelay(1);
    }

    /* if we are here - queue is not enabled */
    dev_err(adap->netdev->dev.parent, "TX queue %d enable failed\n", q);

    /* rollback - disable bit */
    txdctl &= ~TXDCTL_QUEUE_ENABLE;
    writel(txdctl, hw + I82580_TXDCTL(q));

    return -ETIMEDOUT;
}

void i82580_free_tx_ring(struct i82580_adapter *adap)
{
    struct i82580_ring *tx = adap->tx_ring;
    struct pci_dev *pdev = adap->pdev;

    if (!tx)
        return;

    /* Яtx->skbs[], tx->buf_dma[] + free */

    if (tx->desc) {
        dma_free_coherent(&pdev->dev,
                          tx->size,
                          tx->desc,
                          tx->dma);
        tx->desc = NULL;
    }
}

static int i82580_setup_rx_ring(struct i82580_adapter *adap)
{
    struct i82580_ring *rx = adap->tx_ring;
    struct pci_dev *pdev = adap->pdev;

    rx->count   = 256;
    rx->size    = rx->count * sizeof(struct i82580_adv_rx_desc);

    rx->desc    = dma_alloc_coherent(&pdev->dev, rx->size, &rx->dma, GFP_KERNEL);
    if (!rx->desc)
            return -ENOMEM;

    rx->next_to_use     = 0;
    rx->next_to_clean   = 0;
    memset(rx->desc, 0, rx->size);

    /* for buffers' book-keeping */
    rx->skbs = kcalloc(rx->count, sizeof(*rx->skbs), GFP_KERNEL);
    if (!rx->skbs)
        goto err_free_ring;

    rx->buf_dma = kcalloc(rx->count, sizeof(*rx->buf_dma), GFP_KERNEL);
        goto err_free_skbs;

    return 0;

err_free_skbs:
    kfree(rx->skbs);
    rx->skbs = NULL;
err_free_ring:
    dma_free_coherent(&pdev->dev, rx->size, rx->desc, rx->dma);
    rx->desc = NULL;
    return -ENOMEM;
}

static int i82580_alloc_all_rx_buffers(struct i82580_adapter *adap)
{
    int i, err;

    for (i = 0; i < adap->rx_ring->count; i++) {
        err = i82580_alloc_rx_buffer(adap, i);
        if (err)
            goto err_partial;
    }

    return 0;

err_partial:
    while (--i >= 0) {
        if (adap->rx_ring->skbs[i]) {
            dma_unmap_single(&adap->pdev->dev, 
                            adap->rx_ring->buf_dma[i],
                            I82580_RX_BUF_LEN, DMA_FROM_DEVICE);
            dev_kfree_skb_any(adap->rx_ring->skbs[i]);
            adap->rx_ring->skbs[i] = NULL;
        }
    }
    return -ENOMEM;
}

static int i82580_alloc_rx_buffer(struct i82580_adapter *adap, int index)
{
    struct i82580_ring *rx = adap->rx_ring;
    struct i82580_adv_rx_desc *desc;
    struct sk_buff *skb;
    dma_addr_t dma;

    skb = netdev_alloc_skb_ip_align(adap->netdev, I82580_RX_BUF_LEN);
    if (!skb)
            return -ENOMEM;

    dma = dma_map_single(&adap->pdev->dev, skb->data,
                        I82580_RX_BUF_LEN, DMA_FROM_DEVICE);
    if (dma_mapping_error(&adap->pdev->dev, dma)) {
        dev_kfree_skb_any(skb);
        return -ENOMEM;
    }

    desc = (struct i82580_adv_rx_desc *)rx->desc + index;
    desc->pkt_addr = cpu_to_le64(dma);
    desc->hdr_addr = 0; /* for no header split */ 

    rx->skbs[index] = skb;
    rx->buf_dma[index] = dma;

    return 0;
}

/* setting rx registers */
void i82580_rx_program_regs(struct i82580_adapter *adap, int q)
{
    struct i82580_ring *rx = &adap->rx_ring[q];
    void __iomem *hw = adap->hw_addr;
    u32 lo = lower_32_bits(rx->dma);
    u32 hi = upper_32_bits(rx->dma);
    u32 srrctl;

    /* base and length of ring */
    writel(lo, hw + I82580_RDBAL(q));
    writel(hi, hw + I82580_RDBAH(q));
    writel(rx->size, hw + I82580_RDLEN(q));

   /* SRRCTL: descriptor's type + size of packet buffer 
   * one-buffer advandec RX desc
   */
   srrctl = readl(hw + I82580_SRRCTL(q));
   srrctl &= ~(SRRCTL_BSIZEPKT_MASK | SRRCTL_DESCTYPE_MASK);

   srrctl |= ((I82580_RX_BUF_LEN >> SRRCTL_BSIZEPKT_SHIFT) << SRRCTL_BSIZEPKT_SHIFT);
   srrctl |= SRRCTL_DESCTYPE_ADV_ONEBUF;

   writel(srrctl, hw + I82580_SRRCTL(q));

   /* start head/tail. default RDH=0. RDT=count-1,
   * that means "all N descriptors are available for NIC"
   */
  writel(0,             hw + I82580_RDH(q));
  writel(rx->count -1,  hw + I82580_RDT(q));
}

/* rx prep and enable, poll for ready*/
static int i82580_rx_config_queue(struct i82580_adapter *adap, int q)
{
    void __iomem *hw = adap->hw_addr;
    u32 rxdctl;
    int i;

    rxdctl  = readl(hw + I82580_RXDCTL(q));
    rxdctl &= ~(RXDCTL_PTHRESH_MASK | RXDCTL_HTHRESH_MASK | RXDCTL_WTHRESH_MASK);
    rxdctl |= FIELD_PREP(RXDCTL_PTHRESH_MASK, RXDCTL_PTHRESH_DEFAULT);
    rxdctl |= FIELD_PREP(RXDCTL_HTHRESH_MASK, RXDCTL_HTHRESH_DEFAULT);
    rxdctl |= FIELD_PREP(RXDCTL_WTHRESH_MASK, RXDCTL_WTHRESH_DEFAULT);
    rxdctl |= RXDCTL_QUEUE_ENABLE;

    writel(rxdctl, hw + I82580_RXDCTL(q));

    /* polling: waiting for nic get enable */
    for(i = 0; i < 1000; i++) {
        if (readl(hw + I82580_RXDCTL(q)) & RXDCTL_QUEUE_ENABLE)
            return 0;
        udelay(1);
    }

    /* rollback */
    rxdctl &= ~RXDCTL_QUEUE_ENABLE;
    writel(rxdctl, hw + I82580_RXDCTL(q));
    dev_err(&adap->pdev->dev, "RX queue %d failed to enable\n", q);
    return -ETIMEDOUT;
}

/* global enabling rx controller */
static void i82580_rx_enable_mac(struct i82580_adapter *adap)
{
    void __iomem *hw = adap->hw_addr;
    u32 rctl;

    rctl = readl(hw + I82580_RCTL);
    rctl &= ~I82580_RCTL_EN;
    writel(rctl, hw + I82580_RCTL);
    udelay(1);

    rctl &= ~I82580_RCTL_BSIZE_MASK;
    rctl |= I82580_RCTL_BSIZE_2048; 

    // Multicast offset
    rctl &= ~I82580_RCTL_MO_MASK;
    rctl |= I82580_RCTL_MO_0;

    // Broadcast Accept Mode + Strip CRC
    rctl |= I82580_RCTL_BAM;
    rctl |= I82580_RCTL_SECRC;

    rctl |= I82580_RCTL_EN;

    writel(rctl, hw + I82580_RCTL);

    pr_info("RX MAC enabled\n");
}


static void i82580_free_all_rx_buffers(struct i82580_adapter *adap)
{
    struct i82580_ring *rx = adap->rx_ring;
    int i;

    if (!rx || !rx->skbs)
            return;

    for (i = 0; i < rx->count; i++) {
        if (rx->skbs[i]) {
            dma_unmap_single(&adap->pdev->dev, rx->buf_dma[i], 
                            I82580_RX_BUF_LEN, DMA_FROM_DEVICE);
            dev_kfree_skb_any(rx->skbs[i]);
            rx->skbs[i] = NULL;
        }
    }
}

static void i82580_free_rx_ring(struct i82580_adapter *adap)
{
    struct i82580_ring *rx = adap->rx_ring;
    struct pci_dev *pdev = adap->pdev;

    if (!rx) return;

    i82580_free_all_rx_buffers(adap);

    kfree(rx->buf_dma);
    rx->buf_dma = NULL;
    
    kfree(rx->skbs);
    rx->skbs = NULL;

    if (rx->desc) {
        dma_free_coherent(&pdev->dev, rx->size, rx->desc, rx->dma);
        rx->desc = NULL;
    }
}