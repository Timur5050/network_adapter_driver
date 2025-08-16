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
void i82580_free_all_tx_buffers(struct i82580_adapter *adap);
void i82580_free_tx_ring(struct i82580_adapter *adap);
static int i82580_setup_rx_ring(struct i82580_adapter *adap);
static int i82580_alloc_all_rx_buffers(struct i82580_adapter *adap);
static int i82580_alloc_rx_buffer(struct i82580_adapter *adap, int index);
static void i82580_rx_program_regs(struct i82580_adapter *adap, int q);
static int i82580_rx_config_queue(struct i82580_adapter *adap, int q);
static void i82580_rx_enable_mac(struct i82580_adapter *adap);
static void i82580_free_all_rx_buffers(struct i82580_adapter *adap);
static void i82580_free_rx_ring(struct i82580_adapter *adap);
static irqreturn_t i82580_isr(int irq, void *dev_id);
static int i82580_poll(struct napi_struct *napi, int budget);
void i82580_clean_tx(struct i82580_adapter *adap);
static netdev_tx_t i82580_xmit_frame(struct sk_buff *skb, struct net_device *netdev);
static int i82580_close(struct net_device *ndev);
static void i82580_read_mac_addr(struct i82580_adapter *adap);



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
    .ndo_open = i82580_open,
    .ndo_start_xmit = i82580_xmit_frame,
    .ndo_stop = i82580_close
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

    i82580_read_mac_addr(adapter);

    netdev->netdev_ops = &i82580_netdev_ops;

    err = register_netdev(netdev);
    if (err)
        goto err_free_iomap;

    return 0;

err_free_iomap:
err_free_netdev_napi:
    free_netdev(netdev);
err_release_regions:
    pci_release_regions(pdev);
err_disable_pci:
    pci_disable_device(pdev);
    return err;
}

static void i82580_remove(struct pci_dev *pdev)
{
    struct net_device *netdev = pci_get_drvdata(pdev);
    struct i82580_adapter *adap;

    if (!netdev)
        return;

    adap = netdev_priv(netdev);

    unregister_netdev(netdev);
    free_netdev(netdev);
    if (adap->hw_addr)
        pci_iounmap(pdev, adap->hw_addr);

    pci_release_selected_regions(pdev, pci_select_bars(pdev, IORESOURCE_MEM));
    pci_disable_device(pdev);
}

static int i82580_open(struct net_device *ndev)
{
    struct i82580_adapter *adapter = netdev_priv(ndev);
    int err;

    if (test_bit(I82580_TESTING, &adapter->flags))
        return -EBUSY;

    netif_carrier_off(ndev);

    adapter->tx_ring = kzalloc(sizeof(*adapter->tx_ring), GFP_KERNEL);
    if (!adapter->tx_ring)
        return -ENOMEM;

    err = i82580_setup_tx_ring(adapter);
    if (err)
        goto err_tx;

    i82580_tx_program_regs(adapter, 0);
    i82580_tx_config_queue(adapter, 0);
    i82580_tx_enable_mac(adapter);

    adapter->rx_ring = kzalloc(sizeof(*adapter->rx_ring), GFP_KERNEL);
    if (!adapter->rx_ring)
        goto err_rx;

    err = i82580_setup_rx_ring(adapter);
    if (err)
        goto err_rx_setup;

    err = i82580_alloc_all_rx_buffers(adapter);
    if (err)
        goto err_rx_bufs;

    i82580_rx_program_regs(adapter, 0);
    err = i82580_rx_config_queue(adapter, 0);
    if (err)
        goto err_rx_cfg;

    i82580_rx_enable_mac(adapter);

    writel(I82580_ICR_RX_MASK | I82580_ICR_TX_MASK | I82580_ICR_MISC_MASK,
           adapter->hw_addr + I82580_IMS);

    netif_napi_add(ndev, &adapter->napi, i82580_poll);
    napi_enable(&adapter->napi);

    err = request_irq(adapter->pdev->irq, i82580_isr, 0, ndev->name, adapter);
    if (err)
        goto err_irq;

    netif_start_queue(ndev);

    return 0;

err_irq:
    napi_disable(&adapter->napi);
    netif_napi_del(&adapter->napi);
err_rx_cfg:
    i82580_free_all_rx_buffers(adapter);
err_rx_bufs:
    i82580_free_rx_ring(adapter);
err_rx_setup:
    kfree(adapter->rx_ring);
err_rx:
    i82580_free_tx_ring(adapter);
err_tx:
    kfree(adapter->tx_ring);
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

    tx->skbs = kcalloc(tx->count, sizeof(*tx->skbs), GFP_KERNEL);
    if (!tx->skbs)
        goto err_free_ring;

    tx->buf_dma = kcalloc(tx->count, sizeof(*tx->buf_dma), GFP_KERNEL);
        goto err_free_skbs;

err_free_skbs:
    kfree(tx->skbs);
    tx->skbs = NULL;
err_free_ring:
    dma_free_coherent(&pdev->dev, tx->size, tx->desc, tx->dma);
    tx->desc = NULL;
    return -ENOMEM;
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

void i82580_free_all_tx_buffers(struct i82580_adapter *adap)
{
    struct i82580_ring *tx = adap->tx_ring;
    int i;

    if (!tx || !tx->skbs)
        return;

    for (i = 0; i < tx->count; i++) {
        if (tx->skbs[i]) {
            dma_unmap_single(&adap->pdev->dev, tx->buf_dma[i],
                             I82580_RX_BUF_LEN, DMA_TO_DEVICE);
            dev_kfree_skb_any(tx->skbs[i]);
            tx->skbs[i] = NULL;
            tx->buf_dma[i] = 0;
        }
    }
}

void i82580_free_tx_ring(struct i82580_adapter *adap)
{
    struct i82580_ring *tx = adap->tx_ring;
    struct pci_dev *pdev = adap->pdev;

    if (!tx)
        return;

    i82580_free_all_tx_buffers(adap);

    kfree(tx->buf_dma);
    tx->buf_dma = NULL;

    kfree(tx->skbs);
    tx->skbs = NULL;

    if (tx->desc) {
        dma_free_coherent(&pdev->dev, tx->size, tx->desc, tx->dma);
        tx->desc = NULL;
    }
}

static int i82580_setup_rx_ring(struct i82580_adapter *adap)
{
    struct i82580_ring *rx = adap->rx_ring;
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

static irqreturn_t i82580_isr(int irq, void *dev_id)
{
    struct i82580_adapter *adap = dev_id;
    void __iomem *hw = adap->hw_addr;
    u32 icr, handled;

    /* Read cause. ICR is RC/W1C: reading gives pending causes. */
    icr = readl(hw + I82580_ICR);
    if (!icr)
        return IRQ_NONE; /* not our interrupt */

    /* Clear only handled bits */
    handled = icr & (I82580_ICR_RX_MASK | I82580_ICR_TX_MASK | I82580_ICR_MISC_MASK);
    writel(handled, hw + I82580_ICR);

    /* Log unhandled bits for debug */
    if (icr & ~handled)
        netdev_warn(adap->netdev, "Unhandled ICR bits: 0x%x\n", icr & ~handled);

    /* RX events: schedule NAPI to handle bulk receive */
    if (icr & I82580_ICR_RX_MASK) {
        writel(I82580_ICR_RX_MASK, hw + I82580_IMC); /* mask RX bits */
        napi_schedule(&adap->napi);
    }

    /* TX done: clean completed TX descriptors (free skbs) */
    if (icr & I82580_ICR_TX_MASK) {
        i82580_clean_tx(adap);
    }

    /* Link status change */
    if (icr & I82580_ICR_LSC) {
        netdev_info(adap->netdev, "Link status change\n");
        /* TODO: call i82580_check_link(adap); */
    }

    /* other events: GPHY, RXMISS, errors — log or handle */
    if (icr & I82580_ICR_RXMISS) {
        netdev_warn(adap->netdev, "RX buffer overflow (RXMISS)\n");
    }

    return IRQ_HANDLED;
}

static int i82580_poll(struct napi_struct *napi, int budget)
{
    struct i82580_adapter *adap = container_of(napi, struct i82580_adapter, napi);
    struct i82580_ring *rx = adap->rx_ring;
    void __iomem *hw = adap->hw_addr;
    int work_done = 0;
    int i = rx->next_to_clean;

    while (work_done < budget) {
        struct i82580_adv_rx_desc *rxd;
        u16 pkt_len;
        u8 status, errors;
        struct sk_buff *skb;
        dma_addr_t dma;

        rxd = (struct i82580_adv_rx_desc *)rx->desc + i;

        dma_rmb();

        status = rxd->upper.wb.status;
        errors = rxd->upper.wb.errors;
        if (!(status & 0x01))
            break;

        if (errors) {
            adap->stats.rx_errors++;
            if (rx->skbs[i]) {
                dev_kfree_skb_any(rx->skbs[i]);
                rx->skbs[i] = NULL;
                rx->buf_dma[i] = 0;
            }
            goto rx_refill;
        }

        pkt_len = le16_to_cpu(rxd->lower.lengths.length);
        dma = (dma_addr_t)le64_to_cpu(rxd->pkt_addr);

        skb = rx->skbs[i];
        if (!skb) {
            netdev_err(adap->netdev, "rx: missing skb at idx %d\n", i);
            adap->stats.rx_errors++;
            goto rx_refill;
        }

        dma_unmap_single(&adap->pdev->dev, rx->buf_dma[i], I82580_RX_BUF_LEN, DMA_FROM_DEVICE);

        skb_put(skb, pkt_len);
        skb->protocol = eth_type_trans(skb, adap->netdev);
        skb_checksum_none_assert(skb);

        napi_gro_receive(napi, skb);
        adap->stats.rx_packets++;

        rx->skbs[i] = NULL;
        rx->buf_dma[i] = 0;

        work_done++;

rx_refill:
        {
            struct sk_buff *new_skb = netdev_alloc_skb_ip_align(adap->netdev, I82580_RX_BUF_LEN);
            dma_addr_t new_dma;

            if (!new_skb) {
                netdev_warn(adap->netdev, "rx: skb alloc fail at idx %d\n", i);
                adap->stats.rx_errors++;
            } else {
                new_dma = dma_map_single(&adap->pdev->dev, new_skb->data,
                                         I82580_RX_BUF_LEN, DMA_FROM_DEVICE);
                if (dma_mapping_error(&adap->pdev->dev, new_dma)) {
                    dev_kfree_skb_any(new_skb);
                    netdev_err(adap->netdev, "rx: dma_map failed\n");
                    adap->stats.rx_errors++;
                } else {
                    rxd->pkt_addr = cpu_to_le64(new_dma);
                    rxd->hdr_addr = 0;
                    dma_wmb();
                    rx->skbs[i] = new_skb;
                    rx->buf_dma[i] = new_dma;
                    rx->next_to_use = (i + 1) % rx->count;
                }
            }
        }

        i++;
        if (i >= rx->count)
            i = 0;
    }

    rx->next_to_clean = i;

    writel((rx->next_to_use ? rx->next_to_use - 1 : rx->count - 1),
           hw + I82580_RDT(0));

    if (work_done < budget) {
        napi_complete_done(napi, work_done);
        writel(I82580_ICR_RX_MASK, hw + I82580_IMS);
    }

    return work_done;
}

static unsigned int i82580_desc_unused(struct i82580_ring *tx)
{
    return (tx->count + tx->next_to_clean - tx->next_to_use - 1) % tx->count;
}

void i82580_clean_tx(struct i82580_adapter *adap)
{
    struct i82580_ring *tx = adap->tx_ring;
    unsigned int ntc = tx->next_to_clean;
    unsigned int ntu = tx->next_to_use;
    unsigned int count = tx->count;

    while (ntc != ntu) {
        struct i82580_adv_tx_desc *txd = (struct i82580_adv_tx_desc *)tx->desc + ntc;
        u8 status = txd->upper.wb.status;

        /* Check DD bit */
        if (!(status & 0x01))
            break;

        /* descriptor done: free associated skb and unmap dma */
        if (tx->skbs && tx->skbs[ntc]) {
            struct sk_buff *skb = tx->skbs[ntc];
            dma_unmap_single(&adap->pdev->dev, tx->buf_dma[ntc], skb->len, DMA_TO_DEVICE);
            dev_kfree_skb_any(skb);
            tx->skbs[ntc] = NULL;
            tx->buf_dma[ntc] = 0;
        }

        ntc++;
        if (ntc >= count)
            ntc = 0;
    }

    tx->next_to_clean = ntc;

    if (netif_queue_stopped(adap->netdev) && i82580_desc_unused(tx) >= TX_WAKE_THRESHOLD) {
        netif_wake_queue(adap->netdev);
    }
}

static netdev_tx_t i82580_xmit_frame(struct sk_buff *skb, struct net_device *netdev)
{
    struct i82580_adapter *adap = netdev_priv(netdev);
    struct i82580_ring *tx = adap->tx_ring;
    struct i82580_adv_tx_desc *txd;
    dma_addr_t dma;
    unsigned int i;

    /* Check if enough descriptors are available */
    if (i82580_desc_unused(tx) < 2) {
        netif_stop_queue(netdev);
        return NETDEV_TX_BUSY;
    }

    i = tx->next_to_use;
    txd = (struct i82580_adv_tx_desc *)tx->desc + i;

    /* Map skb to DMA */
    dma = dma_map_single(&adap->pdev->dev, skb->data, skb->len, DMA_TO_DEVICE);
    if (dma_mapping_error(&adap->pdev->dev, dma)) {
        dev_kfree_skb_any(skb);
        adap->stats.rx_errors++;
        return NETDEV_TX_OK; /* Drop packet */
    }

    /* Fill descriptor */
    txd->buffer_addr = cpu_to_le64(dma);
    txd->lower.flags.length = cpu_to_le16(skb->len);
    txd->lower.flags.cmd = 0x03 | 0x08; /* EOP | IFCS | RS */
    txd->lower.flags.cso = 0;
    txd->upper.wb.status = 0;

    /* Store for cleanup */
    tx->skbs[i] = skb;
    tx->buf_dma[i] = dma;

    /* Advance next_to_use */
    tx->next_to_use = (i + 1) % tx->count;

    /* Publish to NIC */
    dma_wmb();
    writel(tx->next_to_use, adap->hw_addr + I82580_TDT(0));

    adap->stats.tx_packets++;

    return NETDEV_TX_OK;
}


static int i82580_close(struct net_device *ndev)
{
    struct i82580_adapter *adapter = netdev_priv(ndev);
    netif_stop_queue(ndev);
    napi_disable(&adapter->napi);
    netif_napi_del(&adapter->napi);
    free_irq(adapter->pdev->irq, adapter);
    i82580_free_all_rx_buffers(adapter);
    i82580_free_rx_ring(adapter);
    i82580_free_tx_ring(adapter);
    kfree(adapter->rx_ring);
    kfree(adapter->tx_ring);
    return 0;
}


static void i82580_read_mac_addr(struct i82580_adapter *adap)
{
    void __iomem *hw = adap->hw_addr;
    u32 ral, rah;
    const u8 *mac_addr = adap->netdev->dev_addr;

    /* Read MAC address from RAH/RAL registers (Receive Address 0) */
    ral = readl(hw + I82580_RAL(0));
    rah = readl(hw + I82580_RAH(0));

    /* Check Address Valid bit (AV) in RAH */
    if (!(rah & BIT(31))) {
        dev_warn(&adap->pdev->dev, "MAC address not valid, using random\n");
        eth_hw_addr_random(adap->netdev);
        return;
    }

    /* Since dev_addr is const, we can't modify it directly.
     * Instead, verify the MAC address and log it.
     */
    u8 temp_mac[ETH_ALEN];
    temp_mac[0] = (u8)(ral & 0xFF);
    temp_mac[1] = (u8)((ral >> 8) & 0xFF);
    temp_mac[2] = (u8)((ral >> 16) & 0xFF);
    temp_mac[3] = (u8)((ral >> 24) & 0xFF);
    temp_mac[4] = (u8)(rah & 0xFF);
    temp_mac[5] = (u8)((rah >> 8) & 0xFF);

    /* Compare with netdev->dev_addr */
    if (memcmp(mac_addr, temp_mac, ETH_ALEN) != 0) {
        dev_warn(&adap->pdev->dev, "MAC address mismatch, using random\n");
        eth_hw_addr_random(adap->netdev);
    } else if (!is_valid_ether_addr(mac_addr)) {
        dev_warn(&adap->pdev->dev, "Invalid MAC address, using random\n");
        eth_hw_addr_random(adap->netdev);
    } else {
        dev_info(&adap->pdev->dev, "MAC address: %pM\n", mac_addr);
    }
}
