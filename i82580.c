#include "i82580.h"
#include "i82580_hw.h"
#include "i82580_regs.h"

char i82580_driver_name[] = "i82580";
static char i82580_driver_string[] = "Intel(R) I340 Network Driver";

static const struct pci_device_id i82580_pci_tbl[] = {
    { PCI_DEVICE(0x8086, 0x1516) },
    /* required last entry */
    {0, }
};

MODULE_DEVICE_TABLE(pci, i82580_pci_tbl);

/* all custom funcs */


static int i82580_init_module(void);
static void i82580_exit_module(void);
static int i82580_probe(struct pci_dev *pdev, const struct pci_device_id *ent);
static void i82580_remove(struct pci_dev *pdev);
static int mdic_read(void __iomem *hw, int phy, int reg, u16 *val);
static int mdic_write(void __iomem *hw, int phy, int reg, u16 val);
static int i82580_phy_init(struct i82580_adapter *adap);
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
static unsigned int i82580_desc_unused(struct i82580_ring *tx);
void i82580_clean_tx(struct i82580_adapter *adap);
static netdev_tx_t i82580_xmit_frame(struct sk_buff *skb, struct net_device *netdev);
static int i82580_close(struct net_device *ndev);
static u16 i82580_read_eeprom_word(void __iomem *hw, u16 addr);
static void mac_add(u8 *dst, const u8 *base, u8 add);
static void i82580_read_mac_addr(struct i82580_adapter *adap);
static void i82580_check_link(struct i82580_adapter *adap);



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
    pr_info("%s : init\n", i82580_driver_string);

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
    pr_info("%s : exit\n", i82580_driver_string);
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

    if (PCI_FUNC(pdev->devfn) != 0)
          return -ENODEV;

    /* Enable PCI DEVICE (MMIO only) */
    bars = pci_select_bars(pdev, IORESOURCE_MEM);

    pr_info("%s: enabling PCI device (bars=0x%x)\n", i82580_driver_name, bars);
    err = pci_enable_device_mem(pdev);
    if (err)
            return err;

    err = pci_request_selected_regions(pdev, bars, i82580_driver_name);
    if (err)
            goto err_disable_pci;
    
    pci_set_master(pdev);
    pr_info("%s: bus mastering enabled\n", i82580_driver_name);

    /* DMA mask (82580 supports 64-bit DMA) */
    err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(64));
    if (err) {
        dev_warn(&pdev->dev, "%s : falling back to 32-bit DMA\n", i82580_driver_name);
        err = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
        if (err)
            goto err_release_regions;
    } else {
        dev_info(&pdev->dev, "64-bit DMA endbled\n");
    }

    /* allocate netdev with private adapter */
    pr_info("%s: allocating net_device\n", i82580_driver_name);
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
    pr_info("%s: mapping BAR%d\n", i82580_driver_name, I82580_BAR_MMIO);
    adapter->hw_addr = pci_iomap(pdev, I82580_BAR_MMIO, 0);
    if (!adapter->hw_addr) {
        dev_err(&pdev->dev, i82580_driver_name, " : pci_iomap failed\n");
        err = -ENODEV;
        goto err_free_netdev_napi;
    }
    pr_info("%s: MMIO mapped at %p\n", i82580_driver_name, adapter->hw_addr);

    pr_info("%s adding new napi for rx queues\n", i82580_driver_name);
    netif_napi_add(netdev, &adapter->napi, i82580_poll);

    i82580_read_mac_addr(adapter);

    netdev->netdev_ops = &i82580_netdev_ops;
    pr_info("%s: detected MAC address %pM\n",
        i82580_driver_name, adapter->netdev->dev_addr);

    pr_info("%s: registering net_device\n", i82580_driver_name);
    err = register_netdev(netdev);
    if (err)
        goto err_free_iomap;

    pr_info("DESC SIZE=%zu offsetof(read.pkt_addr)=%zu offsetof(wb.status_error)=%zu sizeof(wb)=%zu\n",
        sizeof(union i82580_adv_rx_desc),
        offsetof(union i82580_adv_rx_desc, read.pkt_addr),
        offsetof(union i82580_adv_rx_desc, wb.status_error),
        sizeof(((union i82580_adv_rx_desc *)0)->wb));

    dev_info(&pdev->dev, "%s: probe successful\n", i82580_driver_name);

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
    struct i82580_adapter *adapter;

    if (!netdev)
        return;

    adapter = netdev_priv(netdev);

    pr_info("%s: remove() called\n", i82580_driver_name);

    unregister_netdev(netdev);

    netif_napi_del(&adapter->napi);

    if (adapter->hw_addr)
        pci_iounmap(pdev, adapter->hw_addr);

    pci_release_selected_regions(pdev, pci_select_bars(pdev, IORESOURCE_MEM));
    pci_disable_device(pdev);

    free_netdev(netdev);
}


static int mdic_read(void __iomem *hw, int phy, int reg, u16 *val)
{
    u32 mdic;
    int timeout = 1000;

    writel((reg << I82580_MDIC_REG_SHIFT) |
           (phy << I82580_MDIC_PHY_SHIFT) |
           I82580_MDIC_OP_READ,
           hw + I82580_MDIC);

    do {
        mdic = readl(hw + I82580_MDIC);
        if (mdic & I82580_MDIC_READY) {
            if (mdic & I82580_MDIC_ERROR)
                return -EIO;
            *val = (u16)(mdic & I82580_MDIC_DATA_MASK);
            return 0;
        }
        udelay(10);
    } while (--timeout);

    return -ETIMEDOUT;
}

static int mdic_write(void __iomem *hw, int phy, int reg, u16 val)
{
    u32 mdic;
    int timeout = 1000;

    writel(val |
           (reg << I82580_MDIC_REG_SHIFT) |
           (phy << I82580_MDIC_PHY_SHIFT) |
           I82580_MDIC_OP_WRITE,
           hw + I82580_MDIC);

    do {
        mdic = readl(hw + I82580_MDIC);
        if (mdic & I82580_MDIC_READY) {
            if (mdic & I82580_MDIC_ERROR)
                return -EIO;
            return 0;
        }
        udelay(10);
    } while (--timeout);

    return -ETIMEDOUT;
}

static int i82580_phy_init(struct i82580_adapter *adap)
{
    void __iomem *hw = adap->hw_addr;
    u16 bmcr, bmsr;
    int err, timeout;

    pr_info("%s: PHY init started\n", i82580_driver_name);

    /* Reset PHY */
    err = mdic_read(hw, 1, MII_BMCR, &bmcr); /* PHY address зазвичай 1 */
    if (err)
        return err;

    bmcr |= BMCR_RESET;
    mdic_write(hw, 1, MII_BMCR, bmcr);

    /* wait reset clear */
    timeout = 1000;
    do {
        err = mdic_read(hw, 1, MII_BMCR, &bmcr);
        if (err) return err;
        if (!(bmcr & BMCR_RESET))
            break;
        udelay(100);
    } while (--timeout);

    if (!timeout)
        return -ETIMEDOUT;

    /* Enable Auto-Neg */
    mdic_read(hw, 1, MII_BMCR, &bmcr);
    bmcr |= BMCR_ANENABLE | BMCR_ANRESTART;
    mdic_write(hw, 1, MII_BMCR, bmcr);

    /* Wait for Auto-Neg complete */
    timeout = 3000;
    do {
        err = mdic_read(hw, 1, MII_BMSR, &bmsr);
        if (err) return err;
        if (bmsr & BMSR_ANEGCOMPLETE) {
            pr_info("%s: Auto-Neg complete\n", i82580_driver_name);
            return 0;
        }
        msleep(10);
    } while (--timeout);

    return -ETIMEDOUT;
}



static int i82580_open(struct net_device *ndev)
{
    struct i82580_adapter *adapter = netdev_priv(ndev);
    int err;

    pr_info("%s: open() started\n", i82580_driver_name);

    if (test_bit(I82580_TESTING, &adapter->flags)) {
        pr_err("%s: device in testing mode, refusing open\n", i82580_driver_name);
        return -EBUSY;
    }

    /* Link down until configured */
    netif_carrier_off(ndev);

    pr_info("%s: allocating mem for tx ring\n", i82580_driver_name);
    adapter->tx_ring = kzalloc(sizeof(*adapter->tx_ring), GFP_KERNEL);
    if (!adapter->tx_ring) {
        pr_err("%s: failed to alloc tx ring\n", i82580_driver_name);
        return -ENOMEM;
    }

    pr_info("%s: setting up tx ring\n", i82580_driver_name);
    err = i82580_setup_tx_ring(adapter);
    if (err) {
        pr_err("%s: setup_tx_ring failed: %d\n", i82580_driver_name, err);
        goto err_tx;
    }

    pr_info("%s: programming tx regs\n", i82580_driver_name);
    i82580_tx_program_regs(adapter, 0);

    pr_info("%s: configuring tx queue\n", i82580_driver_name);
    i82580_tx_config_queue(adapter, 0);

    pr_info("%s: enabling tx in MAC\n", i82580_driver_name);
    i82580_tx_enable_mac(adapter);

    pr_info("%s: allocating mem for rx ring\n", i82580_driver_name);
    adapter->rx_ring = kzalloc(sizeof(*adapter->rx_ring), GFP_KERNEL);
    if (!adapter->rx_ring) {
        pr_err("%s: failed to alloc rx ring\n", i82580_driver_name);
        goto err_rx;
    }

    pr_info("%s: setting up rx ring\n", i82580_driver_name);
    err = i82580_setup_rx_ring(adapter);
    if (err) {
        pr_err("%s: setup_rx_ring failed: %d\n", i82580_driver_name, err);
        goto err_rx_setup;
    }

    pr_info("%s: allocating rx buffers\n", i82580_driver_name);
    err = i82580_alloc_all_rx_buffers(adapter);
    if (err) {
        pr_err("%s: alloc_all_rx_buffers failed: %d\n", i82580_driver_name, err);
        goto err_rx_bufs;
    }

    pr_info("%s: programming rx regs\n", i82580_driver_name);
    i82580_rx_program_regs(adapter, 0);

    pr_info("%s: configuring rx queue\n", i82580_driver_name);
    err = i82580_rx_config_queue(adapter, 0);
    if (err) {
        pr_err("%s: rx_config_queue failed: %d\n", i82580_driver_name, err);
        goto err_rx_cfg;
    }

    pr_info("%s: enabling rx in MAC\n", i82580_driver_name);
    i82580_rx_enable_mac(adapter);

    void __iomem *hw = adapter->hw_addr;
    pr_info("SRRCTL=%08x RXDCTL=%08x RCTL=%08x RDBAL=%08x RDBAH=%08x RDLEN=%08x RDH=%08x RDT=%08x\n",
        readl(hw + I82580_SRRCTL(0)), readl(hw + I82580_RXDCTL(0)),
        readl(hw + I82580_RCTL), readl(hw + I82580_RDBAL(0)), readl(hw + I82580_RDBAH(0)),
        readl(hw + I82580_RDLEN(0)), readl(hw + I82580_RDH(0)), readl(hw + I82580_RDT(0))); 

    pr_info("%s: enabling interrupts\n", i82580_driver_name);
    // writel(I82580_ICR_RX_MASK | I82580_ICR_TX_MASK | I82580_ICR_MISC_MASK,
    //        adapter->hw_addr + I82580_IMS);

    pr_info("%s: requesting irq %d\n", i82580_driver_name, adapter->pdev->irq);
    err = pci_alloc_irq_vectors(adapter->pdev, 1, 1,
        PCI_IRQ_MSI | PCI_IRQ_LEGACY);
    if (err < 0)
            return err;

    int irq = pci_irq_vector(adapter->pdev, 0);
    
    writel(~0U, hw + I82580_ICR);        
    /* flush: */
    readl(hw + I82580_STATUS);
    
    writel(I82580_IRQ_ENABLE_MASK, hw + I82580_IMS);

    err = request_irq(irq, i82580_isr, 0, ndev->name, adapter);
    if (err)
        pci_free_irq_vectors(adapter->pdev);

    pr_info("%s: enabling NAPI\n", i82580_driver_name);
    napi_enable(&adapter->napi);


    pr_info("%s: starting tx queue\n", i82580_driver_name);
    netif_start_queue(ndev);

    pr_info("%s: open() success\n", i82580_driver_name);

    err = i82580_phy_init(adapter);
    if (err) {
        pr_err("%s: PHY init failed: %d\n", i82580_driver_name, err);
        goto err_phy;
    }

    i82580_check_link(adapter);
    //netif_carrier_on(ndev);
    //pr_info("%s: link set UP (forced)\n", i82580_driver_name);

    return 0;

err_phy:
    pr_err("%s: error in request_irq, cleaning up\n", i82580_driver_name);
    napi_disable(&adapter->napi);
    netif_napi_del(&adapter->napi);
err_rx_cfg:
    pr_err("%s: error in rx config, cleaning up\n", i82580_driver_name);
    i82580_free_all_rx_buffers(adapter);
err_rx_bufs:
    pr_err("%s: error in rx bufs alloc, cleaning up\n", i82580_driver_name);
    i82580_free_rx_ring(adapter);
err_rx_setup:
    pr_err("%s: error in rx setup, cleaning up\n", i82580_driver_name);
    kfree(adapter->rx_ring);
err_rx:
    pr_err("%s: error in rx alloc, cleaning up\n", i82580_driver_name);
    i82580_free_tx_ring(adapter);
err_tx:
    pr_err("%s: error in tx setup, cleaning up\n", i82580_driver_name);
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
    if (!tx->desc){
        pr_err("dma_alloc_coherent failed\n");
        return -ENOMEM;
    }

    if (WARN_ON_ONCE(tx->dma & (128 - 1)))
            pr_warn("tx dma not 128B aligned (driver/pool bug)\n");

    tx->next_to_use = 0;
    tx->next_to_clean = 0;

    memset(tx->desc, 0, tx->size);

    tx->skbs = kcalloc(tx->count, sizeof(*tx->skbs), GFP_KERNEL);
    if (!tx->skbs) {
        pr_err("kcalloc skbs failed\n");
        goto err_free_ring;
    }

    tx->buf_dma = kcalloc(tx->count, sizeof(*tx->buf_dma), GFP_KERNEL);
    if (!tx->buf_dma) {
        pr_err("kcalloc buf_dma failed\n");
        goto err_free_skbs;
    }

    return 0;

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
    rx->size    = rx->count * sizeof(union i82580_adv_rx_desc);

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
    if (!rx->buf_dma)
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
    union i82580_adv_rx_desc *desc;
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

    desc = (union i82580_adv_rx_desc *)rx->desc + index;

    memset(desc, 0, sizeof(*desc));

    desc->read.pkt_addr = cpu_to_le64(dma);
    desc->read.hdr_addr = 0; /* for no header split */ 

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

    writel(lo, hw + I82580_RDBAL(q));
    writel(hi, hw + I82580_RDBAH(q));
    writel(rx->size, hw + I82580_RDLEN(q));

    srrctl = readl(hw + I82580_SRRCTL(q));
    srrctl &= ~(SRRCTL_BSIZEPKT_MASK | SRRCTL_DESCTYPE_MASK | SRRCTL_BSIZEHEADER_MASK);
    srrctl |= SRRCTL_BSIZEPKT_2K;
    srrctl |= SRRCTL_DESCTYPE_ADV_ONEBUF;
    writel(srrctl, hw + I82580_SRRCTL(q));

    /* RDH/RDT: всі N дескрипторів доступні для NIC */
    writel(0,             hw + I82580_RDH(q));
    // writel(rx->count - 1, hw + I82580_RDT(q));
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
        if (readl(hw + I82580_RXDCTL(q)) & RXDCTL_QUEUE_ENABLE) {
            dma_wmb();
            writel(adap->rx_ring[q].count - 1, hw + I82580_RDT(q));
            return 0;
        }
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
    u32 rctl = readl(hw + I82580_RCTL);

    rctl &= ~I82580_RCTL_EN;         
    writel(rctl, hw + I82580_RCTL);
    udelay(1);

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
    handled = icr & (I82580_IRQ_RX_MASK | I82580_IRQ_TX_MASK | I82580_IRQ_MISC_MASK);
    writel(icr, hw + I82580_ICR);

    pr_info("%s : IRQ %d, ICR=0x%08x, handled=0x%08x, unhandled=0x%08x\n",
        i82580_driver_name,  irq, icr, handled, icr & ~handled);

    /* Log unhandled bits for debug */
    if (icr & ~handled)
        netdev_warn(adap->netdev, "Unhandled ICR bits: 0x%x\n", icr & ~handled);

    /* RX events: schedule NAPI to handle bulk receive */
    if (icr & (I82580_ICR_RXDW | I82580_ICR_RXDMT0 | I82580_ICR_RXMISS)) {
        pr_info("%s : RX interrupt received\n", i82580_driver_name);

        /* more logs */
        u32 ics = readl(hw + I82580_ICS); 
        u32 icr = readl(hw + I82580_ICR);
        u32 ims = readl(hw + I82580_IMS);
        u32 rxdctl = readl(hw + I82580_RXDCTL(0));
        u32 srrctl = readl(hw + I82580_SRRCTL(0));
        u32 rctl = readl(hw + I82580_RCTL);
        u32 rdlen = readl(hw + I82580_RDLEN(0));
        u32 rdbal = readl(hw + I82580_RDBAL(0));
        u32 rdbah = readl(hw + I82580_RDBAH(0));
        u32 rdh = readl(hw + I82580_RDH(0));
        u32 rdt = readl(hw + I82580_RDT(0));

        pr_info("REGS ICR=0x%08x ICR=0x%08x IMS=0x%08x RXDCTL=0x%08x SRRCTL=0x%08x RCTL=0x%08x\n",
                ics, icr, ims, rxdctl, srrctl, rctl);
        pr_info("RING RDBAL=0x%08x RDBAH=0x%08x RDLEN=0x%08x RDH=%u RDT=%u\n",
                rdbal, rdbah, rdlen, rdh, rdt);

        writel(I82580_IRQ_RX_MASK, hw + I82580_IMC); /* mask RX bits */
        napi_schedule(&adap->napi);
    }

    /* TX done: clean completed TX descriptors (free skbs) */
    if (icr & I82580_IRQ_TX_MASK) {
        pr_info("%s : TX interrupt received\n", i82580_driver_name);
        i82580_clean_tx(adap);
    }

    /* Link status change */
    if (icr & I82580_ICR_LSC) {
        pr_info("%s : link status change interrupt received\n", i82580_driver_name);
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

    pr_info("poll: start (next_to_clean=%d, next_to_use=%d, budget=%d)\n",
        rx->next_to_clean, rx->next_to_use, budget);

    int j;
    union i82580_adv_rx_desc *d = rx->desc;
    for (j = 0; j < min(8, rx->count); j++) {
        pr_info("DUMP[%d] read.pkt=%016llx read.hdr=%016llx wb.stat=%08x wb.len=%u\n",
            j,
            (unsigned long long)le64_to_cpu(d[j].read.pkt_addr),
            (unsigned long long)le64_to_cpu(d[j].read.hdr_addr),
            (unsigned)le32_to_cpu(d[j].wb.status_error),
            (unsigned)le16_to_cpu(d[j].wb.length));
    }

    while (work_done < budget) {
        union i82580_adv_rx_desc *rxd;
        u16 pkt_len;
        u8 status, errors;
        struct sk_buff *skb;
        dma_addr_t dma;

        rxd = (union i82580_adv_rx_desc *)rx->desc + i;

        dma_rmb();

        u32 staterr32 = le32_to_cpu(rxd->wb.status_error);
        status = staterr32 & 0xFF;
        errors = (staterr32 >> 20) & 0xFF;

        /* raw descriptor dump for debug  */
        pr_info("RXD[%d] pkt_addr=%016llx hdr_addr=%016llx staterr=0x%04x len=%u\n",
            i,
            (unsigned long long)le64_to_cpu(rxd->read.pkt_addr),
            (unsigned long long)le64_to_cpu(rxd->read.hdr_addr),
            staterr32,
            (unsigned int)le16_to_cpu(rxd->wb.length));
   

        /* no completion yet */
        if (!(status & I82580_RXD_STAT_DD)) {
            pr_info("poll: idx=%d not ready (status=0x%02x)\n", i, status);
            break;
        }

        /* require EOP for simplicity; if fragmented packet handling needed - adjust */
        if (!(status & I82580_RXD_STAT_EOP)) {
            pr_info("poll: idx=%d DD present but no EOP (status=0x%02x) - skipping\n", i, status);
            /* treat as error for now */
            adap->stats.rx_errors++;
            goto rx_refill;
        }

        pkt_len = le16_to_cpu(rxd->wb.length);

        pr_info("poll: RX complete idx=%d len=%u status=0x%02x errors=0x%02x\n",
            i, pkt_len, status, errors);

        if (errors) {
            pr_info("poll: idx=%d RX error (errors=0x%x)\n", i, errors);
            adap->stats.rx_errors++;
            if (rx->skbs[i]) {
                dev_kfree_skb_any(rx->skbs[i]);
                rx->skbs[i] = NULL;
                rx->buf_dma[i] = 0;
            }
            goto rx_refill;
        }

        /* take skb that we previously allocated and attached to this index */
        skb = rx->skbs[i];
        if (!skb) {
            netdev_err(adap->netdev, "rx: missing skb at idx %d\n", i);
            adap->stats.rx_errors++;
            goto rx_refill;
        }

        /* unmap the DMA mapping we used for this skb */
        dma = rx->buf_dma[i];
        dma_unmap_single(&adap->pdev->dev, dma, I82580_RX_BUF_LEN, DMA_FROM_DEVICE);

        /* finalize skb and pass to stack */
        skb_put(skb, pkt_len);
        skb->protocol = eth_type_trans(skb, adap->netdev);
        skb_checksum_none_assert(skb);

        pr_info("poll: RX packet idx=%d len=%u skb=%p dma=%pad\n",
            i, pkt_len, skb, &dma);

        napi_gro_receive(napi, skb);
        adap->stats.rx_packets++;

        rx->skbs[i] = NULL;
        rx->buf_dma[i] = 0;

        work_done++;

        rx_refill:
        /* refill this descriptor with a fresh skb + DMA mapping */
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
                    netdev_err(adap->netdev, "rx: dma_map failed at idx %d\n", i);
                    adap->stats.rx_errors++;
                } else {
                    /* write new DMA addr into read-format field */
                    rxd->read.pkt_addr = cpu_to_le64(new_dma);
                    rxd->read.hdr_addr = 0;

                    /* clear writeback area so future checks see DD==0 */
                    rxd->wb.status_error = 0;
                    rxd->wb.length = 0;
                    rxd->wb.vlan = 0;

                    /* ensure all writes visible to device before we advance tail */
                    dma_wmb();

                    rx->skbs[i] = new_skb;
                    rx->buf_dma[i] = new_dma;
                    /* next_to_use points to next descriptor we consider filled for HW */
                    rx->next_to_use = (i + 1) % rx->count;

                    pr_info("poll: refill idx=%d new_skb=%p dma=%pad\n", i, new_skb, &new_dma);
                }
            }
        }

        /* advance index */
        i++;
        if (i >= rx->count)
            i = 0;
    } /* while budget */

    rx->next_to_clean = i;

    /* tell NIC where tail is (last available descriptor index) */
    writel(rx->next_to_use, hw + I82580_RDT(0));


    if (work_done < budget) {
        napi_complete_done(napi, work_done);
        /* re-enable RX interrupts */
        writel(I82580_IRQ_RX_MASK, hw + I82580_IMS);
    }

    pr_info("poll: end (work_done=%d, next_to_clean=%d, next_to_use=%d)\n",
            work_done, rx->next_to_clean, rx->next_to_use);

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

    pr_info("clean_tx: start (next_to_clean=%u, next_to_use=%u, count=%u)\n",
            ntc, ntu, count);

    while (ntc != ntu) {
        struct i82580_adv_tx_desc *txd = (struct i82580_adv_tx_desc *)tx->desc + ntc;
        u8 status = txd->upper.fields.status;

        /* Check DD (Descriptor Done) bit */
        if (!(status & 0x01)) {
            pr_info("clean_tx: idx=%u not done (status=0x%x)\n", ntc, status);
            break;
        }

        pr_info("clean_tx: idx=%u done (status=0x%x)\n", ntc, status);

        /* Free associated skb and unmap DMA */
        if (tx->skbs && tx->skbs[ntc]) {
            struct sk_buff *skb = tx->skbs[ntc];

            pr_info("clean_tx: freeing skb=%p dma=%pad len=%u\n",
                    skb, &tx->buf_dma[ntc], skb->len);

            dma_unmap_single(&adap->pdev->dev, tx->buf_dma[ntc], skb->len, DMA_TO_DEVICE);
            dev_kfree_skb_any(skb);
            tx->skbs[ntc] = NULL;
            tx->buf_dma[ntc] = 0;

            adap->stats.tx_packets++;
            adap->stats.tx_bytes += skb->len;
        }

        ntc++;
        if (ntc >= count)
            ntc = 0;
    }

    tx->next_to_clean = ntc;

    if (netif_queue_stopped(adap->netdev) && i82580_desc_unused(tx) >= TX_WAKE_THRESHOLD) {
        pr_info("clean_tx: waking queue (unused=%u)\n", i82580_desc_unused(tx));
        netif_wake_queue(adap->netdev);
    }

    pr_info("clean_tx: end (next_to_clean=%u, next_to_use=%u)\n",
            tx->next_to_clean, tx->next_to_use);
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
        pr_warn("TX: queue stopped (desc_unused=%u)\n", i82580_desc_unused(tx));
        return NETDEV_TX_BUSY;
    }

    i = tx->next_to_use;
    txd = (struct i82580_adv_tx_desc *)tx->desc + i;

    pr_info("TX: start xmit skb_len=%u, idx=%u, unused=%u\n",
        skb->len, i, i82580_desc_unused(tx));

    /* DMA map */
    dma = dma_map_single(&adap->pdev->dev, skb->data, skb->len, DMA_TO_DEVICE);
    if (dma_mapping_error(&adap->pdev->dev, dma)) {
        dev_kfree_skb_any(skb);
        adap->stats.tx_errors++;
        return NETDEV_TX_OK; /* Drop */
    }

    /* Fill descriptor */
    txd->buffer_addr = cpu_to_le64(dma);
    txd->lower.flags.length = cpu_to_le16(skb->len);
    txd->lower.flags.cso = 0;
    txd->lower.flags.cmd = (1 << 0) /* EOP */ |
                           (1 << 1) /* IFCS */ |
                           (1 << 3);/* RS */
    txd->upper.fields.status = 0;

    /* Save skb for cleanup */
    tx->skbs[i] = skb;
    tx->buf_dma[i] = dma;

    /* Advance */
    tx->next_to_use = (i + 1) % tx->count;

    pr_info("TX desc[%u]: dma=%pad len=%u cmd=0x%x\n",
        i, &dma, skb->len, txd->lower.flags.cmd);

    /* Notify HW */
    dma_wmb();
    writel(tx->next_to_use, adap->hw_addr + I82580_TDT(0));

    adap->stats.tx_packets++;
    adap->stats.tx_bytes += skb->len;

    pr_info("TX: done idx=%u next_to_use=%u TDT=%u\n",
        i, tx->next_to_use, readl(adap->hw_addr + I82580_TDT(0)));

    return NETDEV_TX_OK;
}


static int i82580_close(struct net_device *ndev)
{
    struct i82580_adapter *adapter = netdev_priv(ndev);
    void __iomem *hw = adapter->hw_addr;

    pr_info("%s: close() called\n", i82580_driver_name);

    netif_stop_queue(ndev);

    writel(~0U, hw + I82580_IMC);
    (void)readl(hw + I82580_ICR);

    free_irq(pci_irq_vector(adapter->pdev, 0), adapter);
    pci_free_irq_vectors(adapter->pdev);

    napi_disable(&adapter->napi);

    i82580_free_all_rx_buffers(adapter);
    i82580_free_rx_ring(adapter);
    i82580_free_tx_ring(adapter);

    kfree(adapter->rx_ring);
    kfree(adapter->tx_ring);

    return 0;
}


static u16 i82580_read_eeprom_word(void __iomem *hw, u16 addr)
{
    u32 val;

    writel(I82580_EERD_START | (addr << I82580_EERD_ADDR_SHIFT),
           hw + I82580_EERD);

    do {
        val = readl(hw + I82580_EERD);
    } while (!(val & I82580_EERD_DONE));

    return (u16)(val >> I82580_EERD_DATA_SHIFT);
}

/* addd func number to MAC */
static void mac_add(u8 *dst, const u8 *base, u8 add)
{
    int i; 
    u16 carry = add;
    for (i = ETH_ALEN - 1; i >= 0; i--) {
        u16 v = base[i] + (carry & 0xFF);
        dst[i] = (u8)v;
        carry = v >> 8;
    }
}

static void i82580_read_mac_addr(struct i82580_adapter *adap)
{
    void __iomem *hw = adap->hw_addr;
    struct pci_dev *pdev = adap->pdev;
    u8 base_mac[ETH_ALEN], mac[ETH_ALEN];
    u16 mac_word;
    int i;
    u8 fn = PCI_FUNC(pdev->devfn);

    /* базовий MAC з EEPROM */
    for (i = 0; i < 3; i++) {
        mac_word = i82580_read_eeprom_word(hw, i);
        base_mac[2*i]     = mac_word & 0xFF;
        base_mac[2*i + 1] = (mac_word >> 8) & 0xFF;
    }

    /* offset = func */
    mac_add(mac, base_mac, fn);

    if (!is_valid_ether_addr(mac)) {
        dev_warn(&pdev->dev,
                 "EEPROM MAC invalid, assigning random\n");
        eth_hw_addr_random(adap->netdev);
        return;
    }

    /* в netdev */
    eth_hw_addr_set(adap->netdev, mac);

    /* і в RAL0/RAH0 */
    u32 ral = mac[0] | (mac[1] << 8) | (mac[2] << 16) | (mac[3] << 24);
    u32 rah = mac[4] | (mac[5] << 8) | BIT(31);

    writel(ral, hw + I82580_RAL(0));
    writel(rah, hw + I82580_RAH(0));

    dev_info(&pdev->dev, "MAC address (fn%d): %pM\n", fn, mac);
}

static void i82580_check_link(struct i82580_adapter *adap)
{
    void __iomem *hw = adap->hw_addr;
    u32 status = readl(hw + I82580_STATUS);
    bool link_up = !!(status & I82580_STATUS_LU) || (readl(hw + I82580_PCS_LSTS) & I82580_PCS_LSTS_LINK_OK);
    if (link_up) {
        netif_carrier_on(adap->netdev);
        netdev_info(adap->netdev, "Link UP\n");
    } else {
        netif_carrier_off(adap->netdev);
        netdev_info(adap->netdev, "Link DOWN\n");
    }
}
