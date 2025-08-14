#include "i82580.h"
#include "i82580_hw.h"

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

