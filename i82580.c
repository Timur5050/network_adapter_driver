#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/msi.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/slab.h>


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
    pr_info("%s: vendor: 0x%04x, device: 0x%04x\n", i82580_driver_name, pdev->vendor, pdev->device);

    return 0;
}

static void i82580_remove(struct pci_dev *pdev)
{
    pr_info("%s : success if romoving\n", i82580_driver_name);
}

