#ifndef _I82580_H_
#define _I82580_H_

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/pci.h>
#include <linux/msi.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/bitfield.h>

struct i82580_adapter;

#define I82580_BAR_MMIO 0

enum {
    I82580_TESTING = 0,
    I82580_DOWN,
};

struct i82580_ring {
    void *desc;                     // list if descriptors ( DMA )
    dma_addr_t dma;                 // DMA-address for device
    unsigned int count;             // amount of descriptors in ring
    unsigned int size;              // size of ring
    unsigned int next_to_use;       // index of next entry to use
    unsigned int next_to_clean;     // index of next entry to clean

    struct sk_buff  **skbs;         // connected sk_buff for each descriptor
    dma_addr_t      *buf_dma;       // dma of mapped sk_buffs
};


struct i82580_adapter {

    /* OS defigned structs */
    struct net_device *netdev;
    struct pci_dev *pdev;

    /* tx */
    struct i82580_ring *tx_ring;
    int num_tx_queues;
    spinlock_t tx_lock;

    /* rx */
    struct napi_struct napi;
    struct i82580_ring *rx_ring;
    int num_rx_queues;
    spinlock_t rx_lock;

    /* hw mem */
    void __iomem *hw_addr;

    /* interrupts */
    struct msix_entry *msix_entries;
    int num_vectors;

    /* flags & debug */
    unsigned long flags;
};

#endif /* _I82580_H_ */