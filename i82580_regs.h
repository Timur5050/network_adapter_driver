#ifndef _I82580_REGS_H_
#define _I82580_REGS_H_

/* Receive Address Registers */
#define I82580_RAL(n)           (0x5400 + (8 * (n))) /* Receive Address Low, n=0..15 */
#define I82580_RAH(n)           (0x5404 + (8 * (n))) /* Receive Address High, n=0..15 */

/* TX Descriptor Base Address Low/High */
#define I82580_TDBAL(q)   (0xE000 + (q) * 0x40)  
#define I82580_TDBAH(q)   (0xE004 + (q) * 0x40)

/* TX Descriptor Length */
#define I82580_TDLEN(q)   (0xE008 + (q) * 0x40)

/* TX Descriptor Head/Tail */
#define I82580_TDH(q)     (0xE010 + (q) * 0x40)
#define I82580_TDT(q)     (0xE018 + (q) * 0x40)

#define TX_WAKE_THRESHOLD 32

/* -------------------- Transmit Control (TCTL) -------------------- */
#define I82580_TCTL       0x0400
#define I82580_TCTL_EN    (1 << 1)     /* Enable transmitter */
#define I82580_TCTL_PSP   (1 << 3)     /* Pad short packets */
#define I82580_TCTL_CT_SHIFT   4
#define I82580_TCTL_CT_MASK    (0xFF << I82580_TCTL_CT_SHIFT)
#define I82580_TCTL_CT_DEFAULT 0x0F    /* IEEE says 15 retries */
#define I82580_TCTL_BST_SHIFT  12
#define I82580_TCTL_BST_MASK   (0x3FF << I82580_TCTL_BST_SHIFT)
#define I82580_TCTL_BST_DEFAULT 0x40   /* back-off slot time */

/* -------------------- Transmit IPG (TIPG) -------------------- */
#define I82580_TIPG       0x0410
#define I82580_TIPG_DEFAULT 0x00702008 

/* -------------------- Transmit Descriptor Control (TXDCTL) -------------------- */
#define I82580_TXDCTL(q)   (0xE028 + (q) * 0x40)

/* Queue enable bit */
#define TXDCTL_QUEUE_ENABLE    (1 << 25)

/* Prefetch Threshold (PTHRESH) */
#define TXDCTL_PTHRESH_SHIFT   0
#define TXDCTL_PTHRESH_MASK    (0x1F << TXDCTL_PTHRESH_SHIFT)

/* Host Threshold (HTHRESH) */
#define TXDCTL_HTHRESH_SHIFT   8
#define TXDCTL_HTHRESH_MASK    (0x1F << TXDCTL_HTHRESH_SHIFT)

/* Write-back Threshold (WTHRESH) */
#define TXDCTL_WTHRESH_SHIFT   16
#define TXDCTL_WTHRESH_MASK    (0x1F << TXDCTL_WTHRESH_SHIFT)

/* Recommended values from datasheet */
#define TXDCTL_PTHRESH_DEFAULT 0x1F
#define TXDCTL_HTHRESH_DEFAULT 0x08
#define TXDCTL_WTHRESH_DEFAULT 0x01


/* -------------------- RX queue регістри -------------------- */
#define I82580_RDBAL(q)   (0xC000 + (q) * 0x40)
#define I82580_RDBAH(q)   (0xC004 + (q) * 0x40)
#define I82580_RDLEN(q)   (0xC008 + (q) * 0x40)
#define I82580_SRRCTL(q)  (0xC00C + (q) * 0x40)
#define I82580_RDH(q)     (0xC010 + (q) * 0x40)
#define I82580_RDT(q)     (0xC018 + (q) * 0x40)
#define I82580_RXDCTL(q)  (0xC028 + (q) * 0x40)

/* Receive Control Register */
#define I82580_RCTL          0x0100
#define I82580_RCTL_EN       (1 << 1)
#define I82580_RCTL_BAM      (1 << 15)
#define I82580_RCTL_SECRC    (1 << 26)
#define I82580_RCTL_BSIZE_MASK    (3 << 16)
#define I82580_RCTL_BSIZE_2048    (0 << 16)
#define I82580_RCTL_MO_MASK       (3 << 12)
#define I82580_RCTL_MO_0          (0 << 12)

/* RXDCTL thresholds table RXDCTL */
#define RXDCTL_PTHRESH_MASK     GENMASK(4, 0)     /* 0..16 */
#define RXDCTL_HTHRESH_MASK     GENMASK(12, 8)    /* 0..16 */
#define RXDCTL_WTHRESH_MASK     GENMASK(20, 16)   /* 0..15 */
#define RXDCTL_PTHRESH_DEFAULT  0x0C
#define RXDCTL_HTHRESH_DEFAULT  0x0A
#define RXDCTL_WTHRESH_DEFAULT  0x01
#define RXDCTL_QUEUE_ENABLE     BIT(25)

/* SRRCTL — Split and Replication Receive Control */
#define SRRCTL_BSIZEPKT_SHIFT   10
#define SRRCTL_BSIZEPKT_MASK    (0x0000007F << SRRCTL_BSIZEPKT_SHIFT) /* 7 bit */
#define SRRCTL_DESCTYPE_ADV_ONEBUF  (0x2)  /* desc type for  one-buffer mode */
#define SRRCTL_DESCTYPE_MASK        0x00000007

/* RCTL/RXCTRL — global bits for enabling rx controller 
 * RCTL (0x0100) RXCTRL (0x0300)
*/
#define I82580_RCTL      0x0100   /* «from MAC Control Registers» */
#define I82580_RXCTRL    0x0300   /* «from Receive Control» */

/* exact semantics */
#define RCTL_EN          BIT(1)   /* legacy RX */
#define RCTL_BAM         BIT(15)  /* accept broadcast */
#define RCTL_SECRC       BIT(26)  /* strip ethernet CRC */
#define RXCTRL_RXEN      BIT(0)   /* modern RX */

/* RX Buffer Length */
#define I82580_RX_BUF_LEN    1522

/* Interrupt registers (from your datasheet) */
#define I82580_ICR       0x1500  /* Interrupt Cause Read (RC/W1C) */
#define I82580_ICS       0x1504  /* Interrupt Cause Set (WO) */
#define I82580_IMS       0x1508  /* Interrupt Mask Set/Read */
#define I82580_IMC       0x150C  /* Interrupt Mask Clear */

/* ICR bits (from datasheet excerpt you pasted) */
#define I82580_ICR_TXDW      (1U << 0)   /* Transmit Descriptor Written Back */
#define I82580_ICR_LSC       (1U << 2)   /* Link Status Change */
#define I82580_ICR_RXDMT0    (1U << 4)   /* Receive Descriptor Minimum Threshold Reached */
#define I82580_ICR_RXMISS    (1U << 6)   /* Rx Missed packet (overflow) */
#define I82580_ICR_RXDW      (1U << 7)   /* Receiver Descriptor Write Back */
#define I82580_ICR_SWMB      (1U << 8)   /* SW mailbox write */
#define I82580_ICR_GPHY      (1U << 10)  /* PHY interrupt */

/* Useful masks for our driver */
#define I82580_ICR_RX_MASK   (I82580_ICR_RXDW | I82580_ICR_RXDMT0 | I82580_ICR_RXMISS)
#define I82580_ICR_TX_MASK   (I82580_ICR_TXDW)
#define I82580_ICR_MISC_MASK (I82580_ICR_LSC | I82580_ICR_GPHY)

#endif /* _I82580_REGS_H_ */