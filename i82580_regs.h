#ifndef _I82580_REGS_H_
#define _I82580_REGS_H_


/* CTRL Register (Device Control, 0x0000) */
#define I82580_CTRL         0x0000
#define I82580_CTRL_PHY_RST BIT(18) /* PHY Reset: 1=reset, 0=normal */

/* PCS Registers (for auto-negotiation) */
#define I82580_PCS_CFG      0x4200  /* PCS Configuration */
#define I82580_PCS_LCTL     0x4208  /* PCS Link Control */
#define I82580_PCS_LSTS     0x4210  /* PCS Link Status */
#define I82580_PCS_ANADV    0x4218  /* Auto-Neg Advertisement */

/* PCS Bits */
#define I82580_PCS_CFG_PCS_EN BIT(31) /* PCS Enable */
#define I82580_PCS_LCTL_AN_ENABLE BIT(0) /* Auto-Neg Enable */
#define I82580_PCS_LCTL_AN_RESTART BIT(1) /* Auto-Neg Restart */
#define I82580_PCS_LSTS_AN_COMPLETE BIT(5) /* Auto-Neg Complete */
#define I82580_PCS_LSTS_LINK_OK BIT(0) /* Link OK */

/* SCTL (SerDes Control, for SGMII/SerDes mode if needed) */
#define I82580_SCTL         0x04218
#define I82580_EERD       0x14
#define I82580_EERD_START BIT(0)
#define I82580_EERD_DONE  BIT(1)
#define I82580_EERD_ADDR_SHIFT 2
#define I82580_EERD_DATA_SHIFT 16

/* MDIC for phy management */
#define I82580_MDIC      0x20  /* MDI Control register */
#define I82580_MDIC_DATA_MASK   0xFFFF
#define I82580_MDIC_REG_SHIFT   16
#define I82580_MDIC_PHY_SHIFT   21
#define I82580_MDIC_OP_READ     (2 << 26)
#define I82580_MDIC_OP_WRITE    (1 << 26)
#define I82580_MDIC_READY       (1 << 28)
#define I82580_MDIC_ERROR       (1 << 30)

/* STATUS Register */
#define I82580_STATUS       0x0008  /* Status Register (R) */

/* STATUS bits */
#define I82580_STATUS_FD    BIT(0)  /* Full Duplex: 1=full, 0=half */
#define I82580_STATUS_LU    BIT(1)  /* Link Up: 1=link up, 0=link down */
#define I82580_STATUS_LS_MASK   GENMASK(3, 2) /* Link Speed */
#define I82580_STATUS_LS_10M    (0 << 2) /* 10 Mb/s */
#define I82580_STATUS_LS_100M   (1 << 2) /* 100 Mb/s */
#define I82580_STATUS_LS_1000M  (2 << 2) /* 1000 Mb/s */
#define I82580_STATUS_TXOFF BIT(6)  /* Transmission Paused */
#define I82580_STATUS_SD_MASK   GENMASK(11, 10) /* Speed/Duplex Resolved */
#define I82580_STATUS_SD_10H    (0 << 10) /* 10Mb/s half-duplex */
#define I82580_STATUS_SD_100H   (1 << 10) /* 100Mb/s half-duplex */
#define I82580_STATUS_SD_1000F  (3 << 10) /* 1000Mb/s full-duplex */
#define I82580_STATUS_PHYRA BIT(19) /* PHY Reset Asserted */

/* Receive Address Registers */
#define I82580_RAL(n)           (0x5400 + (8 * (n))) /* Receive Address Low, n=0..15 */
#define I82580_RAH(n)           (0x5404 + (8 * (n))) /* Receive Address High, n=0..15 */
#define E1000_RAH_AV  (1 << 31)

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
#define SRRCTL_BSIZEPKT_SHIFT       10
#define SRRCTL_BSIZEPKT_MASK        (0x0000007F << SRRCTL_BSIZEPKT_SHIFT) /* Bits 16:10 */
#define SRRCTL_BSIZEPKT_2K          (0x2 << SRRCTL_BSIZEPKT_SHIFT)         /* 2KB buffer */
#define SRRCTL_DESCTYPE_MASK        0x0E000000                            /* Bits 27:25 */
#define SRRCTL_DESCTYPE_ADV_ONEBUF  0x02000000                            /* Advanced one buffer (001b) */
#define SRRCTL_BSIZEHEADER_MASK     0x00003F00                            /* Bits 13:8, for header buffer size */

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
#define I82580_RX_BUF_LEN      2048

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
#define I82580_IRQ_RX_MASK   (I82580_ICR_RXDW | I82580_ICR_RXDMT0 | I82580_ICR_RXMISS)
#define I82580_IRQ_TX_MASK   (I82580_ICR_TXDW)
#define I82580_IRQ_MISC_MASK (I82580_ICR_LSC | I82580_ICR_GPHY)
#define I82580_IRQ_ENABLE_MASK (I82580_IRQ_RX_MASK | I82580_IRQ_TX_MASK | I82580_IRQ_MISC_MASK)

/* rx adcanved desc */
#define I82580_RXD_STAT_DD  0x01 /* Descriptor Done */
#define I82580_RXD_STAT_EOP 0x02 /* End of Packet */

#endif /* _I82580_REGS_H_ */
