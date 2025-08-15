#ifndef _I82580_REGS_H_
#define _I82580_REGS_H_

/* TX Descriptor Base Address Low/High */
#define I82580_TDBAL(q)   (0xE000 + (q) * 0x40)  
#define I82580_TDBAH(q)   (0xE004 + (q) * 0x40)

/* TX Descriptor Length */
#define I82580_TDLEN(q)   (0xE008 + (q) * 0x40)

/* TX Descriptor Head/Tail */
#define I82580_TDH(q)     (0xE010 + (q) * 0x40)
#define I82580_TDT(q)     (0xE018 + (q) * 0x40)

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


#endif /* _I82580_REGS_H_ */