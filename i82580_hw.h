#ifndef _I82580_HW_H_
#define _I82580_HW_H_

#include <linux/types.h>

struct i82580_adv_tx_desc {
    __le64 buffer_addr;    /* Data buffer address */
    union {
        __le32 data;
        struct {
            __le16 length;    /* Data buffer length */
            u8 cso;           /* Checksum offset */
            u8 cmd;           /* Descriptor control (EOP, IFCS, RS, etc.) */
        } flags;
    } lower;
    union {
        __le32 data;
        struct {
            u8 status;        /* Descriptor status */
            u8 css;           /* Checksum start */
            __le16 vlan;      /* VLAN tag */
        } fields;
    } upper;
};
union i82580_adv_rx_desc {
    struct {                   /* READ (driver -> HW) */
        __le64 pkt_addr;
        __le64 hdr_addr;
    } read;
    struct {                   /* WRITE-BACK (HW -> driver) */
        __le32 mrq;
        union {
            __le32 rss;
            __le32 ip_id;
        } hi_dword;
        __le32 status_error;
        __le16 length;
        __le16 vlan;
    } wb;
};

#endif /* _I82580_HW_H_ */