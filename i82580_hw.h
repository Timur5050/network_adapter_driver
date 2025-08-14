#ifndef _I82580_HW_H_
#define _I82580_HW_H_

#include <linux/types.h>

struct i82580_adv_tx_desc {
    __le64 buffer_addr;   // Address of data buffer
    union {
        __le32 data;      // For read format
        struct {
            __le16 length;  // Data buffer length
            u8     cso;     // Checksum offset
            u8     cmd;     // Command flags (EOP, IFCS, RS, etc.)
        } flags;
    } lower;
    union {
        __le32 data;      // For write-back format
        struct {
            u8 status;     // Descriptor status
            u8 css;        // Checksum start
            __le16 special;
        } wb;
    } upper;
};

struct i82580_adv_rx_desc {
    __le64 pkt_addr;    // Packet buffer address
    __le64 hdr_addr;    // Header buffer address (can be 0 if unused)
    union {
        __le32 data;    // For write-back format
        struct {
            __le16 length;   // Packet length
            __le16 csum;     // Packet checksum
        } lengths;
    } lower;
    union {
        __le32 data;
        struct {
            u8 status;       // Descriptor status (DD, EOP, etc.)
            u8 errors;       // Error flags
            __le16 vlan;     // VLAN tag if enabled
        } wb;
    } upper;
};


#endif /* _I82580_HW_H_ */