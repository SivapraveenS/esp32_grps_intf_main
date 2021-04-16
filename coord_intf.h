/*
 *********************************************************************
 * File Name : coord_intf.h
 * Author : ram krishnan (rkris@wisense.in)
 * Created : Feb/3/2021
 *
 * Copyright (c) WiSense Technologies Pvt Ltd
 * All rights reserved.
 *********************************************************************
 */

#ifndef __COORD_INTF_ENA__
#define __COORD_INTF_ENA__

#define WSN_SHORT_BROADCAST_ADDR  0xffff

#define COORD_INTF_STS_SUCCESS         0x0
#define COORD_INTF_STS_DEV_BUSY        0x1
#define COORD_INTF_STS_HW_TX_ERR       0x2
#define COORD_INTF_STS_INV_PARAMS      0x3
#define COORD_INTF_STS_TX_IN_PROGRESS  0x4
#define COORD_INTF_STS_IDLE            0x5

#define COORD_INTF_MSG_TYPE_ACK   0x0

// Bit definitions of the flags field in the message header
#define COORD_INTF_HDR_ACK_BM  (1 << 7)
#define COORD_INTF_PYLD_ACK_BM  (~(1 << 7))

#define COORD_INTF_ACK_STS_OK_BM               (1 << 0)
#define COORD_INTF_ACK_STS_OOM_BM              (1 << 1)
#define COORD_INTF_ACK_STS_FRAME_TOO_LONG_BM   (1 << 2)
#define COORD_INTF_ACK_STS_INV_CRC             (1 << 3)
#define COORD_INTF_ACK_STS_RELAY_IN_PROGRESS   (1 << 4)
#define COORD_INTF_ACK_STS_HDR_BYTES_MISSING   (1 << 5)
#define COORD_INTF_ACK_STS_PYLD_BYTES_MISSING  (1 << 6)
#define COORD_INTF_ACK_STS_NODE_NOT_ASSOCIATED (3)


#define COORD_INTF_FRAME_HDR_MSG_TYPE_FIELD_LEN   2
#define COORD_INTF_FRAME_HDR_FLAGS_FIELD_LEN      1
#define COORD_INTF_FRAME_HDR_SEQ_NR_FIELD_LEN     1
#define COORD_INTF_FRAME_HDR_PYLD_LEN_FIELD_LEN   2
#define COORD_INTF_FRAME_HDR_CRC_FIELD_LEN        2

#define COORD_INTF_FRAME_HDR_HDR_CRC_FIELD_LEN   COORD_INTF_FRAME_HDR_CRC_FIELD_LEN
#define COORD_INTF_FRAME_HDR_PYLD_CRC_FIELD_LEN  COORD_INTF_FRAME_HDR_CRC_FIELD_LEN

#define COORD_INTF_FRAME_HDR_MSG_TYPE_FIELD_OFF   0

#define COORD_INTF_FRAME_HDR_FLAGS_FIELD_OFF \
        COORD_INTF_FRAME_HDR_MSG_TYPE_FIELD_LEN

#define COORD_INTF_FRAME_HDR_SEQ_NR_FIELD_OFF \
        (COORD_INTF_FRAME_HDR_FLAGS_FIELD_OFF + COORD_INTF_FRAME_HDR_FLAGS_FIELD_LEN)

#define COORD_INTF_FRAME_HDR_PYLD_LEN_FIELD_OFF \
        (COORD_INTF_FRAME_HDR_SEQ_NR_FIELD_OFF + COORD_INTF_FRAME_HDR_SEQ_NR_FIELD_LEN)

#define COORD_INTF_FRAME_HDR_HDR_CRC_FIELD_OFF  \
        (COORD_INTF_FRAME_HDR_PYLD_LEN_FIELD_OFF + COORD_INTF_FRAME_HDR_PYLD_LEN_FIELD_LEN)

#define COORD_INTF_FRAME_HDR_PYLD_CRC_FIELD_OFF \
        (COORD_INTF_FRAME_HDR_HDR_CRC_FIELD_OFF + COORD_INTF_FRAME_HDR_HDR_CRC_FIELD_LEN)

#define COORD_INTF_FRAME_MAX_PYLD_LEN  128

#define COORD_INTF_FRAME_HDR_LEN  (COORD_INTF_FRAME_HDR_MSG_TYPE_FIELD_LEN \
                                   + COORD_INTF_FRAME_HDR_FLAGS_FIELD_LEN \
                                   + COORD_INTF_FRAME_HDR_SEQ_NR_FIELD_LEN \
                                   + COORD_INTF_FRAME_HDR_PYLD_LEN_FIELD_LEN \
                                   + COORD_INTF_FRAME_HDR_HDR_CRC_FIELD_LEN \
                                   + COORD_INTF_FRAME_HDR_PYLD_CRC_FIELD_LEN)

#endif
