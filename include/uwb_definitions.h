#include <stdint.h>

#define UWB_BROADCAST_ID 0xff
#define UWB_MAGIC_WORD "cskth"

#define UWB_CONFIG {\
    9,               /* Channel number. */\
    DWT_PLEN_128,    /* Preamble length. Used in TX only. */\
    DWT_PAC8,        /* Preamble acquisition chunk size. Used in RX only. */\
    9,               /* TX preamble code. Used in TX only. */\
    9,               /* RX preamble code. Used in RX only. */\
    1,               /* 0 to use standard 8 symbol SFD, 1 to use non-standard 8 symbol, 2 for non-standard 16 symbol SFD and 3 for 4z 8 symbol SDF type */\
    DWT_BR_6M8,      /* Data rate. */\
    DWT_PHRMODE_STD, /* PHY header mode. */\
    DWT_PHRRATE_STD, /* PHY header rate. */\
    (129 + 8 - 8),   /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */\
    DWT_STS_MODE_OFF, /* STS disabled */\
    DWT_STS_LEN_64,/* STS length see allowed values in Enum dwt_sts_lengths_e */\
    DWT_PDOA_M0      /* PDOA mode off */\
}

enum uwb_msg_types {
    UWB_MSG_POLL_RANGING,
    UWB_MSG_RESPONSE_RANGING,
    UWB_MSG_RESPONSE_RANGING_POSITION
};

#pragma pack(1)
struct uwb_header {
    char magic[5]; //{"cskth"}
    uint8_t msg_type;
    uint8_t sender;
    uint8_t receiver;
};

#pragma pack(1)
struct uwb_poll_msg {
    struct uwb_header header;
    
};

#pragma pack(1)
struct uwb_response_msg {
    struct uwb_header header;
    uint64_t rx_timestamp;
    uint64_t tx_timestamp;
};

#pragma pack(1)
struct uwb_response_ranging_position {
    struct uwb_header header;
    uint64_t rx_timestamp;
    uint64_t tx_timestamp;
    int64_t position_x;
    int64_t position_y;
    int64_t position_z;
};