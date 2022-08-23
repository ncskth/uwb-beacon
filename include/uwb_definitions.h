#include <stdint.h>

#define UWB_BROADCAST_ID 0xff

#define UWB_MAGIC_WORD "cskth"

#define BROADCAST_RESPONSE_SLOT 10000 // 10ms

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

#define UWB_HAS_3D_POSITION_BITMASK (1 << 0)
#define UWB_HAS_2D_POSITION_BITMASK (1 << 1)
#define UWB_IS_ANCHOR_BITMASK (1 << 2)

enum uwb_msg_types {
    UWB_MSG_POLL_RANGING,
    UWB_MSG_RESPONSE_RANGING,
    UWB_MSG_RESPONSE_RANGING_POSITION,
    UWB_MSG_CALIBRATE_BEACONS,
    UWB_MSG_POLL_ALIVE,
    UWB_MSG_IS_ALIVE,
    UWB_MSG_GIVE_WORD
};

enum uwb_purposes {
    UWB_PURPOSE_BEACON_ORIGIN,
    UWB_PURPOSE_BEACON_X,
    UWB_PURPOSE_BEACON_Y,
    UWB_PURPOSE_BEACON_Z,
    UWB_PURPOSE_BEACON_REPEATER,
    UWB_PURPOSE_USER
}

#pragma pack(1)
struct uwb_header {
    char magic[5]; //{"cskth"}
    uint8_t msg_type;
    uint8_t sender;
    uint8_t receiver;
    uint8_t seq_num;
};

#pragma pack(1)
struct uwb_poll_msg {
    struct uwb_header header;
};

#pragma pack(1)
struct uwb_response_msg {
    struct uwb_header header;
    uint32_t rx_timestamp;
    uint32_t tx_timestamp;
};

#pragma pack(1)
struct uwb_response_ranging_position {
    struct uwb_header header;
    uint32_t rx_timestamp;
    uint32_t tx_timestamp;
    int32_t position_x; // in mm
    int32_t position_y; // in mm
    int32_t position_z; // in mm
    uint8_t flags;
    uint16_t error; // in mm
};

#pragma pack(1)
struct uwb_query_metadata {
    struct uwb_header header;
    uint8_t num_beacons;
    uint8_t num_users;
};

#pragma pack(1)
struct uwb_calibrate_beacons {
    struct uwb_header header;
};

#pragma pack(1)
struct uwb_is_alive {
    struct uwb_header header;
    uint8_t purpose;
};

#pragma pack(1)
struct uwb_give_word {
    struct uwb_header header;
    uint32_t duration;   
};