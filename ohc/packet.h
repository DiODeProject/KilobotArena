// 01010101
#define PAGE_SIZE 128
#define PACKET_HEADER 0x55
#define PACKET_SIZE   PAGE_SIZE+4
enum {
    PACKET_STOP,
    PACKET_LEDTOGGLE,
    PACKET_FORWARDMSG,
    PACKET_FORWARDRAWMSG,
    PACKET_BOOTPAGE
};

typedef enum {
    NORMAL = 0,
    GPS,
    SPECIAL = 0x80,
    BOOT = 0x80,
    BOOTPGM_PAGE,
    BOOTPGM_SIZE,
    RESET,
    SLEEP,
    WAKEUP,
    CHARGE,
    VOLTAGE,
    RUN,
    READUID,
    CALIB
} message_type_t;

enum {
    CALIB_SAVE,
    CALIB_UID,
    CALIB_TURN_LEFT,
    CALIB_TURN_RIGHT,
    CALIB_STRAIGHT
};

#pragma pack(1)
typedef  struct  __attribute__((__packed__)) {
    uint8_t mode;
    uint16_t uid;
    uint8_t turn_left;
    uint8_t turn_right;
    uint8_t straight_left;
    uint8_t straight_right;
    uint16_t unused;
} calibmsg_t;
