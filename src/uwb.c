#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <string.h>
#include <stdint.h>
#include <driver/gpio.h>

#include "pt.h"
#include "deca_device_api.h"
#include "beacon.h"
#include "deca_regs.h"
#include "hardware.h"
#include "uwb_definitions.h"

/* Default antenna delay values for 64 MHz PRF. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define SPEED_OF_LIGHT (299792458)

#define UUS_TO_DWT_TIME 63898

static void send_message(uint8_t* buf, uint8_t len, uint64_t ts);
static void uwb_parse_message();
static void uwb_task();
static uint64_t get_tx_timestamp_u64(void);
static uint64_t get_rx_timestamp_u64(void);

SemaphoreHandle_t uwb_mutex;
uint8_t living_nodes[16];
uint8_t living_nodes_index;
uint64_t has_word_until = 0;
uint8_t positioning_initialized = 0;

IRAM_ATTR void uwb_irq(TaskHandle_t *uwb_task) {
    int has_awoken;
    vTaskNotifyGiveFromISR(*uwb_task, &has_awoken);
    portYIELD_FROM_ISR(has_awoken);
}

void init_uwb() {
    // uwb spi
    spi_bus_config_t spi_config = {
        .sclk_io_num = PIN_SPI_UWB_SCK,
        .miso_io_num = PIN_SPI_UWB_MISO,
        .mosi_io_num = PIN_SPI_UWB_MOSI,
    };
    spi_bus_initialize(SPI_UWB_HOST, &spi_config, SPI_DMA_CH_AUTO);
    spi_device_interface_config_t uwb_conf = {
        .address_bits = 0,
        .clock_speed_hz = SPI_UWB_FREQ,
        .command_bits = 0,
        .dummy_bits = 0,
        .mode = 0,
        .input_delay_ns = 0,
        .spics_io_num = PIN_SPI_UWB_CS,
        .queue_size = 1,
    };
    spi_bus_add_device(SPI_UWB_HOST, &uwb_conf, &deca_spi_device);

    while (!dwt_checkidlerc()) {}
    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        printf("uwb error\n");
    }

    dwt_config_t config = UWB_CONFIG;

    dwt_txconfig_t txconfig_options = {
        0x34,       /* PG delay. */
        0xfdfdfdfd, /* TX power. */
        0x0         /*PG count*/
    };

    if (dwt_configure(&config)) {
        printf("uwb error2\n");
    }

    dwt_configuretxrf(&txconfig_options);

    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    dwt_setlnapamode(DWT_LNA_ENABLE | DWT_PA_ENABLE);

    dwt_setrxtimeout(0);
    dwt_setpreambledetecttimeout(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    dwt_setinterrupt(SYS_ENABLE_LO_RXFCG_ENABLE_BIT_MASK, 0, DWT_ENABLE_INT_ONLY);

    uwb_mutex = xSemaphoreCreateMutex();
    static TaskHandle_t uwb_reading_task_handle;
    xTaskCreatePinnedToCore(uwb_reading_task, "uwb_reading_task", 4096, NULL, configMAX_PRIORITIES - 1, &uwb_reading_task_handle, 1);
    xTaskCreatePinnedToCore(uwb_misc_task, "uwb_misc_task", 4096, NULL, 6, NULL, 1); // 6 is a good priority i like it

    gpio_set_direction(PIN_UWB_IRQ, GPIO_MODE_INPUT);
    gpio_set_intr_type(PIN_UWB_IRQ, GPIO_INTR_POSEDGE);
    gpio_intr_enable(PIN_UWB_IRQ);
    gpio_isr_handler_add(PIN_UWB_IRQ, uwb_irq, &uwb_reading_task_handle);
}

void uwb_reading_task() {
    uint64_t last_poll = esp_timer_get_time();
    uint64_t random_delay = 0;

    while (1) {
        uint32_t notification_value;
        xTaskNotifyWait(0, 0, &notification_value, 10);
        xSemaphoreTake(uwb_mutex, portMAX_DELAY);
        uint32_t sys_status = dwt_read32bitreg(SYS_STATUS_ID);
        if (sys_status & SYS_STATUS_RXFCG_BIT_MASK) {
            uwb_parse_message();
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK); // clear bit
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }
        else if (sys_status & SYS_STATUS_ALL_RX_ERR) {
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        } else {

        }
        xSemaphoreGive(uwb_mutex);
    }
}

void uwb_misc_task() {
    while (true) {

    }
}

static uint64_t get_tx_timestamp_u64(void)
{
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    int8_t i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

static uint64_t get_rx_timestamp_u64(void)
{
    uint8_t ts_tab[5];
    uint64_t ts = 0;
    int8_t i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

static void send_message(uint8_t* buf, uint8_t len, uint64_t ts) {
    dwt_forcetrxoff();
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK); //clear bit
    dwt_writetxdata(len, buf, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(len + 2, 0, 1); /* Zero offset in TX buffer, ranging. */
    uint8_t ret;
    if (ts == 0) {
        ret = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    } else {
        dwt_setdelayedtrxtime(ts);
        ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
    }
    
    while (ret == DWT_SUCCESS && (dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK) == 0) {
        vTaskDelay(1);
    }
}

static void uwb_parse_message() {
    uint8_t rx_buf[64];

    uint32_t length = dwt_read32bitreg(RX_FINFO_ID) & RX_BUFFER_MAX_LEN;
    if (length > sizeof(rx_buf)) {
        printf("too long\n");
        return;
    }
    dwt_readrxdata(rx_buf, length, 0);
    struct uwb_header *header = (struct uwb_header *) rx_buf; 
    if (memcmp(header->magic, UWB_MAGIC_WORD, sizeof(header->magic)) != 0) {
        printf("invalid header %.*s\n", sizeof(header->magic), header->magic);
        return;
    }
    if (header->receiver != node_id && header->receiver != 0xff) {
        return;
    }
    if (header->msg_type == UWB_MSG_POLL_RANGING) {
        uint64_t poll_rx_ts = get_rx_timestamp_u64();
        uint64_t extra_time = living_nodes_index * PER_MESSAGE;
        uint64_t random_delay = (extra_time * esp_random()) / UINT32_MAX;
        uint64_t resp_tx_time = (poll_rx_ts + ((random_delay + PER_MESSAGE) * UUS_TO_DWT_TIME)) >> 8;
        uint64_t resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
        uint8_t flags = 0;
        if (dimensions == 2) {
            flags |= UWB_HAS_2D_POSITION_BITMASK;
            flags |= UWB_IS_ANCHOR_BITMASK;
        } else 
        if (dimensions == 3) {
            flags |= UWB_HAS_3D_POSITION_BITMASK;
            flags |= UWB_IS_ANCHOR_BITMASK;
        }
        struct uwb_response_ranging_position tx_msg = {
            .header.magic = UWB_MAGIC_WORD,
            .header.msg_type = UWB_MSG_RESPONSE_RANGING,
            .header.sender = node_id,
            .header.receiver = header->sender,
            .rx_timestamp = poll_rx_ts,
            .tx_timestamp = resp_tx_ts,
            .position_x = pos_x,
            .position_y = pos_y,
            .position_z = pos_z,
            .flags = flags,
        };
        send_message(&tx_msg, sizeof(tx_msg), resp_tx_time);
    }
}