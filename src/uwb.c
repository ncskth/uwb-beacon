#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <string.h>
#include <stdint.h>
#include <driver/gpio.h>
#include <nvs.h>

#include "pt.h"
#include "deca_device_api.h"
#include "beacon.h"
#include "deca_regs.h"
#include "hardware.h"
#include "uwb_definitions.h"
#include "position_solver.h"

/* Default antenna delay values for 64 MHz PRF. */
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define SPEED_OF_LIGHT (299792458)

#define UUS_TO_DWT_TIME 63898

#define MIN_TRX_DELAY 700

static void send_message_delayed(uint8_t* buf, uint8_t len, uint64_t ts);
static void send_message_global(uint8_t* buf, uint8_t len);
static void send_message_instantly(uint8_t* buf, uint8_t len);
static void uwb_parse_message();
static void uwb_misc_task();
static void uwb_reading_task();
static void uwb_send_task();

static uint64_t get_tx_timestamp_u64(void);
static uint64_t get_rx_timestamp_u64(void);

SemaphoreHandle_t uwb_mutex;
volatile uint8_t user_node_ids[32];
volatile uint8_t user_node_ids_index = 0;

volatile uint64_t has_word_until = 0;
volatile uint64_t message_received = 0;

volatile uint8_t origin_beacon_id;
volatile uint8_t x_beacon_id;

volatile uint8_t y_beacon_id;
volatile uint8_t z_beacon_id;
volatile uint8_t repeater_beacon_ids[32];
volatile uint8_t repeater_beacon_ids_index = 0;

TaskHandle_t uwb_misc_task_handle;
TaskHandle_t uwb_send_task_handle;
volatile uint8_t* uwb_send_task_buf; // i really can't be bothered to set up a RTOS queue for this...
volatile uint8_t uwb_send_task_len;


volatile struct distance_measurement distance_measurements[32];
volatile uint8_t distance_measurements_index = 0;

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

    gpio_set_direction(PIN_UWB_RESET, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_UWB_RESET, 1);
    vTaskDelay(1);
    gpio_set_level(PIN_UWB_RESET, 0);
    vTaskDelay(1);
    gpio_set_level(PIN_UWB_RESET, 1);
    vTaskDelay(1);

    while (!dwt_checkidlerc()) {}
    if (dwt_initialise(DWT_DW_INIT) == DWT_ERROR) {
        printf("uwb error\n");
    }

    dwt_config_t config = UWB_CONFIG;

    dwt_txconfig_t txconfig_options = {
        0x34,       /* PG delay. */
        0xFEFEFEFE, /* TX power. */
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
    xTaskCreatePinnedToCore(uwb_misc_task, "uwb_misc_task", 4096, NULL, 6, &uwb_misc_task_handle, 1);
    xTaskCreatePinnedToCore(uwb_send_task, "uwb_send_task", 4096, NULL, 7, &uwb_send_task_handle, 1);

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
        xSemaphoreGive(uwb_mutex);

        if (sys_status & SYS_STATUS_RXFCG_BIT_MASK) {
            uwb_parse_message();
            xSemaphoreTake(uwb_mutex, portMAX_DELAY);
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK); // clear bit
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            xSemaphoreGive(uwb_mutex);
        }
        else if (sys_status & SYS_STATUS_ALL_RX_ERR) {
            xSemaphoreTake(uwb_mutex, portMAX_DELAY);
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
            xSemaphoreGive(uwb_mutex);
        } else {

        }
        xTaskNotify(uwb_misc_task_handle, 0, 0);
    }
}

void uwb_send_task() {
    while (true) {
        xTaskNotifyWait(0, 0, NULL, portMAX_DELAY);
        vTaskDelay(GET_BROADCAST_WAIT_MS(node_id));
        send_message_instantly(uwb_send_task_buf, uwb_send_task_len);
        printf("sent globally %.*s\n", uwb_send_task_len, uwb_send_task_buf);
    }
}

void change_system_status(uint8_t system_status, uint8_t send) {
    positioning_system_status = system_status;
    nvs_handle_t nvs;
    ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &nvs));
    ESP_ERROR_CHECK(nvs_set_blob(nvs, "system_status", &positioning_system_status, sizeof(positioning_system_status)));
    if (send) {
        struct uwb_system_status_msg msg = {
            .header.magic = UWB_MAGIC_WORD,
            .header.msg_type = UWB_MSG_SYSTEM_STATUS,
            .header.receiver = UWB_BROADCAST_ID,
            .header.sender = node_id,
            .status = system_status,
        };
        send_message_instantly(&msg, sizeof(msg));
    }
}

void uwb_misc_task() {
    uint8_t had_word = 0;

    while (true) {
        if (gpio_get_level(PIN_BUTTON_SENSE) == 0) {
            vTaskDelay(500);
            change_system_status(UWB_SYSTEM_STATUS_CALIBRATING, true);
        }

        //when given word (x, y, z)
        if (esp_timer_get_time() < has_word_until && purpose != UWB_PURPOSE_BEACON_REPEATER) {
            if (!had_word) {
                pos_x = 0;
                pos_y = 0;
                pos_z = 0;
            }
            had_word = 1;
            printf("has word non repeater\n");
            //poll fast
            struct uwb_poll_ranging msg = {
                .header.magic = UWB_MAGIC_WORD,
                .header.msg_type = UWB_MSG_POLL_RANGING,
                .header.receiver = origin_beacon_id,
                .header.sender = node_id,
            };
            send_message_instantly(&msg, sizeof(msg));
            xTaskNotifyWait(0, 0, NULL, SINGLE_BUSY_FOR_MS);
            int32_t *p_pos = NULL;
            if (distance_measurements_index == 0) {
                continue;
            }

            if (purpose == UWB_PURPOSE_BEACON_X) {
                p_pos = &pos_x; 
            }
            if (purpose == UWB_PURPOSE_BEACON_Y) {
                p_pos = &pos_y; 
            }
            if (purpose == UWB_PURPOSE_BEACON_Z) {
                p_pos = &pos_z; 
            }
            if (*p_pos == 0) {
                *p_pos = distance_measurements[0].distance;
            }
            *p_pos = (distance_measurements[0].distance * 10 + *p_pos * 90) / 100;
            printf("distance %d %d\n", *p_pos, distance_measurements[0].distance);
            distance_measurements_index = 0;
        }

        //when given word (repeater)
        if (esp_timer_get_time() < has_word_until && purpose == UWB_PURPOSE_BEACON_REPEATER) {
            if (!had_word) {
                pos_x = 0;
                pos_y = 0;
                pos_z = 0;
            }
            had_word = 1;
            printf("has word repeater\n");
            //poll fast
            struct uwb_poll_ranging msg = {
                .header.magic = UWB_MAGIC_WORD,
                .header.msg_type = UWB_MSG_POLL_RANGING,
                .header.receiver = origin_beacon_id,
                .header.sender = node_id,
            };
            send_message_instantly(&msg, sizeof(msg));
            // xTaskNotifyWait(0, 0, NULL, SINGLE_BUSY_FOR_MS);
            vTaskDelay(SINGLE_BUSY_FOR_MS);

            msg.header.receiver = x_beacon_id;
            send_message_instantly(&msg, sizeof(msg));
            // xTaskNotifyWait(0, 0, NULL, SINGLE_BUSY_FOR_MS);
            vTaskDelay(SINGLE_BUSY_FOR_MS);

            msg.header.receiver = y_beacon_id;
            send_message_instantly(&msg, sizeof(msg));
            // xTaskNotifyWait(0, 0, NULL, SINGLE_BUSY_FOR_MS);
            vTaskDelay(SINGLE_BUSY_FOR_MS);
            
            msg.header.receiver = z_beacon_id;
            send_message_instantly(&msg, sizeof(msg));
            // xTaskNotifyWait(0, 0, NULL, SINGLE_BUSY_FOR_MS);
            vTaskDelay(SINGLE_BUSY_FOR_MS);

            struct pos_solver_position pos = {pos_x / 1000.0, pos_y / 1000.0, pos_z / 1000.0,};
            uint8_t fix;
            solve_for_position(distance_measurements, distance_measurements_index, &pos, &fix);
            distance_measurements_index = 0;
            if (pos_x == 0 && pos_y == 0 && pos_z == 0) {
                pos_x = pos.x;
                pos_y = pos.y;
                pos_z = pos.z;
            }
            if (fix == 3) {
                pos_x = (pos.x * 1000 * 20 + pos_x * 80) / 100;
                pos_y = (pos.y * 1000 * 20 + pos_y * 80) / 100;
                pos_z = (pos.z * 1000 * 20 + pos_z * 80) / 100;
                printf("pos %f %f %f %f %f %f\n", pos_x / 1000.0, pos_y / 1000.0, pos_z / 1000.0, pos.x, pos.y, pos.z);
            }
        }

        if (esp_timer_get_time() > has_word_until && had_word) {
            had_word = 0;
            printf("calibrated position %f %f %f\n", pos_x / 1000.0, pos_y / 1000.0, pos_z / 1000.0);
        }

        //the calibration process
        if (positioning_system_status == UWB_SYSTEM_STATUS_CALIBRATING && purpose == UWB_PURPOSE_BEACON_ORIGIN) {
            vTaskDelay(5000);
            printf("starting calibration\n");
            vTaskDelay(1000); // wait a second because why not
            x_beacon_id = UWB_BROADCAST_ID;
            y_beacon_id = UWB_BROADCAST_ID;
            z_beacon_id = UWB_BROADCAST_ID;
            repeater_beacon_ids_index = 0;

            struct uwb_poll_alive msg = {
                .header.magic = UWB_MAGIC_WORD,
                .header.msg_type = UWB_MSG_POLL_ALIVE,
                .header.receiver = UWB_BROADCAST_ID,
                .header.sender = node_id,
            };
            printf("poll alive\n");
            send_message_instantly(&msg, sizeof(msg));
            struct uwb_is_alive tx_msg = {
                .header.magic = UWB_MAGIC_WORD,
                .header.msg_type = UWB_MSG_IS_ALIVE,
                .header.sender = node_id,
                .header.receiver = UWB_BROADCAST_ID,
                .purpose = purpose,
            };
            send_message_global(&tx_msg, sizeof(tx_msg));
            vTaskDelay(BROADCAST_BUSY_FOR_MS);
            printf("beacons x:%d y:%d z:%d\n", x_beacon_id, y_beacon_id, z_beacon_id);
            if (x_beacon_id == UWB_BROADCAST_ID || y_beacon_id == UWB_BROADCAST_ID || z_beacon_id == UWB_BROADCAST_ID) {
                change_system_status(UWB_SYSTEM_STATUS_ERROR, true);
                printf("not enough beacons\n");
                continue;
            }

            printf("give word x %d\n", x_beacon_id);
            //give word x
            struct uwb_give_word word_msg = {
                .header.magic = UWB_MAGIC_WORD,
                .header.msg_type = UWB_MSG_GIVE_WORD,
                .header.receiver = x_beacon_id,
                .header.sender = node_id,
                .duration = 10000,
            };
            send_message_instantly(&word_msg, sizeof(word_msg));
            vTaskDelay(11000);

            //give word y
            printf("give word y %d\n", y_beacon_id);
            word_msg.header.receiver = y_beacon_id;
            send_message_instantly(&word_msg, sizeof(word_msg));
            vTaskDelay(11000);
            
            //give word z
            printf("give word z %d\n", z_beacon_id);
            word_msg.header.receiver = z_beacon_id;
            send_message_instantly(&word_msg, sizeof(word_msg));
            vTaskDelay(11000);

            for (uint8_t i = 0; i < repeater_beacon_ids_index; i++) {
                //give word i
                printf("give word i %d\n", repeater_beacon_ids[i]);
                word_msg.header.receiver = repeater_beacon_ids[i];
                send_message_instantly(&word_msg, sizeof(word_msg));
                vTaskDelay(11000);
            }
            //send system good
            printf("calibration done\n");
            change_system_status(UWB_SYSTEM_STATUS_GOOD, true);
        }
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

static void send_message_instantly(uint8_t* buf, uint8_t len) {
    xSemaphoreTake(uwb_mutex, portMAX_DELAY);
    dwt_forcetrxoff();
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK); //clear bit
    dwt_writetxdata(len, buf, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(len + 2, 0, 1); /* Zero offset in TX buffer, ranging. */
    uint8_t ret;
    ret = dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
    while (ret == DWT_SUCCESS && (dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK) == 0) {
        vTaskDelay(1);
    }
    xSemaphoreGive(uwb_mutex);
}

static void send_message_delayed(uint8_t* buf, uint8_t len, uint64_t ts) {
    xSemaphoreTake(uwb_mutex, portMAX_DELAY);
    dwt_forcetrxoff();
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS_BIT_MASK); //clear bit
    dwt_writetxdata(len, buf, 0); /* Zero offset in TX buffer. */
    dwt_writetxfctrl(len + 2, 0, 1); /* Zero offset in TX buffer, ranging. */
    uint8_t ret;
    dwt_setdelayedtrxtime(ts);
    ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);
    while (ret == DWT_SUCCESS && (dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS_BIT_MASK) == 0) {
        vTaskDelay(1);
    }
    xSemaphoreGive(uwb_mutex);
}

static void send_message_global(uint8_t* buf, uint8_t len) {
    static uint8_t static_buf[64];
    memcpy(static_buf, buf, len);
    uwb_send_task_buf = static_buf;
    uwb_send_task_len = len;
    xTaskNotifyGive(uwb_send_task_handle);
}

static void uwb_parse_message() {
    uint8_t rx_buf[64];
    xSemaphoreTake(uwb_mutex, portMAX_DELAY);
    uint32_t length = dwt_read32bitreg(RX_FINFO_ID) & RX_BUFFER_MAX_LEN;
    xSemaphoreGive(uwb_mutex);
    if (length > sizeof(rx_buf)) {
        printf("too long\n");
        return;
    }
    xSemaphoreTake(uwb_mutex, portMAX_DELAY);
    dwt_readrxdata(rx_buf, length, 0);
    xSemaphoreGive(uwb_mutex);

    struct uwb_header *header = (struct uwb_header *) rx_buf; 
    if (memcmp(header->magic, UWB_MAGIC_WORD, sizeof(header->magic)) != 0) {
        printf("invalid header %.*s\n", sizeof(header->magic), header->magic);
        return;
    }
    if (header->receiver != node_id && header->receiver != UWB_BROADCAST_ID) {
        return;
    }
    if (header->msg_type == UWB_MSG_POLL_RANGING) {
        uint8_t flags = 0;
        // if (positioning_system_status == UWB_SYSTEM_STATUS_GOOD) {
        flags |= UWB_HAS_3D_POSITION_BITMASK;
        // }
        flags |= UWB_IS_ANCHOR_BITMASK;

        uint64_t poll_rx_ts = get_rx_timestamp_u64();
        uint64_t resp_tx_time = (poll_rx_ts + ((MIN_TRX_DELAY) * UUS_TO_DWT_TIME)) >> 8;
        uint64_t resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
        struct uwb_response_ranging tx_msg = {
            .header.magic = UWB_MAGIC_WORD,
            .header.msg_type = UWB_MSG_RESPONSE_RANGING,
            .header.sender = node_id,
            .header.receiver = header->sender,
            .rx_timestamp = poll_rx_ts,
            .tx_timestamp = resp_tx_ts,
            .flags = flags,
            .position_x = pos_x,
            .position_y = pos_y,
            .position_z = pos_z,
        };
        send_message_delayed(&tx_msg, sizeof(tx_msg), resp_tx_time);
    } else
    if (header->msg_type == UWB_MSG_RESPONSE_RANGING) {
        struct uwb_response_ranging *rx_msg = (struct uwb_response_ranging*) rx_buf;
        uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
        int32_t rtd_init, rtd_resp;
        float clockOffsetRatio ;
        /* Read carrier integrator value and calculate clock offset ratio. */
        clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1<<26);
        /* Get timestamps embedded in response message. */
        poll_rx_ts = rx_msg->rx_timestamp;
        resp_tx_ts = rx_msg->tx_timestamp;
        // poll_tx_ts = dwt_readtxtimestamplo32();
        poll_tx_ts = dwt_readtxtimestamplo32();
        resp_rx_ts = dwt_readrxtimestamplo32();
        /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
        rtd_init = resp_rx_ts - poll_tx_ts;
        rtd_resp = resp_tx_ts - poll_rx_ts;
        // not hardware accelerated but what can a man do
        double tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
        if (tof < 0) {
            tof = 0;
        }
        struct distance_measurement new_measurement;
        new_measurement.id = header->sender;
        new_measurement.distance = 1000 * tof * SPEED_OF_LIGHT;
        new_measurement.error = rx_msg->error;
        new_measurement.position_x = rx_msg->position_x;
        new_measurement.position_y = rx_msg->position_y;
        new_measurement.position_z = rx_msg->position_z;
        new_measurement.flags = rx_msg->flags,
        distance_measurements[distance_measurements_index] = new_measurement;
        distance_measurements_index++;
    } else 
    if (header->msg_type == UWB_MSG_IS_ALIVE) {
        struct uwb_is_alive *rx_msg = (struct uwb_is_alive*) rx_buf;
        printf("lolcat %d %d\n", rx_msg->header.sender, rx_msg->purpose);
        if (rx_msg->purpose == UWB_PURPOSE_BEACON_X) {
            x_beacon_id = rx_msg->header.sender;
        }
        if (rx_msg->purpose == UWB_PURPOSE_BEACON_Y) {
            y_beacon_id = rx_msg->header.sender;
        }
        if (rx_msg->purpose == UWB_PURPOSE_BEACON_Z) {
            z_beacon_id = rx_msg->header.sender;
        }
        if (rx_msg->purpose == UWB_PURPOSE_BEACON_ORIGIN) {
            origin_beacon_id = rx_msg->header.sender;
        }
        if (rx_msg->purpose == UWB_PURPOSE_BEACON_REPEATER) {
            repeater_beacon_ids[repeater_beacon_ids_index] = rx_msg->header.sender;
            repeater_beacon_ids_index++;
        }
        if (rx_msg->purpose == UWB_PURPOSE_USER) {
            user_node_ids[user_node_ids_index] = rx_msg->header.sender;
            user_node_ids_index++;
        }
    } else
    if (header->msg_type == UWB_MSG_POLL_ALIVE) {
        struct uwb_is_alive tx_msg = {
            .header.magic = UWB_MAGIC_WORD,
            .header.msg_type = UWB_MSG_IS_ALIVE,
            .header.sender = node_id,
            .header.receiver = UWB_BROADCAST_ID,
            .purpose = purpose,
        };
        send_message_global(&tx_msg, sizeof(tx_msg));
    } else
    if (header->msg_type == UWB_MSG_GIVE_WORD) {
        struct uwb_give_word *rx_msg = (struct uwb_give_word*) rx_buf;
        origin_beacon_id = rx_msg->header.sender;
        has_word_until = esp_timer_get_time() + rx_msg->duration * 1000;
    } else
    if (header->msg_type == UWB_MSG_SYSTEM_STATUS) {
        struct uwb_system_status_msg *rx_msg = (struct uwb_system_status_msg*) rx_buf;
        change_system_status(rx_msg->status, false);
    }
}