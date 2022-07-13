#include <driver/gpio.h>
#include <driver/ledc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>
#include <string.h>
#include <esp_timer.h>
#include <esp_pm.h>


#include "deca_device_api.h"
#include "deca_regs.h"
#include "hardware.h"
#include "pt.h"

// #define RECEIVER
#define TRANSMITTER


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

#define UWB_MAGIC_WORD "biker"
#define TX_ANT_DLY 16385
#define RX_ANT_DLY 16385
#define SPEED_OF_LIGHT (299792458)
#define POLL_EVERY (1 * 1000000ull)
#define UUS_TO_DWT_TIME 63898
#define RX_TX_DELAY 1000

#define MAX_DISTANCE 2

enum uwb_msg_types {
    UWB_MSG_POLL_RANGING,
    UWB_MSG_RESPONSE_RANGING,
};

#pragma pack(1)
struct uwb_header {
    char magic[5];
    uint8_t msg_type;
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

void init_rgb();
void init_uwb();
static void send_message(uint8_t* buf, uint8_t len, uint64_t ts);
static void uwb_parse_message();
static void update_uwb();
static uint64_t get_tx_timestamp_u64(void);
static uint64_t get_rx_timestamp_u64(void);

float center_distance = 2;
float current_distance = 0;
uint16_t attempts = 0;
void HSVtoRGB( float *r, float *g, float *b, float h, float s, float v) {
	int i;
	float f, p, q, t;

	if( s == 0 ) {
		// achromatic (grey)
		*r = *g = *b = v;
		return;
	}

	h /= 60;			// sector 0 to 5
	i = floor( h );
	f = h - i;			// factorial part of h
	p = v * ( 1 - s );
	q = v * ( 1 - s * f );
	t = v * ( 1 - s * ( 1 - f ) );

	switch( i ) {
		case 0:
			*r = v;
			*g = t;
			*b = p;
			break;
		case 1:
			*r = q;
			*g = v;
			*b = p;
			break;
		case 2:
			*r = p;
			*g = v;
			*b = t;
			break;
		case 3:
			*r = p;
			*g = q;
			*b = v;
			break;
		case 4:
			*r = t;
			*g = p;
			*b = v;
			break;
		default:		// case 5:
			*r = v;
			*g = p;
			*b = q;
			break;
	}
}

void set_led(uint8_t r, uint8_t g, uint8_t b) {
    ledc_set_duty(LED_SPEED_MODE, CHANNEL_LED_R, r);
    ledc_set_duty(LED_SPEED_MODE, CHANNEL_LED_G, g);
    ledc_set_duty(LED_SPEED_MODE, CHANNEL_LED_B, b);

    ledc_update_duty(LED_SPEED_MODE, CHANNEL_LED_R);
    ledc_update_duty(LED_SPEED_MODE, CHANNEL_LED_G);
    ledc_update_duty(LED_SPEED_MODE, CHANNEL_LED_B);   
}

volatile uint8_t lol = 0;
IRAM_ATTR void rx_isr(void *misc) {
    lol++;
}

void app_main() {
    init_uwb();
    init_rgb();
    gpio_set_direction(PIN_BUTTON_SENSE, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_BUTTON_SENSE, GPIO_PULLUP_ONLY);

    gpio_set_direction(PIN_UWB_INTR, GPIO_MODE_INPUT);
    gpio_set_intr_type(PIN_UWB_INTR, GPIO_INTR_POSEDGE);
    gpio_intr_enable(PIN_UWB_INTR);
    gpio_install_isr_service(ESP_INTR_FLAG_IRAM);
    gpio_isr_handler_add(PIN_UWB_INTR, rx_isr, NULL);

    uint64_t last_poll = esp_timer_get_time();
    uint64_t last_poll_received = esp_timer_get_time();
    uint64_t random_delay = 0;

    while (1) {
        uint32_t sys_status = dwt_read32bitreg(SYS_STATUS_ID);
        if (sys_status & SYS_STATUS_RXFCG_BIT_MASK) {
            printf("rx\n");
            uwb_parse_message(); // this function can take up to 2ms because of delayed tx
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG_BIT_MASK); // clear bit
        }
        else if (sys_status & SYS_STATUS_RXFCE_BIT_MASK) {
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        } else {
        
        }

        #ifdef TRANSMITTER
        if (esp_timer_get_time() - last_poll > random_delay) {
            printf("poll\n");
            printf("intr %d\n", lol);
            last_poll = esp_timer_get_time();
            struct uwb_poll_msg msg = {
                .header.magic = UWB_MAGIC_WORD,
                .header.msg_type = UWB_MSG_POLL_RANGING,
            };
            send_message(&msg, sizeof(msg), 0);
            random_delay = POLL_EVERY;
            attempts++;
            if (attempts > 15) {
                set_led(255, 0, 0);
            }
        }
        if (gpio_get_level(PIN_BUTTON_SENSE) == 0) {
            center_distance = current_distance;
        }
        

        #endif
    }
}

void init_uwb() {
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

    dwt_setinterrupt(SYS_ENABLE_LO_RXFCG_ENABLE_BIT_MASK, 0, DWT_ENABLE_INT_ONLY);
    #ifdef RECEIVER
    dwt_setrxtimeout(0);
    dwt_setpreambledetecttimeout(0);
    dwt_rxenable(DWT_START_RX_IMMEDIATE);
    #else
    dwt_setrxtimeout(RX_TX_DELAY * 2);
    #endif
}

void init_rgb() {
    // led
    ledc_timer_config_t led_timer_conf_r = {
        .speed_mode = LED_SPEED_MODE,
        .timer_num = TIMER_LED_R,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = LED_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config_t led_timer_conf_g = {
        .speed_mode = LED_SPEED_MODE,
        .timer_num = TIMER_LED_G,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = LED_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config_t led_timer_conf_b = {
        .speed_mode = LED_SPEED_MODE,
        .timer_num = TIMER_LED_B,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .freq_hz = LED_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    
    ledc_channel_config_t led_channel_conf_r = {
        .speed_mode = LED_SPEED_MODE,
        .channel = CHANNEL_LED_R,
        .timer_sel = TIMER_LED_R,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PIN_LED_R,
        .duty = 0, // Set duty to 0%
        .hpoint = 0
    };
    ledc_channel_config_t led_channel_conf_g = {
        .speed_mode = LED_SPEED_MODE,
        .channel = CHANNEL_LED_G,
        .timer_sel = TIMER_LED_G,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PIN_LED_G,
        .duty = 0, // Set duty to 0%
        .hpoint = 0
    };
    ledc_channel_config_t led_channel_conf_b = {
        .speed_mode = LED_SPEED_MODE,
        .channel = CHANNEL_LED_B,
        .timer_sel = TIMER_LED_B,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PIN_LED_B,
        .duty = 0, // Set duty to 0%
        .hpoint = 0
    };

    ledc_timer_config(&led_timer_conf_r);
    ledc_timer_config(&led_timer_conf_g);
    ledc_timer_config(&led_timer_conf_b);

    ledc_channel_config(&led_channel_conf_r);
    ledc_channel_config(&led_channel_conf_g);
    ledc_channel_config(&led_channel_conf_b);
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
    printf("type %d\n", header->msg_type);
    if (header->msg_type == UWB_MSG_POLL_RANGING) {
        uint64_t poll_rx_ts = get_rx_timestamp_u64();
        uint64_t resp_tx_time = (poll_rx_ts + ((RX_TX_DELAY) * UUS_TO_DWT_TIME)) >> 8;
        uint64_t resp_tx_ts = (((uint64_t)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

        struct uwb_response_msg tx_msg = {
            .header.magic = UWB_MAGIC_WORD,
            .header.msg_type = UWB_MSG_RESPONSE_RANGING,
            .rx_timestamp = poll_rx_ts,
            .tx_timestamp = resp_tx_ts
        };
        send_message(&tx_msg, sizeof(tx_msg), resp_tx_time);
        uint64_t rx_again = (poll_rx_ts + ((POLL_EVERY - 1000ull) * UUS_TO_DWT_TIME)) >> 8;
        dwt_forcetrxoff();
        dwt_setdelayedtrxtime(rx_again);
        dwt_rxenable(DWT_START_RX_DELAYED);

    }
    else if (header->msg_type == UWB_MSG_RESPONSE_RANGING) {
        struct uwb_response_msg *rx_msg = (struct uwb_response_msg *) rx_buf;
        uint32_t poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
        int32_t rtd_init, rtd_resp;
        float clockOffsetRatio ;

        /* Read carrier integrator value and calculate clock offset ratio. */
        clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1<<26);

        /* Get timestamps embedded in response message. */
        poll_rx_ts = rx_msg->rx_timestamp;
        resp_tx_ts = rx_msg->tx_timestamp;
    
        poll_tx_ts = dwt_readtxtimestamplo32();
        resp_rx_ts = dwt_readrxtimestamplo32();

        /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
        rtd_init = resp_rx_ts - poll_tx_ts;
        rtd_resp = resp_tx_ts - poll_rx_ts;

        // not hardware accelerated but what can a man do
        double tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
        float distance = tof * SPEED_OF_LIGHT;
        float angle = 120 - (distance - center_distance) / (MAX_DISTANCE) * 120;
        current_distance = distance;
        angle = fmaxf(fminf(angle, 240), 0);
        printf("angle %f dist %f c %f\n", angle, distance, center_distance);
        float r, g, b;
        HSVtoRGB(&r, &g, &b, angle, 1.0, 1.0);
        set_led(r * 255, g * 255, b * 255);
        attempts = 0;

        static uint8_t flip = 255;
        #ifdef TRANSMITTER
        set_led(255, 0, 0);
        #else
        set_led(125,125,215);
        #endif
        set_led(flip, flip, flip);
        flip = 255 - flip;
    }  
    else {

    }
}