/*
 * main.c - HAGPS RTK System
 *
 * A dual-mode RTK GPS system using ESP32 and u-blox ZED-F9P:
 *
 *   BASE mode  : Reads RTCM3 corrections from ZED-F9P and publishes
 *                them to an MQTT broker over WiFi.
 *
 *   ROVER mode : Subscribes to RTCM3 corrections via MQTT, forwards
 *                them to ZED-F9P, and displays the resulting RTK
 *                position on the serial console.
 *
 * Device role and WiFi credentials are configured interactively
 * through the serial terminal on first boot, then stored in NVS.
 */

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_log.h"
#include "driver/uart.h"

#include "role_config.h"
#include "wifi_config.h"
#include "command_handler.h"
#include "wifi_comm.h"
#include "mqtt_comm.h"
#include "gnss_uart.h"
#include "nmea_parser.h"

/* ── MQTT broker settings ──────────────────────────────────────────── */
#define MQTT_BROKER_URI  "mqtt://13.214.212.87:1883"
#define MQTT_USERNAME    "mqtt"
#define MQTT_PASSWORD    "ICPHmqtt!"
#define MQTT_TOPIC       "rtk/corrections"

/* ── Timing constants ──────────────────────────────────────────────── */
#define LOOP_DELAY_MS       20      /* Main loop period (ms)            */
#define STATUS_INTERVAL_MS  5000    /* Status report interval (ms)      */

/* ── Logging tag ───────────────────────────────────────────────────── */
static const char *TAG = "MAIN";
static bool g_base_fixed_mode = false;

#define RUNTIME_CMD_BUF_SIZE  160

/* ── System initialization (shared by both modes) ──────────────────── */

static void system_init(void)
{
    /* NVS flash (required for WiFi + config storage) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* Default event loop (required for WiFi & MQTT) */
    ESP_ERROR_CHECK(esp_event_loop_create_default());
}

/**
 * Ensure device role is configured; prompt user if not.
 * Returns DEVICE_ROLE_BASE or DEVICE_ROLE_ROVER.
 */
static int ensure_role_configured(void)
{
    if (!role_config_exists()) {
        printf("\n*** Device role not configured ***\n");
        command_handler_init();
        command_handler_prompt();
        /* command_handler_prompt() will reboot after saving */
    }
    return role_config_load();
}

/**
 * Ensure WiFi credentials are configured; prompt user if not.
 * Loads SSID and password into the provided buffers.
 */
static void ensure_wifi_configured(char *ssid, size_t ssid_len,
                                   char *pass, size_t pass_len)
{
    if (!wifi_config_exists()) {
        printf("\n*** WiFi credentials not configured ***\n");
        command_handler_init();
        command_handler_prompt();
    }

    if (!wifi_config_load(ssid, ssid_len, pass, pass_len)) {
        ESP_LOGE(TAG, "Failed to load WiFi credentials");
        esp_restart();
    }
}

/**
 * Connect to WiFi and MQTT, blocking until both are ready.
 * If WiFi fails, clears credentials and reboots to re-prompt.
 */
static void connect_network(const char *ssid, const char *pass,
                            const char *client_id)
{
    printf("Connecting to WiFi '%s'...\n", ssid);

    /* Try to connect with a 15-second timeout */
    if (!wifi_init_sta(ssid, pass, 15000)) {
        printf("\n*** WiFi connection failed! ***\n");
        printf("Clearing credentials and restarting...\n\n");
        wifi_stop();
        wifi_config_clear();
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }

    mqtt_comm_init(MQTT_BROKER_URI, MQTT_USERNAME, MQTT_PASSWORD,
                   client_id, MQTT_TOPIC);

    printf("Waiting for MQTT connection...\n");
    while (!mqtt_is_connected()) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    printf("MQTT connected.\n\n");
}

/* ── BASE station task ─────────────────────────────────────────────── */

/* UBX-NAV-SVIN parser - extracts survey-in status from ZED-F9P */
typedef struct {
    uint32_t dur;       /* Survey duration in seconds */
    uint32_t meanAcc;   /* Mean accuracy in 0.1mm units */
    bool     valid;     /* Survey complete flag */
    bool     active;    /* Survey active flag */
} ubx_svin_t;

/**
 * Parse UBX-NAV-SVIN message (0xB5 0x62 0x01 0x3B).
 * Returns true if found and parsed successfully.
 */
static bool parse_ubx_svin(const uint8_t *buf, size_t len, ubx_svin_t *svin)
{
    static bool logged_search = false;
    
    /* Look for UBX sync chars + NAV-SVIN header */
    for (size_t i = 0; i + 48 < len; i++) {
        if (buf[i] == 0xB5 && buf[i+1] == 0x62 &&
            buf[i+2] == 0x01 && buf[i+3] == 0x3B) {
            
            if (!logged_search) {
                ESP_LOGI("PARSE", "Found UBX-NAV-SVIN header at offset %d", i);
                logged_search = true;
            }
            
            /* Payload length should be 40 bytes */
            uint16_t payload_len = buf[i+4] | (buf[i+5] << 8);
            if (payload_len != 40) {
                ESP_LOGW("PARSE", "NAV-SVIN payload length mismatch: %d (expected 40)", payload_len);
                continue;
            }
            
            const uint8_t *payload = &buf[i + 6];
            
            /* Extract fields (little-endian) */
            svin->dur = payload[8] | (payload[9] << 8) | 
                       (payload[10] << 16) | (payload[11] << 24);
            
            svin->meanAcc = payload[28] | (payload[29] << 8) |
                           (payload[30] << 16) | (payload[31] << 24);
            
            svin->valid = payload[36];
            svin->active = payload[37];
            
            logged_search = true;
            return true;
        }
    }
    return false;
}

/**
 * Save current configuration to ZED-F9P flash memory.
 * This makes the settings persistent across power cycles.
 */
static void save_config_to_flash(void)
{
    /* UBX-CFG-CFG: Save current config to flash (BBR, Flash, all layers) */
    uint8_t cfg_cmd[] = {
        0xB5, 0x62,       /* Sync */
        0x06, 0x09,       /* CFG-CFG */
        0x0D, 0x00,       /* Length: 13 bytes */
        0x00, 0x00, 0x00, 0x00,  /* clearMask: don't clear anything */
        0xFF, 0xFF, 0x00, 0x00,  /* saveMask: save all (ioPort, msgConf, infMsg, navConf, rxmConf, rinvConf, antConf) */
        0x00, 0x00, 0x00, 0x00,  /* loadMask: don't load */
        0x1F,             /* deviceMask: BBR, Flash, EEPROM, SPI, etc. */
        0x00, 0x00        /* Checksum */
    };

    /* Calculate checksum */
    uint8_t ck_a = 0, ck_b = 0;
    for (int i = 2; i < 21; i++) {
        ck_a += cfg_cmd[i];
        ck_b += ck_a;
    }
    cfg_cmd[21] = ck_a;
    cfg_cmd[22] = ck_b;

    gnss_uart_write(cfg_cmd, sizeof(cfg_cmd));
    vTaskDelay(pdMS_TO_TICKS(500));  /* Wait for flash write */

    ESP_LOGI("CONFIG", "Configuration saved to ZED-F9P flash memory");
}

static void configure_base_survey_in(uint32_t min_dur_sec, float acc_limit_m, 
                                      bool *rtcm_active, size_t *svin_len, size_t *nmea_len,
                                      bool *received_svin, bool *received_any_ubx)
{
    // STEP 1: Disable BASE mode first to stop any existing RTCM output
    uint8_t tmode3_disable[48] = {
        0xB5, 0x62, 0x06, 0x71, 0x28, 0x00,
        0x00, 0x00,
        0x00, 0x00,  // Mode = 0 (DISABLED)
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00,
        0x00, 0x00  // Checksum (filled below)
    };
    
    uint8_t ck_a = 0, ck_b = 0;
    for (int i = 2; i < 46; i++) {
        ck_a += tmode3_disable[i];
        ck_b += ck_a;
    }
    tmode3_disable[46] = ck_a;
    tmode3_disable[47] = ck_b;
    
    ESP_LOGI("BASE", "Disabling BASE mode...");
    gnss_uart_write(tmode3_disable, sizeof(tmode3_disable));
    vTaskDelay(pdMS_TO_TICKS(500));  // Wait for ZED to stop RTCM
    gnss_uart_flush();  // Clear old RTCM
    vTaskDelay(pdMS_TO_TICKS(300));  // Wait for any final data
    gnss_uart_flush();  // Flush again
    
    // STEP 1.5: Enable UBX-NAV-SVIN messages BEFORE configuring Survey-In
    uint8_t enable_svin_pre[] = {
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00,
        0x01, 0x3B,  // NAV-SVIN (class 0x01, ID 0x3B)
        0x00, 0x05, 0x05, 0x00, 0x00, 0x00,  // Rate=5 (every 5 epochs) on UART1/UART2
        0x00, 0x00   // Checksum
    };
    ck_a = 0;
    ck_b = 0;
    for (int i = 2; i < 14; i++) {
        ck_a += enable_svin_pre[i];
        ck_b += ck_a;
    }
    enable_svin_pre[14] = ck_a;
    enable_svin_pre[15] = ck_b;
    ESP_LOGI("BASE", "Enabling UBX-NAV-SVIN output (rate=5)...");
    gnss_uart_write(enable_svin_pre, sizeof(enable_svin_pre));
    vTaskDelay(pdMS_TO_TICKS(200));  // Wait for ZED to process
    
    // STEP 2: Now configure Survey-In mode
    uint32_t acc_limit_01mm = (uint32_t)(acc_limit_m * 10000.0f);
    uint8_t tmode3_survey[48] = {
        0xB5, 0x62, 0x06, 0x71, 0x28, 0x00,
        0x00, 0x00,
        0x01, 0x00,  // Mode = 1 (Survey-In)
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00,
        0x00, 0x00  // Checksum bytes (filled in below)
    };

    tmode3_survey[30] = (uint8_t)(min_dur_sec & 0xFF);
    tmode3_survey[31] = (uint8_t)((min_dur_sec >> 8) & 0xFF);
    tmode3_survey[32] = (uint8_t)((min_dur_sec >> 16) & 0xFF);
    tmode3_survey[33] = (uint8_t)((min_dur_sec >> 24) & 0xFF);

    tmode3_survey[34] = (uint8_t)(acc_limit_01mm & 0xFF);
    tmode3_survey[35] = (uint8_t)((acc_limit_01mm >> 8) & 0xFF);
    tmode3_survey[36] = (uint8_t)((acc_limit_01mm >> 16) & 0xFF);
    tmode3_survey[37] = (uint8_t)((acc_limit_01mm >> 24) & 0xFF);

    ck_a = 0;
    ck_b = 0;
    for (int i = 2; i < 46; i++) {
        ck_a += tmode3_survey[i];
        ck_b += ck_a;
    }
    tmode3_survey[46] = ck_a;
    tmode3_survey[47] = ck_b;

    ESP_LOGI("BASE", "Starting NEW Survey-In...");
    gnss_uart_write(tmode3_survey, sizeof(tmode3_survey));
    
    // Wait for ZED to process and start outputting SVIN messages
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Clear all buffers and state when restarting Survey-In
    if (svin_len) *svin_len = 0;  // Clear UBX buffer
    if (nmea_len) *nmea_len = 0;  // Clear NMEA buffer
    if (received_svin) *received_svin = false;  // Reset SVIN tracking
    if (received_any_ubx) *received_any_ubx = false;  // Reset UBX tracking
    if (rtcm_active) {
        *rtcm_active = false;  // Reset RTCM detection
        ESP_LOGI("BASE", "RTCM detection reset - waiting for Survey-In completion");
    }
    g_base_fixed_mode = false;
    
    ESP_LOGI("BASE", "Survey-In triggered: duration=%lus, acc=%.2fm",
             (unsigned long)min_dur_sec, acc_limit_m);
}

static void configure_base_fixed_position(double lat_deg, double lon_deg, double alt_m)
{
    int32_t lat_1e7 = (int32_t)(lat_deg * 1e7);
    int32_t lon_1e7 = (int32_t)(lon_deg * 1e7);
    int32_t alt_cm = (int32_t)(alt_m * 100.0);

    uint8_t tmode3[48] = {
        0xB5, 0x62, 0x06, 0x71, 0x28, 0x00,
        0x00, 0x00,
        0x02, 0x01,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x20, 0x4E, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00,
        0x00, 0x00  // Checksum bytes (filled in below)
    };

    tmode3[10] = (uint8_t)(lat_1e7 & 0xFF);
    tmode3[11] = (uint8_t)((lat_1e7 >> 8) & 0xFF);
    tmode3[12] = (uint8_t)((lat_1e7 >> 16) & 0xFF);
    tmode3[13] = (uint8_t)((lat_1e7 >> 24) & 0xFF);

    tmode3[14] = (uint8_t)(lon_1e7 & 0xFF);
    tmode3[15] = (uint8_t)((lon_1e7 >> 8) & 0xFF);
    tmode3[16] = (uint8_t)((lon_1e7 >> 16) & 0xFF);
    tmode3[17] = (uint8_t)((lon_1e7 >> 24) & 0xFF);

    tmode3[18] = (uint8_t)(alt_cm & 0xFF);
    tmode3[19] = (uint8_t)((alt_cm >> 8) & 0xFF);
    tmode3[20] = (uint8_t)((alt_cm >> 16) & 0xFF);
    tmode3[21] = (uint8_t)((alt_cm >> 24) & 0xFF);

    uint8_t ck_a = 0, ck_b = 0;
    for (int i = 2; i < 46; i++) {
        ck_a += tmode3[i];
        ck_b += ck_a;
    }
    tmode3[46] = ck_a;
    tmode3[47] = ck_b;

    gnss_uart_write(tmode3, sizeof(tmode3));
    vTaskDelay(pdMS_TO_TICKS(200));
    g_base_fixed_mode = true;
    ESP_LOGI("BASE", "Fixed mode set: lat=%.7f lon=%.7f alt=%.2fm", lat_deg, lon_deg, alt_m);
}

static void runtime_base_cmd_init(void)
{
    // UART0 is already configured by ESP-IDF console
    // We'll use stdin which is non-blocking in ESP-IDF
    // No additional setup needed
}

static void runtime_base_cmd_poll(bool *rtcm_active, size_t *svin_len, size_t *nmea_len,
                                   bool *received_svin, bool *received_any_ubx)
{
    static char line[RUNTIME_CMD_BUF_SIZE];
    static int line_pos = 0;
    int c;

    // Read from stdin (getchar is non-blocking in ESP-IDF)
    while ((c = getchar()) != EOF) {
        if (c == '\n' || c == '\r') {
            if (line_pos > 0) {
                printf("\n");  // Echo newline
                line[line_pos] = '\0';
                
                unsigned long dur_ul = 0;
                uint32_t dur = 0;
                float acc = 0.0f;
                double lat = 0.0, lon = 0.0, alt = 0.0;

                if (sscanf(line, "zed surveyin %lu %f", &dur_ul, &acc) == 2) {
                    dur = (uint32_t)dur_ul;
                    if (dur < 60) dur = 60;
                    if (acc < 0.1f) acc = 0.1f;
                    configure_base_survey_in(dur, acc, rtcm_active, svin_len, nmea_len,
                                            received_svin, received_any_ubx);
                    save_config_to_flash();
                    role_config_mark_zed_configured();  // Mark as configured
                    printf("\n*** Survey-In RESTARTED ***\n");
                    printf("*** Old RTCM stopped, new survey started from scratch ***\n");
                    printf("*** Duration: %lus, Accuracy: %.2fm ***\n", dur_ul, acc);
                    printf("*** Settings saved - will persist after reboot ***\n\n");
                } else if (sscanf(line, "zed fixed %lf %lf %lf", &lat, &lon, &alt) == 3) {
                    configure_base_fixed_position(lat, lon, alt);
                    save_config_to_flash();
                    role_config_mark_zed_configured();  // Mark as configured
                    if (rtcm_active) {
                        *rtcm_active = true;  // Fixed mode = immediate RTCM
                    }
                    printf("\n*** FIXED BASE MODE SET ***\n");
                    printf("*** Position: lat=%.7f lon=%.7f alt=%.2fm ***\n", lat, lon, alt);
                    printf("*** RTCM corrections active immediately ***\n");
                    printf("*** Settings saved - will persist after reboot ***\n\n");
                } else if (strcmp(line, "zed help") == 0) {
                    printf("\nZED runtime commands:\n");
                    printf("  zed surveyin <seconds> <acc_m>  - Start new Survey-In\n");
                    printf("  zed fixed <lat> <lon> <alt_m>   - Set fixed base position\n");
                    printf("\nExamples:\n");
                    printf("  zed surveyin 300 2.0\n");
                    printf("  zed fixed 1.3520833 103.8198360 15.2\n");
                    printf("\nNote: Settings are saved to ZED-F9P flash and persist after reboot.\n\n");
                } else if (strncmp(line, "zed ", 4) == 0) {
                    printf("Invalid zed command. Type 'zed help'.\n");
                }
                
                line_pos = 0;
            }
        } else if (c == 127 || c == 8) {  // Backspace or DEL
            if (line_pos > 0) {
                line_pos--;
                printf("\b \b");  // Erase character on screen
            }
        } else if (c >= 32 && c < 127) {
            if (line_pos < RUNTIME_CMD_BUF_SIZE - 1) {
                line[line_pos++] = (char)c;
                putchar(c);  // Echo the character
                fflush(stdout);  // Make sure it appears immediately
            }
        }
    }
}

/**
 * Configure ZED-F9P for BASE station mode with Survey-In.
 * Sets 300 second survey duration and 2.0m accuracy target.
 */
static void configure_base_mode(void)
{
    g_base_fixed_mode = false;
    ESP_LOGI("BASE", "Configuring ZED-F9P for Survey-In mode...");
    
    // First disable any existing BASE mode to ensure clean start
    uint8_t tmode3_disable[48] = {
        0xB5, 0x62, 0x06, 0x71, 0x28, 0x00,
        0x00, 0x00,
        0x00, 0x00,  // Mode = 0 (DISABLED)
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00,
        0x00, 0x00,
        0x00, 0x00  // Checksum (filled below)
    };
    
    uint8_t ck_a = 0, ck_b = 0;
    for (int i = 2; i < 46; i++) {
        ck_a += tmode3_disable[i];
        ck_b += ck_a;
    }
    tmode3_disable[46] = ck_a;
    tmode3_disable[47] = ck_b;
    
    gnss_uart_write(tmode3_disable, sizeof(tmode3_disable));
    vTaskDelay(pdMS_TO_TICKS(300));
    gnss_uart_flush();
    
    /* UBX-CFG-TMODE3: Enable Survey-In mode
     * Survey duration: 300 seconds (0x0000012C)
     * Accuracy limit: 20000 (0.1mm units = 2.0m)
     */
    uint8_t tmode3[] = {
        0xB5, 0x62,       /* Sync */
        0x06, 0x71,       /* CFG-TMODE3 */
        0x28, 0x00,       /* Length: 40 bytes */
        0x00,             /* version */
        0x00,             /* reserved1 */
        0x01, 0x00,       /* flags: Survey-In mode (bit 0 = 1) */
        0x00, 0x00, 0x00, 0x00,  /* ecefXOrLat */
        0x00, 0x00, 0x00, 0x00,  /* ecefYOrLon */
        0x00, 0x00, 0x00, 0x00,  /* ecefZOrAlt */
        0x00,             /* ecefXOrLatHP */
        0x00,             /* ecefYOrLonHP */
        0x00,             /* ecefZOrAltHP */
        0x00,             /* reserved2 */
        0x20, 0x4E, 0x00, 0x00,  /* fixedPosAcc: 20000 (2.0m in 0.1mm) */
        0x2C, 0x01, 0x00, 0x00,  /* svinMinDur: 300 seconds */
        0x20, 0x4E, 0x00, 0x00,  /* svinAccLimit: 20000 (2.0m) */
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  /* reserved3 */
        0x00, 0x00        /* Checksum */
    };
    
    /* Calculate checksum */
    ck_a = 0;
    ck_b = 0;
    for (int i = 2; i < 46; i++) {
        ck_a += tmode3[i];
        ck_b += ck_a;
    }
    tmode3[46] = ck_a;
    tmode3[47] = ck_b;
    
    gnss_uart_write(tmode3, sizeof(tmode3));
    vTaskDelay(pdMS_TO_TICKS(200));
    
    /* Enable RTCM3 messages for BASE transmission */
    const uint8_t rtcm_msgs[][16] = {
        /* RTCM3.3 1005 - Station coordinates (10s) */
        {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF5, 0x05, 0x00, 0x0A, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00},
        /* RTCM3.3 1077 - GPS MSM7 (1s) */
        {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF5, 0x4D, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00},
        /* RTCM3.3 1087 - GLONASS MSM7 (1s) */
        {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF5, 0x57, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00},
        /* RTCM3.3 1097 - Galileo MSM7 (1s) */
        {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF5, 0x61, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00},
        /* RTCM3.3 1127 - BeiDou MSM7 (1s) */
        {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF5, 0x7F, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00},
        /* RTCM3.3 1230 - GLONASS code-phase biases (10s) */
        {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF5, 0xE6, 0x00, 0x0A, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00},
    };
    
    for (int i = 0; i < 6; i++) {
        uint8_t cmd[16];
        memcpy(cmd, rtcm_msgs[i], 16);
        
        /* Calculate checksum */
        ck_a = 0; ck_b = 0;
        for (int j = 2; j < 14; j++) {
            ck_a += cmd[j];
            ck_b += ck_a;
        }
        cmd[14] = ck_a;
        cmd[15] = ck_b;
        
        gnss_uart_write(cmd, 16);
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    ESP_LOGI("BASE", "Survey-In configured: 300s duration, 2.0m accuracy");
    save_config_to_flash();
}

/**
 * Poll for UBX-NAV-SVIN status (active request).
 * Clears RTCM-heavy buffer first, sends poll, waits for response.
 * Returns true if a response was received and parsed.
 */
static bool poll_svin_status(void)
{
    /* Drain any pending RTCM data from UART to make room for poll response */
    uint8_t drain_buf[512];
    int drain_len;
    do {
        drain_len = gnss_uart_read(drain_buf, sizeof(drain_buf));
    } while (drain_len > 0);  /* Keep reading until buffer is empty */
    
    /* UBX-NAV-SVIN poll request */
    uint8_t poll[] = {
        0xB5, 0x62,       /* Sync */
        0x01, 0x3B,       /* NAV-SVIN */
        0x00, 0x00,       /* Length: 0 (poll) */
        0x00, 0x00        /* Checksum */
    };
    
    /* Calculate checksum */
    uint8_t ck_a = 0, ck_b = 0;
    for (int i = 2; i < 6; i++) {
        ck_a += poll[i];
        ck_b += ck_a;
    }
    poll[6] = ck_a;
    poll[7] = ck_b;
    
    gnss_uart_write(poll, sizeof(poll));
    /* Give ZED time to respond to poll (300ms should be plenty) */
    vTaskDelay(pdMS_TO_TICKS(300));
    return true;
}

/**
 * Enable UBX-NAV-SVIN messages (sent once at init).
 * UBX-CFG-MSG: Enable NAV-SVIN on all ports to ensure we receive it.
 */
static void enable_svin_messages(void)
{
    /* UBX-CFG-MSG: Set message rate for NAV-SVIN */
    uint8_t cmd[] = {
        0xB5, 0x62,       /* Sync chars */
        0x06, 0x01,       /* CFG-MSG */
        0x08, 0x00,       /* Length: 8 bytes */
        0x01, 0x3B,       /* NAV-SVIN (class 0x01, ID 0x3B) */
        0x00,             /* I2C/DDC rate: 0=disabled */
        0x05,             /* UART1 rate: 5=every 5 navigation epochs */
        0x05,             /* UART2 rate: 5=every 5 navigation epochs */
        0x00,             /* USB rate: 0=disabled */
        0x00,             /* SPI rate: 0=disabled */
        0x00,             /* Reserved */
        0x00, 0x00        /* Checksum (placeholder) */
    };
    
    /* Calculate UBX checksum (Fletcher) */
    uint8_t ck_a = 0, ck_b = 0;
    for (int i = 2; i < 14; i++) {
        ck_a += cmd[i];
        ck_b += ck_a;
    }
    cmd[14] = ck_a;
    cmd[15] = ck_b;
    
    gnss_uart_write(cmd, sizeof(cmd));
    ESP_LOGI("BASE", "Enabled UBX-NAV-SVIN status messages (rate=5)");
    vTaskDelay(pdMS_TO_TICKS(200));
}

static void run_base_station(void)
{
    printf("========================================\n");
    printf("  RTK BASE STATION\n");
    printf("========================================\n\n");

    gnss_uart_init();
    
    /* First, test if ZED-F9P responds at all by requesting version info */
    ESP_LOGI("BASE", "Testing UART connection with ZED-F9P...");
    uint8_t ver_poll[] = {0xB5, 0x62, 0x0A, 0x04, 0x00, 0x00, 0x0E, 0x34};  /* UBX-MON-VER poll */
    gnss_uart_write(ver_poll, sizeof(ver_poll));
    vTaskDelay(pdMS_TO_TICKS(500));
    
    uint8_t test_buf[512];
    int test_len = gnss_uart_read(test_buf, sizeof(test_buf));
    if (test_len <= 0) {
        ESP_LOGE("BASE", "==========================================");
        ESP_LOGE("BASE", "  NO DATA FROM ZED-F9P!");
        ESP_LOGE("BASE", "==========================================");
        ESP_LOGE("BASE", "Check wiring:");
        ESP_LOGE("BASE", "  ESP32 GPIO16 (RX) -> ZED-F9P UART2 TX");
        ESP_LOGE("BASE", "  ESP32 GPIO17 (TX) -> ZED-F9P UART2 RX");
        ESP_LOGE("BASE", "  Common GND");
        ESP_LOGE("BASE", "Check power: ZED-F9P LED should be ON");
        ESP_LOGE("BASE", "==========================================");
    } else {
        ESP_LOGI("BASE", "ZED-F9P UART OK! Received %d bytes", test_len);
    }
    
    // Check if ZED-F9P has been configured before
    bool zed_configured = role_config_is_zed_configured();
    
    if (!zed_configured) {
        // First boot - configure with defaults
        ESP_LOGI("BASE", "First boot detected - configuring ZED-F9P with defaults");
        configure_base_mode();
        role_config_mark_zed_configured();
    } else {
        // Subsequent boot - use saved config from ZED-F9P flash
        ESP_LOGI("BASE", "Using saved ZED-F9P configuration from flash memory");
        ESP_LOGI("BASE", "(Runtime settings from previous boot preserved)");
    }
    
    enable_svin_messages();
    runtime_base_cmd_init();
    printf("\n========================================\n");
    printf("Runtime commands enabled!\n");
    printf("Type 'zed help' and press ENTER\n");
    printf("Your typing will be echoed on screen\n");
    printf("========================================\n\n");

    uint8_t  buf[512];
    static uint8_t svin_buf[2048];      /* UBX survey-in parser buffer */
    static uint8_t base_nmea_buf[2048]; /* BASE GPS/NMEA parser buffer */
    size_t   svin_len = 0;
    size_t   base_nmea_len = 0;
    int      packet_count = 0;
    bool     rtcm_active = false;
    ubx_svin_t svin_status = {0};
    int      last_progress = -1;
    TickType_t last_status_tick = xTaskGetTickCount();
    TickType_t last_poll_tick = xTaskGetTickCount();
    TickType_t last_debug_tick = xTaskGetTickCount();
    bool     received_svin = false;  /* Track if we've seen any SVIN messages */
    bool     received_any_ubx = false; /* Track if ANY UBX messages received */

    while (1) {
        runtime_base_cmd_poll(&rtcm_active, &svin_len, &base_nmea_len,
                             &received_svin, &received_any_ubx);

        int len = gnss_uart_read(buf, sizeof(buf));
        if (len > 0) {
            /* Debug: Check for ANY UBX messages */
            if (!received_any_ubx) {
                for (int i = 0; i < len - 1; i++) {
                    if (buf[i] == 0xB5 && buf[i+1] == 0x62) {
                        received_any_ubx = true;
                        ESP_LOGI("BASE", "UBX messages detected from ZED-F9P");
                        break;
                    }
                }
            }
            
            /* Debug: Periodically log what we're receiving */
            TickType_t now_debug = xTaskGetTickCount();
            if (!rtcm_active && !received_svin &&
                (now_debug - last_debug_tick) >= pdMS_TO_TICKS(5000)) {
                last_debug_tick = now_debug;
                ESP_LOGI("BASE", "Received %d bytes, RTCM:%d UBX:%d SVIN:%d", 
                        len, rtcm_active, received_any_ubx, received_svin);
                /* Show first several bytes to look for UBX sync patterns */
                if (len >= 20) {
                    ESP_LOGI("BASE", "First 20 bytes: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
                            buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9],
                            buf[10], buf[11], buf[12], buf[13], buf[14], buf[15], buf[16], buf[17], buf[18], buf[19]);
                } else if (len >= 6) {
                    ESP_LOGI("BASE", "First bytes: %02X %02X %02X %02X %02X %02X",
                            buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]);
                }
            }
            
            /* Always parse for UBX-NAV-SVIN messages (mixed with RTCM data) */
            /* Accumulate all data for UBX parsing - don't skip based on rtcm_active */
            if (svin_len + len < sizeof(svin_buf)) {
                memcpy(svin_buf + svin_len, buf, len);
                svin_len += len;
                
                /* Try to parse survey-in status */
                if (parse_ubx_svin(svin_buf, svin_len, &svin_status)) {
                    if (!received_svin) {
                        ESP_LOGI("BASE", "========================================");
                        ESP_LOGI("BASE", "UBX-NAV-SVIN Message Received!");
                        ESP_LOGI("BASE", "========================================");
                    }
                    received_svin = true;
                    last_poll_tick = xTaskGetTickCount();
                    
                    /* Check if Survey-In just completed */
                    if (svin_status.valid && !rtcm_active) {
                        rtcm_active = true;
                        printf("\r\n\n========================================\n");
                        printf("*** Survey-In COMPLETE! ***\n");
                        printf("Duration: %lu seconds\n", (unsigned long)svin_status.dur);
                        printf("Accuracy: %.3f meters\n", svin_status.meanAcc / 10000.0f);
                        printf("*** RTCM corrections now active ***\n");
                        printf("========================================\n\n");
                    }
                    else if (svin_status.active && !svin_status.valid) {
                        /* Survey in progress - show detailed status like u-center */
                        float acc_m = svin_status.meanAcc / 10000.0f;
                        
                        /* Always show current status regardless of progress value */
                        printf("\r[Survey-In] Duration: %3lus | Accuracy: %6.2fm | Active: YES | Valid: %s%s",
                               (unsigned long)svin_status.dur,
                               acc_m,
                               svin_status.valid ? "YES" : "NO ",
                               "     ");
                        fflush(stdout);
                        last_progress = svin_status.dur;  // Track by duration instead
                    } else if (!svin_status.active && !svin_status.valid) {
                        /* Survey-In configured but not yet active */
                        if (!received_svin || last_progress < 0) {
                            ESP_LOGI("BASE", "Survey-In configured, waiting to become active...");
                            last_progress = 0;
                        }
                    }
                }
                
                /* Keep buffer from growing too large */
                if (svin_len > 1024) {
                    memmove(svin_buf, svin_buf + 512, svin_len - 512);
                    svin_len -= 512;
                }
            }

            /* Accumulate stream for BASE GPS info parsing */
            if (base_nmea_len + len < sizeof(base_nmea_buf)) {
                memcpy(base_nmea_buf + base_nmea_len, buf, len);
                base_nmea_len += len;
            } else {
                memmove(base_nmea_buf,
                        base_nmea_buf + (base_nmea_len / 2),
                        base_nmea_len - (base_nmea_len / 2));
                base_nmea_len = base_nmea_len - (base_nmea_len / 2);
                if (base_nmea_len + len < sizeof(base_nmea_buf)) {
                    memcpy(base_nmea_buf + base_nmea_len, buf, len);
                    base_nmea_len += len;
                }
            }
            
            mqtt_comm_publish(buf, len);
            packet_count++;
        }

        /* Poll for Survey-In status every 2 seconds if not yet complete */
        TickType_t now = xTaskGetTickCount();
        if (!rtcm_active && (now - last_poll_tick) >= pdMS_TO_TICKS(2000)) {
            last_poll_tick = now;
            /* poll_svin_status() now drains buffer, sends poll, waits for response */
            poll_svin_status();
            /* After draining and polling, read any UBX response data */
            int response_len = gnss_uart_read(buf, sizeof(buf));
            if (response_len > 0) {
                ESP_LOGI("BASE", "Got %d bytes after poll request", response_len);
                /* Check for UBX sync bytes */
                bool has_ubx = false;
                for (int i = 0; i < response_len - 1; i++) {
                    if (buf[i] == 0xB5 && buf[i+1] == 0x62) {
                        has_ubx = true;
                        ESP_LOGI("BASE", "Found UBX sync at offset %d", i);
                        break;
                    }
                }
                if (!has_ubx) {
                    ESP_LOGW("BASE", "Poll response contains no UBX data, first bytes: %02X %02X %02X %02X %02X %02X",
                            response_len > 0 ? buf[0] : 0,
                            response_len > 1 ? buf[1] : 0,
                            response_len > 2 ? buf[2] : 0,
                            response_len > 3 ? buf[3] : 0,
                            response_len > 4 ? buf[4] : 0,
                            response_len > 5 ? buf[5] : 0);
                }
                /* Add to svin_buf for parsing */
                if (svin_len + response_len < sizeof(svin_buf)) {
                    memcpy(svin_buf + svin_len, buf, response_len);
                    svin_len += response_len;
                }
            } else {
                ESP_LOGW("BASE", "No response to poll request (got 0 bytes)");
            }
            if (!received_svin) {
                printf("\r[Survey-In] Waiting for ZED-F9P response... (polling every 2s)%20s", "");
                fflush(stdout);
            }
        }

        /* Periodic status (every 30 seconds) */
        if ((now - last_status_tick) >= pdMS_TO_TICKS(STATUS_INTERVAL_MS)) {
            last_status_tick = now;

            nmea_gga_data_t base_gps = nmea_parse_gga(base_nmea_buf, base_nmea_len);
            if (base_gps.valid) {
                if (!rtcm_active && !g_base_fixed_mode) {
                    printf("BASE GPS: %.7f, %.7f | Alt: %.1fm | Sats: %d | Base not valid until Survey-In complete\n",
                           base_gps.latitude,
                           base_gps.longitude,
                           base_gps.altitude,
                           base_gps.satellites);
                } else {
                    printf("BASE GPS: %.7f, %.7f | Alt: %.1fm | Sats: %d | Fix: %s\n",
                           base_gps.latitude,
                           base_gps.longitude,
                           base_gps.altitude,
                           base_gps.satellites,
                           nmea_fix_quality_str(base_gps.fix_quality));
                }
            } else {
                if (!rtcm_active && !g_base_fixed_mode) {
                    printf("BASE GPS: Waiting for fix | Base not valid until Survey-In complete\n");
                } else {
                    printf("BASE GPS: Waiting for fix...\n");
                }
            }

            if (rtcm_active) {
                printf("RTCM: Broadcasting (%d packets sent)\n", packet_count);
            } else if (!received_svin) {
                /* Fallback if no UBX-NAV-SVIN messages received */
                printf("\rSurvey-In: Polling ZED-F9P for status...%30s", "");
                fflush(stdout);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
    }
}

/* ── ROVER station task ────────────────────────────────────────────── */

#define GNSS_BUF_SIZE 2048

/**
 * Configure ZED-F9P for ROVER mode (moving base).
 */
static void configure_rover_mode(void)
{
    ESP_LOGI("ROVER", "Configuring ZED-F9P for Rover mode...");
    
    /* UBX-CFG-TMODE3: Disable Survey-In (mode = 0) */
    uint8_t tmode3[] = {
        0xB5, 0x62,       /* Sync */
        0x06, 0x71,       /* CFG-TMODE3 */
        0x28, 0x00,       /* Length: 40 bytes */
        0x00,             /* version */
        0x00,             /* reserved1 */
        0x00, 0x00,       /* flags: Disabled (mode = 0) */
        0x00, 0x00, 0x00, 0x00,  /* ecefXOrLat */
        0x00, 0x00, 0x00, 0x00,  /* ecefYOrLon */
        0x00, 0x00, 0x00, 0x00,  /* ecefZOrAlt */
        0x00,             /* ecefXOrLatHP */
        0x00,             /* ecefYOrLonHP */
        0x00,             /* ecefZOrAltHP */
        0x00,             /* reserved2 */
        0x00, 0x00, 0x00, 0x00,  /* fixedPosAcc */
        0x00, 0x00, 0x00, 0x00,  /* svinMinDur */
        0x00, 0x00, 0x00, 0x00,  /* svinAccLimit */
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,  /* reserved3 */
        0x00, 0x00        /* Checksum */
    };
    
    /* Calculate checksum */
    uint8_t ck_a = 0, ck_b = 0;
    for (int i = 2; i < 46; i++) {
        ck_a += tmode3[i];
        ck_b += ck_a;
    }
    tmode3[46] = ck_a;
    tmode3[47] = ck_b;
    
    gnss_uart_write(tmode3, sizeof(tmode3));
    vTaskDelay(pdMS_TO_TICKS(200));
    
    /* Enable NMEA GGA messages (1Hz on UART2) */
    uint8_t gga_cmd[] = {
        0xB5, 0x62,       /* Sync */
        0x06, 0x01,       /* CFG-MSG */
        0x08, 0x00,       /* Length */
        0xF0, 0x00,       /* NMEA GGA */
        0x00,             /* I2C rate */
        0x00,             /* UART1 rate */
        0x01,             /* UART2 rate (1Hz) */
        0x00,             /* USB rate */
        0x00,             /* SPI rate */
        0x00,             /* Reserved */
        0x00, 0x00        /* Checksum */
    };
    
    ck_a = 0; ck_b = 0;
    for (int i = 2; i < 14; i++) {
        ck_a += gga_cmd[i];
        ck_b += ck_a;
    }
    gga_cmd[14] = ck_a;
    gga_cmd[15] = ck_b;
    
    gnss_uart_write(gga_cmd, sizeof(gga_cmd));
    vTaskDelay(pdMS_TO_TICKS(100));
    
    ESP_LOGI("ROVER", "Rover mode configured (moving base)");
    save_config_to_flash();
}

static void run_rover_station(void)
{
    printf("========================================\n");
    printf("  RTK ROVER\n");
    printf("========================================\n\n");

    gnss_uart_init();
    configure_rover_mode();

    static uint8_t gnss_buf[GNSS_BUF_SIZE];
    uint8_t        mqtt_buf[512];
    size_t         gnss_len       = 0;
    int            rtcm_total     = 0;
    int            rtcm_last      = 0;
    TickType_t     last_status_tick = xTaskGetTickCount();

    while (1) {
        /* ── 1. Drain MQTT queue → forward RTCM to ZED-F9P ──────── */
        int len;
        while ((len = mqtt_comm_receive(mqtt_buf, sizeof(mqtt_buf))) > 0) {
            gnss_uart_write(mqtt_buf, len);
            rtcm_total++;
        }

        /* ── 2. Read NMEA sentences from ZED-F9P ────────────────── */
        int read_len = gnss_uart_read(gnss_buf + gnss_len,
                                      GNSS_BUF_SIZE - gnss_len);
        if (read_len > 0) {
            gnss_len += read_len;

            /* Keep buffer from overflowing by discarding old data */
            if (gnss_len > GNSS_BUF_SIZE - 256) {
                size_t discard = 1024;
                if (discard > gnss_len) discard = gnss_len;
                memmove(gnss_buf, gnss_buf + discard, gnss_len - discard);
                gnss_len -= discard;
            }
        }

        /* ── 3. Periodic status display ──────────────────────────── */
        TickType_t now = xTaskGetTickCount();
        if ((now - last_status_tick) >= pdMS_TO_TICKS(STATUS_INTERVAL_MS)) {
            last_status_tick = now;

            /* Parse and display position */
            nmea_gga_data_t gps = nmea_parse_gga(gnss_buf, gnss_len);
            if (gps.valid) {
                printf("Position : %.7f, %.7f\n", gps.latitude, gps.longitude);
                printf("Altitude : %.1f m\n", gps.altitude);
                printf("Sats     : %d\n", gps.satellites);
                printf("Fix      : %s\n", nmea_fix_quality_str(gps.fix_quality));
            } else {
                printf("GPS      : Waiting for fix...\n");
            }

            /* RTCM correction status */
            int received = rtcm_total - rtcm_last;
            if (received > 0) {
                printf("RTCM     : Active (%d new, %d total)\n",
                       received, rtcm_total);
            } else {
                printf("RTCM     : No corrections received\n");
            }
            rtcm_last = rtcm_total;

            printf("----------------------------------------\n\n");
        }

        vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
    }
}

/* ── Entry point ───────────────────────────────────────────────────── */

void app_main(void)
{
    system_init();

    int device_role = ensure_role_configured();

    char wifi_ssid[64];
    char wifi_pass[64];
    ensure_wifi_configured(wifi_ssid, sizeof(wifi_ssid),
                           wifi_pass, sizeof(wifi_pass));

    const char *role_name = (device_role == DEVICE_ROLE_BASE) ? "BASE" : "ROVER";
    const char *client_id = (device_role == DEVICE_ROLE_BASE) ? "base_station"
                                                               : "rover_station";

    printf("\n");
    printf("========================================\n");
    printf("  HAGPS RTK System\n");
    printf("  Role : %s\n", role_name);
    printf("  WiFi : %s\n", wifi_ssid);
    printf("========================================\n\n");

    connect_network(wifi_ssid, wifi_pass, client_id);

    if (device_role == DEVICE_ROLE_BASE) {
        run_base_station();
    } else {
        run_rover_station();
    }
}
