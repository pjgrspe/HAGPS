#include "command_handler.h"
#include "wifi_config.h"
#include "role_config.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_system.h"
#include <string.h>
#include <stdio.h>

#define CMD_UART_NUM UART_NUM_0
#define CMD_BUF_SIZE 256

static bool configured = false;

void command_handler_init(void)
{
    // UART0 is already initialized by default for console
    // Just configure for command input
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    
    uart_param_config(CMD_UART_NUM, &uart_config);
    uart_driver_install(CMD_UART_NUM, CMD_BUF_SIZE * 2, 0, 0, NULL, 0);
}

static void parse_wifi_command(const char *line)
{
    char ssid[64] = {0};
    char password[64] = {0};
    
    // Parse: wifi <ssid> <password>
    if (sscanf(line, "wifi %63s %63s", ssid, password) == 2) {
        printf("Setting WiFi: SSID='%s', Password='%s'\n", ssid, password);
        
        if (wifi_config_save(ssid, password)) {
            printf("WiFi credentials saved! Restarting...\n");
            configured = true;
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            esp_restart();
        } else {
            printf("ERROR: Failed to save credentials\n");
        }
    } else {
        printf("ERROR: Invalid format. Use: wifi <ssid> <password>\n");
    }
}

static void parse_role_command(const char *line)
{
    char role_str[16] = {0};
    
    // Parse: role <base|rover>
    if (sscanf(line, "role %15s", role_str) == 1) {
        int role = -1;
        
        if (strcmp(role_str, "base") == 0 || strcmp(role_str, "BASE") == 0) {
            role = DEVICE_ROLE_BASE;
        } else if (strcmp(role_str, "rover") == 0 || strcmp(role_str, "ROVER") == 0) {
            role = DEVICE_ROLE_ROVER;
        } else {
            printf("ERROR: Invalid role. Use 'base' or 'rover'\n");
            return;
        }
        
        const char *role_name = (role == DEVICE_ROLE_BASE) ? "BASE STATION" : "ROVER";
        printf("Setting device role: %s\n", role_name);
        
        if (role_config_save(role)) {
            printf("Role saved! Restarting...\n");
            configured = true;
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            esp_restart();
        } else {
            printf("ERROR: Failed to save role\n");
        }
    } else {
        printf("ERROR: Invalid format. Use: role <base|rover>\n");
    }
}

void command_handler_prompt(void)
{
    printf("\n");
    printf("=========================================\n");
    printf("  Configuration Required\n");
    printf("=========================================\n");
    printf("Commands:\n");
    printf("  role <base|rover>       - Set device role\n");
    printf("  wifi <ssid> <password>  - Set WiFi credentials\n");
    printf("  clear                   - Clear all settings\n");
    printf("  help                    - Show this help\n");
    printf("=========================================\n");
    printf("\n");
    
    char line[CMD_BUF_SIZE];
    int line_pos = 0;
    
    while (!configured) {
        printf("> ");
        fflush(stdout);
        
        // Read line from UART
        line_pos = 0;
        while (line_pos < CMD_BUF_SIZE - 1) {
            uint8_t c;
            int len = uart_read_bytes(CMD_UART_NUM, &c, 1, portMAX_DELAY);
            
            if (len > 0) {
                if (c == '\n' || c == '\r') {
                    printf("\n");
                    line[line_pos] = '\0';
                    break;
                } else if (c == 127 || c == 8) { // Backspace
                    if (line_pos > 0) {
                        line_pos--;
                        printf("\b \b"); // Erase character on screen
                        fflush(stdout);
                    }
                } else if (c >= 32 && c < 127) { // Printable characters
                    line[line_pos++] = c;
                    printf("%c", c); // Echo character
                    fflush(stdout);
                }
            }
        }
        
        // Process command
        if (strlen(line) == 0) {
            continue;
        }
        
        if (strncmp(line, "role ", 5) == 0) {
            parse_role_command(line);
        } else if (strncmp(line, "wifi ", 5) == 0) {
            parse_wifi_command(line);
        } else if (strcmp(line, "clear") == 0) {
            wifi_config_clear();
            role_config_clear();
            printf("All settings cleared\n");
        } else if (strcmp(line, "help") == 0) {
            printf("\nCommands:\n");
            printf("  role <base|rover>       - Set device role\n");
            printf("  wifi <ssid> <password>  - Set WiFi credentials\n");
            printf("  clear                   - Clear all settings\n");
            printf("  help                    - Show this help\n");
            printf("\n");
        } else {
            printf("Unknown command: '%s'. Type 'help' for available commands.\n", line);
        }
    }
}
