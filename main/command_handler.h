#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

// Initialize command handler (sets up UART for console)
void command_handler_init(void);

// Start command prompt (blocking - waits for WiFi credentials)
void command_handler_prompt(void);

#endif // COMMAND_HANDLER_H
