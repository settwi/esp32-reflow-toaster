#pragma once

static void try_start(void);
static void stop(void);
void start(void* _);
void init_spi(void);
void init_serial(void);
void init_gpio(void);
void init(void);
uint16_t read_raw_temp(void);
double read_temp(void);
void send_temp(void* _);
void handle_cmd(const char* data);
void cmd_listen(void* _);
void app_main(void);
