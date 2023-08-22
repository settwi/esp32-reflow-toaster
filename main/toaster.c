#include <stdio.h>
#include <math.h>
#include <inttypes.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/uart.h"

#include "toaster.h"

static volatile bool stop_stuff = true;

static const uint8_t RXD_PIN = 2;
static const uint8_t TXD_PIN = 0;
static const size_t RX_BUF_SIZE = 1024;
static const int MISO_PIN = 34,
                 SCK_PIN = 23,
                 CS_PIN = 22,
                 HOST = SPI2_HOST;
static const uint8_t GPIO_CTRL_PIN = 18;

spi_device_handle_t spi_han = NULL;

typedef struct {
    double set_point_degc;
    double time_sec;
    bool done;
} pid_ctrl_t;

// Chipquik SMD291AX leaded paste reflow profile
// probably good for any leaded solder (?)
static const pid_ctrl_t LEAD_REFLOW[10] = {
    {.set_point_degc = 90.,  .time_sec = 30., false},
    {.set_point_degc = 150., .time_sec = 90., false},
    {.set_point_degc = 183., .time_sec = 30., false},
    {.set_point_degc = 235., .time_sec = 60., false},
    {.set_point_degc = 170., .time_sec = 30., false},
    {0, 0, true}
    // then open the door lmao
};

static void try_start(void) {
    printf("start!\n");
    if (read_temp() > 100) {
        printf("Cannot start when temp > 100C!");
        return;
    }

    xTaskCreate(start, "start", 2048, NULL, configMAX_PRIORITIES, NULL);
}

void start(void* _) {
    static const double KP = 0.0005;
    static const double KI = 0.0001;
    static const double KD = 0.001;
    

    // set now so that we reset any previous
    // "stop" commands
    stop_stuff = false;


    for (size_t i = 0; i < 10; ++i) {
        if (LEAD_REFLOW[i].done) {
            printf("OPEN THE DOOR\n");
            break;
        }

        double goal_setpoint = LEAD_REFLOW[i].set_point_degc;
        double cur_temp;
        while ((cur_temp = read_temp()) < goal_setpoint) {
            double integral = 0;
            double last_err = 0;
            double t = read_temp();
            while (!stop_stuff && t < goal_setpoint) {
                t = read_temp();
                double err = goal_setpoint - t;
                double prop = KP * err;
                integral += KI * err;
                double derivative = KD * (last_err - err);
                double ctrl = prop + integral + derivative;

                if (ctrl > 0) {
                    gpio_set_level(GPIO_CTRL_PIN, 1);
                }
                else {
                    gpio_set_level(GPIO_CTRL_PIN, 0);
                }

                printf("ctrl is %f\n", ctrl);
                printf("abs(err) is %f\n", fabs(err));
                last_err = err;

                vTaskDelay(1000 / portTICK_PERIOD_MS);
            }
            if (stop_stuff) { break; }
        }
    }

    gpio_set_level(GPIO_CTRL_PIN, 0);
    printf("done\n");
    vTaskDelete(NULL);
}

static void stop(void) {
    printf("stop!\n");
    gpio_set_level(GPIO_CTRL_PIN, 0);
    stop_stuff = true;
}

void init_spi(void) {
    spi_bus_config_t buscfg = {
        .miso_io_num=MISO_PIN,
        .mosi_io_num=-1,
        .sclk_io_num=SCK_PIN,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1
    };
    esp_err_t ret = spi_bus_initialize(HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);

    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1000000ULL,
        .mode = 0,
        .spics_io_num = CS_PIN,
        .queue_size = 8
    };
    ret = spi_bus_add_device(HOST, &devcfg, &spi_han);
    ESP_ERROR_CHECK(ret);
    printf("SPI initialized\n");
}

void init_serial(void) {
    const uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t ret = uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
    ESP_ERROR_CHECK(ret);
    ret = uart_param_config(UART_NUM_1, &uart_config);
    ESP_ERROR_CHECK(ret);
    ret = uart_set_pin(UART_NUM_1, TXD_PIN, RXD_PIN, -1, -1);
    ESP_ERROR_CHECK(ret);
    printf("UART initialized\n");
}

void init_gpio(void) {
    static const uint64_t GPIO_OUTPUT_BITMASK = 1ULL << GPIO_CTRL_PIN;

    // Configure the single pin for output only
    gpio_config_t io_conf;
    memset(&io_conf, 0, sizeof(io_conf));
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = GPIO_OUTPUT_BITMASK;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;

    esp_err_t rc = gpio_config(&io_conf);
    ESP_ERROR_CHECK(rc);

    // low to start
    gpio_set_level(GPIO_CTRL_PIN, 0);
}

void init(void) {
    init_spi();
    init_serial();
    init_gpio();
}

uint16_t read_raw_temp(void) {
    spi_transaction_t t;
    esp_err_t ret;
    uint8_t rx[2];

    memset(&t, 0, sizeof(t));
    memset(rx, 0, sizeof(rx));
    // # bits, not bytes
    t.length = 8*sizeof(rx);
    t.rx_buffer = rx;

    ret = spi_device_polling_transmit(spi_han, &t);
    ESP_ERROR_CHECK(ret);

    return (((uint16_t)rx[0]) << 8) | ((uint16_t)rx[1]);
}

double read_temp(void) {
    // degc/ct
    static const double GAIN = 0.25;

    uint16_t rt = read_raw_temp();
    // only these 12 bits matter for the temp
    // and then, multiply by gain to get degC
    double temp = GAIN * ((rt >> 3) & 0xfff);
    printf("temp is: %lf\n", temp);
    return temp;
}

void send_temp(void* _) {
    while (true) {
        // forward to computer
        uint16_t t = read_raw_temp();
        uart_write_bytes(UART_NUM_1, &t, 2); 
        vTaskDelay(200 / portTICK_PERIOD_MS);
    }
}

void handle_cmd(const char* data) {
    if (strcmp(data, "start") == 0) {
        try_start();
    }
    else if (strcmp(data, "stop") == 0) {
        stop();
    }
    else {
        printf("uh oh: %s", data);
    }
}

void cmd_listen(void* _) {
    static const char *LISTEN_TAG = "LISTENER";
    esp_log_level_set(LISTEN_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(RX_BUF_SIZE+1);

    while (true) {
        const int rxBytes = uart_read_bytes(
            UART_NUM_1, data, RX_BUF_SIZE,
            10 / portTICK_PERIOD_MS
        );
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            handle_cmd((char*)data);
        }
    }

    free(data);
}

void app_main(void) {
    init();
    xTaskCreate(send_temp, "send_out_temp", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(cmd_listen, "cmd_listen", 1024*2, NULL, configMAX_PRIORITIES - 1, NULL);
}
