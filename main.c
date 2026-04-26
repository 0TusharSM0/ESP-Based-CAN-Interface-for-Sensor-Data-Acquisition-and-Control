#include <stdio.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/adc.h"
#include "driver/twai.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

#include "esp_err.h"

/* ================= ADC CONFIG ================= */
#define ADC_CHANNEL     ADC1_CHANNEL_4   // GPIO32
#define ADC_ATTEN       ADC_ATTEN_DB_12
#define ADC_WIDTH_CFG   ADC_WIDTH_BIT_12

/* ================= PWM CONFIG ================= */
#define LED_GPIO        25
#define PWM_TIMER       LEDC_TIMER_0
#define PWM_CHANNEL     LEDC_CHANNEL_0
#define PWM_FREQ_HZ     1000
#define PWM_RESOLUTION  LEDC_TIMER_12_BIT   // 0–4095

/* ================= CAN CONFIG ================= */
#define CAN_TX_GPIO     GPIO_NUM_17
#define CAN_RX_GPIO     GPIO_NUM_16

#define CAN_ID_ADC_TX   0x123
#define CAN_ID_LED_RX   0x100

#define CAN_TX_PERIOD_MS 100

/* ================= GLOBAL STATE ================= */
static volatile bool led_enable = true;

/* ================= CAN INIT ================= */
static void can_init(void)
{
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(
            CAN_TX_GPIO,
            CAN_RX_GPIO,
            TWAI_MODE_NORMAL
        );

    twai_timing_config_t t_config =
        TWAI_TIMING_CONFIG_500KBITS();

    twai_filter_config_t f_config =
        TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
}

/* ================= PWM INIT ================= */
static void pwm_init(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = PWM_TIMER,
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz          = PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_cfg);

    ledc_channel_config_t ch_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = PWM_CHANNEL,
        .timer_sel  = PWM_TIMER,
        .gpio_num   = LED_GPIO,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&ch_cfg);
}

/* ================= CAN RX TASK ================= */
void can_rx_task(void *arg)
{
    printf("CAN RX TASK STARTED\n");
    twai_message_t rx_msg;

    while (1)
    {
        if (twai_receive(&rx_msg, portMAX_DELAY) == ESP_OK)
        {
            if (rx_msg.identifier == CAN_ID_LED_RX &&
                rx_msg.data_length_code == 1)
            {
                if (rx_msg.data[0] == 0x01)
                {
                    led_enable = true;
                    printf("LED ENABLED\n");
                }
                else if (rx_msg.data[0] == 0x00)
                {
                    led_enable = false;
                    printf("LED DISABLED\n");
                }
            }
        }
    }
}

/* ================= ADC + CAN TX TASK ================= */
void can_adc_tx_task(void *arg)
{
    TickType_t last_wake = xTaskGetTickCount();
    uint16_t adc_raw;

    adc1_config_width(ADC_WIDTH_CFG);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);

    twai_message_t tx_msg = {
        .identifier = CAN_ID_ADC_TX,
        .extd = 0,
        .rtr  = 0,
        .data_length_code = 2
    };

    while (1)
    {
        adc_raw = adc1_get_raw(ADC_CHANNEL);

        /* Update PWM only if enabled */
        if (led_enable)
        {
            ledc_set_duty(
                LEDC_LOW_SPEED_MODE,
                PWM_CHANNEL,
                adc_raw
            );
        }
        else
        {
            ledc_set_duty(
                LEDC_LOW_SPEED_MODE,
                PWM_CHANNEL,
                0
            );
        }
        ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL);

        /* Pack ADC (INTEL / little-endian) */
        tx_msg.data[0] = adc_raw & 0xFF;
        tx_msg.data[1] = (adc_raw >> 8) & 0xFF;

        twai_transmit(&tx_msg, pdMS_TO_TICKS(10));

        printf("ADC=%u  LED=%s\n",
               adc_raw,
               led_enable ? "ON" : "OFF");

        vTaskDelayUntil(&last_wake,
                         pdMS_TO_TICKS(CAN_TX_PERIOD_MS));
    }
}

/* ================= APP MAIN ================= */
void app_main(void)
{
    printf("APP STARTED\n");

    pwm_init();
    can_init();

    xTaskCreate(
        can_adc_tx_task,
        "can_adc_tx_task",
        4096,
        NULL,
        5,
        NULL
    );

    xTaskCreate(
        can_rx_task,
        "can_rx_task",
        4096,
        NULL,
        6,   // higher priority (event-driven)
        NULL
    );
}