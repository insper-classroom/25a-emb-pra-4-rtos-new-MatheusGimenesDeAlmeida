#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#include "ssd1306.h"
#include "gfx.h"

// Definições
const int TRIGGER_PIN=16;
const int ECHO_PIN=17;

const int vel_som=0.0343f;

const int dist_max=400;

// Filas 
static QueueHandle_t xQueueTime     = NULL; 
static QueueHandle_t xQueueDistance = NULL; 

// Interrupção no pino ECHO para capturar bordas de subida e descida.
void pin_callback(uint gpio, uint32_t events) {
    //armazena o tempo das bordas
    static uint64_t start_time_us = 0;
    
    if (gpio == ECHO_PIN) {
        uint64_t now_us = time_us_64();

        if (events & GPIO_IRQ_EDGE_RISE) {
            start_time_us = now_us;
        }
        // Borda de descida: envia o delta para a fila xQueueTime
        else if (events & GPIO_IRQ_EDGE_FALL) {
            uint64_t delta_us = now_us - start_time_us;
            // Envia de forma não-bloqueante (from ISR)
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xQueueSendFromISR(xQueueTime, &delta_us, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

// trigger_task
void trigger_task(void *pvParameters) {
    const TickType_t xDelay = pdMS_TO_TICKS(1000); 
    while (true) {
        gpio_put(TRIGGER_PIN, 1);
        sleep_us(10);
        gpio_put(TRIGGER_PIN, 0);

        vTaskDelay(xDelay);
    }
}

// echo_task 
//lê o delta de tempo do pulso e converte em distância, depois envia para xQueueDistance
void echo_task(void *pvParameters) {
    uint64_t delta_us;
    float distance_cm;

    while (true) {
        if (xQueueReceive(xQueueTime, &delta_us, portMAX_DELAY) == pdTRUE) {
            distance_cm = (delta_us * vel_som) / 2.0f;

            if (distance_cm > dist_max) {
                distance_cm = -1.0f;  
            }
            xQueueSend(xQueueDistance, &distance_cm, 0);
        }
    }
}

//  oled_task 
void oled_task(void *pvParameters) {
    ssd1306_t disp;
    ssd1306_init();
    gfx_init(&disp, 128, 32);

    float distance_cm;
    char buffer[32];
    int contador_falhas = 0;
    const int maximo_falha = 3;

    while (true) {
        if (xQueueReceive(xQueueDistance, &distance_cm, portMAX_DELAY) == pdTRUE) {
            gfx_clear_buffer(&disp);

            if (distance_cm < 0) {
                contador_falhas++;
                if (contador_falhas >= maximo_falha) {
                    gfx_draw_string(&disp, 0, 0, 1, "Sensor Falhou");
                } else {
                    gfx_draw_string(&disp, 0, 0, 1, "Falha na leitura");
                }
            } else {
                contador_falhas = 0;
                snprintf(buffer, sizeof(buffer), "Dist: %.1f cm", distance_cm);
                gfx_draw_string(&disp, 0, 0, 1, buffer);

                int bar_width = (int)((distance_cm / 100.0f) * 128);
                if (bar_width > 128) bar_width = 128;
                if (bar_width < 0)   bar_width = 0;

                gfx_draw_line(&disp, 0, 20, bar_width, 20);
            }
            gfx_show(&disp);
        }
    }
}
 
int main(void) {
    stdio_init_all();
    sleep_ms(2000); 

    gpio_init(TRIGGER_PIN);
    gpio_set_dir(TRIGGER_PIN, GPIO_OUT);
    gpio_put(TRIGGER_PIN, 0);

    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
    gpio_set_pulls(ECHO_PIN, false, true); 

    gpio_set_irq_enabled_with_callback(
        ECHO_PIN,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true,
        pin_callback
    );

    xQueueTime = xQueueCreate(5, sizeof(uint64_t));
    xQueueDistance = xQueueCreate(5, sizeof(float));

    xTaskCreate(trigger_task, "TriggerTask", 256, NULL, 1, NULL);
    xTaskCreate(echo_task,    "EchoTask",    256, NULL, 1, NULL);
    xTaskCreate(oled_task,    "OledTask",   4096, NULL, 1, NULL);

    vTaskStartScheduler();

    while (1) {
        tight_loop_contents();
    }
    return 0;
}