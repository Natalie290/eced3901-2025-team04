// Minimal R-Pi Pico USB File

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"

#define PIN_LED PICO_DEFAULT_LED_PIN

int main() {
    stdio_init_all();
    stdio_set_translate_crlf(&stdio_usb, false);

    //WARNING: Default library needs flow control set to RTS/CTS - otherwise port will
    // not connect. The LED turns on once port is detected in firmware.
    while (!stdio_usb_connected());

    adc_init();
    adc_gpio_init(26);
    adc_select_input(0);

    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_put(PIN_LED, 1);

    int adc;

    while(1){
        uint16_t adc = adc_read();
        char command = getchar();

        switch(command){
            case '?':
                putchar('A');
                break;
            case 'L':
                gpio_put(PIN_LED, 0);
                break;
            case 'l':
                gpio_put(PIN_LED, 1);
                break;
            case 'S':
                printf("ADC: %d\n", adc);
                break;

            default:
                putchar('N');            
        }
    }

    return 0;
}

