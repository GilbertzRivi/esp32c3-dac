#include <stdio.h>
#include "driver/i2c_master.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "esp_lcd.c"
#include "esp_timer.h"
#include <math.h>

#define SDA 8
#define SCL 9
#define BTN1 0
#define BTN2 7
#define BTN3 20
#define BTN4 10
#define DAC_RESOLUTION 65535
#define PI 3.14159265358979323846

const char menu[4][10] = {
    "Sine    ",
    "Square  ",
    "Triangle",
    "Saw     "
};

int waveSel = 0;
bool notPressed = true;

int delay = 50;
bool generate = false;
uint32_t cycle = 0;

i2c_master_bus_handle_t bus_handle;
i2c_master_bus_config_t i2c_mst_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = -1,
    .scl_io_num = SCL,
    .sda_io_num = SDA,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};

i2c_master_dev_handle_t dev_handle;
i2c_device_config_t dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = 0x4c,
    .scl_speed_hz = 1000000,
};

int dataPins[4] = {4, 3, 2, 1};
lcd_t lcd;

void beginDAC(){
    
    uint8_t command = 0x40;

    uint16_t data = 0x0000;
    data |= ((uint16_t)0b00 << 13);
    data |= ((uint16_t)0b0 << 12);
    data |= ((uint16_t)0b1 << 11);

    uint8_t highByte = (data >> 8) & 0xFF;
    uint8_t lowByte = data & 0xFF;

    uint8_t buffer[3] = {command, highByte, lowByte};

    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, buffer, 3, -1));
}

void writeDAC(uint16_t value){
    
    uint8_t command = 0x30;

    uint8_t highByte = (value >> 8) & 0xFF;
    uint8_t lowByte = value & 0xFF;

    uint8_t buffer[3] = {command, highByte, lowByte};

    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, buffer, 3, -1));
}


void setup(){
    gpio_set_direction(BTN1, GPIO_MODE_INPUT);
    gpio_set_direction(BTN2, GPIO_MODE_INPUT);
    gpio_set_direction(BTN3, GPIO_MODE_INPUT);
    gpio_set_direction(BTN4, GPIO_MODE_INPUT);
    gpio_pulldown_dis(BTN1);
    gpio_pulldown_dis(BTN2);
    gpio_pulldown_dis(BTN3);
    gpio_pulldown_dis(BTN4);
    gpio_pullup_en(BTN1);
    gpio_pullup_en(BTN2);
    gpio_pullup_en(BTN3);
    gpio_pullup_en(BTN4);

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
    
    lcdCtor(&lcd, dataPins, 6, 5);
    lcdInit(&lcd);
    lcdClear(&lcd);
    lcdSetText(&lcd, menu[waveSel], 0, 0);
    char buff[5];
    sprintf(buff, "%-4d", delay);
    lcdSetText(&lcd, buff, 0, 1);
    lcdSetText(&lcd, "*20 us delay", 4, 1);
}

void btn1(){
    waveSel ++;
    if (waveSel >= 4){
        waveSel = 0;
    }
    lcdSetText(&lcd, menu[waveSel], 0, 0);
}

void btn2(){
    delay --;
    if(delay <= 1){
        delay = 1;
    }
    char buff[5];
    sprintf(buff, "%-4d", delay);
    lcdSetText(&lcd, buff, 0, 1);
    lcdSetText(&lcd, "*20 us delay", 4, 1);
}

void btn3(){
    delay ++;
    if(delay > 100){
        delay = 100;
    }
    char buff[5];
    sprintf(buff, "%-4d", delay);
    lcdSetText(&lcd, buff, 0, 1);
    lcdSetText(&lcd, "*20 us delay", 4, 1);
}

void btn4(){
    generate = !generate;
}

void writeToDac(){
    uint16_t dac_value = 0;

    if(1){
        cycle ++;
        float phase = (float)((float)(cycle % 0xff) / 0xff);
        switch (waveSel) {
            case 0:
                dac_value = (DAC_RESOLUTION / 2) * (sin(2 * PI * phase) + 1);
                break;

            case 1:
                dac_value = (phase < 0.5) ? 0 : DAC_RESOLUTION;
                break;

            case 2:
                dac_value = (phase < 0.5) ? (DAC_RESOLUTION * 2 * phase) : (DAC_RESOLUTION * (2 * (1 - phase)));
                break;

            case 3:
                dac_value = DAC_RESOLUTION * phase;
                break;
        }   
        writeDAC(dac_value);
        esp_rom_delay_us(delay*20);
    }
}

void app_main(void){
    setup();
    beginDAC();
    
    while(1){
        if (generate){
            writeToDac();
            if(!gpio_get_level(BTN4)){
                btn4();
            }
        }
        else{
            if(!gpio_get_level(BTN1) && notPressed){
                notPressed = false;
                btn1();
            }
            else if(!gpio_get_level(BTN2) && notPressed){
                notPressed = false;
                btn2();
            }
            else if(!gpio_get_level(BTN3) && notPressed){
                notPressed = false;
                btn3();
            }
            else if(!gpio_get_level(BTN4) && notPressed){
                notPressed = false;
                btn4();
            }
            if(gpio_get_level(BTN1) && gpio_get_level(BTN2) && gpio_get_level(BTN3) && gpio_get_level(BTN4)){
                notPressed = true;
            }
        }
    }
}