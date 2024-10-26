#include "dmp_utils.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_attr.h"
#include "esp_log.h"
#include <stdio.h>
#include <string.h>

static i2c_master_bus_handle_t bus_handle;
static i2c_master_dev_handle_t dev_handle;



int reg_int_cb(struct int_param_s *int_param)
{
    const char* TAG = "REG_INT_CB";
    int ret = 0;
    gpio_config_t init;
    init.intr_type = GPIO_INTR_NEGEDGE;
    init.mode = GPIO_MODE_INPUT;
    init.pin_bit_mask = (1ULL << (int_param->pin));
    init.pull_up_en = GPIO_PULLUP_ENABLE;
    init.pull_down_en = GPIO_PULLDOWN_DISABLE;
    ret = gpio_config(&init);
    if(ret != ESP_OK)
    {
        ESP_LOGE(TAG, "gpio_config failed with code = %d", ret);
    }
    ret = gpio_install_isr_service(ESP_INTR_FLAG_EDGE);
    if(ret != ESP_OK)
    {
        ESP_LOGE(TAG, "gpio_install_isr_service failed with code = %d", ret);
    }   
    ret = gpio_isr_handler_add(int_param->pin, (gpio_isr_t)(int_param->cb), NULL);
    if(ret != ESP_OK)
    {
        ESP_LOGE(TAG, "gpio_isr_handler_add failed with code = %d", ret);
    }   
    ret = gpio_intr_enable(int_param->pin);
    if(ret != ESP_OK)
    {
        ESP_LOGE(TAG, "gpio_intr_enable failed with code = %d", ret);
    }  
    return ret;
}

int setup_i2c_device(i2c_master_bus_config_t i2c_mst_config, i2c_device_config_t dev_cfg, struct int_param_s *int_param){
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
    ESP_ERROR_CHECK(reg_int_cb(int_param));
    return ESP_OK;
}

void uninstall_i2c_bus()
{
    i2c_master_bus_rm_device(dev_handle);
}


/* The following functions must be defined for this platform:
 * i2c_write(unsigned char slave_addr, unsigned char reg_addr,
 *      unsigned char length, unsigned char const *data)
 * i2c_read(unsigned char slave_addr, unsigned char reg_addr,
 *      unsigned char length, unsigned char *data)
 * delay_ms(unsigned long num_ms)
 * get_ms(unsigned long *count)
 */
int i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char len, unsigned char const *data)
{
    int ret = 0;
    int data_len = len + 1;
    uint8_t* data_buf = (uint8_t*)malloc(data_len);
    data_buf[0] = reg_addr;
    for(int i = 0; i < len; i++)
    {
        data_buf[i+1] = data[i];
    }
    ret = i2c_master_transmit(dev_handle, data_buf, data_len, -1);
    return ret;
}

int i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data)
{
    int ret = 0;
    ret = i2c_master_transmit_receive(dev_handle, (uint8_t*)&reg_addr, sizeof(reg_addr), (uint8_t*)data, (size_t)length, -1);
    return ret;
}

void delay_ms(uint32_t num_ms)
{
    TickType_t xDelay = num_ms / portTICK_PERIOD_MS;
    vTaskDelay( xDelay );
}

void get_ms(uint32_t *count)
{
    TickType_t ticks = xTaskGetTickCount();
    *count = ticks * portTICK_PERIOD_MS;
}




