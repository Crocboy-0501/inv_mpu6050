#ifndef _MPU6050_H_
#define _MPU6050_H_
#include "inv_mpu.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include <stdio.h>
/* The following functions must be defined for this platform:
 * i2c_write(unsigned char slave_addr, unsigned char reg_addr,
 *      unsigned char length, unsigned char const *data)
 * i2c_read(unsigned char slave_addr, unsigned char reg_addr,
 *      unsigned char length, unsigned char *data)
 * delay_ms(unsigned long num_ms)
 * get_ms(unsigned long *count)
 * reg_int_cb(void (*cb)(void), unsigned char port, unsigned char pin)
 * labs(long x)
 * fabsf(float x)
 * min(int a, int b)
 */
#define log_i(format, ...)  ESP_LOGI("MPU6050", format, ##__VA_ARGS__)
#define log_e(format, ...)  ESP_LOGE("MPU6050", format, ##__VA_ARGS__)

int i2c_read(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);
int i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char len, unsigned char const *data);
void delay_ms(uint32_t num_ms);
void get_ms(uint32_t *count);

static inline int min(int a, int b){
    return a < b ? a : b;
} 

int setup_i2c_device(i2c_master_bus_config_t i2c_mst_config, i2c_device_config_t dev_cfg, struct int_param_s *int_param);
#endif