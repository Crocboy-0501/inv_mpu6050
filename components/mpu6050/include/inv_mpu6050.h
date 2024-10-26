#ifndef _INV_MPU6050_H_
#define _INV_MPU6050_H_
#include <stdio.h>
/* IIC controller config */
#define AUTO_SELET_PORT     (-1)
#define I2C_MASTER_SCL_IO   (3)
#define I2C_MASTER_SDA_IO   (17)
#define I2C_INT_IO          GPIO_NUM_10
#define I2C_DEV_ADDR        0x68
#define I2C_SCL_SPEED       100000
/* Data requested by client. */
#define ACCEL_FLAG          1
#define GYRO_FLAG           0
#define QUAT_FLAG           0
/* Configurating DMP output */
#define ACCEL_ON            0x01
#define GYRO_ON             0x02
/* Starting sampling rate. */
#define DEFAULT_MPU_HZ      200
#ifdef __cplusplus
extern "C" {
#endif
typedef void (*send_pkg_cb)(char*, char);
void app_dmp(send_pkg_cb);
#ifdef __cplusplus
}
#endif
#endif