# Inv_mpu6050
 Porting driver of DMP(MPU6050) on esp32s3

The DMP is a fast, low power, programmable, embedded lightweight processor in the MPU devices. It is  design to offload functionality, like sensor fusion and gesture recognition, from the MCU to save overall  power in the system. 

Dir tree:

│  CMakeLists.txt
├─eMPL
│  │  dmp_utils.c
│  │  inv_mpu.c
│  │  inv_mpu_dmp_motion_driver.c
│  └─include
│          dmpKey.h
│          dmpmap.h
│          dmp_utils.h
│          inv_mpu.h
│          inv_mpu_dmp_motion_driver.h
├─include
│      inv_mpu6050.h
└─src
        inv_mpu6050.c

As we all known, InvenSense has provided the DMP driver. You would need to provide the following APIs to support I2C read/write functionality, system clock access, hardware interrupts callbacks and logging corresponding to the platform on which the MD6.12 is to be ported. 

`dmp_utils.c ` complete this functions like above. 

You can edit `inv_mpu6050.h` to configure I2C parameters like IO, mod, etc.

It also supports the callback function `send_pkg_cb` to handle data from the DMP.
