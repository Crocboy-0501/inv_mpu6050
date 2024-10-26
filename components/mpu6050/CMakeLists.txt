idf_build_get_property(target IDF_TARGET)

set(srcs "eMPL/inv_mpu_dmp_motion_driver.c"
         "eMPL/inv_mpu.c"
         "eMPL/dmp_utils.c"
         "src/inv_mpu6050.c")
set(includes "include")
set(priv_includes "eMPL/include")

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS ${includes}
    PRIV_INCLUDE_DIRS ${priv_includes} 
    PRIV_REQUIRES esp_driver_gpio esp_driver_i2c
)
