idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS ""
    REQUIRES 
        servo_controller 
        mpu6050 
        gait 
        kinematics 
        hexapod 
        leg 
        communication 
        config
        driver
        esp_wifi
        nvs_flash
        esp_netif
)

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error=format)
