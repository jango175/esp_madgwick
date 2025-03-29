idf_component_register(SRCS "esp_madgwick.cpp"
                    INCLUDE_DIRS "include"
                    REQUIRES driver esp_mpu6050 esp_hmc5883l esp_qmc5883l espressif__esp-dsp esp_timer)
