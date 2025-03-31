/**
 * @file esp_madgwick.h
 * @author jango175
 * @brief ESP-IDF Madgwick MARG filter
 * 
 * @copyright Apache 2.0
*/

#include <stdio.h>
#include <math.h>
#include "dsp_platform.h"
#include "esp_dsp.h"

#ifdef __cplusplus
extern "C" {
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "esp_mpu6050.h"

// choose one magnetometer type
#define MADGWICK_MAGNETOMETER_HMC 1
// #define MADGWICK_MAGNETOMETER_QMC 1

#ifdef MADGWICK_MAGNETOMETER_HMC
    #include "esp_hmc5883l.h"
#elif defined(MADGWICK_MAGNETOMETER_QMC)
    #include "esp_qmc5883l.h"
#endif

#define DT_MS              2 // time step in ms
#define GYRO_MEAN_ERROR    M_PI*5.0f/180.0f // gyroscope mean error in rad/s
#define MADGWICK_BETA      sqrtf(3.0f/4.0f)*GYRO_MEAN_ERROR // filter gain

// #define TEST_PERFORMANCE   1 // uncomment for performance measurement

typedef struct {
    i2c_port_num_t i2c_port;
    gpio_num_t sda_pin;
    gpio_num_t scl_pin;
    uint32_t i2c_freq;
} esp_madgwick_conf_t;

typedef struct {
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float mx;
    float my;
    float mz;
} esp_madgwick_sensors_t;


void esp_madgwick_init(esp_madgwick_conf_t* conf);

void esp_madgwick_read_sensors(esp_madgwick_sensors_t* sensors);

void esp_madgwick_get_attitude(float* roll, float* pitch, float* yaw);

void esp_madgwick_get_gyro(float* gx, float* gy, float* gz);

void esp_madgwick_restart();

#ifdef __cplusplus
}
#endif