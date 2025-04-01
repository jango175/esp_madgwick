/**
 * @file esp_madgwick.cpp
 * @author jango175
 * @brief ESP-IDF Madgwick MARG filter
 * 
 * @copyright Apache 2.0
*/

#include <stdio.h>
#include "esp_madgwick.h"

i2c_master_bus_handle_t bus_handle;

static portMUX_TYPE spinlock = portMUX_INITIALIZER_UNLOCKED;
static bool filter_restart = false;
static SemaphoreHandle_t mutex = NULL;
static dspm::Mat q_attitude(4, 1);
static dspm::Mat q_gyro(4, 1);

static mpu6050_conf_t mpu;
#ifdef MADGWICK_MAGNETOMETER_HMC
    static hmc5883l_conf_t hmc;
#elif defined(MADGWICK_MAGNETOMETER_QMC)
    static qmc5883l_conf_t qmc;
#endif

static const char* TAG = "esp_madgwick";


/**
 * @brief convert quaternion to euler angles
 * 
 * @param q quaternion
 * @param roll pointer to roll angle in radians
 * @param pitch pointer to pitch angle in radians
 * @param yaw pointer to yaw angle in radians
 */
static void quat_to_euler(dspm::Mat q, float* roll, float* pitch, float* yaw)
{
    *roll = atan2f(2.0f*(q(0, 0)*q(1, 0) + q(2, 0)*q(3, 0)),
                    powf(q(0, 0), 2.0f) - powf(q(1, 0), 2.0f) - powf(q(2, 0), 2.0f) + powf(q(3, 0), 2.0f));

    *pitch = asinf(2.0f*(q(0, 0)*q(2, 0) - q(3, 0)*q(1, 0)));

    *yaw = atan2f(2.0f*(q(1, 0)*q(2, 0) + q(0, 0)*q(3, 0)),
                    powf(q(0, 0), 2.0f) + powf(q(1, 0), 2.0f) - powf(q(2, 0), 2.0f) - powf(q(3, 0), 2.0f));
}


/**
 * @brief get attitude from quaternion
 * 
 * @param roll pointer to roll angle in degrees
 * @param pitch pointer to pitch angle in degrees
 * @param yaw pointer to yaw angle in degrees
 */
extern "C" void esp_madgwick_get_attitude(float* roll, float* pitch, float* yaw)
{
    dspm::Mat q(4, 1);

    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE)
    {
        q = q_attitude;
        xSemaphoreGive(mutex);

        // convert quaternion to euler angles
        quat_to_euler(q, roll, pitch, yaw);

        // convert radians to degrees
        *roll = *roll*180.0f/M_PI;
        *pitch = *pitch*180.0f/M_PI;
        *yaw = *yaw*180.0f/M_PI;
    }
    else
        ESP_LOGE(TAG, "Failed to take mutex");
}


/**
 * @brief get gyroscope data
 * 
 * @param gx pointer to gyroscope x axis data in deg/s
 * @param gy pointer to gyroscope y axis data in deg/s
 * @param gz pointer to gyroscope z axis data in deg/s
 */
extern "C" void esp_madgwick_get_gyro(float* gx, float* gy, float* gz)
{
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE)
    {
        *gx = q_gyro(1, 0)*180.0f/M_PI;
        *gy = q_gyro(2, 0)*180.0f/M_PI;
        *gz = q_gyro(3, 0)*180.0f/M_PI;
        xSemaphoreGive(mutex);
    }
    else
        ESP_LOGE(TAG, "Failed to take mutex");
}


/**
 * @brief quaternion multiplication
 * 
 * @param q1 first quaternion
 * @param q2 second quaternion
 * 
 * @return q1q2 output quaternion q1 x q2
 */
static dspm::Mat quat_mult(dspm::Mat q1, dspm::Mat q2)
{
    dspm::Mat q1q2(4, 1);

    q1q2(0, 0) = q1(0, 0)*q2(0, 0) - q1(1, 0)*q2(1, 0) - q1(2, 0)*q2(2, 0) - q1(3, 0)*q2(3, 0);
    q1q2(1, 0) = q1(0, 0)*q2(1, 0) + q1(1, 0)*q2(0, 0) + q1(2, 0)*q2(3, 0) - q1(3, 0)*q2(2, 0);
    q1q2(2, 0) = q1(0, 0)*q2(2, 0) - q1(1, 0)*q2(3, 0) + q1(2, 0)*q2(0, 0) + q1(3, 0)*q2(1, 0);
    q1q2(3, 0) = q1(0, 0)*q2(3, 0) + q1(1, 0)*q2(2, 0) - q1(2, 0)*q2(1, 0) + q1(3, 0)*q2(0, 0);

    return q1q2;
}


/**
 * @brief quaternion conjugate
 * 
 * @param q quaternion
 * 
 * @return q_conj conjugate of quaternion q
 */
static dspm::Mat quat_conj(dspm::Mat q)
{
    dspm::Mat q_conj(4, 1);

    q_conj(0, 0) = q(0, 0);
    q_conj(1, 0) = -q(1, 0);
    q_conj(2, 0) = -q(2, 0);
    q_conj(3, 0) = -q(3, 0);

    return q_conj;
}


/**
 * @brief Madgwick filter task
 * 
 * @param arg pointer to the task argument
 */
static IRAM_ATTR void esp_madgwick_filter_task(void* arg)
{
    esp_madgwick_sensors_t sensors;
    TickType_t xLastWakeTime = 0;
    float dt = DT_MS / 1000.0f;

    dspm::Mat q_origin(4, 1);
    dspm::Mat q_est(4, 1);
    dspm::Mat q_diff(4, 1);
    dspm::Mat q_change(4, 1);
    dspm::Mat q_conj(4, 1);

    dspm::Mat a(4, 1);
    dspm::Mat m(4, 1);
    dspm::Mat h(4, 1);
    dspm::Mat b(4, 1);

    dspm::Mat f(6, 1);
    dspm::Mat J(6, 4);
    dspm::Mat gradient(6, 1);
    float gradient_norm = 0.0f;

    // initial quaternion
    q_est(0, 0) = 1.0f;
    q_est(1, 0) = 0.0f;
    q_est(2, 0) = 0.0f;
    q_est(3, 0) = 0.0f;

    while (1)
    {
#ifdef TEST_PERFORMANCE
        int64_t loop_start = esp_timer_get_time();
#endif

        xLastWakeTime = xTaskGetTickCount();

        if (filter_restart)
        {
            // restart the filter
            q_est(0, 0) = 1.0f;
            q_est(1, 0) = 0.0f;
            q_est(2, 0) = 0.0f;
            q_est(3, 0) = 0.0f;

            portENTER_CRITICAL(&spinlock);
            filter_restart = false;
            portEXIT_CRITICAL(&spinlock);
        }

        // read sensors data
        esp_madgwick_read_sensors(&sensors);

        // accelerometer quaternion
        a(0, 0) = 0.0f;
        a(1, 0) = sensors.ax;
        a(2, 0) = sensors.ay;
        a(3, 0) = sensors.az;
        a = a/a.norm();

        // gyroscope quaternion
        q_origin(0, 0) = 0.0f;
        q_origin(1, 0) = sensors.gx;
        q_origin(2, 0) = sensors.gy;
        q_origin(3, 0) = sensors.gz;

        // compute the quaternion difference
        q_diff = quat_mult(q_est, q_origin);

        // compute the quaternion change
        q_change = 0.5f*q_diff;

        // magnetic field correction
        m(0, 0) = 0.0f;
        m(1, 0) = sensors.mx;
        m(2, 0) = sensors.my;
        m(3, 0) = sensors.mz;
        m = m/m.norm();

        h = quat_mult(quat_mult(q_est, m), quat_conj(q_est));

        b(0, 0) = 0.0f;
        b(1, 0) = sqrt(powf(h(1, 0), 2.0f) + powf(h(2, 0), 2.0f));
        b(2, 0) = 0.0f;
        b(3, 0) = h(3, 0);

        // objective function
        f(0, 0) = 2.0f*(q_est(1, 0)*q_est(3, 0) - q_est(0, 0)*q_est(2, 0)) - a(1, 0);
        f(1, 0) = 2.0f*(q_est(0, 0)*q_est(1, 0) + q_est(2, 0)*q_est(3, 0)) - a(2, 0);
        f(2, 0) = 2.0f*(0.5f - powf(q_est(1, 0), 2.0f) - powf(q_est(2, 0), 2.0f)) - a(3, 0);
        f(3, 0) = 2.0f*b(1, 0)*(0.5f - powf(q_est(2, 0), 2.0f) - powf(q_est(3, 0), 2.0f)) +
                    2.0f*b(2, 0)*(q_est(0, 0)*q_est(3, 0) + q_est(1, 0)*q_est(2, 0)) +
                    2.0f*b(3, 0)*(q_est(1, 0)*q_est(3, 0) - q_est(0, 0)*q_est(2, 0)) - m(1, 0);
        f(4, 0) = 2.0f*b(1, 0)*(q_est(1, 0)*q_est(2, 0) - q_est(0, 0)*q_est(3, 0)) +
                    2.0f*b(2, 0)*(0.5f - pow(q_est(1, 0), 2.0f) - powf(q_est(3, 0), 2.0f)) +
                    2.0f*b(3, 0)*(q_est(0, 0)*q_est(1, 0) + q_est(2, 0)*q_est(3, 0))- m(2, 0);
        f(5, 0) = 2.0f*b(1, 0)*(q_est(0, 0)*q_est(2, 0) + q_est(1, 0)*q_est(3, 0)) +
                    2.0f*b(2, 0)*(q_est(2, 0)*q_est(3, 0) - q_est(0, 0)*q_est(1, 0)) +
                    2.0f*b(3, 0)*(0.5f - powf(q_est(1, 0), 2.0f) - powf(q_est(2, 0), 2.0f)) - m(3, 0);

        // calculate the Jacobian form
        J(0, 0) = -2.0f*q_est(2, 0);
        J(0, 1) = 2.0f*q_est(3, 0);
        J(0, 2) = -2.0f*q_est(0, 0);
        J(0, 3) = 2.0f*q_est(1, 0);
        J(1, 0) = 2.0f*q_est(1, 0);
        J(1, 1) = 2.0f*q_est(0, 0);
        J(1, 2) = 2.0f*q_est(3, 0);
        J(1, 3) = 2.0f*q_est(2, 0);
        J(2, 0) = 0.0f;
        J(2, 1) = -4.0f*q_est(1, 0);
        J(2, 2) = -4.0f*q_est(2, 0);
        J(2, 3) = 0.0f;
        J(3, 0) = 2.0f*b(2, 0)*q_est(3, 0) - 2.0f*b(3, 0)*q_est(2, 0);
        J(3, 1) = 2.0f*b(2, 0)*q_est(2, 0) + 2.0f*b(3, 0)*q_est(3, 0);
        J(3, 2) = -4.0f*b(1, 0)*q_est(2, 0) + 2.0f*b(2, 0)*q_est(1, 0) - 2.0f*b(3, 0)*q_est(0, 0);
        J(3, 3) = -4.0f*b(1, 0)*q_est(3, 0) + 2.0f*b(2, 0)*q_est(0, 0) + 2.0f*b(3, 0)*q_est(1, 0);
        J(4, 0) = -2.0f*b(1, 0)*q_est(3, 0) + 2.0f*b(3, 0)*q_est(1, 0);
        J(4, 1) = 2.0f*b(1, 0)*q_est(2, 0) - 4.0f*b(2, 0)*q_est(1, 0) + 2.0f*b(3, 0)*q_est(0, 0);
        J(4, 2) = 2.0f*b(1, 0)*q_est(1, 0) + 2.0f*b(3, 0)*q_est(3, 0);
        J(4, 3) = -2.0f*b(1, 0)*q_est(0, 0) - 4.0f*b(2, 0)*q_est(3, 0) + 2.0f*b(3, 0)*q_est(2, 0);
        J(5, 0) = 2.0f*b(1, 0)*q_est(2, 0) - 2.0f*b(2, 0)*q_est(1, 0);
        J(5, 1) = 2.0f*b(1, 0)*q_est(3, 0) - 2.0f*b(2, 0)*q_est(0, 0) - 4.0f*b(3, 0)*q_est(1, 0);
        J(5, 2) = 2.0f*b(1, 0)*q_est(0, 0) + 2.0f*b(2, 0)*q_est(3, 0) - 4.0f*b(3, 0)*q_est(2, 0);
        J(5, 3) = 2.0f*b(1, 0)*q_est(1, 0) + 2.0f*b(2, 0)*q_est(2, 0);

        // compute the normalised gradient
        gradient = J.t()*f;
        gradient_norm = gradient.norm();
        gradient = gradient/gradient_norm;

        // compute new quaternion after gradient descent
        q_est = q_est + (q_change - MADGWICK_BETA*gradient)*dt;
        q_est = q_est/q_est.norm();

        // update quaternion
        if (xSemaphoreTake(mutex, DT_MS/portTICK_PERIOD_MS) == pdTRUE)
        {
            q_attitude = q_est;
            q_gyro = q_origin;
            xSemaphoreGive(mutex);
        }
        else
            ESP_LOGE(TAG, "Failed to take mutex");

#ifdef TEST_PERFORMANCE
        uint64_t loop_end = esp_timer_get_time() - loop_start;
        ESP_LOGI(TAG, "%lld us", loop_end);
#endif

        // delay for a while
        xTaskDelayUntil(&xLastWakeTime, DT_MS/portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}


/**
 * @brief initialize sensors for Madgwick filter
 * 
 * @param conf struct with Madgwick filter parameters
 */
extern "C" void esp_madgwick_init(esp_madgwick_conf_t* conf)
{
    mutex = xSemaphoreCreateMutex();
    if (mutex == NULL)
    {
        ESP_LOGE(TAG, "Failed to create mutex");
        return;
    }

    portENTER_CRITICAL(&spinlock);
    q_attitude(0, 0) = 1.0f;
    q_attitude(1, 0) = 0.0f;
    q_attitude(2, 0) = 0.0f;
    q_attitude(3, 0) = 0.0f;

    q_gyro(0, 0) = 0.0f;
    q_gyro(1, 0) = 0.0f;
    q_gyro(2, 0) = 0.0f;
    q_gyro(3, 0) = 0.0f;
    portEXIT_CRITICAL(&spinlock);

    // init I2C bus
    i2c_master_bus_config_t i2c_mst_config;
    i2c_mst_config.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_mst_config.i2c_port = conf->i2c_port;
    i2c_mst_config.scl_io_num = conf->scl_pin;
    i2c_mst_config.sda_io_num = conf->sda_pin;
    i2c_mst_config.glitch_ignore_cnt = 7;
    i2c_mst_config.flags.enable_internal_pullup = true;
    i2c_mst_config.intr_priority = 3;
    i2c_mst_config.trans_queue_depth = 0;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    // init MPU6050
    mpu.i2c_port = conf->i2c_port;
    mpu.sda_pin = conf->sda_pin;
    mpu.scl_pin = conf->scl_pin;
    mpu.i2c_freq = conf->i2c_freq;
    mpu.i2c_addr = MPU6050_ADDR_0;
    mpu.fs_sel = MPU6050_FS_SEL_500;
    mpu.afs_sel = MPU6050_AFS_SEL_4G;
    mpu6050_init(mpu);
    mpu6050_i2c_passthrough(mpu);

#ifdef MADGWICK_MAGNETOMETER_HMC
    // init HMC5883L
    hmc.i2c_port = conf->i2c_port;
    hmc.sda_pin = conf->sda_pin;
    hmc.scl_pin = conf->scl_pin;
    hmc.i2c_freq = conf->i2c_freq;
    hmc.drdy_pin = GPIO_NUM_NC;
    hmc5883l_init(hmc);
    hmc5883l_write_config(hmc, HMC5883L_OVER_SAMPLE_8, HMC5883L_DATA_OUTPUT_RATE_75_HZ,
                            HMC5883L_MODE_NORMAL, HMC5883L_GAIN_1090);
    hmc5883l_write_mode(hmc, HMC5883L_CONTINUOUS_MODE);
#elif defined(MADGWICK_MAGNETOMETER_QMC)
    // init QMC5883L
    qmc.i2c_port = conf->i2c_port;
    qmc.sda_pin = conf->sda_pin;
    qmc.scl_pin = conf->scl_pin;
    qmc.i2c_freq = conf->i2c_freq;
    qmc.drdy_pin = GPIO_NUM_NC;
    qmc5883l_init(qmc);
    qmc5883l_write_control(qmc, QMC5883L_OVER_SAMPLE_RATIO_512, QMC5883L_FULL_SCALE_2G,
                            QMC5883L_DATA_OUTPUT_RATE_200, QMC5883L_CONTINUOUS_MODE,
                            QMC5883L_POINTER_ROLLOVER_FUNCTION_NORMAL, QMC5883L_INTERRUPT_DISABLE);
#endif

    // wait for sensors to initialize
    vTaskDelay(100/portTICK_PERIOD_MS);

    // optional calibration
    // mpu6050_calibrate_gyro(mpu);
    // hmc5883l_calibrate(hmc);

    xTaskCreate(esp_madgwick_filter_task, "esp_madgwick_filter_task", 4096, NULL, 17, NULL);
}


/**
 * @brief read sensors data
 * 
 * @param sensors struct with sensors data
 */
extern "C" void esp_madgwick_read_sensors(esp_madgwick_sensors_t* sensors)
{
    // read accelerometer and gyroscope data
    mpu6050_read_accelerometer(mpu, &sensors->ax, &sensors->ay, &sensors->az);
    mpu6050_read_gyroscope(mpu, &sensors->gx, &sensors->gy, &sensors->gz);

    // read magnetometer data
#ifdef MADGWICK_MAGNETOMETER_HMC
    hmc5883l_read_magnetometer(hmc, &sensors->mx, &sensors->my, &sensors->mz);
#elif defined(MADGWICK_MAGNETOMETER_QMC)
    qmc5883l_read_magnetometer(qmc, &sensors->mx, &sensors->my, &sensors->mz);
#endif

    // correct accelerometer axes
    float buf = sensors->ax;
    sensors->ax = sensors->ay;
    sensors->ay = -buf;

    // correct gyroscope axes
    buf = sensors->gx;
    sensors->gx = sensors->gy;
    sensors->gy = -buf;

    // correct magnetometer axes
    buf = sensors->mx;
    sensors->mx = sensors->my;
    sensors->my = -buf;

    // debug
    // ESP_LOGI(TAG, "acc:\t%f\t%f\t%f", sensors->ax, sensors->ay, sensors->az);
    // ESP_LOGI(TAG, "gyro:\t%f\t%f\t%f", sensors->gx, sensors->gy, sensors->gz);
    // ESP_LOGI(TAG, "mag:\t%f\t%f\t%f", sensors->mx, sensors->my, sensors->mz);

    // change gyroscope readings to radians
    sensors->gx = sensors->gx*M_PI/180.0f;
    sensors->gy = sensors->gy*M_PI/180.0f;
    sensors->gz = sensors->gz*M_PI/180.0f;
}


/**
 * @brief restart Madgwick filter
 */
extern "C" void esp_madgwick_restart()
{
    portENTER_CRITICAL(&spinlock);
    filter_restart = true;
    portEXIT_CRITICAL(&spinlock);
}
