#include "Common-define.h"

#define MAX_PWM (819)
#define MIN_PWM (410)
#define ACCEL_SENSITIVITY_8G 4096.0f // LSB/g for ±8g range
#define GRAVITY 9.81f                // Gravitational acceleration (m/s^2)

const float GYRO_TO_DEG_PER_SEC = 0.01526717557;
const float YAW_COMPENSATION_FACTOR = 0.000001066;
const float RAD_TO_DEG = 180.0 / M_PI;

static const char *PROCESS_TAG = "process_task";
// const float alpha = 0.9996;

float gyroXOffset = 0.0;
float gyroYOffset = 0.0;
float gyroZOffset = 0.0;

i2c_master_dev_handle_t dev_handle;

PRYT_data_t parsedata(const char *data)
{
    // Expected format: yaw:30.5,pitch:15.0,roll:45.0,throttle:70
    const char *yawKey = "yaw:";
    const char *pitchKey = "pitch:";
    const char *rollKey = "roll:";
    const char *throttleKey = "throttle:";

    // Find positions of keys
    const char *yawPos = strstr(data, yawKey);
    const char *pitchPos = strstr(data, pitchKey);
    const char *rollPos = strstr(data, rollKey);
    const char *throttlePos = strstr(data, throttleKey);

    PRYT_data_t return_data;

    if (yawPos && pitchPos && rollPos && throttlePos)
    {
        // Extract yaw value
        yawPos += strlen(yawKey);
        return_data.yaw = strtof(yawPos, NULL);

        // Extract pitch value
        pitchPos += strlen(pitchKey);
        return_data.pitch = strtof(pitchPos, NULL);

        // Extract roll value
        rollPos += strlen(rollKey);
        return_data.roll = strtof(rollPos, NULL);

        // Extract throttle value and calculate mapped throttle
        throttlePos += strlen(throttleKey);
        char throttleBuffer[4] = {0};            // Max 3 characters for throttle + null terminator``
        strncpy(throttleBuffer, throttlePos, 3); // Copy only first 5 characters
        throttleBuffer[3] = '\0';                // Ensure null-terminated
        return_data.throttle = 980 + (strtof(throttleBuffer, NULL) * 10.0);
    }
    else
    {
        ESP_LOGI(PROCESS_TAG, "Invalid data format");
    }
    return return_data;
}

PRYT_data_t calculatePID(float pitch_error, float roll_error, float yaw_error)
{

    // PID constants (tune these values for your setup)
    float Kp = 3.44, Ki = 0.048, Kd = 1.92;

    static float pitch_previous_error = 0;
    static float roll_previous_error = 0;
    static float pitch_integral = 0, roll_integral = 0, yaw_integral = 0;

    static uint32_t previousTime = 0;

    TickType_t tick_count = xTaskGetTickCount();
    uint32_t currentTime = pdTICKS_TO_MS(tick_count);
    float deltaTime = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;

    PRYT_data_t PID_data;

    // Pitch PID
    if (-3 < pitch_error && pitch_error < 3)
    {
        pitch_integral += pitch_error * deltaTime;
    }
    float pitch_derivative = (pitch_error - pitch_previous_error) / deltaTime;
    PID_data.pitch = Kp * pitch_error + Ki * pitch_integral + Kd * pitch_derivative;
    pitch_previous_error = pitch_error;

    // Roll PID
    if (-3 < roll_error && roll_error < 3)
    {
        roll_integral += roll_error * deltaTime;
    }
    float roll_derivative = (roll_error - roll_previous_error) / deltaTime;
    PID_data.roll = Kp * roll_error + Ki * roll_integral + Kd * roll_derivative;
    roll_previous_error = roll_error;

    // Yaw PID
    if (-3 < yaw_error && yaw_error < 3)
    {
        yaw_integral += yaw_error * deltaTime;
    }
    PID_data.yaw = Kp * yaw_error + Ki * yaw_integral;

    return PID_data;
}

// MPU6050 configuration and calibration functions
void configureMPU6050()
{
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = -1,
        .scl_io_num = GPIO_NUM_22,
        .sda_io_num = GPIO_NUM_21,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));

    i2c_device_config_t MPU6050 = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = 0b1101000,
        .scl_speed_hz = 100000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &MPU6050, &dev_handle));

    // Power Management 1: Wake up the MPU6050 and use the internal oscillator
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, (uint8_t[]){0x6B, 0x00}, 2, -1));

    // Accelerometer Configuration: Set ±8g range
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, (uint8_t[]){0x1C, 0x10}, 2, -1));

    // Gyroscope Configuration: Set ±500 °/s range
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, (uint8_t[]){0x1B, 0x08}, 2, -1));

    // Enable DLPF with 42 Hz bandwidth
    uint8_t dlpf_config = 0x03; // DLPF enabled, 42 Hz bandwidth
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, (uint8_t[]){0x1A, dlpf_config}, 2, -1));

    // Set Sample Rate Divider for 100 Hz sampling rate
    uint8_t sample_rate_divider = 9; // 1 kHz / (1 + 9) = 100 Hz
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, (uint8_t[]){0x19, sample_rate_divider}, 2, -1));

    esp_rom_delay_us(100000);
}

void calibrateGyroscope()
{
    const int numSamples = 2000;
    float gyroXSum = 0.0, gyroYSum = 0.0, gyroZSum = 0.0;
    uint8_t data[6];
    uint8_t addr = 0x43;
    imu_data_t g;

    for (int i = 0; i < numSamples; i++)
    {
        ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, &addr, 1, -1));
        ESP_ERROR_CHECK(i2c_master_receive(dev_handle, (uint8_t *)&data, 6, -1));
        g.gyro.x = data[0] << 8 | data[1];
        g.gyro.y = data[2] << 8 | data[3];
        g.gyro.z = data[4] << 8 | data[5];
        gyroXSum += g.gyro.x;
        gyroYSum += g.gyro.y;
        gyroZSum += g.gyro.z;
        esp_rom_delay_us(1000);
    }

    gyroXOffset = gyroXSum / numSamples;
    gyroYOffset = gyroYSum / numSamples;
    gyroZOffset = gyroZSum / numSamples;

    ESP_LOGI(PROCESS_TAG, "Gyroscope calibration complete!");
    ESP_LOGI(PROCESS_TAG, "Offsets -> X: %.2f, Y: %.2f, Z: %.2f", gyroXOffset, gyroYOffset, gyroZOffset);
}

void GetIMUData(imu_data_t *a, imu_data_t *g, uint8_t addr)
{
    uint8_t data[14];
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, &addr, 1, -1));
    ESP_ERROR_CHECK(i2c_master_receive(dev_handle, (uint8_t *)&data, 14, -1));

    a->acceleration.x = (data[0] << 8) | data[1];
    a->acceleration.y = (data[2] << 8) | data[3];
    a->acceleration.z = (data[4] << 8) | data[5];
    g->gyro.x = data[8] << 8 | data[9];
    g->gyro.y = data[10] << 8 | data[11];
    g->gyro.z = data[12] << 8 | data[13];
}

void predictQuaternion(float *q, float P[4][4], float Q[4][4], float gyroX, float gyroY, float gyroZ, float dt)
{
    // Subtract bias from gyroscope readings
    float wx = (gyroX - gyroXOffset) / 65.5f * (M_PI / 180.0f); // Convert to rad/s
    float wy = (gyroY - gyroYOffset) / 65.5f * (M_PI / 180.0f);
    float wz = (gyroZ - gyroZOffset) / 65.5f * (M_PI / 180.0f);

    // Quaternion derivative equation
    float dq[4];
    dq[0] = -0.5f * (q[1] * wx + q[2] * wy + q[3] * wz);
    dq[1] = 0.5f * (q[0] * wx + q[2] * wz - q[3] * wy);
    dq[2] = 0.5f * (q[0] * wy + q[3] * wx - q[1] * wz);
    dq[3] = 0.5f * (q[0] * wz + q[1] * wy - q[2] * wx);

    // Update quaternion
    q[0] += dq[0] * dt;
    q[1] += dq[1] * dt;
    q[2] += dq[2] * dt;
    q[3] += dq[3] * dt;

    // Normalize quaternion
    float norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] /= norm;
    q[1] /= norm;
    q[2] /= norm;
    q[3] /= norm;

    // Predict covariance matrix
    // Approximate F as identity matrix (for simplicity)
    // P = F * P * F^T + Q => P = P + Q
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            P[i][j] += Q[i][j];
        }
    }
}

void invertMatrix3x3(float A[3][3], float invA[3][3])
{
    float det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
                A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
                A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);

    if (det == 0.0f)
    {
        // Handle singular matrix (e.g., set identity matrix or raise an error)
        return;
    }

    float invDet = 1.0f / det;

    invA[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]) * invDet;
    invA[0][1] = (A[0][2] * A[2][1] - A[0][1] * A[2][2]) * invDet;
    invA[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]) * invDet;
    invA[1][0] = (A[1][2] * A[2][0] - A[1][0] * A[2][2]) * invDet;
    invA[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]) * invDet;
    invA[1][2] = (A[0][2] * A[1][0] - A[0][0] * A[1][2]) * invDet;
    invA[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]) * invDet;
    invA[2][1] = (A[0][1] * A[2][0] - A[0][0] * A[2][1]) * invDet;
    invA[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]) * invDet;
}

void updateCovariance(float P[4][4], float K[4][3], float H[3][4])
{
    // Compute (I - K * H)
    float I_minus_KH[4][4] = {0};
    for (int i = 0; i < 4; i++)
    {
        I_minus_KH[i][i] = 1.0f; // Identity matrix
        for (int j = 0; j < 4; j++)
        {
            for (int k = 0; k < 3; k++)
            {
                I_minus_KH[i][j] -= K[i][k] * H[k][j];
            }
        }
    }

    // Update covariance matrix: P = (I - K * H) * P
    float temp[4][4] = {0};
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            for (int k = 0; k < 4; k++)
            {
                temp[i][j] += I_minus_KH[i][k] * P[k][j];
            }
        }
    }

    // Copy result back to P
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            P[i][j] = temp[i][j];
        }
    }
}

void computeKalmanGain(float P[4][4], float H[3][4], float R[3][3], float K[4][3])
{
    // Compute H * P
    float HP[3][4] = {0};
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            for (int k = 0; k < 4; k++)
            {
                HP[i][j] += H[i][k] * P[k][j];
            }
        }
    }

    // Compute H * P * H^T
    float HPHt[3][3] = {0};
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            for (int k = 0; k < 4; k++)
            {
                HPHt[i][j] += HP[i][k] * H[j][k];
            }
        }
    }

    // Add R to HPHt
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            HPHt[i][j] += R[i][j];
        }
    }

    // Invert HPHt
    float HPHt_inv[3][3];
    invertMatrix3x3(HPHt, HPHt_inv);

    // Compute K = P * H^T * (HPHt + R)^-1
    float PHt[4][3] = {0};
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            for (int k = 0; k < 4; k++)
            {
                PHt[i][j] += P[i][k] * H[j][k];
            }
        }
    }

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            K[i][j] = 0.0f;
            for (int k = 0; k < 3; k++)
            {
                K[i][j] += PHt[i][k] * HPHt_inv[k][j];
            }
        }
    }
}

void updateKalmanFilter(float *q, float P[4][4], float H[3][4], float R[3][3], float K[4][3], float ax, float ay, float az)
{
    // Transform global gravity vector to local frame using quaternion
    float gLocal[3];
    gLocal[0] = 2.0f * (q[1] * q[3] - q[0] * q[2]);
    gLocal[1] = 2.0f * (q[0] * q[1] + q[2] * q[3]);
    gLocal[2] = q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];

    // Normalize the local gravity vector
    float norm_gLocal = sqrt(gLocal[0] * gLocal[0] + gLocal[1] * gLocal[1] + gLocal[2] * gLocal[2]);
    gLocal[0] /= norm_gLocal;
    gLocal[1] /= norm_gLocal;
    gLocal[2] /= norm_gLocal;


    // Normalize the accelerometer readings
    float norm_accel = sqrt(ax * ax + ay * ay + az * az);
    float ax_normalized = ax / norm_accel;
    float ay_normalized = ay / norm_accel;
    float az_normalized = az / norm_accel;

    // Compute error between normalized accelerometer readings and local gravity
    float error[3];
    error[0] = ax_normalized - gLocal[0];
    error[1] = ay_normalized - gLocal[1];
    error[2] = az_normalized - gLocal[2];


    computeKalmanGain(P, H, R, K);

    // Update quaternion using Kalman gain
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            q[i] += K[i][j] * error[j];
        }
    }

    // Normalize quaternion
    float norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (fabs(norm - 1.0f) > 1e-6)
    { // Normalize only if deviation exceeds a small threshold
        for (int i = 0; i < 4; i++)
        {
            q[i] /= norm;
        }
    }

    ESP_LOGI(PROCESS_TAG, "Updated Quaternion af: (%.2f, %.2f, %.2f, %.2f)", q[0], q[1], q[2], q[3]);

    updateCovariance(P, K, H);
}

void quaternionToEuler(float *q, float *pitch, float *roll, float *yaw)
{
    *pitch = asin(2.0f * (q[0] * q[2] - q[1] * q[3])) * RAD_TO_DEG;
    *roll = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2])) * RAD_TO_DEG;
    *yaw = atan2(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3])) * RAD_TO_DEG;
}

// Data processing task
void DataProcessingTask(void *parameter)
{
    imu_data_t a, g;
    uint8_t addr = 0x3B; // start address of the required register
    uint32_t previousTime = 0;
    uint32_t currentTime = 0;
    float pitch = 0, roll = 0, yaw = 0;
    ESC_out_t esc_data;
    char receive_buffer[128] = {0};
    // bool set_gyro_angles = false;

    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // [qw, qx, qy, qz]
    // Kalman filter variables
    float P[4][4] = {
        {0.01, 0.0, 0.0, 0.0},
        {0.0, 0.01, 0.0, 0.0},
        {0.0, 0.0, 0.01, 0.0},
        {0.0, 0.0, 0.0, 0.01}}; // Covariance matrix

        float Q[4][4] = {
            {1e-5, 0,    0,    0},
            {0,    1e-5, 0,    0},
            {0,    0,    1e-5, 0},
            {0,    0,    0,    1e-5}
        }; // Process noise covariance

        float R[3][3] = {
            {1e-2, 0,    0},
            {0,    1e-2, 0},
            {0,    0,    1e-2}
        }; // Measurement noise covariance

    float K[4][3] = {0}; // Kalman gain
    float H[3][4] = {
        {0, 0, 0, -2}, // Partial derivatives for g_x
        {0, 0, 2, 0},  // Partial derivatives for g_y
        {0, -2, 0, 0}  // Partial derivatives for g_z
    };

    while (1)
    {
        GetIMUData(&a, &g, addr);
        ESP_LOGI(PROCESS_TAG, "Accel: (%d, %d, %d), Gyro: (%d, %d, %d)", a.acceleration.x, a.acceleration.y, a.acceleration.z, g.gyro.x, g.gyro.y, g.gyro.z);
        TickType_t tick_count = xTaskGetTickCount();
        currentTime = pdTICKS_TO_MS(tick_count);
        float deltaTime = (currentTime - previousTime) / 1000.0;
        previousTime = currentTime;

        predictQuaternion(q, P, Q, g.gyro.x, g.gyro.y, g.gyro.z, deltaTime);

        // Convert raw accelerometer readings to m/s^2
        float accel_x_ms2 = (float)a.acceleration.x / ACCEL_SENSITIVITY_8G * GRAVITY;
        float accel_y_ms2 = (float)a.acceleration.y / ACCEL_SENSITIVITY_8G * GRAVITY;
        float accel_z_ms2 = (float)a.acceleration.z / ACCEL_SENSITIVITY_8G * GRAVITY;

        updateKalmanFilter(q, P, H, R, K, accel_x_ms2, accel_y_ms2, accel_z_ms2);

        ESP_LOGI(PROCESS_TAG, "Quaternion: (%.2f, %.2f, %.2f, %.2f)", q[0], q[1], q[2], q[3]);

        quaternionToEuler(q, &roll, &pitch, &yaw);
        ESP_LOGI(PROCESS_TAG, "Pitch: %.2f, Roll: %.2f, Yaw: %.2f", pitch, roll, yaw);

        xQueueReceive(Control_data_queue, receive_buffer, portMAX_DELAY);
        PRYT_data_t parsed_data = parsedata(receive_buffer);

        float pitch_error = pitch - parsed_data.pitch;
        float roll_error = roll - parsed_data.roll;
        float yaw_error = yaw - parsed_data.yaw;

        PRYT_data_t PID_data = calculatePID(pitch_error, roll_error, yaw_error);

        esc_data.ESC1_out = (uint16_t)(parsed_data.throttle - PID_data.pitch - PID_data.roll - PID_data.yaw);
        esc_data.ESC2_out = (uint16_t)(parsed_data.throttle + PID_data.pitch - PID_data.roll + PID_data.yaw);
        esc_data.ESC3_out = (uint16_t)(parsed_data.throttle + PID_data.pitch + PID_data.roll - PID_data.yaw);
        esc_data.ESC4_out = (uint16_t)(parsed_data.throttle - PID_data.pitch + PID_data.roll + PID_data.yaw);
        ESP_LOGI(PROCESS_TAG, "ESC1: %d, ESC2: %d, ESC3: %d, ESC4: %d", esc_data.ESC1_out, esc_data.ESC2_out, esc_data.ESC3_out, esc_data.ESC4_out);

        xQueueSend(ESC_data_queue, &esc_data, 0);
    }
}