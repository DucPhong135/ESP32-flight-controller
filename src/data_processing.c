#include "Common-define.h"

#define PI 3.1412
#define MAX_PWM (819)
#define MIN_PWM (410)

static const char *PROCESS_TAG = "process_task";
const float alpha = 0.9996;

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
void configureMPU6050() {
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
        .scl_speed_hz = 300000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &MPU6050, &dev_handle));

    uint8_t buffer[3][2] = {{0x6B, 0x00}, {0x1C, 0x10}, {0x1B, 0x08}};
    for (uint8_t i = 0; i < 3; i++) {
        ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, (uint8_t *)&buffer[i], 2, -1));
    }
    esp_rom_delay_us(100000);
}

void calibrateGyroscope() {
    const int numSamples = 2000;
    float gyroXSum = 0.0, gyroYSum = 0.0, gyroZSum = 0.0;
    uint8_t data[6];
    uint8_t addr = 0x43;
    imu_data_t g;

    for (int i = 0; i < numSamples; i++) {
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

// Data processing task
void DataProcessingTask(void *parameter) {
    imu_data_t a, g;
    uint8_t addr = 0x3B; // start address of the required register
    uint8_t data[14];
    uint32_t previousTime = 0;
    uint32_t currentTime = 0;
    float pitch = 0, roll = 0, yaw = 0;
    float current_pitch = 0, current_roll = 0;
    ESC_out_t esc_data;
    char receive_buffer[128] = {0};
    bool set_gyro_angles = false;

    ESC_data_queue = xQueueCreate(20, sizeof(ESC_out_t));

    while (1) {
        ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, &addr, 1, -1));
        ESP_ERROR_CHECK(i2c_master_receive(dev_handle, (uint8_t *)&data, 14, -1));

        a.acceleration.x = (data[0] << 8) | data[1];
        a.acceleration.y = (data[2] << 8) | data[3];
        a.acceleration.z = (data[4] << 8) | data[5];
        g.gyro.x = data[8] << 8 | data[9];
        g.gyro.y = data[10] << 8 | data[11];
        g.gyro.z = data[12] << 8 | data[13];

        TickType_t tick_count = xTaskGetTickCount();
        currentTime = pdTICKS_TO_MS(tick_count);
        float deltaTime = (currentTime - previousTime) / 1000.0;
        previousTime = currentTime;

        pitch += (g.gyro.x - gyroXOffset) * deltaTime * 0.01526717557; // Degrees
        roll += (g.gyro.y - gyroYOffset) * deltaTime * 0.01526717557;  // Degrees
        yaw += (g.gyro.z - gyroZOffset) * deltaTime * 0.01526717557;   // Degrees

        pitch += roll * sin(g.gyro.z * 0.000001066);
        roll -= pitch * sin(g.gyro.z * 0.000001066);

        float acc_total_vector = sqrt((a.acceleration.x * a.acceleration.x) + (a.acceleration.y * a.acceleration.y) + (a.acceleration.z * a.acceleration.z));
        float accelPitch = asin((float)a.acceleration.y / acc_total_vector) * 57.296;
        float accelRoll = asin((float)a.acceleration.x / acc_total_vector) * -57.296;

        if (set_gyro_angles) {
            pitch = pitch * alpha + accelPitch * (1 - alpha);
            roll = roll * alpha + accelRoll * (1 - alpha);
        } else {
            pitch = accelPitch;
            roll = accelRoll;
            set_gyro_angles = true;
        }

        current_pitch = current_pitch * 0.9 + pitch * 0.1;
        current_roll = current_roll * 0.9 + roll * 0.1;

        xQueueReceive(Control_data_queue, receive_buffer, portMAX_DELAY);
        PRYT_data_t parsed_data = parsedata(receive_buffer);

        float pitch_error = current_pitch - parsed_data.pitch;
        float roll_error = current_roll - parsed_data.roll;
        float yaw_error = yaw - parsed_data.yaw;

        PRYT_data_t PID_data = calculatePID(pitch_error, roll_error, yaw_error);

        esc_data.ESC1_out = (uint16_t)(parsed_data.throttle - PID_data.pitch - PID_data.roll - PID_data.yaw);
        esc_data.ESC2_out = (uint16_t)(parsed_data.throttle + PID_data.pitch - PID_data.roll + PID_data.yaw);
        esc_data.ESC3_out = (uint16_t)(parsed_data.throttle + PID_data.pitch + PID_data.roll - PID_data.yaw);
        esc_data.ESC4_out = (uint16_t)(parsed_data.throttle - PID_data.pitch + PID_data.roll + PID_data.yaw);

        xQueueSend(ESC_data_queue, &esc_data, 0);
    }
}