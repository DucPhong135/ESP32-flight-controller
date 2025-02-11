#include "Common-define.h"


// Task Handle for 3 tasks
TaskHandle_t DataProcessingHandle;
TaskHandle_t MotorControlHandle;
TaskHandle_t ReceiveDataHandle;

//Queue handle for 2 queues
QueueHandle_t ESC_data_queue;     // queue for esc data
QueueHandle_t Control_data_queue; // queue for data from app


extern void configureESC();
extern void configureMPU6050();
extern void calibrateGyroscope();
extern void ReceiveDataSetup();
extern void ReceiveDataTask(void *parameter);
extern void DataProcessingTask(void *parameter);
extern void MotorControlTask(void *parameter);



void app_main()
{
    configureESC();
    configureMPU6050();
    calibrateGyroscope();
    ReceiveDataSetup();

    ESC_data_queue = xQueueCreate(64, sizeof(ESC_out_t));
    Control_data_queue = xQueueCreate(64, sizeof(char) * 128);

    xTaskCreate(ReceiveDataTask, "ReceiveData", 2048, NULL, 1, &ReceiveDataHandle);
    xTaskCreate(DataProcessingTask, "DataProcessing", 4096, NULL, 1, &DataProcessingHandle);
    xTaskCreate(MotorControlTask, "MotorControl", 2048, NULL, 1, &MotorControlHandle);
}
