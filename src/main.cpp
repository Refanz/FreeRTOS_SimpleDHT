#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <DHT.h>

DHT dht;

QueueHandle_t temperature_q;
QueueHandle_t humidity_q;

void TaskDHT_Temperature(void *pvParameters) {
    float current_temperature;

    while (true) {
        Serial.println("TaskDHTTemp");
        current_temperature = dht.getTemperature();
        Serial.println(current_temperature);

        if (!isnan(current_temperature)) {
            xQueueSend(temperature_q, &current_temperature, portMAX_DELAY);
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void TaskDHT_Humidity(void *pvParameters) {
    float current_humidity;

    while (true) {
        Serial.println("TaskDHTHum");
        current_humidity = dht.getHumidity();
        Serial.println(current_humidity);

        if (!isnan(current_humidity)) {
            xQueueSend(humidity_q, &current_humidity, portMAX_DELAY);
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void setup() {
    Serial.begin(9600);

    dht.setup(A0, DHT::DHT11);

    // Membuat temperature queue
    temperature_q = xQueueCreate(3, sizeof(float));

    // Membuat humidity queue
    humidity_q = xQueueCreate(3, sizeof(float));

    if (temperature_q == NULL || humidity_q == NULL) {
        Serial.println("Queue can not be created!");
    }

    xTaskCreate(TaskDHT_Temperature, "DHT Temp Task", 128, NULL, 1, NULL);
    xTaskCreate(TaskDHT_Humidity, "DHT Hum Task", 128, NULL, 1, NULL);

    vTaskStartScheduler();
}

void loop() {

}