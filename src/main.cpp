#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <semphr.h>

DHT dht;

LiquidCrystal_I2C lcd(0x27, 16, 2);

QueueHandle_t DHT_Queue;

TaskHandle_t taskHandle_LCD;
TaskHandle_t taskHandle_DHT;
TaskHandle_t taskHandle_Flame;

SemaphoreHandle_t xBinarySemaphore;

#define DHT_PIN A0
#define RELAY_PIN 2
#define FLAME_PIN 9
#define BUZZER_PIN 6

void TaskDHT_Read(void *params) {
    uint8_t temp_val, humd_val;

    while (true) {

        if (xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdTRUE) {
            temp_val = (uint8_t) dht.getTemperature();
            humd_val = (uint8_t) dht.getHumidity();

            if (!isnan(temp_val)) {
                Serial.print("Task DHT Read Temp: ");
                Serial.println(temp_val);

                xQueueSend(DHT_Queue, &temp_val, portMAX_DELAY);
            }

            if (!isnan(humd_val)) {
                Serial.print("Task DHT Humd: ");
                Serial.println(humd_val);
                xQueueSend(DHT_Queue, &humd_val, portMAX_DELAY);
            }

            if (!isnan(temp_val) && temp_val > 30) {
                //Serial.println("Fan On");
                digitalWrite(RELAY_PIN, HIGH);

            } else {
                digitalWrite(RELAY_PIN, LOW);
            }

            xSemaphoreGive(xBinarySemaphore);
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void TaskFlame_Read(void *params) {
    uint8_t flame_val;

    while (true) {

        if (xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdTRUE) {
            flame_val = digitalRead(FLAME_PIN);

            Serial.print("Flame Read Task: ");
            Serial.println(flame_val);

            if (flame_val == 1) {
                //Serial.println("Fire..");
                tone(BUZZER_PIN, 100);

                lcd.clear();
                lcd.setCursor(0, 0);
                lcd.print("Fire..");

            } else {
                //Serial.println("No Fire");
                noTone(BUZZER_PIN);

                lcd.clear();
                lcd.setCursor(0, 0);
                lcd.print("No Fire");
            }

            xSemaphoreGive(xBinarySemaphore);
        }

        vTaskDelay(pdMS_TO_TICKS(700));
    }
}

void TaskDisplay_LCD(void *params) {
    uint8_t temp_val, humd_val;

    while (true) {

        if (xSemaphoreTake(xBinarySemaphore, portMAX_DELAY) == pdTRUE) {
            Serial.println("LCD Task");

            if (xQueueReceive(DHT_Queue, &temp_val, portMAX_DELAY) == pdPASS &&
                xQueueReceive(DHT_Queue, &humd_val, portMAX_DELAY) == pdPASS) {

                lcd.clear();
                lcd.setCursor(0, 0);
                lcd.print("Humidity: ");
                lcd.setCursor(10, 0);
                lcd.print(humd_val);

                lcd.setCursor(0, 1);
                lcd.print("Temperature: ");
                lcd.setCursor(13, 1);
                lcd.print(temp_val);
            }

        }

        xSemaphoreGive(xBinarySemaphore);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void setup() {
    Serial.begin(9600);

    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("TA SOWN FREERTOS");
    lcd.setCursor(0, 1);
    lcd.print("KELOMPOK 3");

    dht.setup(DHT_PIN, DHT::DHT11);

    pinMode(RELAY_PIN, OUTPUT);

    pinMode(FLAME_PIN, INPUT);

    pinMode(BUZZER_PIN, INPUT);

    DHT_Queue = xQueueCreate(2, sizeof(uint8_t));


    if (DHT_Queue == NULL) {
        Serial.println("Queue can not be created!");
    }

    xBinarySemaphore = xSemaphoreCreateBinary();

    xTaskCreate(TaskDHT_Read, "DHT Read Task", 113, NULL, 1, &taskHandle_DHT);
    xTaskCreate(TaskFlame_Read, "Flame Read Task", 113, NULL, 1, &taskHandle_Flame);
    xTaskCreate(TaskDisplay_LCD, "DHT Display LCD", 113, NULL, 1, &taskHandle_LCD);

    xSemaphoreGive(xBinarySemaphore);

    vTaskStartScheduler();
}

void loop() {

}