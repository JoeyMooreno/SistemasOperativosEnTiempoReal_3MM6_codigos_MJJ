#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_AHTX0.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Configuración del LCD I2C
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Selección de CPU para FreeRTOS
#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

// Declaración del mutex para manejar la concurrencia en el LCD
static SemaphoreHandle_t mutex;

// Pines para los sensores
static const int trigPin = 12;  // HC-SR04 TRIG en GPIO 12
static const int echoPin = 13;  // HC-SR04 ECHO en GPIO 13

// Instancia del sensor AHT20
Adafruit_AHTX0 aht;

// Variables para almacenar la distancia y temperatura
static float distance;
static float temperature;

// Tarea para leer la distancia del sensor HC-SR04
void ReadDistance(void *parameters) {
  while (1) {
    // Generar pulso TRIG para iniciar la medición
    digitalWrite(trigPin, LOW);
    vTaskDelay(2 / portTICK_PERIOD_MS);
    digitalWrite(trigPin, HIGH);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    digitalWrite(trigPin, LOW);

    // Medir duración del pulso ECHO
    long duration = pulseIn(echoPin, HIGH);

    // Convertir la duración en distancia (cm)
    distance = duration * 0.034 / 2;

    // Actualizar el LCD con la distancia
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
      lcd.setCursor(0, 0);
      lcd.print("Distancia:     ");
      lcd.setCursor(0, 0);
      lcd.print("Distancia: ");
      lcd.print(distance);
      lcd.print(" cm");
      xSemaphoreGive(mutex);
    }

    // Imprimir la distancia en el monitor serial
    Serial.print("Distancia: ");
    Serial.print(distance);
    Serial.println(" cm");

    vTaskDelay(1000 / portTICK_PERIOD_MS); // Intervalo de medición
  }
}

// Tarea para leer la temperatura del sensor AHT20
void ReadTemperature(void *parameters) {
  while (1) {
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);  // Lee el sensor AHT20
    
    temperature = temp.temperature;

    if (!isnan(temperature)) {
      if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        lcd.setCursor(0, 1);
        lcd.print("Temp:         "); // Limpia la línea
        lcd.setCursor(0, 1);
        lcd.print("Temp: ");
        lcd.print(temperature);
        lcd.print(" C");
        xSemaphoreGive(mutex);
      }

      // Imprimir la temperatura en el monitor serial
      Serial.print("Temperatura: ");
      Serial.print(temperature);
      Serial.println(" C");
      Serial.println("--------------------------");
    } else {
      Serial.println("Error al leer el AHT20");
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS); // Intervalo de medición
  }
}

void setup() {
  Serial.begin(9600);

  // Configurar pines del HC-SR04
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Iniciar el bus I2C y el LCD
  Wire.begin(14, 15);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Lectura sensores");

  // Inicializar el sensor AHT20
  if (!aht.begin()) {
    Serial.println("No se pudo encontrar AHT20. Verifique la conexión.");
    while (1);
  }

  // Crear el mutex
  mutex = xSemaphoreCreateMutex();

  // Crear las tareas de FreeRTOS
  xTaskCreate(ReadDistance, "ReadDistance", 2048, NULL, 1, NULL);
  xTaskCreate(ReadTemperature, "ReadTemperature", 2048, NULL, 1, NULL);

  vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void loop() {
  // El loop está vacío ya que las tareas se manejan mediante FreeRTOS
}
