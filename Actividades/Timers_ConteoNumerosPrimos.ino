#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/timers.h>

#define LED_PIN 4

#if CONFIG_FREERTOS_UNICORE
  static const BaseType_t app_cpu = 0;
#else
  static const BaseType_t app_cpu = 1;
#endif

// Variables globales
int primeCount = 0;         // Contador de números primos
int targetCount = 0;        // Contador objetivo ingresado por el usuario
bool reset = false;         // Indica si se debe reiniciar el conteo
int seconds = 0;            // Contador de segundos

// Timer de FreeRTOS
static TimerHandle_t auto_reload = NULL;

// Función para verificar si un número es primo
bool isPrime(int num) {
    if (num <= 1) return false;
    for (int i = 2; i <= sqrt(num); i++) {
        if (num % i == 0) return false;
    }
    return true;
}

// Callback para el timer de FreeRTOS
void myTimer(TimerHandle_t xTimer) {
    seconds++;

    if (reset) {
        seconds = 0;
        primeCount = 0;
        reset = false;
        Serial.println("Reinicio del conteo.");
    }

    // Imprimir el número de segundos actual
    Serial.print("Segundos: ");
    Serial.println(seconds);

    // Verificar si el número actual es primo
    if (isPrime(seconds)) {
        if (primeCount < targetCount) {
            digitalWrite(LED_PIN, HIGH);  // Enciende el LED
            Serial.println("¡Primo!");
            primeCount++;
            vTaskDelay(1000 / portTICK_PERIOD_MS);  // Mantener el LED encendido por 1 segundo
            digitalWrite(LED_PIN, LOW);   // Apaga el LED
        }
    }

    // Detener el conteo si se alcanzaron 100 segundos
    if (seconds >= 100) {
        Serial.println("Conteo alcanzado.");
        xTimerStop(auto_reload, 0);  // Detiene el timer
    }
}

// Función de configuración inicial
void setup() {
    Serial.begin(9600);
    pinMode(LED_PIN, OUTPUT);

    // Crear el Timer auto-reload de 1 segundo
    auto_reload = xTimerCreate(
        "Timer 01",                       // Nombre del timer
        1000 / portTICK_PERIOD_MS,        // Periodo en Ticks (1 segundo)
        pdTRUE,                           // Configuración de auto-reload
        (void*)1,                         // Identificador
        myTimer                           // Función de callback
    );

    // Iniciar el Timer
    xTimerStart(auto_reload, portMAX_DELAY);
}

// Función de loop principal
void loop() {
    if (Serial.available() > 0) {
        int input = Serial.parseInt();  // Leer el valor ingresado
        if (input > 0) {
            targetCount = input;        // Actualizar el objetivo
            primeCount = 0;             // Reiniciar el contador de primos
            reset = true;               // Indicar que se debe reiniciar
            Serial.println("Reinicio del conteo.");
        }
    }    
}