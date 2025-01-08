// LIBRERÍAS
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "semphr.h"
#include <MPU6050.h>

// PINES
static const int button1_pin = 2; // Botón - Tarea 1
static const int led1_pin = 3; // Led - Tarea 1
static const int button2_pin = 4; // Botón - Tarea 3
static const int led2_pin = 5; // Led - Tarea 3
static const int led3_pin = 6; // Led - Tarea 5
static const int ADC_pin = 26; // Pin ADC - Tarea 2
static const int button3_pin = 7; // Botón - Tarea 6

// Instancia del MPU6050
MPU6050 mpu;

// CONFIGURACIONES, CONSTANTES Y VARIABLES
#define BRIGHTNESS_MAX 255 // Valor máximo de PWM
uint16_t rawValue; // Valor de lectura del pin ADC
float voltage; // Variable de voltaje
float delayMs = 10; // Retardo inicial
float seno = 0; // Seno resultado del polinomio
const int limite = 20; // Grado del polinomio
float AngGrad = 0; // Ángulo en grados
float AngRad = 0; // Ángulo en radianes
const char msgTaylor[] = "Serie de Taylor de grado 20: "; 
int msgTaylor_leng = strlen(msgTaylor); 
SemaphoreHandle_t xMutex; // Identificador para primer mutex
SemaphoreHandle_t xMutex2; // Identificador para segundo mutex

// Tiempos acumulados en cada cara (6 caras del cubo)
float faceTimes[6] = {0, 0, 0, 0, 0, 0};

// Variables de orientación y estado
int currentFace = -1;   // Cara actual
int previousFace = -1;  // Cara anterior
bool timerRunning = false;

// Tiempo de inicio
unsigned long startTime = 0;

// Configuración del LCD
LiquidCrystal_I2C lcd(0x27, 16, 2); // Cambia la dirección I2C si es diferente

// FUNCIÓN DE INICIALIZACIÓN
void setup() {
    //Inicialización
    Serial.begin(115200); // Comunicación serial a 115200 baudios
    xMutex = xSemaphoreCreateMutex(); // Creación del primer mutex
    if (xMutex == NULL) { // Verificación de creación del mutex
      Serial.println("Error al crear el mutex");
      while (1);
    }
    xMutex2 = xSemaphoreCreateMutex(); // Creación del segundo mutex
    if (xMutex2 == NULL) { // Verificación de creación del mutex
      Serial.println("Error al crear el mutex");
      while (1);
    }
    // Inicializar el ADC
    analogReadResolution(12); // Configura el ADC a 12 bits

    // Configuración de I2C con pines personalizados
    Wire.setSDA(20); // Usar GP20 para SDA
    Wire.setSCL(21); // Usar GP21 para SCL
    Wire.begin(); // Usar GP20 para SDA y GP21 para SCL

    // Inicializar el LCD
    lcd.begin();  // LCD de 16x2
    lcd.backlight(); // Enciende la luz de fondo

    // Inicializar MPU6050
    mpu.initialize();
    if (!mpu.testConnection()) {
      Serial.println("Error: No se pudo conectar al MPU6050");
      while (true); // Detener ejecución si no se detecta el sensor
    }
    
    // Configuración de pines
     pinMode(button1_pin, INPUT);
     pinMode(led1_pin, OUTPUT);
     pinMode(button2_pin, INPUT);
     pinMode(led2_pin, OUTPUT);
     pinMode(led3_pin, OUTPUT);
     pinMode(ADC_pin, INPUT);
     pinMode(button3_pin, INPUT);

    // CREACION DE TAREAS
    xTaskCreate( // Tarea 1 - Secuencia de encendido y parpadeo de LED
        TaskSecuenciaLED,         // Función de la tarea
        "Secuencia LED",          // Nombre de la tarea
        1024,                     // Tamaño de la pila en palabras
        NULL,                     // Parámetro de entrada
        2,                        // Prioridad de la tarea
        NULL                      // Manejo de la tarea (opcional)
    );

    xTaskCreate( // Tarea 3 - Secuencia de parpadeo de LED
        TaskParpadeoLED,         // Función de la tarea
        "Parpadeo LED",          // Nombre de la tarea
        1024,                    // Tamaño de la pila en palabras
        NULL,                    // Parámetro de entrada
        2,                       // Prioridad de la tarea
        NULL                     // Manejo de la tarea (opcional)
    );

     xTaskCreate( // Tarea 5 - Brillo de LED con PWM
        TaskBrilloLED,           // Función de la tarea
        "Brillo LED",            // Nombre de la tarea
        1024,                    // Tamaño de la pila en palabras
        NULL,                    // Parámetro de entrada
        3,                       // Prioridad de la tarea
        NULL                     // Manejo de la tarea (opcional)
    );   

     xTaskCreate( // Tarea 2 - Lectura ADC e impresión en pantalla LCD
        TaskLeeryMostrarADC,     // Función de la tarea
        "Leer y Mostrar ADC",    // Nombre de la tarea
        2048,                    // Tamaño de la pila en palabras
        NULL,                    // Parámetro de entrada
        3,                       // Prioridad de la tarea
        NULL                     // Manejo de la tarea (opcional)
    ); 

     xTaskCreate( // Tarea 4 - Comunicación por UART sobre el estado de cada tarea
        TaskUART,                 // Función de la tarea
        "Mostrar en UART",        // Nombre de la tarea
        1024,                     // Tamaño de la pila en palabras
        NULL,                     // Parámetro de entrada
        1,                        // Prioridad de la tarea
        NULL                      // Manejo de la tarea (opcional)
    ); 

     xTaskCreate( // Tarea 6 - Medición de tiempo en cada cara de un cubo con MPU6050
        TaskMPU6050,             // Función de la tarea
        "Caras en MPU6050",      // Nombre de la tarea
        2048,                    // Tamaño de la pila en palabras
        NULL,                    // Parámetro de entrada
        1,                       // Prioridad de la tarea
        NULL                     // Manejo de la tarea (opcional)
    ); 
     xTaskCreate( // Tarea 7 - Calculo de Serie de Taylor
        TaskSerieTaylor,        // Función de la tarea
        "Serie de Taylor",      // Nombre de la tarea
        2048,                   // Tamaño de la pila en palabras
        NULL,                   // Parámetro de entrada
        1,                      // Prioridad de la tarea
        NULL                    // Manejo de la tarea (opcional)
    ); 
}

// TAREA 1 - SECUENCIA DE ENCENDIDO Y PARPADEO DE LED
void TaskSecuenciaLED(void *parameter) {
    while (1) {
        // Leer el estado del botón
        if (digitalRead(button1_pin) == HIGH) { // Botón presionado
            // Encender el LED por 3 segundos
            digitalWrite(led1_pin, HIGH);
            vTaskDelay(3000 / portTICK_PERIOD_MS);
            // Parpadeo infinito cada 1 segundo
            while (digitalRead(button1_pin) == HIGH) { // Mientras el botón esté presionado
                   digitalWrite(led1_pin, HIGH);
                   vTaskDelay(500 / portTICK_PERIOD_MS); // LED encendido
                   digitalWrite(led1_pin, LOW);
                   vTaskDelay(500 / portTICK_PERIOD_MS); // LED apagado
              }
            } 
        else{
              digitalWrite(led1_pin, LOW); // Asegurarse de que el LED esté apagado si no se presiona el botón
            }
        vTaskDelay(100 / portTICK_PERIOD_MS); // Reducir el uso de CPU
      }
}

// TAREA 3 - SECUENCIA DE PARPADEO DE LED
void TaskParpadeoLED(void *parameter) {
  while (1) {
    while (digitalRead(button2_pin)==HIGH){ // Botón presionado
          digitalWrite(led2_pin, HIGH);
          vTaskDelay(500 / portTICK_PERIOD_MS); // LED encendido
          digitalWrite(led2_pin, LOW);
          vTaskDelay(500 / portTICK_PERIOD_MS); // LED apagado
    }
    vTaskDelay(100 / portTICK_PERIOD_MS); // Reducir el uso de CPU
  }
}

// TAREA 5 - BRILLO DE LED CON PWM
void TaskBrilloLED(void *parameter) {
  int brightness = 0; // Valor inicial para el nivel de brillo
  int step = 1;  // Dirección de conteo para el ciclo

  while(1){
    // Ciclo PWM por software
    for (int i = 0; i < BRIGHTNESS_MAX; i++){
      if(i <= brightness){
        digitalWrite(led3_pin, HIGH); // LED encendido
      }
      else{
        digitalWrite(led3_pin, LOW); // LED apagado
      }
    }
    brightness = brightness + step; // Cambiar el nivel de brillo para la siguiente iteración
    // Cambiar la dirección  si alcanza los límites
    if(brightness >= BRIGHTNESS_MAX || brightness <= 0){
      step = -step; // Invertir la dirección
    }
    vTaskDelay(delayMs / portTICK_PERIOD_MS); // Controlar la resolución del software PWM    
  }
}

// TAREA 2 - LECTURA ADC E IMPRESIÓN EN PANTALLA LCD
void TaskLeeryMostrarADC(void *parameter) {
    while (1) {

        rawValue = analogRead(ADC_pin); // Lectura analógica de voltaje
        voltage = (rawValue / 4095.0) * 3.3; // Conversión del voltaje

            lcd.clear(); // Limpiar pantalla
            lcd.setCursor(0, 0); // Posición inicial de cursor
            lcd.print("ADC: ");
            lcd.setCursor(0, 1); // Salto de línea de cursor
            lcd.print(voltage, 2);
            lcd.print(" V");
        vTaskDelay(500 / portTICK_PERIOD_MS); // Actualizar cada 50 ms
    }
}

// TAREA 4 - COMUNICACIÓN POR UART SOBRE EL ESTADO DE CADA TAREA
void TaskUART(void *parameter){

    char command[4];  // Buffer para el comando UART

    while (1) {
        // Lee el comando del UART
        int i = 0;
        while (Serial.available() > 0 && i < 4) {
            command[i++] = Serial.read(); // Lee un byte del UART
        }
        command[i] = '\0'; // Termina la cadena

        // Verifica el comando recibido y responder
        if (i > 0) { // Si se recibió un comando
            if (strcmp(command, "ET01") == 0) { // Estado de la Tarea 1
                // Responde con el estado del botón
                if (digitalRead(button1_pin)==HIGH) {
                    Serial.println("Botón presionado");
                } else {
                    Serial.println("Botón no presionado");
                }
            }

            else if (strcmp(command, "ET03") == 0) { // Estado de la Tarea 3
                // Responde con el estado del botón
                if (digitalRead(button2_pin)==HIGH) {
                    Serial.println("Botón presionado");
                } else {
                    Serial.println("Botón no presionado");
                }
            }
            else if (strcmp(command, "ET02") == 0) { // Estado de la Tarea 2
               // Responde con el valor ADC del voltaje
                Serial.print("ADC: ");
                Serial.print(voltage, 2);
                Serial.println(" V");
            }
            else if (strcmp(command, "ET04") == 0) { // Estado de la Tarea 4
              //  Responde indicando la espera de conamndos
                Serial.print("Esperando comandos...");
            }
            else if (strcmp(command, "ET05") == 0) { // Estado de la Tarea 5
              String receivedData = ""; // Almacena la cadena recibida
              int flag = 1; // Bandera
               while (1 && flag == 1){
                  while (Serial.available()) {
                    char c = Serial.read(); // Leer valor de retardo
                    if (c == '\n') { // Fin del comando
                      float newDelay = receivedData.toFloat(); // Conversión de la cadena a flotante
                      if (newDelay > 0) {
                        xSemaphoreTake(xMutex, portMAX_DELAY); // Toma el mutex para proteger el acceso a la variable compartida
                        delayMs = newDelay; // Actualiza el valor del retardo
                        xSemaphoreGive(xMutex); // Libera el mutex una vez que se ha actualizado la variable
                        Serial.print("Nuevo retardo establecido: ");
                        Serial.println(newDelay); // Imprime el valor de nuevo retardo
                        flag = 0;
                      }
                      receivedData = ""; // Reinicia el buffer
                    } else {
                      receivedData += c; //  Agrega el carácter leído al buffer
                    }
                  }

        }
        vTaskDelay(100 / portTICK_PERIOD_MS); // Reducir el uso de CPU
    }
            else if (strcmp(command, "ET07") == 0) { // Estado de la Tarea 7

              String receivedData2 = ""; // Almacena la cadena recibida
              int flag = 1; // Bandera
               while (1 && flag == 1){
                  while (Serial.available()) {
                    char v = Serial.read();
                    if (v == '\n') { // Fin del comando
                      float ntaylor = receivedData2.toFloat(); // Conversión de cadena a flotante
                      if (ntaylor > 0) {
                        xSemaphoreTake(xMutex2, portMAX_DELAY); // Toma el mutex para proteger el acceso a la variable compartida
                        AngGrad = ntaylor; // Asigna el valor del ángulo introducido a la variable
                        xSemaphoreGive(xMutex2); // Libera el mutex una vez que se ha actualizado la variable
                            Serial.println();
                            for (int i = 0; i < msgTaylor_leng; i++) {
                              Serial.print(msgTaylor[i]);
                              //vTaskDelay(10 / portTICK_PERIOD_MS);
                            }
                            Serial.println();
                            Serial.printf("Ángulo ingresado: %.2f°\nSeno aproximado: %.5f\n", AngGrad, seno);   
                        flag = 0;
                      }
                      receivedData2 = ""; // Reinicia el buffer
                    } else {
                      receivedData2 += v; // Agrega el carácter leído al buffer
                    }
                  }

        }
        vTaskDelay(100 / portTICK_PERIOD_MS); // Reducir el uso de CPU

            }
  }
}
}

// TAREA 6 - MEDICIÓN DE TIEMPO EN CADA CARA DE UN CUBO CON MPU6050
void TaskMPU6050(void *parameter) {

  bool buttonPressed = false; // Estado del botón

  while (1) {
    // --- Manejo del botón ---
    if (digitalRead(button3_pin) == HIGH && !buttonPressed) {
      // Botón presionado
      buttonPressed = true;

      if (!timerRunning) {
        // Inicia la grabación
        timerRunning = true;
        startTime = millis();
        for (int i = 0; i < 6; i++) {
          faceTimes[i] = 0;
        }
        previousFace = -1;
        Serial.println("Comienza grabación...");
      } else {
        // Detiene la grabación
        timerRunning = false;
        Serial.println("Grabación terminada. Resultados:");
        for (int i = 0; i < 6; i++) {
          Serial.print("Cara ");
          Serial.print(i);
          Serial.print(": ");
          Serial.print(faceTimes[i]/1000);
          Serial.println(" s");
        }
      }
    } else if (digitalRead(button3_pin) == LOW) {
      buttonPressed = false;
    }

    // --- Manejo del sensor MPU6050 ---
    if (timerRunning) {
      int16_t ax, ay, az; // Variables para lecturas del MPU6050
      mpu.getAcceleration(&ax, &ay, &az); // Lecturas del MPU6050

      // Determinar la cara del cubo
       if (az > 14000) currentFace = 0; // Parámetro para cara 1
       else if (ax > 14000) currentFace = 1; // Parámetro para cara 2
       else if (ax < -14000) currentFace = 2; // Parámetro para cara 3
       else if (ay < -14000) currentFace = 3; // Parámetro para cara 4
       else if (ay > 14000) currentFace = 4; // Parámetro para cara 5
       else if (az < -14000) currentFace = 5; // Parámetro para cara 6
       else currentFace = -1; // 

      // Actualizar tiempos si la cara cambia
      if (currentFace != -1 && currentFace != previousFace) {
        unsigned long currentTime = millis();
        if (previousFace != -1) {
          faceTimes[previousFace] += (currentTime - startTime);
        }
        startTime = currentTime;
        previousFace = currentFace;
      }
    }

    // Espera breve para evitar alto consumo de CPU
    vTaskDelay(500 / portTICK_PERIOD_MS); // Actualiza cada 500 ms    
  }
}

// TAREA 7 - CÁLCULO DE SERIE DE TAYLOR
void TaskSerieTaylor(void *parameter) {

  while (1) {
      AngRad = AngGrad * (3.14159 / 180.0); // Conversión de ángulo de grados a radianes
      seno = 0; // Variable para resultado
      for (int i = 0; i <= limite; i++) {  // Ciclo para el cálculo del polinomio de Taylor de grado 20
        float valor = pow(-1,i)*pow(AngRad, 2*(i+1)) / tgamma(2 *(i+2)); // Valor del término de Taylor
        seno = seno+valor;
        vTaskDelay(10 / portTICK_PERIOD_MS); // Retardo
      }
    vTaskDelay(100 / portTICK_PERIOD_MS); // Espera breve para evitar alto consumi de CPU
  } 
}

void loop() {
    // La función en bucle no se usa en FreeRTOS
}
