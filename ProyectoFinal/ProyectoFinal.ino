//LIBRERÍAS

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <Servo.h>       // Para los servos
#include "pico/stdlib.h" // Funciones básicas

//Botones
static const int START = 0;
static const int STOP = 1;

int motorSpeed = 0; // Valor inicial: motor detenido


//Motores
uint8_t MOTOR1_PWM_PIN = 2;
uint8_t M1IN1 = 10;
uint8_t M1IN2 = 11;

uint8_t MOTOR2_PIN =20;
uint8_t MOTOR3_PIN =19;
uint8_t MOTOR4_PIN =18;

// Servo and motor objects
Servo servo1, servo2;

uint8_t SERVO_PIN[2]={12,13};
int anguloini[2]={0,0};
int angulofin[2]={60,60};

int pwmValue = 0; // Valor del PWM (0-255)

// Variables compartidas
//motor1
volatile int motor1Speed = 0; // 0-100%
int onTime = 0;      // Tiempo en que el LED estará encendido
int offTime = 0;     // Tiempo en que el LED estará apagado
int period = 1000;   // Periodo total del ciclo

//sensores ultrasonicos

#define NUM_SENSORS 3
bool objectDetected[3] = {0, 0, 0};

// Pines del sensor ultrasonico
int triggerPins[NUM_SENSORS] = {3, 6, 8};
int echoPins[NUM_SENSORS] = {4, 7, 9};

// Constante para la distancia objetivo (10 cm)
const int targetDistance = 10;

// Vector para almacenar las distancias medidas.
int distances[3];

//sensores IR

int irPin [3]= {27,26,21}; // Pin del sensor IR

//contadores
volatile int counterGrande = 0;
volatile int counterMediano = 0;
volatile int counterChico = 0;

int motorTime = 10000;           // Tiempo que el motor estará encendido (en milisegundos)
int servo1Time = 5000;           // Tiempo que el motor estará encendido (en milisegundos)

// Variables para detectar la presencia de un objeto IR
bool objectDetectedIR[3] = {false, false, false};    // Flag para verificar si un objeto ya fue detectado
unsigned long lastDetectionTime = 0;  // Para controlar el tiempo entre detecciones


//botones
volatile bool emergencyStop = false;
volatile bool systemRunning = false;
String serialCommand = "";


SemaphoreHandle_t serialMutex;
SemaphoreHandle_t motorSpeedMutex; // Mutex para proteger motor1Speed
SemaphoreHandle_t objectCountMutex; // Semaphore para proteger el contador
SemaphoreHandle_t xMutex4;

// Prototipos de las tareas
void taskSerialHandler(void *pvParameters);
void taskServo1(void *pvParameters);
void taskServo2(void *pvParameters);
void taskConteoGrandes(void *pvParameters);


void setup() {
  Serial.begin(115200);
  //Botones Inicio y paro
    pinMode(START, INPUT);
    pinMode(STOP, INPUT);


  // Create mutex
  serialMutex = xSemaphoreCreateMutex(); //Mutex para el serial
  motorSpeedMutex = xSemaphoreCreateMutex(); // Crear el mutex para motor1Speed
  objectCountMutex = xSemaphoreCreateMutex(); // Crear el mutex para proteger el contador de objetos
  xMutex4 = xSemaphoreCreateMutex();

  // Attach servos
  servo1.attach(SERVO_PIN[0]); 
  servo2.attach(SERVO_PIN[1]);
  
  servo1.write(anguloini[0]);
  servo2.write(anguloini[1]);

  //Motores 
  pinMode(MOTOR1_PWM_PIN, OUTPUT);
  pinMode(M1IN1, OUTPUT);
  pinMode(M1IN2, OUTPUT);
  pinMode(MOTOR2_PIN, OUTPUT);
  pinMode(MOTOR3_PIN, OUTPUT);
  pinMode(MOTOR4_PIN, OUTPUT);

  //Sensores ultrasonicos 
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(triggerPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }

  // Configurar el pin del sensor IR y del motor
  for (int i = 0; i < NUM_SENSORS; i++) {
    pinMode(irPin[i], INPUT);
  }

  // Dirección motores
  digitalWrite(M1IN1, LOW);
  digitalWrite(M1IN2, HIGH);

  // Create tasks
  xTaskCreate(taskSerialHandler, "Serial Handler", 2048, NULL, 4, NULL);
  xTaskCreate(TaskControlMotor1, "PWM Motor", 2048, NULL, 3, NULL);
  xTaskCreate(TaskBotones, "Botones", 2048, NULL, 5, NULL);
  xTaskCreate(taskServo1, "C1", 2048, NULL, 2, NULL);
  xTaskCreate(taskServo2, "C2", 2048, NULL, 2, NULL);
  xTaskCreate(taskConteoGrandes, "IR1", 2048, NULL, 1, NULL);
  xTaskCreate(taskConteoMedianos, "IR2", 2048, NULL, 1, NULL);
  xTaskCreate(taskConteoChicos, "IR3", 2048, NULL, 1, NULL);

}

void loop() {
  // Empty. Tasks are handled by FreeRTOS.
}

// TAREA - MANEJO DE COMANDOS SERIALES
void taskSerialHandler(void *pvParameters) {
    while (true) {
        if (Serial.available()) {
            String command = Serial.readStringUntil('\n');
            command.trim(); // Eliminar espacios en blanco alrededor del comando

            if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
                Serial.print("Received command: ");
                Serial.println(command);

                if (command.startsWith("VEL")) {
                    // Obtener la velocidad deseada desde el comando
                    int newSpeed = command.substring(3).toInt();
                    newSpeed = constrain(newSpeed, 0, 100); // Limitar entre 0% y 100%

                    // Proteger motor1Speed con el mutex
                    if (xSemaphoreTake(motorSpeedMutex, portMAX_DELAY) == pdTRUE) {
                        motor1Speed = newSpeed;
                        xSemaphoreGive(motorSpeedMutex);
                    }

                    Serial.print("Velocidad actualizada a: ");
                    Serial.println(newSpeed);
                } else if (command == "CONT") {
                    // Obtener los contadores (opcionalmente proteger con un mutex si es necesario)
                    Serial.print("Counter Grande: ");
                    Serial.println(counterGrande);
                    Serial.print("Counter Mediano: ");
                    Serial.println(counterMediano);
                    Serial.print("Counter Chico: ");
                    Serial.println(counterChico);
               // } else if (flag == 1) {
               //     systemRunning = true;
               //     emergencyStop = false;
               //     Serial.println("System Started");
               // } else if (flag == 0) {
               //    emergencyStop = true;
               //    systemRunning = false;
               //    Serial.println("Emergency Stop Activated");
                } else {
                    Serial.println("Unknown command");
                }

                xSemaphoreGive(serialMutex); // Liberar el mutex para el puerto serie
            }
        }
        // Retardo para permitir que otras tareas se ejecuten
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// TAREA - CONTROL DEL MOTOR1
void TaskControlMotor1(void *parameter) {
    while (1) {

    if(systemRunning){
        int localSpeed = 0;

        // Acceder a motor1Speed bajo protección del mutex
        if (xSemaphoreTake(motorSpeedMutex, portMAX_DELAY) == pdTRUE) {
            localSpeed = motor1Speed;
            xSemaphoreGive(motorSpeedMutex);
        }

        // Convertir el porcentaje de velocidad (0-100%) a rango PWM 
        pwmValue = map(localSpeed, 0, 100, 155, 255);
       

        analogWrite(MOTOR1_PWM_PIN, pwmValue); // Ajustar velocidad del motor
        //Serial.println(localSpeed);
      }
      else{
        analogWrite(MOTOR1_PWM_PIN, 0); // Ajustar velocidad del motor

      }
      // Retardo para evitar sobrecargar la CPU
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}      

void TaskBotones(void *parameter) {
    while (1) {
        if (digitalRead(START) == HIGH) { // Botón START presionado
            xSemaphoreTake(xMutex4, portMAX_DELAY);
            systemRunning = true;
            Serial.println("Sistema encendido");
            xSemaphoreGive(xMutex4);
        } else if (digitalRead(STOP) == HIGH) { // Botón STOP presionado
            xSemaphoreTake(xMutex4, portMAX_DELAY);
            systemRunning = false;
            Serial.println("Sistema apagado");
            xSemaphoreGive(xMutex4);
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Reducir el uso de CPU
    }
}


 void taskServo1(void *pvParameters) {
  while (true) {
    if (systemRunning) {
       objectDetected[0] = isObjectDetected(triggerPins[0], echoPins[0], targetDistance);
       objectDetected[1] = isObjectDetected(triggerPins[1], echoPins[1], targetDistance);
        
       if (objectDetected[0] && objectDetected[1])
       {
        servo1.write(angulofin[0]);
        Serial.println("Grande");
       }
       else {
        vTaskDelay(pdMS_TO_TICKS(servo1Time)); 
        servo1.write(anguloini[0]);
        
       }
       
    } 
    else{
      servo1.write(anguloini[1]);
      }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void taskServo2(void *pvParameters) {
  while (true) {
    if (systemRunning) {
       objectDetected[0] = isObjectDetected(triggerPins[0], echoPins[0], targetDistance);
       objectDetected[1] = isObjectDetected(triggerPins[1], echoPins[1], targetDistance);
       objectDetected[2] = isObjectDetected(triggerPins[2], echoPins[2], targetDistance); 

       if (objectDetected[1] && !objectDetected[0])
       {

        servo2.write(angulofin[1]);
        Serial.println("Mediano");
       }
       else  if(objectDetected[2]){
        servo2.write(anguloini[1]);
        Serial.println("Chico");
       }
       else {
        vTaskDelay(pdMS_TO_TICKS(servo1Time)); 
        servo2.write(anguloini[1]);
       }
       
    } 
    else{
      servo2.write(anguloini[1]);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}


// Tarea para detectar objetos y controlar el motor
void taskConteoGrandes(void *pvParameters) {
  while (true) {
    if (systemRunning) {
    // Leer el estado del sensor IR (detecta objeto si la señal es LOW)
    bool currentDetection1 = digitalRead(irPin[0]) == LOW;  // Asumiendo que el sensor activa LOW al detectar
    // Verificar si se ha detectado un objeto y si no hemos contado este objeto previamente
    if (currentDetection1 && !objectDetectedIR[0]) {
      objectDetectedIR[0] = true;// Registrar que un objeto ha sido detectado

      // Enciende el motor
      digitalWrite(MOTOR2_PIN, HIGH);  // Enciende el motor
      vTaskDelay(pdMS_TO_TICKS(motorTime));  // Mantener el motor encendido durante el tiempo especificado
      digitalWrite(MOTOR2_PIN, LOW);  // Apagar el motor

      // Incrementar el contador de objetos
      if (xSemaphoreTake(objectCountMutex, portMAX_DELAY) == pdTRUE) {
        counterGrande++;  // Incrementa el contador de objetos detectados
        xSemaphoreGive(objectCountMutex);
      }

      // Guardar el tiempo del último objeto detectado
      lastDetectionTime = millis();
    }

    // Comprobar si el objeto ha pasado y liberar la detección
    if (!currentDetection1 && objectDetectedIR[0]) {
      // Solo liberar el flag si ha pasado un tiempo suficiente (para evitar ruido)
      if (millis() - lastDetectionTime > 200) {  // Tiempo de espera para evitar doble detección
        objectDetectedIR[0] = false;  // Reiniciar el estado de detección
      }
    }

    // Esperar un pequeño retraso para evitar lecturas rápidas innecesarias
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  }
}

// Tarea para detectar objetos y controlar el motor
void taskConteoMedianos(void *pvParameters) {
  while (true) {
    if (systemRunning) {
    // Leer el estado del sensor IR (detecta objeto si la señal es LOW)
    bool currentDetection2 = digitalRead(irPin[1]) == LOW;  // Asumiendo que el sensor activa LOW al detectar
    // Verificar si se ha detectado un objeto y si no hemos contado este objeto previamente
    if (currentDetection2 && !objectDetectedIR[1]) {
      objectDetectedIR[0] = true;// Registrar que un objeto ha sido detectado

      // Enciende el motor
      digitalWrite(MOTOR3_PIN, HIGH);  // Enciende el motor
      vTaskDelay(pdMS_TO_TICKS(motorTime));  // Mantener el motor encendido durante el tiempo especificado
      digitalWrite(MOTOR3_PIN, LOW);  // Apagar el motor

      // Incrementar el contador de objetos
      if (xSemaphoreTake(objectCountMutex, portMAX_DELAY) == pdTRUE) {
        counterMediano++;  // Incrementa el contador de objetos detectados
        xSemaphoreGive(objectCountMutex);
      }

      // Guardar el tiempo del último objeto detectado
      lastDetectionTime = millis();
    }

    // Comprobar si el objeto ha pasado y liberar la detección
    if (!currentDetection2 && objectDetectedIR[1]) {
      // Solo liberar el flag si ha pasado un tiempo suficiente (para evitar ruido)
      if (millis() - lastDetectionTime > 200) {  // Tiempo de espera para evitar doble detección
        objectDetectedIR[1] = false;  // Reiniciar el estado de detección
      }
    }

    // Esperar un pequeño retraso para evitar lecturas rápidas innecesarias
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  }
}
// Tarea para detectar objetos y controlar el motor
void taskConteoChicos(void *pvParameters) {
  while (true) {
    if (systemRunning) {
    // Leer el estado del sensor IR (detecta objeto si la señal es LOW)
    bool currentDetection3 = digitalRead(irPin[2]) == LOW;  // Asumiendo que el sensor activa LOW al detectar
    // Verificar si se ha detectado un objeto y si no hemos contado este objeto previamente
    if (currentDetection3 && !objectDetectedIR[2]) {
      objectDetectedIR[2] = true;// Registrar que un objeto ha sido detectado

      // Enciende el motor
      digitalWrite(MOTOR4_PIN, HIGH);  // Enciende el motor
      vTaskDelay(pdMS_TO_TICKS(motorTime));  // Mantener el motor encendido durante el tiempo especificado
      digitalWrite(MOTOR4_PIN, LOW);  // Apagar el motor

      // Incrementar el contador de objetos
      if (xSemaphoreTake(objectCountMutex, portMAX_DELAY) == pdTRUE) {
        counterChico++;  // Incrementa el contador de objetos detectados
        xSemaphoreGive(objectCountMutex);
      }

      // Guardar el tiempo del último objeto detectado
      lastDetectionTime = millis();
    }

    // Comprobar si el objeto ha pasado y liberar la detección
    if (!currentDetection3 && objectDetectedIR[2]) {
      // Solo liberar el flag si ha pasado un tiempo suficiente (para evitar ruido)
      if (millis() - lastDetectionTime > 200) {  // Tiempo de espera para evitar doble detección
        objectDetectedIR[2] = false;  // Reiniciar el estado de detección
      }
    }

    // Esperar un pequeño retraso para evitar lecturas rápidas innecesarias
    vTaskDelay(pdMS_TO_TICKS(100));
  }
  }
}

// Función para detectar un objeto
bool isObjectDetected(int triggerPin, int echoPin, int threshold) {
  // Generar un pulso en el pin Trigger
  digitalWrite(triggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPin, LOW);

  // Leer el tiempo que tarda el pulso en regresar
  long duration = pulseIn(echoPin, HIGH);

  // Calcular la distancia en cm
  int distance = duration * 0.034 / 2;
  
  // Retornar true si la distancia es menor al umbral
  return distance < threshold;
}