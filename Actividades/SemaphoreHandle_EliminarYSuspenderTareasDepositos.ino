/*
Ejercicio 1 de Práctica 3. Eliminar y Suspender Tareas
By MJJH, JJMM y JSZM
*/

#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

// Definición de pines de entrada (botones y sensores)
const int sp1 = 34;
const int sp2 = 35;
const int DL_ALTO = 32;
const int DL_BAJO = 33;

// Variables para almacenar el estado de los sensores
bool estadosp1;
bool estadosp2;
bool estadoDL_ALTO;
bool estadoDL_BAJO;

// Prototipos de las tareas
void estado01(void *parameters);
void estado02(void *parameters);
void estado03(void *parameters);
void estado04(void *parameters);

// Semáforos para la sincronización entre tareas
static SemaphoreHandle_t xSemaphore1, xSemaphore2, xSemaphore3, xSemaphore4;

void setup() {
  // Inicialización del puerto serial
  Serial.begin(115200);

  // Configuración de los pines de los sensores como entradas
  pinMode(sp1, INPUT_PULLDOWN);
  pinMode(sp2, INPUT_PULLDOWN);
  pinMode(DL_ALTO, INPUT_PULLDOWN);
  pinMode(DL_BAJO, INPUT_PULLDOWN);

  // Creación de semáforos binarios
  xSemaphore1 = xSemaphoreCreateBinary();
  xSemaphore2 = xSemaphoreCreateBinary();
  xSemaphore3 = xSemaphoreCreateBinary();
  xSemaphore4 = xSemaphoreCreateBinary();

  // Creación de tareas en el núcleo especificado
  xTaskCreatePinnedToCore(estado01, "estado01", 1024, NULL, 1, NULL, app_cpu);
  xTaskCreatePinnedToCore(estado02, "estado02", 1024, NULL, 1, NULL, app_cpu);
  xTaskCreatePinnedToCore(estado03, "estado03", 1024, NULL, 1, NULL, app_cpu);
  xTaskCreatePinnedToCore(estado04, "estado04", 1024, NULL, 1, NULL, app_cpu);

  // Arrancar la máquina de estados en el estado inicial (estado 1)
  xSemaphoreGive(xSemaphore1);
}

void loop() {
 
}

// Estado 1: Control inicial
void estado01(void *parameters) {
  while (1) {
    if (xSemaphoreTake(xSemaphore1, 0) == pdTRUE) {
      Serial.println("Estado 1: Valvula A activada.");
      estadosp1 = digitalRead(sp1); // Leer el estado del botón SP1

      if (estadosp1 == HIGH) {
        xSemaphoreGive(xSemaphore2); // Pasar al siguiente estado
      } else {
        xSemaphoreGive(xSemaphore1); // Mantenerse en el mismo estado
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS); // Retardo de 100 ms
  }
}

// Estado 2: Control intermedio
void estado02(void *parameters) {
  while (1) {
    if (xSemaphoreTake(xSemaphore2, 0) == pdTRUE) {
      Serial.println("Estado 2: Activando válvula B.");
      estadosp2 = digitalRead(sp2); // Leer el estado del botón SP2

      if (estadosp2 == HIGH) {
        xSemaphoreGive(xSemaphore3); // Pasar al siguiente estado
      } else {
        xSemaphoreGive(xSemaphore2); // Mantenerse en el mismo estado
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS); // Retardo de 100 ms
  }
}

// Estado 3: Control avanzado
void estado03(void *parameters) {
  while (1) {
    if (xSemaphoreTake(xSemaphore3, 0) == pdTRUE) {
      Serial.println("Estado 3: Verificando DL_ALTO.");
      estadoDL_ALTO = digitalRead(DL_ALTO); // Leer el estado del sensor DL_ALTO

      if (estadoDL_ALTO == HIGH) {
        Serial.println("Activando válvula 2 y motor por 10 segundos.");
        vTaskDelay(10000 / portTICK_PERIOD_MS); // Activar válvula 2 y motor por 10 segundos
        Serial.println("Estado 4: Compuerta activada. Esperando DL_BAJO.");
        xSemaphoreGive(xSemaphore4); // Pasar al siguiente estado
      } else {
        xSemaphoreGive(xSemaphore3); // Mantenerse en el mismo estado
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS); // Retardo de 100 ms
  }
}

// Estado 4: Activación completa y regreso a estado 1
void estado04(void *parameters) {
  while (1) {
    if (xSemaphoreTake(xSemaphore4, 0) == pdTRUE) {
      estadoDL_BAJO = digitalRead(DL_BAJO); // Leer el estado del sensor DL_BAJO

      if (estadoDL_BAJO == HIGH) {
        Serial.println("DL_BAJO detectado: Regresando a Estado 1.");
        xSemaphoreGive(xSemaphore1); // Regresar al estado inicial
      } else {
        xSemaphoreGive(xSemaphore4); // Mantenerse en el mismo estado
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS); // Retardo de 100 ms
  }
}
