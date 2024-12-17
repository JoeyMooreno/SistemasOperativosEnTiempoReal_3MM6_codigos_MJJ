#if CONFIG_FREERTOS_UNICORE
    static const BaseType_t app_cpu = 0;
#else
    static const BaseType_t app_cpu = 1;
#endif

// Definir longitud de colas
static const uint8_t queue_len_A = 5; 
static const uint8_t queue_len_B = 5;

// Declaración de manejadores de colas
static QueueHandle_t queue_A;
static QueueHandle_t queue_B;

void setup() {

    Serial.begin(115200); // Inicializar comunicación serial a 115200 baudios
    Serial.println("Mensaje de inicio");
    delay(500); // Retardo

    // Creación de colas
    queue_A = xQueueCreate(queue_len_A, sizeof(int)); // Cola A
    queue_B = xQueueCreate(queue_len_B, sizeof(int)); // Cola B

    // Creación de tareas
    xTaskCreatePinnedToCore(TareaA_Process, "Tarea A", 1024, NULL, 1, NULL, app_cpu); // Tarea A
    xTaskCreatePinnedToCore(TareaB_Process, "Tarea B", 1024, NULL, 1, NULL, app_cpu); // Tarea B

}

// Función de la Tarea A
void TareaA_Process(void *parameters) {

    static int numA = 0; // Contador de mensajes enviados en la Tarea A
    int item; // Variable para almacenar el dato recibido desde la cola

    while (1) {

        if (xQueueReceive(queue_A, (void*)&item, 0) == pdTRUE) { // Intenta recibir un dato de la Cola A sin bloqueo
            Serial.print("Tarea A received: ");
            Serial.println(item); // Imprime el dato recibido si es exitoso
        }
        vTaskDelay(50 / portTICK_PERIOD_MS); // Espera 50 ms

        if (xQueueSend(queue_A, (void*)&numA, 10) != pdTRUE) { // Envía numA a la Cola A con un tiempo de espera de 10 ticks
            Serial.println("Queue A full"); // Si la Cola A está llena, imprime mensaje de error
        }
        vTaskDelay(100 / portTICK_PERIOD_MS); // Espera 100 ms

        numA++; // Incrementa contador de la Tarea A

    }

}

// Función de la Tarea B
void TareaB_Process(void *parameters) {

    int item; // Variable para almacenar el dato recibido desde la cola
    static int numB = 0; // Contador de mensajes enviados en la Tarea B

    while (1) {

        if (xQueueReceive(queue_B, (void*)&item, 0) == pdTRUE) { // Intenta recibir un dato de la Cola B sin bloqueo
            Serial.print("Tarea B received: ");
            Serial.println(item); // Imprime el dato recibido si es exitoso
        }
        vTaskDelay(50 / portTICK_PERIOD_MS); // Espera 50 ms

        if (xQueueSend(queue_B, (void*)&numB, 10) != pdTRUE) { // Envía numB a la Cola B con un tiempo de espera de 10 ticks
            Serial.println("Queue B full"); // Si la Cola B está llena, imprime mensaje de error
        }
        vTaskDelay(100 / portTICK_PERIOD_MS); // Espera 100 ms

        numB++; // Incrementa contador de la Tarea B
    }

}

void loop() {
  // Vacío
}