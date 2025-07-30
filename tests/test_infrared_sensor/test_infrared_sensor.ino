
/**
 * @file test_infrared_sensor.ino
 * @brief Código de teste para um sensor infravermelho de 5 canais com ESP32.
 * 
 * Objetivo: Detectar uma linha preta no chão e imprimir o status de cada sensor
 * no Serial Monitor. O sensor detecta a linha quando a leitura é LOW (baixa reflexão).
 */

// --- Configuração dos Pinos ---
// Mude o valor padrão e defina seu pino.
const int sensorPins[5] = {13, 12, 14, 27, 26}; // Pinos para os 5 sensores infravermelhos

void setup() {
  // Inicia a comunicação serial a uma taxa de 115200 bps
  Serial.begin(115200);
  Serial.println("Teste do Sensor Infravermelho de 5 Canais");

  // Configura cada pino do sensor como entrada
  for (int i = 0; i < 5; i++) {
    pinMode(sensorPins[i], INPUT);
  }
}

void loop() {
  Serial.println("--- Nova Leitura ---");
  bool lineDetected = false;

  // Lê cada sensor e verifica se a linha foi detectada
  for (int i = 0; i < 5; i++) {
    // Sensores de linha IR geralmente retornam LOW quando detectam uma superfície escura (preta)
    int sensorValue = digitalRead(sensorPins[i]);
    
    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print(" (Pino ");
    Serial.print(sensorPins[i]);
    Serial.print("): ");

    if (sensorValue == LOW) {
      Serial.println("Linha preta detectada");
      lineDetected = true;
    } else {
      Serial.println("Nenhuma linha");
    }
  }

  if (!lineDetected) {
    Serial.println("Status: Nenhuma linha detectada por nenhum sensor.");
  }

  // Aguarda 1 segundo antes da próxima leitura para não sobrecarregar o monitor serial
  delay(1000);
}
