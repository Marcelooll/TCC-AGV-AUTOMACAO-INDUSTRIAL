
/**
 * @file test_ultrasonic_sensor.ino
 * @brief Código de teste para o sensor ultrassônico HC-SR04 com ESP32.
 * 
 * Objetivo: Medir a distância de um objeto à frente e reportar se está
 * dentro de um limite pré-definido.
 */

// --- Configuração dos Pinos ---
// Mude o valor padrão e defina seu pino.
const int TRIG_PIN = 23; // Pino de Trigger (envio do pulso)
const int ECHO_PIN = 22; // Pino de Echo (recebimento do pulso)

// --- Constantes ---
const int DISTANCE_THRESHOLD = 20; // Distância em cm. Altere para o seu limite.

void setup() {
  // Inicia a comunicação serial a uma taxa de 115200 bps
  Serial.begin(115200);
  Serial.println("Teste do Sensor Ultrassonico HC-SR04");

  // Configura os pinos de Trigger e Echo
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  long duration, distance;

  // Gera o pulso ultrassônico
  // 1. Garante que o pino de trigger esteja em estado baixo
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);

  // 2. Envia um pulso de 10 microssegundos
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // 3. Mede o tempo que o pino de echo permanece em estado alto
  duration = pulseIn(ECHO_PIN, HIGH);

  // 4. Calcula a distância com base no tempo medido
  // A velocidade do som é aproximadamente 343 m/s, ou 0.0343 cm/µs.
  // A distância é dividida por 2 porque o som vai e volta.
  distance = duration * 0.0343 / 2;

  // Imprime a distância medida
  Serial.print("Distancia: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Verifica se a distância está dentro do limite
  if (distance > 0 && distance < DISTANCE_THRESHOLD) {
    Serial.println("Status: Objeto detectado proximo!");
  } else {
    Serial.println("Status: Nenhum objeto proximo.");
  }

  // Aguarda 1 segundo antes da próxima medição
  delay(1000);
}
