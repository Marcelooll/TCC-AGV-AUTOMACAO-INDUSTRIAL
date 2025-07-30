
/**
 * @file test_motor_bts7960.ino
 * @brief Código de teste para um motor DC com o driver de ponte H BTS7960 e ESP32.
 * 
 * Objetivo: Testar o funcionamento de um motor, girando-o para frente,
 * para trás e parando em sequência.
 */

// --- Configuração dos Pinos ---
// Mude o valor padrão e defina seu pino.
const int R_EN_PIN = 21;  // Pino de habilitação (Enable) para rotação à direita (frente)
const int L_EN_PIN = 19;  // Pino de habilitação (Enable) para rotação à esquerda (trás)
const int RPWM_PIN = 18;  // Pino PWM para controle de velocidade à direita (frente)
const int LPWM_PIN = 5;   // Pino PWM para controle de velocidade à esquerda (trás)

// --- Constantes ---
const int MOTOR_SPEED = 200; // Velocidade do motor (0 a 255)

void setup() {
  // Inicia a comunicação serial
  Serial.begin(115200);
  Serial.println("Teste do Motor com Ponte H BTS7960");

  // Configura todos os pinos como saída
  pinMode(R_EN_PIN, OUTPUT);
  pinMode(L_EN_PIN, OUTPUT);
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);

  // Habilita o driver da ponte H
  digitalWrite(R_EN_PIN, HIGH);
  digitalWrite(L_EN_PIN, HIGH);
  
  Serial.println("Driver habilitado. Iniciando sequencia de teste...");
}

void moveForward() {
  Serial.println("Acao: Girando motor para FRENTE");
  analogWrite(RPWM_PIN, MOTOR_SPEED);
  analogWrite(LPWM_PIN, 0);
}

void moveBackward() {
  Serial.println("Acao: Girando motor para TRAS");
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, MOTOR_SPEED);
}

void stopMotor() {
  Serial.println("Acao: PARANDO motor");
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);
}

void loop() {
  // Gira para frente por 3 segundos
  moveForward();
  delay(3000);

  // Para por 2 segundos
  stopMotor();
  delay(2000);

  // Gira para trás por 3 segundos
  moveBackward();
  delay(3000);

  // Para por 2 segundos
  stopMotor();
  delay(2000);
}
