
/**
 * @file test_robot_rotation.ino
 * @brief Código de teste para um sistema com 2 motores e 2 pontes H BTS7960.
 * 
 * Objetivo: Fazer o robô girar sobre seu próprio eixo nos sentidos
 * horário e anti-horário.
 */

// --- Configuração dos Pinos do Motor 1 (Direito) ---
// Mude o valor padrão e defina seu pino.
const int M1_R_EN_PIN = 21; // Enable Rotação Direita
const int M1_L_EN_PIN = 19; // Enable Rotação Esquerda
const int M1_RPWM_PIN = 18; // PWM Rotação Direita (Frente)
const int M1_LPWM_PIN = 5;  // PWM Rotação Esquerda (Trás)

// --- Configuração dos Pinos do Motor 2 (Esquerdo) ---
// Mude o valor padrão e defina seu pino.
const int M2_R_EN_PIN = 17; // Enable Rotação Direita
const int M2_L_EN_PIN = 16; // Enable Rotação Esquerda
const int M2_RPWM_PIN = 4;  // PWM Rotação Direita (Frente)
const int M2_LPWM_PIN = 2;  // PWM Rotação Esquerda (Trás)

// --- Constantes ---
const int ROTATION_SPEED = 200; // Velocidade de rotação (0 a 255)

void setup() {
  Serial.begin(115200);
  Serial.println("Teste de Rotacao do Robo com 2 Motores");

  // Configura todos os pinos como saída
  pinMode(M1_R_EN_PIN, OUTPUT);
  pinMode(M1_L_EN_PIN, OUTPUT);
  pinMode(M1_RPWM_PIN, OUTPUT);
  pinMode(M1_LPWM_PIN, OUTPUT);
  
  pinMode(M2_R_EN_PIN, OUTPUT);
  pinMode(M2_L_EN_PIN, OUTPUT);
  pinMode(M2_RPWM_PIN, OUTPUT);
  pinMode(M2_LPWM_PIN, OUTPUT);

  // Habilita ambos os drivers
  digitalWrite(M1_R_EN_PIN, HIGH);
  digitalWrite(M1_L_EN_PIN, HIGH);
  digitalWrite(M2_R_EN_PIN, HIGH);
  digitalWrite(M2_L_EN_PIN, HIGH);

  Serial.println("Drivers habilitados. Iniciando teste de rotacao...");
}

// Para o robô
void stopRobot() {
  Serial.println("Acao: PARANDO");
  analogWrite(M1_RPWM_PIN, 0);
  analogWrite(M1_LPWM_PIN, 0);
  analogWrite(M2_RPWM_PIN, 0);
  analogWrite(M2_LPWM_PIN, 0);
}

// Gira no sentido horário (motor direito para trás, motor esquerdo para frente)
void turnClockwise() {
  Serial.println("Acao: Girando no sentido HORARIO");
  // Motor Direito (M1) para trás
  analogWrite(M1_RPWM_PIN, 0);
  analogWrite(M1_LPWM_PIN, ROTATION_SPEED);
  // Motor Esquerdo (M2) para frente
  analogWrite(M2_RPWM_PIN, ROTATION_SPEED);
  analogWrite(M2_LPWM_PIN, 0);
}

// Gira no sentido anti-horário (motor direito para frente, motor esquerdo para trás)
void turnCounterClockwise() {
  Serial.println("Acao: Girando no sentido ANTI-HORARIO");
  // Motor Direito (M1) para frente
  analogWrite(M1_RPWM_PIN, ROTATION_SPEED);
  analogWrite(M1_LPWM_PIN, 0);
  // Motor Esquerdo (M2) para trás
  analogWrite(M2_RPWM_PIN, 0);
  analogWrite(M2_LPWM_PIN, ROTATION_SPEED);
}

void loop() {
  // Gira no sentido horário por 3 segundos
  turnClockwise();
  delay(3000);

  // Para por 2 segundos
  stopRobot();
  delay(2000);

  // Gira no sentido anti-horário por 3 segundos
  turnCounterClockwise();
  delay(3000);

  // Para por 2 segundos
  stopRobot();
  delay(2000);
}
