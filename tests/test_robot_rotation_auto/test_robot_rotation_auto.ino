/**
 * @file test_robot_rotation_auto.ino
 * @brief Código de teste automático para a rotação do robô com feedback de LED.
 * 
 * Objetivo: Testar o sistema de rotação do robô (motores e drivers) de forma
 * automática, sem a necessidade de botões ou comandos externos. LEDs de feedback
 * visual indicam a direção da rotação.
 * 
 * Como funciona:
 * - Ao ser ligado, o robô executará uma sequência contínua de movimentos.
 * - Ele girará no sentido horário por 3 segundos, com um LED da direita aceso.
 * - Depois, parará por 2 segundos, com ambos os LEDs apagados.
 * - Em seguida, girará no sentido anti-horário por 3 segundos, com um LED da esquerda aceso.
 * - Parará por 2 segundos, e o ciclo se repetirá.
 */

// --- Pinos de Configuração dos LEDs (Saídas) ---
// Usamos pinos seguros para os LEDs.
const int LED_DIREITA_PIN = 22;  // LED que indica rotação horária
const int LED_ESQUERDA_PIN = 23; // LED que indica rotação anti-horária

// --- Configuração dos Pinos do Motor 1 (Direito) ---
const int M1_R_EN_PIN = 21; // Enable Rotação Direita
const int M1_L_EN_PIN = 19; // Enable Rotação Esquerda
const int M1_RPWM_PIN = 18; // PWM Rotação Direita (Frente)
const int M1_LPWM_PIN = 5;  // PWM Rotação Esquerda (Trás)

// --- Configuração dos Pinos do Motor 2 (Esquerdo) ---
const int M2_R_EN_PIN = 17; // Enable Rotação Direita
const int M2_L_EN_PIN = 16; // Enable Rotação Esquerda
const int M2_RPWM_PIN = 4;  // PWM Rotação Direita (Frente)
const int M2_LPWM_PIN = 2;  // PWM Rotação Esquerda (Trás)

// --- Constantes de Controle ---
const int ROTATION_SPEED = 200; // Velocidade de rotação (0 a 255)

/**
 * @brief Função de configuração (setup).
 * Executada uma vez ao ligar o ESP32.
 */
void setup() {
  Serial.begin(115200);
  Serial.println(">>> Teste Automatico de Rotacao com LEDs <<<");

  // Configura os pinos dos LEDs como SAÍDA
  pinMode(LED_DIREITA_PIN, OUTPUT);
  pinMode(LED_ESQUERDA_PIN, OUTPUT);

  // Configura todos os pinos dos motores como SAÍDA
  pinMode(M1_R_EN_PIN, OUTPUT);
  pinMode(M1_L_EN_PIN, OUTPUT);
  pinMode(M1_RPWM_PIN, OUTPUT);
  pinMode(M1_LPWM_PIN, OUTPUT);
  
  pinMode(M2_R_EN_PIN, OUTPUT);
  pinMode(M2_L_EN_PIN, OUTPUT);
  pinMode(M2_RPWM_PIN, OUTPUT);
  pinMode(M2_LPWM_PIN, OUTPUT);

  // Habilita ambos os drivers da ponte H
  digitalWrite(M1_R_EN_PIN, HIGH);
  digitalWrite(M1_L_EN_PIN, HIGH);
  digitalWrite(M2_R_EN_PIN, HIGH);
  digitalWrite(M2_L_EN_PIN, HIGH);

  Serial.println("Drivers habilitados. Iniciando sequencia de teste...");
}

/**
 * @brief Para o robô e apaga os LEDs.
 */
void stopRobot() {
  Serial.println("Acao: PARANDO");
  
  // Desliga os motores
  analogWrite(M1_RPWM_PIN, 0);
  analogWrite(M1_LPWM_PIN, 0);
  analogWrite(M2_RPWM_PIN, 0);
  analogWrite(M2_LPWM_PIN, 0);

  // Apaga os LEDs
  digitalWrite(LED_DIREITA_PIN, LOW);
  digitalWrite(LED_ESQUERDA_PIN, LOW);
}

/**
 * @brief Gira no sentido horário (motor direito para trás, esquerdo para frente).
 *        Acende o LED da direita.
 */
void turnClockwise() {
  Serial.println("Acao: Girando no sentido HORARIO");
  
  // Motor Direito (M1) para trás
  analogWrite(M1_RPWM_PIN, 0);
  analogWrite(M1_LPWM_PIN, ROTATION_SPEED);
  // Motor Esquerdo (M2) para frente
  analogWrite(M2_RPWM_PIN, ROTATION_SPEED);
  analogWrite(M2_LPWM_PIN, 0);

  // Acende o LED da direita e apaga o da esquerda
  digitalWrite(LED_DIREITA_PIN, HIGH);
  digitalWrite(LED_ESQUERDA_PIN, LOW);
}

/**
 * @brief Gira no sentido anti-horário (motor direito para frente, esquerdo para trás).
 *        Acende o LED da esquerda.
 */
void turnCounterClockwise() {
  Serial.println("Acao: Girando no sentido ANTI-HORARIO");
  
  // Motor Direito (M1) para frente
  analogWrite(M1_RPWM_PIN, ROTATION_SPEED);
  analogWrite(M1_LPWM_PIN, 0);
  // Motor Esquerdo (M2) para trás
  analogWrite(M2_RPWM_PIN, 0);
  analogWrite(M2_LPWM_PIN, ROTATION_SPEED);

  // Acende o LED da esquerda e apaga o da direita
  digitalWrite(LED_DIREITA_PIN, LOW);
  digitalWrite(LED_ESQUERDA_PIN, HIGH);
}

/**
 * @brief Função de loop principal.
 * Executa a sequência de movimentos continuamente.
 */
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