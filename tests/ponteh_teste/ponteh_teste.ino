/**
 * @file ponteh_teste.ino
 * @brief Teste automático para um motor com Ponte H e feedback de LED.
 * 
 * Objetivo: Testar o funcionamento de um motor DC com um driver de Ponte H (como o BTS7960),
 * executando uma sequência automática de movimentos para frente e para trás.
 * Um LED indicador permanece aceso sempre que o motor estiver em movimento.
 * 
 * Como funciona:
 * - Ao ser ligado, o ESP32 iniciará uma sequência contínua.
 * - O motor girará para frente por 3 segundos, com o LED de status aceso.
 * - O motor parará por 2 segundos, e o LED se apagará.
 * - O motor girará para trás por 3 segundos, com o LED de status aceso novamente.
 * - O motor parará por 2 segundos, e o LED se apagará.
 * - O ciclo se repetirá indefinidamente.
 */

// --- Pinos de Configuração ---

// Pino de SAÍDA (OUTPUT) para o LED de status
// Este LED indica que o motor está em funcionamento.
const int LED_STATUS_PIN = 22; // Usando um pino seguro

// Pinos de controle do Motor com a Ponte H (ex: BTS7960)
const int R_EN_PIN = 21;  // Pino de habilitação (Enable) para rotação à direita (frente)
const int L_EN_PIN = 19;  // Pino de habilitação (Enable) para rotação à esquerda (trás)
const int RPWM_PIN = 18;  // Pino PWM para controle de velocidade à direita (frente)
const int LPWM_PIN = 5;   // Pino PWM para controle de velocidade à esquerda (trás)

// --- Constantes de Controle ---
const int MOTOR_SPEED = 200; // Velocidade do motor (0 a 255)

/**
 * @brief Função de configuração (setup).
 * Executada uma vez ao ligar o ESP32.
 */
void setup() {
  // Inicia a comunicação serial
  Serial.begin(115200);
  Serial.println(">>> Teste Automatico de Motor com Ponte H <<<");

  // Configura o pino do LED como SAÍDA
  pinMode(LED_STATUS_PIN, OUTPUT);

  // Configura todos os pinos do motor como SAÍDA
  pinMode(R_EN_PIN, OUTPUT);
  pinMode(L_EN_PIN, OUTPUT);
  pinMode(RPWM_PIN, OUTPUT);
  pinMode(LPWM_PIN, OUTPUT);

  // Habilita o driver da ponte H
  digitalWrite(R_EN_PIN, HIGH);
  digitalWrite(L_EN_PIN, HIGH);
  
  Serial.println("Driver habilitado. Iniciando sequencia de teste...");
}

/**
 * @brief Move o motor para frente e acende o LED.
 */
void moveForward() {
  Serial.println("Acao: Girando motor para FRENTE");
  digitalWrite(LED_STATUS_PIN, HIGH); // Acende o LED de status
  analogWrite(RPWM_PIN, MOTOR_SPEED);
  analogWrite(LPWM_PIN, 0);
}

/**
 * @brief Move o motor para trás e acende o LED.
 */
void moveBackward() {
  Serial.println("Acao: Girando motor para TRAS");
  digitalWrite(LED_STATUS_PIN, HIGH); // Acende o LED de status
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, MOTOR_SPEED);
}

/**
 * @brief Para o motor e apaga o LED.
 */
void stopMotor() {
  Serial.println("Acao: PARANDO motor");
  digitalWrite(LED_STATUS_PIN, LOW); // Apaga o LED de status
  analogWrite(RPWM_PIN, 0);
  analogWrite(LPWM_PIN, 0);
}

/**
 * @brief Função de loop principal.
 * Executa a sequência de movimentos continuamente.
 */
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