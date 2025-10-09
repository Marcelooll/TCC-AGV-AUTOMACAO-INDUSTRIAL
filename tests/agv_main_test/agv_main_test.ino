/**
 * @file agv_main_test.ino
 * @brief Código de teste integrado para AGV com ESP32.
 * @author Marcelol
 * @date 11/09/2025
 *
 * @details
 * Este programa implementa um sistema de controle completo para um AGV (Automated Guided Vehicle),
 * incluindo as seguintes funcionalidades:
 * - Seguidor de linha com 5 sensores IR (TCRT5000).
 * - Desvio de obstáculos com sensor ultrassônico (HC-SR04).
 * - Controle de dois motores DC com drivers de Ponte H (BTS7960).
 * - Máquina de estados para gerenciar o comportamento do AGV (Parado, Seguindo Linha, Desviando).
 * - Controle por botões de Start/Stop.
 * - Sinalização de status com LEDs.
 *
 *
 */

// =================================================================
// 1. CONFIGURAÇÕES E CONSTANTES
// =================================================================

// --- Pinos do Motor Direito (Motor 1) ---
const int M1_RPWM_PIN = 25;
const int M1_LPWM_PIN = 26;
const int M1_EN_R_PIN = 33;
const int M1_EN_L_PIN = 32;

// --- Pinos do Motor Esquerdo (Motor 2) ---
const int M2_RPWM_PIN = 19;
const int M2_LPWM_PIN = 18;
const int M2_EN_R_PIN = 21;
const int M2_EN_L_PIN = 23;

// --- Pinos dos Sensores ---
const int ULTRA_TRIG_PIN = 5;
const int ULTRA_ECHO_PIN = 17;

const int IR_PIN_1 = 39; // Mais à esquerda
const int IR_PIN_2 = 34;
const int IR_PIN_3 = 36; // Centro
const int IR_PIN_4 = 35;
const int IR_PIN_5 = 4;  // Mais à direita
const int IR_PINS[5] = {IR_PIN_1, IR_PIN_2, IR_PIN_3, IR_PIN_4, IR_PIN_5};

// --- Pinos de Controle (Botões e LEDs) ---
// Botão START: Conectar usando o contato NA (Normalmente Aberto)
const int BUTTON_START_PIN = 16;
// Botão STOP: Conectar usando o contato NF (Normalmente Fechado)
const int BUTTON_STOP_PIN = 22;

const int LED_IDLE_PIN = 13;
const int LED_RUN_1_PIN = 14;
const int LED_RUN_2_PIN = 27;

// --- Parâmetros de Comportamento ---
const int OBSTACLE_DISTANCE_CM = 20; // Distância para acionar o desvio
const int BASE_SPEED = 150;          // Velocidade base dos motores (0-255)
const float KP = 40.0;               // Constante Proporcional (para o seguidor de linha)

// --- Constantes de Temporização ---
const unsigned long LED_BLINK_INTERVAL = 500; // Intervalo de pisca do LED (ms)

// =================================================================
// 2. VARIÁVEIS GLOBAIS E ESTADOS
// =================================================================

// Máquina de Estados do AGV
enum AGVState {
  STATE_STOPPED, // AGV totalmente parado e desarmado
  STATE_IDLE,    // Armado, mas aguardando comando para iniciar
  STATE_FOLLOWING_LINE,
  STATE_AVOIDING_OBSTACLE
};

AGVState currentState = STATE_STOPPED;

// Variáveis para controle de tempo (LEDs e botões)
unsigned long lastBlinkTime = 0;
bool ledState = false;

// =================================================================
// 3. FUNÇÕES DE SETUP
// =================================================================

void setup() {
  Serial.begin(115200);
  Serial.println(">>> AGV Teste Integrado - Iniciando <<<");

  setupPins();
  setupMotorDrivers();

  Serial.println("Setup concluido. AGV em modo STOPPED.");
}

/**
 * @brief Configura o modo (INPUT/OUTPUT) de todos os pinos.
 */
void setupPins() {
  // Motores
  pinMode(M1_RPWM_PIN, OUTPUT);
  pinMode(M1_LPWM_PIN, OUTPUT);
  pinMode(M1_EN_R_PIN, OUTPUT);
  pinMode(M1_EN_L_PIN, OUTPUT);
  pinMode(M2_RPWM_PIN, OUTPUT);
  pinMode(M2_LPWM_PIN, OUTPUT);
  pinMode(M2_EN_R_PIN, OUTPUT);
  pinMode(M2_EN_L_PIN, OUTPUT);

  // Sensores
  pinMode(ULTRA_TRIG_PIN, OUTPUT);
  pinMode(ULTRA_ECHO_PIN, INPUT);
  for (int i = 0; i < 5; i++) {
    pinMode(IR_PINS[i], INPUT);
  }

  // Botões (com pull-up interno, ativos em LOW)
  pinMode(BUTTON_START_PIN, INPUT_PULLUP);
  pinMode(BUTTON_STOP_PIN, INPUT_PULLUP);

  // LEDs
  pinMode(LED_IDLE_PIN, OUTPUT);
  pinMode(LED_RUN_1_PIN, OUTPUT);
  pinMode(LED_RUN_2_PIN, OUTPUT);
}

/**
 * @brief Habilita os drivers da ponte H.
 */
void setupMotorDrivers() {
  digitalWrite(M1_EN_R_PIN, HIGH);
  digitalWrite(M1_EN_L_PIN, HIGH);
  digitalWrite(M2_EN_R_PIN, HIGH);
  digitalWrite(M2_EN_L_PIN, HIGH);
  Serial.println("Drivers dos motores habilitados.");
}


// =================================================================
// 4. FUNÇÕES DE CONTROLE DOS COMPONENTES
// =================================================================

/**
 * @brief Define a velocidade e direção de cada motor.
 * @param leftSpeed Velocidade do motor esquerdo (-255 a 255).
 * @param rightSpeed Velocidade do motor direito (-255 a 255).
 */
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  // Motor Esquerdo
  if (leftSpeed >= 0) {
    analogWrite(M2_RPWM_PIN, leftSpeed);
    analogWrite(M2_LPWM_PIN, 0);
  } else {
    analogWrite(M2_RPWM_PIN, 0);
    analogWrite(M2_LPWM_PIN, -leftSpeed);
  }

  // Motor Direito
  if (rightSpeed >= 0) {
    analogWrite(M1_RPWM_PIN, rightSpeed);
    analogWrite(M1_LPWM_PIN, 0);
  } else {
    analogWrite(M1_RPWM_PIN, 0);
    analogWrite(M1_LPWM_PIN, -rightSpeed);
  }
}

/**
 * @brief Para ambos os motores.
 */
void stopMotors() {
  setMotorSpeeds(0, 0);
}

/**
 * @brief Lê a distância do sensor ultrassônico.
 * @return A distância em centímetros.
 */
long readUltrasonicDistance() {
  digitalWrite(ULTRA_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRA_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRA_TRIG_PIN, LOW);
  
  long duration = pulseIn(ULTRA_ECHO_PIN, HIGH);
  return duration * 0.034 / 2;
}

/**
 * @brief Lê os sensores de linha e calcula a posição da linha.
 * @return Um valor de erro ponderado. 0 = centro, <0 = esquerda, >0 = direita.
 *         Retorna 1000 se a linha for perdida.
 */
float readLinePosition() {
  bool irValues[5];
  int onLineCount = 0;
  float weightedSum = 0;

  // Lê os sensores (assumindo que LOW = sobre a linha)
  for (int i = 0; i < 5; i++) {
    irValues[i] = (digitalRead(IR_PINS[i]) == LOW);
    if (irValues[i]) {
      onLineCount++;
    }
  }

  // Calcula a posição ponderada
  // Posições: -2, -1, 0, 1, 2
  if (onLineCount > 0) {
    weightedSum = (irValues[0] * (-2) + irValues[1] * (-1) + irValues[2] * 0 + irValues[3] * 1 + irValues[4] * 2);
    return weightedSum / onLineCount;
  }

  // Linha perdida
  return 1000.0; 
}

/**
 * @brief Atualiza o estado dos LEDs de acordo com o estado do AGV.
 */
void updateLEDs() {
  unsigned long currentTime = millis();

  // Controla o pisca-pisca
  if (currentTime - lastBlinkTime >= LED_BLINK_INTERVAL) {
    lastBlinkTime = currentTime;
    ledState = !ledState;
  }

  // Define quais LEDs devem piscar
  switch (currentState) {
    case STATE_STOPPED:
      digitalWrite(LED_IDLE_PIN, LOW);
      digitalWrite(LED_RUN_1_PIN, LOW);
      digitalWrite(LED_RUN_2_PIN, LOW);
      break;
    case STATE_IDLE:
      digitalWrite(LED_IDLE_PIN, ledState);
      digitalWrite(LED_RUN_1_PIN, LOW);
      digitalWrite(LED_RUN_2_PIN, LOW);
      break;
    case STATE_FOLLOWING_LINE:
    case STATE_AVOIDING_OBSTACLE:
      digitalWrite(LED_IDLE_PIN, LOW);
      digitalWrite(LED_RUN_1_PIN, ledState);
      digitalWrite(LED_RUN_2_PIN, ledState);
      break;
  }
}

// =================================================================
// 5. LÓGICA DA MÁQUINA DE ESTADOS
// =================================================================

/**
 * @brief Lógica principal para o estado SEGUINDO LINHA.
 */
void handleFollowingLine() {
  // Verifica se há um obstáculo
  if (readUltrasonicDistance() < OBSTACLE_DISTANCE_CM) {
    Serial.println("Obstaculo detectado! Mudando para AVOIDING_OBSTACLE.");
    currentState = STATE_AVOIDING_OBSTACLE;
    stopMotors(); // Para antes de iniciar a manobra
    delay(500);
    return;
  }

  // Calcula o erro da posição da linha
  float error = readLinePosition();

  if (error == 1000.0) {
    // Linha perdida, para o robô
    stopMotors();
  } else {
    // Calcula a correção usando o controlador Proporcional
    int correction = (int)(KP * error);

    // Ajusta a velocidade dos motores
    int leftSpeed = BASE_SPEED - correction;
    int rightSpeed = BASE_SPEED + correction;

    setMotorSpeeds(leftSpeed, rightSpeed);
  }
}

/**
 * @brief Lógica principal para o estado DESVIANDO DE OBSTÁCULO.
 */
void handleAvoidingObstacle() {
  Serial.println("Executando manobra de desvio...");

  // 1. Gira 90 graus para a direita
  setMotorSpeeds(BASE_SPEED, -BASE_SPEED);
  delay(1000); // Ajuste este tempo para uma curva de 90 graus

  // 2. Avança por um tempo
  setMotorSpeeds(BASE_SPEED, BASE_SPEED);
  delay(1500);

  // 3. Gira 90 graus para a esquerda
  setMotorSpeeds(-BASE_SPEED, BASE_SPEED);
  delay(1000);

  // 4. Avança até encontrar a linha novamente
  setMotorSpeeds(BASE_SPEED, BASE_SPEED);
  while(readLinePosition() == 1000.0) {
    // Continua avançando
    delay(10);
  }

  Serial.println("Manobra concluida. Voltando a seguir a linha.");
  currentState = STATE_FOLLOWING_LINE;
}

/**
 * @brief Verifica os botões de controle e atualiza o estado.
 */
void checkButtons() {
  // Botão de STOP (NF - Normalmente Fechado) tem prioridade máxima.
  // A lógica é invertida: o pino fica em HIGH quando o botão é pressionado (circuito aberto).
  if (digitalRead(BUTTON_STOP_PIN) == HIGH) {
    if (currentState != STATE_STOPPED) {
      Serial.println("Botao STOP pressionado. Desarmando AGV.");
      currentState = STATE_STOPPED;
      stopMotors();
      delay(500); // Debounce
    }
    return;
  }

  // Botão de START
  if (digitalRead(BUTTON_START_PIN) == LOW) {
    delay(50); // Debounce simples
    if (digitalRead(BUTTON_START_PIN) == LOW) {
      switch (currentState) {
        case STATE_STOPPED:
          Serial.println("Botao START pressionado. Armado, em modo IDLE.");
          currentState = STATE_IDLE;
          break;
        case STATE_IDLE:
          Serial.println("Botao START pressionado. Iniciando operacao.");
          currentState = STATE_FOLLOWING_LINE;
          break;
        case STATE_FOLLOWING_LINE:
        case STATE_AVOIDING_OBSTACLE:
          // Se já está rodando, o botão de start não faz nada.
          break;
      }
      delay(500); // Evita múltiplas leituras
    }
  }
}


// =================================================================
// 6. LOOP PRINCIPAL
// =================================================================

void loop() {
  // 1. Verifica os botões para transições de estado manuais
  checkButtons();

  // 2. Executa a lógica correspondente ao estado atual
  switch (currentState) {
    case STATE_STOPPED:
      // Não faz nada, os motores já estão parados.
      break;
    case STATE_IDLE:
      // Não faz nada, apenas aguarda o comando de início.
      break;
    case STATE_FOLLOWING_LINE:
      handleFollowingLine();
      break;
    case STATE_AVOIDING_OBSTACLE:
      handleAvoidingObstacle();
      break;
  }

  // 3. Atualiza os LEDs de status
  updateLEDs();
  
  delay(10); // Pequeno delay para estabilidade
}
