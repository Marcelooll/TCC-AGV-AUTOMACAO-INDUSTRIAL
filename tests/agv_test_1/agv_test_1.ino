// =======================================================================
//
//      PROJETO FINAL AGV - TCC | VERSÃO ELITE (100% NÃO-BLOQUEANTE)
//      Autor: Gemini
//      Data: 27/08/2025
//
//      - Sistema 100% não-bloqueante, incluindo botão de emergência.
//      - Filtro de Mediana Otimizado (Insertion Sort) para o Ultrassom.
//      - Funções de movimento abstraídas (princípio DRY).
//      - Constantes de calibração globais para fácil ajuste.
//
// =======================================================================

// =======================================================================
//  DEFINIÇÕES DE HARDWARE E PARÂMETROS
// =======================================================================

// --- Hardware Pins ---
const int irPin1 = 23, irPin2 = 22, irPin3 = 21, irPin4 = 19, irPin5 = 18;
const int echoPin = 2, trigPin = 15;
const int motorR_IN1 = 13, motorR_IN2 = 12, motorL_IN3 = 32, motorL_IN4 = 14;
const int startButton = 27, stopButton = 25;
const int sinaleiroPowerPin = 26, sinaleiroStatusPin = 33;

// --- Parâmetros de Navegação e Sensores ---
const int OBSTACLE_DISTANCE_CM = 15;
const int NUM_US_READINGS = 5;
const int NO_OBSTACLE_DETECTED = 999;

// --- Parâmetros de Calibração de Desvio (Ajuste Conforme o Robô Real) ---
const int T_REVERSE = 300, T_TURN_90 = 500, T_PASS = 1000, T_REALIGN = 1200;

// --- Configuração do PWM ---
const int PWM_FREQ = 5000, PWM_RESOLUTION = 8;
const int PWM_CHANNEL_R1 = 0, PWM_CHANNEL_R2 = 1, PWM_CHANNEL_L1 = 2, PWM_CHANNEL_L2 = 3;

// --- Parâmetros de Velocidade ---
const int VEL_MAXIMA = 200, VEL_CURVA_SUAVE = 150, VEL_CURVA_FECHADA = 180;

// =======================================================================
//  VARIÁVEIS GLOBAIS DE ESTADO
// =======================================================================

enum SystemState { STATE_NORMAL, STATE_EMERGENCY };
SystemState systemState = STATE_NORMAL;

enum AvoidState { IDLE, STEP_1_REVERSING, STEP_2_TURNING_RIGHT, STEP_3_FORWARD_1, STEP_4_TURNING_LEFT, STEP_5_FORWARD_2, STEP_6_TURNING_LEFT_2, STEP_7_FORWARD_3, FINISHING };
AvoidState avoidState = IDLE;
unsigned long avoidStepStartTime = 0;

bool isRunning = false;

// --- Timers e Estados para Lógicas Não-Bloqueantes ---
unsigned long previousSinaleiroMillis = 0;
bool sinaleiroState = LOW;

unsigned long lastDebounceTime = 0;
int lastStartButtonState = HIGH;
const long debounceDelay = 50;

// =======================================================================
//  SETUP
// =======================================================================
void setup() {
  pinMode(irPin1, INPUT); pinMode(irPin2, INPUT); pinMode(irPin3, INPUT); pinMode(irPin4, INPUT); pinMode(irPin5, INPUT);
  pinMode(trigPin, OUTPUT); pinMode(echoPin, INPUT);
  pinMode(startButton, INPUT_PULLUP); pinMode(stopButton, INPUT_PULLUP);
  pinMode(sinaleiroPowerPin, OUTPUT); pinMode(sinaleiroStatusPin, OUTPUT);

  ledcSetup(PWM_CHANNEL_R1, PWM_FREQ, PWM_RESOLUTION); ledcSetup(PWM_CHANNEL_R2, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_L1, PWM_FREQ, PWM_RESOLUTION); ledcSetup(PWM_CHANNEL_L2, PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(motorR_IN1, PWM_CHANNEL_R1); ledcAttachPin(motorR_IN2, PWM_CHANNEL_R2);
  ledcAttachPin(motorL_IN3, PWM_CHANNEL_L1); ledcAttachPin(motorL_IN4, PWM_CHANNEL_L2);

  stopMotors();
}

// =======================================================================
//  LOOP PRINCIPAL
// =======================================================================
void loop() {
  handleButtons();
  handleSinaleiro();

  switch (systemState) {
    case STATE_NORMAL:
      if (isRunning) {
        if (avoidState == IDLE) {
          if (getFilteredDistance() <= OBSTACLE_DISTANCE_CM) {
            avoidState = STEP_1_REVERSING;
            avoidStepStartTime = millis();
          }
        }

        if (avoidState != IDLE) {
          handleObstacleAvoidance();
        } else {
          followLine();
        }
      }
      break;

    case STATE_EMERGENCY:
      // Em estado de emergência, nenhuma lógica de movimento é executada.
      // Apenas os sinaleiros e botões são monitorados.
      break;
  }
}

// =======================================================================
//  MÁQUINAS DE ESTADO E HANDLERS
// =======================================================================

void handleSinaleiro() {
  unsigned long currentMillis = millis();
  const long BLINK_INTERVAL_NORMAL = 300;
  const long BLINK_INTERVAL_EMERGENCY = 100;
  long currentBlinkInterval = (systemState == STATE_EMERGENCY) ? BLINK_INTERVAL_EMERGENCY : BLINK_INTERVAL_NORMAL;

  if (currentMillis - previousSinaleiroMillis >= currentBlinkInterval) {
    previousSinaleiroMillis = currentMillis;
    sinaleiroState = !sinaleiroState;
  }

  if (systemState == STATE_EMERGENCY) {
    digitalWrite(sinaleiroPowerPin, sinaleiroState);
    digitalWrite(sinaleiroStatusPin, sinaleiroState);
  } else {
    if (isRunning) {
      digitalWrite(sinaleiroPowerPin, sinaleiroState);
      digitalWrite(sinaleiroStatusPin, sinaleiroState);
    } else {
      digitalWrite(sinaleiroPowerPin, HIGH);
      digitalWrite(sinaleiroStatusPin, LOW);
    }
  }
}

void handleButtons() {
  // --- Botão de Emergência (NF) ---
  if (digitalRead(stopButton) == HIGH) { // Botão de emergência pressionado
    if (systemState == STATE_NORMAL) {
      systemState = STATE_EMERGENCY;
      isRunning = false;
      avoidState = IDLE;
      stopMotors();
    }
  } else { // Botão de emergência liberado
    if (systemState == STATE_EMERGENCY) {
      systemState = STATE_NORMAL;
    }
  }

  // --- Botão de Start/Pause (NA) com Debounce 100% Não-Bloqueante ---
  int reading = digitalRead(startButton);
  if (reading != lastStartButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // A leitura é considerada estável, verifica se houve uma transição de HIGH para LOW
    if (reading == LOW && lastStartButtonState == HIGH) {
      isRunning = !isRunning;
      if (!isRunning) {
        stopMotors();
        avoidState = IDLE;
      }
    }
  }
  lastStartButtonState = reading;
}

void handleObstacleAvoidance() {
  unsigned long currentTime = millis();
  switch (avoidState) {
    case STEP_1_REVERSING:      moveBackward();   if (currentTime - avoidStepStartTime > T_REVERSE) { avoidState = STEP_2_TURNING_RIGHT; avoidStepStartTime = currentTime; } break;
    case STEP_2_TURNING_RIGHT:  turnSharpRight(); if (currentTime - avoidStepStartTime > T_TURN_90) { avoidState = STEP_3_FORWARD_1; avoidStepStartTime = currentTime; } break;
    case STEP_3_FORWARD_1:      moveForward();    if (currentTime - avoidStepStartTime > T_PASS) { avoidState = STEP_4_TURNING_LEFT; avoidStepStartTime = currentTime; } break;
    case STEP_4_TURNING_LEFT:   turnSharpLeft();  if (currentTime - avoidStepStartTime > T_TURN_90) { avoidState = STEP_5_FORWARD_2; avoidStepStartTime = currentTime; } break;
    case STEP_5_FORWARD_2:      moveForward();    if (currentTime - avoidStepStartTime > T_REALIGN) { avoidState = STEP_6_TURNING_LEFT_2; avoidStepStartTime = currentTime; } break;
    case STEP_6_TURNING_LEFT_2: turnSharpLeft();  if (currentTime - avoidStepStartTime > T_TURN_90) { avoidState = STEP_7_FORWARD_3; avoidStepStartTime = currentTime; } break;
    case STEP_7_FORWARD_3:      moveForward();    if (currentTime - avoidStepStartTime > T_PASS) { avoidState = FINISHING; } break;
    case FINISHING:             stopMotors();     avoidState = IDLE; break;
  }
}

// =======================================================================
//  FUNÇÕES DE SENSORES E NAVEGAÇÃO
// =======================================================================

int getFilteredDistance() {
  int readings[NUM_US_READINGS];
  for (int i = 0; i < NUM_US_READINGS; i++) {
    readings[i] = getRawDistance();
    delayMicroseconds(200); // Pausa mínima entre leituras para estabilização do sensor
  }

  // Insertion Sort - eficiente para N pequeno
  for (int i = 1; i < NUM_US_READINGS; i++) {
    int key = readings[i];
    int j = i - 1;
    while (j >= 0 && readings[j] > key) {
      readings[j + 1] = readings[j];
      j = j - 1;
    }
    readings[j + 1] = key;
  }
  return readings[NUM_US_READINGS / 2]; // Retorna a mediana
}

int getRawDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long raw_duration = pulseIn(echoPin, HIGH, 25000);
  return (raw_duration == 0) ? NO_OBSTACLE_DETECTED : raw_duration * 0.034 / 2;
}

void followLine() {
  int s1 = digitalRead(irPin1), s2 = digitalRead(irPin2), s3 = digitalRead(irPin3), s4 = digitalRead(irPin4), s5 = digitalRead(irPin5);
  if (s1 && s2 && !s3 && s4 && s5) moveForward();
  else if (s1 && !s2 && s3 && s4 && s5) turnSlightLeft();
  else if (s1 && s2 && s3 && !s4 && s5) turnSlightRight();
  else if (!s1 && s2 && s3 && s4 && s5) turnSharpLeft();
  else if (s1 && s2 && s3 && s4 && !s5) turnSharpRight();
  else if (s1 && !s2 && !s3 && s4 && s5) turnSharpLeft();
  else if (s1 && s2 && !s3 && !s4 && s5) turnSharpRight();
  else stopMotors();
}

// =======================================================================
//  FUNÇÕES DE MOVIMENTO (ABSTRAÍDAS)
// =======================================================================

void _setMotorPWM(int R1, int R2, int L1, int L2) {
  ledcWrite(PWM_CHANNEL_R1, R1); ledcWrite(PWM_CHANNEL_R2, R2);
  ledcWrite(PWM_CHANNEL_L1, L1); ledcWrite(PWM_CHANNEL_L2, L2);
}

void moveForward()      { _setMotorPWM(VEL_MAXIMA, 0, VEL_MAXIMA, 0); }
void moveBackward()     { _setMotorPWM(0, VEL_MAXIMA, 0, VEL_MAXIMA); }
void turnSlightRight()  { _setMotorPWM(VEL_CURVA_SUAVE, 0, VEL_MAXIMA, 0); }
void turnSlightLeft()   { _setMotorPWM(VEL_MAXIMA, 0, VEL_CURVA_SUAVE, 0); }
void turnSharpRight()   { _setMotorPWM(0, VEL_CURVA_FECHADA, VEL_CURVA_FECHADA, 0); }
void turnSharpLeft()    { _setMotorPWM(VEL_CURVA_FECHADA, 0, 0, VEL_CURVA_FECHADA); }
void stopMotors()       { _setMotorPWM(0, 0, 0, 0); }