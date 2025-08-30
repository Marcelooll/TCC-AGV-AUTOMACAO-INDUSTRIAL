// ===================================================================================
//
//      PROJETO FINAL AGV - TCC | VERSÃO SUPER DIDÁTICA
//      Data: 27/08/2025
//
//      Este código controla um robô seguidor de linha com desvio de obstáculos.
//      Ele foi escrito com comentários super detalhados para ajudar iniciantes
//      a entender não só a lógica do robô, mas também conceitos básicos de
//      programação na linguagem C/C++ (usada pelo Arduino/ESP32).
//
// ===================================================================================

// ===================================================================================
//  SEÇÃO DE CALIBRAÇÃO E AJUSTES
//  Aqui ficam todas as variáveis que você provavelmente precisará ajustar.
// ===================================================================================

// --- Parâmetros de Velocidade ---
// 'const' significa que o valor desta variável não pode ser alterado durante a execução do programa.
// 'int' é um tipo de variável para números inteiros.
const int VEL_MAXIMA = 200;       // Velocidade principal do robô (valor entre 0 e 255, que é o máximo do PWM de 8 bits).
const int VEL_BASE_CURVA = 180;   // Velocidade usada ao fazer curvas fechadas (giro no próprio eixo).

// --- Sintonia do Controlador de Linha (P-Control) ---
// 'float' é um tipo de variável para números com casas decimais (ex: 3.14).
// O 'KP' (Ganho Proporcional) é o ajuste mais importante para o seguidor de linha.
// - Se o robô oscila muito (ziguezagueia) na linha, diminua o KP.
// - Se o robô faz as curvas de forma muito lenta ou aberta, aumente o KP.
const float KP = 0.08;

// --- Parâmetros de Navegação e Sensores ---
const int OBSTACLE_DISTANCE_CM = 20;  // Distância em cm que o robô detecta um obstáculo e inicia o desvio.
const int NUM_US_READINGS = 5;        // O sensor ultrassônico fará 5 leituras para tirar a média e evitar erros.
const int NO_OBSTACLE_DETECTED = 999; // Um valor alto para representar "nenhum obstáculo".

// --- Tempos da Manobra de Desvio (em milissegundos) ---
// Ajuste estes tempos para que o robô faça a manobra de desvio corretamente.
const int T_REVERSE = 300;          // Tempo que o robô anda de ré.
const int T_TURN_90 = 500;          // Tempo para girar 90 graus.
const int T_PASS_OBSTACLE = 1000;   // Tempo para passar ao lado do obstáculo.
const int T_REALIGN = 1200;         // Tempo máximo para procurar a linha antes de se considerar perdido.

// ===================================================================================
//  DEFINIÇÕES DE HARDWARE E ESTADOS (Geralmente não precisa mexer aqui)
// ===================================================================================

// --- Pinos do Hardware ---
const int irPin1 = 23, irPin2 = 22, irPin3 = 21, irPin4 = 19, irPin5 = 18; // Sensores infravermelhos (de linha)

// '[]' indica que a variável é um "array" (uma coleção de valores do mesmo tipo).
// Agrupamos os pinos em um array para podermos usar loops (laços de repetição) para configurá-los e lê-los.
const int irPins[] = {irPin1, irPin2, irPin3, irPin4, irPin5};

const int echoPin = 2, trigPin = 15; // Sensor ultrassônico (de obstáculo)
const int motorR_IN1 = 13, motorR_IN2 = 12, motorL_IN3 = 32, motorL_IN4 = 14; // Pinos de controle dos motores
const int startButton = 27, stopButton = 25; // Botões de Start e Emergência
const int sinaleiroPowerPin = 26, sinaleiroStatusPin = 33; // LEDs de status

// --- Configuração do PWM (para controle de velocidade do ESP32) ---
const int PWM_FREQ = 5000, PWM_RESOLUTION = 8; // Frequência e Resolução do PWM. 8 bits = valores de 0 a 255.

// Canais de PWM do ESP32. Cada "ledc" (controlador de LED) pode gerar um sinal PWM independente.
const int PWM_CHANNEL_R1 = 0, PWM_CHANNEL_R2 = 1, PWM_CHANNEL_L1 = 2, PWM_CHANNEL_L2 = 3;

// --- Máquina de Estados Principal ---
// 'enum' (enumeração) cria um novo tipo de variável personalizado.
// Aqui, criamos o tipo 'SystemState' que só pode ter um dos valores definidos na lista.
// Isso organiza o código e evita erros, pois o robô só pode estar em um estado de cada vez.
enum SystemState { STATE_STOPPED, STATE_FOLLOWING_LINE, STATE_AVOIDING_OBSTACLE, STATE_EMERGENCY, STATE_LINE_LOST };
SystemState systemState = STATE_STOPPED; // O robô começa no estado "parado".

// --- Sub-estados para o desvio de obstáculo ---
enum AvoidState { IDLE, REVERSING, TURNING_RIGHT, GOING_AROUND, TURNING_LEFT, FINDING_LINE, REALIGNING_ON_LINE };
AvoidState avoidState = IDLE;
unsigned long avoidStepStartTime = 0; // 'unsigned long' é um tipo para números inteiros muito grandes e sem sinal (não negativos). Usado para tempo.

// --- Variáveis Globais ---
// 'volatile' é um qualificador especial que diz ao compilador que esta variável pode mudar a qualquer momento de forma inesperada.
// É OBRIGATÓRIO para variáveis que são modificadas dentro de uma função de interrupção (ISR).
// 'bool' é um tipo de variável que só pode ser 'true' (verdadeiro) ou 'false' (falso).
volatile bool emergencyFlag = false;

bool isRunning = false; // Controla se o robô está em modo "play" ou "pause".
unsigned long lastDebounceTime = 0; // Variáveis para o "debounce" do botão (evitar leituras duplas).
int lastStartButtonState = HIGH;
const long debounceDelay = 50;
int lastKnownError = 0; // Guarda a última posição da linha para saber para onde procurar se ela for perdida.

// ===================================================================================
//  FUNÇÃO DE INTERRUPÇÃO (ISR - Interrupt Service Routine)
// ===================================================================================
// 'void' significa que a função não retorna nenhum valor.
// 'IRAM_ATTR' é um atributo específico do ESP32 que armazena esta função na memória RAM, tornando-a muito mais rápida de ser acessada, o que é ideal para interrupções.
// Esta função é especial. Ela é chamada AUTOMATICAMENTE pelo hardware do ESP32
// no exato momento em que o pino do botão de emergência é ativado.
void IRAM_ATTR emergencyStopISR() {
  emergencyFlag = true; // Apenas levanta uma "bandeira". O loop principal fará o resto.
}

// ===================================================================================
//  SETUP - Executado uma única vez quando o robô liga
// ===================================================================================
void setup() {
  // Este é um "for-each loop". Ele passa por cada item do array 'irPins'.
  // Para cada 'int pin' dentro de 'irPins', ele executa o código no bloco.
  for (int pin : irPins) {
    // 'pinMode' configura um pino digital para ser uma ENTRADA (INPUT) ou SAÍDA (OUTPUT).
    pinMode(pin, INPUT);
  }

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // PULLUP significa que o pino já tem uma resistência interna que o mantém em nível ALTO (HIGH).
  // Quando o botão é pressionado, ele conecta o pino ao terra (GND), forçando a leitura para BAIXO (LOW).
  pinMode(startButton, INPUT_PULLUP);
  pinMode(stopButton, INPUT_PULLUP);

  // 'attachInterrupt' é a função que ativa a interrupção.
  // Argumentos:
  // 1. O pino que vai gerar a interrupção (convertido para o número da interrupção).
  // 2. A função que será chamada (nossa ISR).
  // 3. O modo de acionamento: RISING (na borda de subida, quando o sinal vai de LOW para HIGH).
  attachInterrupt(digitalPinToInterrupt(stopButton), emergencyStopISR, RISING);

  pinMode(sinaleiroPowerPin, OUTPUT);
  pinMode(sinaleiroStatusPin, OUTPUT);

  // 'ledcSetup' é uma função do ESP32 para configurar um canal de PWM.
  ledcSetup(PWM_CHANNEL_R1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_R2, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_L1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_L2, PWM_FREQ, PWM_RESOLUTION);

  // 'ledcAttachPin' conecta um pino físico a um canal de PWM já configurado.
  ledcAttachPin(motorR_IN1, PWM_CHANNEL_R1);
  ledcAttachPin(motorR_IN2, PWM_CHANNEL_R2);
  ledcAttachPin(motorL_IN3, PWM_CHANNEL_L1);
  ledcAttachPin(motorL_IN4, PWM_CHANNEL_L2);

  stopMotors();
  digitalWrite(sinaleiroPowerPin, HIGH);
  digitalWrite(sinaleiroStatusPin, LOW);
}

// ===================================================================================
//  LOOP - Executado repetidamente sem parar
// ===================================================================================
void loop() {
  handleEmergency();
  // 'return' dentro de uma função 'void' faz com que a função pare sua execução imediatamente.
  if (systemState == STATE_EMERGENCY) return;

  handleButtons();
  handleSinaleiro();

  // O operador '!' significa "NÃO". Então, '!isRunning' é o mesmo que 'isRunning == false'.
  if (isRunning) {
    if (systemState != STATE_AVOIDING_OBSTACLE && getFilteredDistance() <= OBSTACLE_DISTANCE_CM) {
      systemState = STATE_AVOIDING_OBSTACLE;
      avoidState = REVERSING;
      avoidStepStartTime = millis(); // 'millis()' retorna o número de milissegundos desde que o robô ligou.
    }

    // 'switch' é uma forma de substituir uma cadeia de 'if-else if-else'.
    // Ele verifica o valor de 'systemState' e executa o bloco de código ('case') correspondente.
    switch (systemState) {
      case STATE_FOLLOWING_LINE:
        followLine_PControl();
        break; // 'break' é essencial para sair do switch após um 'case' ser executado.
      case STATE_AVOIDING_OBSTACLE:
        handleObstacleAvoidance();
        break;
      case STATE_LINE_LOST:
        handleLineLost();
        break;
      case STATE_STOPPED:
        stopMotors();
        break;
    }
  } else {
    stopMotors();
    systemState = STATE_STOPPED;
  }
}

// ===================================================================================
//  FUNÇÕES DE LÓGICA (State Handlers)
// ===================================================================================

void handleEmergency() {
  if (emergencyFlag) {
    isRunning = false;
    systemState = STATE_EMERGENCY;
    stopMotors();
    // O operador '%' (módulo) retorna o resto de uma divisão.
    // (millis() / 100) % 2 resulta em 0 e 1 alternadamente, criando um efeito de pisca-pisca.
    digitalWrite(sinaleiroPowerPin, HIGH);
    digitalWrite(sinaleiroStatusPin, (millis() / 100) % 2);

    if (digitalRead(stopButton) == LOW) {
      emergencyFlag = false;
      systemState = STATE_STOPPED;
    }
  }
}

void handleSinaleiro() {
  if (systemState == STATE_EMERGENCY) return;

  if (isRunning) {
    bool blinkState = (millis() / 300) % 2;
    digitalWrite(sinaleiroPowerPin, blinkState);
    digitalWrite(sinaleiroStatusPin, blinkState);
  } else {
    digitalWrite(sinaleiroPowerPin, HIGH);
    digitalWrite(sinaleiroStatusPin, LOW);
  }
}

void handleButtons() {
  int reading = digitalRead(startButton);
  if (reading != lastStartButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading == LOW && lastStartButtonState == HIGH) {
      isRunning = !isRunning; // Inverte o valor booleano. true vira false, false vira true.
      if (isRunning) {
        systemState = STATE_FOLLOWING_LINE;
      } else {
        systemState = STATE_STOPPED;
      }
    }
  }
  lastStartButtonState = reading;
}

void handleObstacleAvoidance() {
  unsigned long currentTime = millis();
  switch (avoidState) {
    case REVERSING:
      moveBackward();
      if (currentTime - avoidStepStartTime > T_REVERSE) { avoidState = TURNING_RIGHT; avoidStepStartTime = currentTime; }
      break;
    case TURNING_RIGHT:
      turnSharpRight();
      if (currentTime - avoidStepStartTime > T_TURN_90) { avoidState = GOING_AROUND; avoidStepStartTime = currentTime; }
      break;
    case GOING_AROUND:
      moveForward();
      if (currentTime - avoidStepStartTime > T_PASS_OBSTACLE) { avoidState = TURNING_LEFT; avoidStepStartTime = currentTime; }
      break;
    case TURNING_LEFT:
      turnSharpLeft();
      if (currentTime - avoidStepStartTime > T_TURN_90) { avoidState = FINDING_LINE; avoidStepStartTime = currentTime; }
      break;
    case FINDING_LINE:
      moveForward();
      if (currentTime - avoidStepStartTime > T_REALIGN) { systemState = STATE_LINE_LOST; avoidState = IDLE; break; }
      // O operador '||' significa "OU". A condição é verdadeira se QUALQUER uma das partes for verdadeira.
      if (!digitalRead(irPin2) || !digitalRead(irPin3) || !digitalRead(irPin4)) { avoidState = REALIGNING_ON_LINE; avoidStepStartTime = currentTime; }
      break;
    case REALIGNING_ON_LINE:
      turnSharpLeft();
      if (currentTime - avoidStepStartTime > T_TURN_90) { avoidState = IDLE; systemState = STATE_FOLLOWING_LINE; }
      break;
  }
}

void handleLineLost() {
    if (lastKnownError > 0) {
        turnSharpRight();
    } else {
        turnSharpLeft();
    }

    bool lineFound = false;
    for (int pin : irPins) {
        if (digitalRead(pin) == LOW) { lineFound = true; break; }
    }

    if (lineFound) {
        systemState = STATE_FOLLOWING_LINE;
    }
}

// ===================================================================================
//  FUNÇÕES DE NAVEGAÇÃO E SENSORES
// ===================================================================================

void followLine_PControl() {
  bool irReadings[5];
  int activeSensorCount = 0;
  float weightedSum = 0;

  for (int i = 0; i < 5; i++) {
    irReadings[i] = !digitalRead(irPins[i]);
    if (irReadings[i]) {
      // O operador '+=' é um atalho. 'x += y' é o mesmo que 'x = x + y'.
      weightedSum += i * 1000;
      activeSensorCount++;
    }
  }

  if (activeSensorCount == 0) {
    systemState = STATE_LINE_LOST;
    return;
  }

  float position = weightedSum / activeSensorCount;
  int error = position - 2000;
  lastKnownError = error;

  int adjustment = KP * error;

  int leftSpeed = constrain(VEL_MAXIMA - adjustment, 0, VEL_MAXIMA);
  int rightSpeed = constrain(VEL_MAXIMA + adjustment, 0, VEL_MAXIMA);

  _setMotorPWM(rightSpeed, 0, leftSpeed, 0);
}

int getFilteredDistance() {
  int readings[NUM_US_READINGS];
  for (int i = 0; i < NUM_US_READINGS; i++) {
    readings[i] = getRawDistance();
    delayMicroseconds(200);
  }

  for (int i = 1; i < NUM_US_READINGS; i++) {
    int key = readings[i];
    int j = i - 1;
    // O operador '&&' significa "E". A condição é verdadeira se AMBAS as partes forem verdadeiras.
    while (j >= 0 && readings[j] > key) {
      readings[j + 1] = readings[j];
      j = j - 1;
    }
    readings[j + 1] = key;
  }
  return readings[NUM_US_READINGS / 2];
}

int getRawDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // 'long' é um tipo para números inteiros maiores que 'int'.
  long raw_duration = pulseIn(echoPin, HIGH, 25000);

  // Este é um "operador ternário", um atalho para um if-else.
  // A sintaxe é: (condição) ? (valor se verdadeiro) : (valor se falso).
  // Lê-se: "a duração é zero? Se sim, retorne NO_OBSTACLE_DETECTED. Se não, retorne o cálculo da distância."
  return (raw_duration == 0) ? NO_OBSTACLE_DETECTED : raw_duration * 0.034 / 2;
}

// ===================================================================================
//  FUNÇÕES DE CONTROLE DOS MOTORES (Baixo Nível)
// ===================================================================================

// O '_' no início do nome da função é uma convenção para indicar que esta é uma
// função "interna" ou de "baixo nível", que não deve ser chamada diretamente pela lógica principal.
void _setMotorPWM(int R1, int R2, int L1, int L2) {
  // 'ledcWrite' envia o valor de PWM (0-255) para o canal especificado.
  ledcWrite(PWM_CHANNEL_R1, R1);
  ledcWrite(PWM_CHANNEL_R2, R2);
  ledcWrite(PWM_CHANNEL_L1, L1);
  ledcWrite(PWM_CHANNEL_L2, L2);
}

// Funções de movimento "abstraídas". Elas simplificam o código principal.
// Em vez de chamar '_setMotorPWM' com 4 números, chamamos 'moveForward()', que é mais fácil de ler.
void moveForward()      { _setMotorPWM(VEL_MAXIMA, 0, VEL_MAXIMA, 0); }
void moveBackward()     { _setMotorPWM(0, VEL_MAXIMA, 0, VEL_MAXIMA); }
void turnSharpRight()   { _setMotorPWM(VEL_BASE_CURVA, 0, 0, VEL_BASE_CURVA); }
void turnSharpLeft()    { _setMotorPWM(0, VEL_BASE_CURVA, VEL_BASE_CURVA, 0); }
void stopMotors()       { _setMotorPWM(0, 0, 0, 0); }
