/**
 * @file agv_main_production.ino
 * @brief Sistema de controle para AGV (Automated Guided Vehicle) com ESP32
 * @author Marcelol
 * @date 11/09/2025
 * @version 2.0
 *
 * @details
 * Sistema completo de controle para veículo guiado automaticamente incluindo:
 * - Seguidor de linha usando 5 sensores infravermelhos TCRT5000
 * - Desvio automático de obstáculos com sensor ultrassônico HC-SR04
 * - Controle de dois motores DC via drivers BTS7960
 * - Máquina de estados finita para gerenciamento de comportamento
 * - Interface de controle via botões START/STOP com segurança
 * - Sinalização visual de status via LEDs
 *
 * Características de segurança:
 * - Botão STOP com prioridade absoluta e resposta em tempo real
 * - Sistema de armar em dois estágios para prevenir partida acidental
 * - Timeouts e limitações para prevenir comportamento errático
 * - Debounce em software para entradas de botões
 * - Validação de limites em todas as saídas PWM
 */

// =============================================================================
// SECAO 1: CONFIGURACAO DE HARDWARE
// =============================================================================

// Pinos do motor direito (M1) - Driver BTS7960
const int M1_RPWM_PIN = 25;  // PWM para rotacao para frente
const int M1_LPWM_PIN = 26;  // PWM para rotacao para tras
const int M1_EN_R_PIN = 33;  // Enable para rotacao para frente
const int M1_EN_L_PIN = 32;  // Enable para rotacao para tras

// Pinos do motor esquerdo (M2) - Driver BTS7960
const int M2_RPWM_PIN = 19;
const int M2_LPWM_PIN = 18;
const int M2_EN_R_PIN = 21;
const int M2_EN_L_PIN = 23;

// Pinos do sensor ultrassonico HC-SR04
const int ULTRA_TRIG_PIN = 5;   // Pino de disparo do pulso
const int ULTRA_ECHO_PIN = 17;  // Pino de recepcao do eco

// Pinos dos sensores infravermelhos TCRT5000
// Distribuicao fisica: [IR1][IR2][IR3][IR4][IR5]
//                      Esquerda <- Centro -> Direita
const int IR_PIN_1 = 39;  // Extrema esquerda
const int IR_PIN_2 = 34;  // Meio esquerda
const int IR_PIN_3 = 36;  // Centro
const int IR_PIN_4 = 35;  // Meio direita
const int IR_PIN_5 = 4;   // Extrema direita
const int IR_PINS[5] = {IR_PIN_1, IR_PIN_2, IR_PIN_3, IR_PIN_4, IR_PIN_5};

// Pinos de interface com usuario
const int BUTTON_START_PIN = 16;  // Botao START (contato NA - Normalmente Aberto)
const int BUTTON_STOP_PIN = 22;   // Botao STOP (contato NF - Normalmente Fechado)
const int LED_IDLE_PIN = 13;      // LED de status: armado aguardando
const int LED_RUN_1_PIN = 14;     // LED de status: operacao ativa
const int LED_RUN_2_PIN = 27;     // LED de status: operacao ativa

// =============================================================================
// SECAO 2: PARAMETROS DE CONFIGURACAO
// =============================================================================

// Parametros de controle de movimento
const int OBSTACLE_DISTANCE_CM = 20;     // Distancia minima para deteccao de obstaculo
const int BASE_SPEED = 150;              // Velocidade base dos motores (0-255)
const float KP = 40.0;                   // Ganho proporcional do controlador P
const int MAX_MOTOR_SPEED = 255;         // Velocidade maxima permitida
const int MIN_MOTOR_SPEED = -255;        // Velocidade minima permitida (reverso)

// Parametros de temporização
const unsigned long LED_BLINK_INTERVAL = 500;        // Periodo de piscada dos LEDs (ms)
const unsigned long DEBOUNCE_DELAY = 50;             // Tempo de debounce para botoes (ms)
const unsigned long STABILIZATION_DELAY = 300;       // Tempo de estabilizacao apos parada (ms)
const unsigned long TURN_DURATION = 1000;            // Duracao estimada de giro de 90 graus (ms)
const unsigned long FORWARD_DURATION = 1500;         // Duracao de avanco durante desvio (ms)
const unsigned long LINE_SEARCH_TIMEOUT = 5000;      // Timeout para busca de linha (ms)
const unsigned long OBSTACLE_CONFIRM_TIME = 200;     // Tempo para confirmar obstaculo (ms)

// Parametros do sensor ultrassonico
const int ULTRASONIC_READINGS = 3;           // Numero de leituras para media
const unsigned long ULTRASONIC_TIMEOUT = 30000;  // Timeout do pulseIn em microsegundos

// =============================================================================
// SECAO 3: TIPOS E ESTRUTURAS DE DADOS
// =============================================================================

// Estados da maquina de estados finita do AGV
enum AGVState {
  STATE_STOPPED,           // Sistema desligado, motores parados
  STATE_IDLE,              // Sistema armado, aguardando comando de inicio
  STATE_FOLLOWING_LINE,    // Modo operacional: seguindo linha
  STATE_AVOIDING_OBSTACLE  // Modo operacional: executando desvio de obstaculo
};

// Fases da manobra de desvio de obstaculo
enum AvoidancePhase {
  AVOID_TURN_RIGHT,    // Fase 1: Girar 90 graus para direita
  AVOID_MOVE_FORWARD,  // Fase 2: Avancar contornando obstaculo
  AVOID_TURN_LEFT,     // Fase 3: Girar 90 graus para esquerda
  AVOID_FIND_LINE      // Fase 4: Buscar e retornar a linha
};

// =============================================================================
// SECAO 4: VARIAVEIS GLOBAIS
// =============================================================================

// Estado atual do sistema
AGVState currentState = STATE_STOPPED;

// Variaveis de controle de LED
unsigned long lastBlinkTime = 0;
bool ledState = false;

// Variaveis de debounce para botoes
unsigned long lastStartPress = 0;
unsigned long lastStopPress = 0;
bool startButtonLastState = HIGH;
bool stopButtonLastState = HIGH;

// Variaveis de controle de desvio de obstaculo
AvoidancePhase avoidancePhase = AVOID_TURN_RIGHT;
unsigned long avoidancePhaseStartTime = 0;
bool isAvoiding = false;

// Variaveis de estabilizacao
unsigned long stabilizationStartTime = 0;
bool isStabilizing = false;

// Variaveis de deteccao de obstaculo
unsigned long obstacleFirstDetected = 0;
bool obstacleConfirmed = false;

// =============================================================================
// SECAO 5: FUNCOES DE INICIALIZACAO
// =============================================================================

void setup() {
  Serial.begin(115200);
  Serial.println("===========================================");
  Serial.println("AGV Control System v2.0");
  Serial.println("Inicializando sistema...");
  Serial.println("===========================================");

  configurePins();
  enableMotorDrivers();
  
  Serial.println("Sistema inicializado com sucesso");
  Serial.println("Estado: STOPPED - Pressione START para armar");
  Serial.println("===========================================");
}

/**
 * Configura todos os pinos do microcontrolador
 * Define modo INPUT/OUTPUT e ativa pull-ups internos quando necessario
 */
void configurePins() {
  // Configuracao dos pinos dos motores
  pinMode(M1_RPWM_PIN, OUTPUT);
  pinMode(M1_LPWM_PIN, OUTPUT);
  pinMode(M1_EN_R_PIN, OUTPUT);
  pinMode(M1_EN_L_PIN, OUTPUT);
  pinMode(M2_RPWM_PIN, OUTPUT);
  pinMode(M2_LPWM_PIN, OUTPUT);
  pinMode(M2_EN_R_PIN, OUTPUT);
  pinMode(M2_EN_L_PIN, OUTPUT);

  // Configuracao dos pinos do sensor ultrassonico
  pinMode(ULTRA_TRIG_PIN, OUTPUT);
  pinMode(ULTRA_ECHO_PIN, INPUT);

  // Configuracao dos pinos dos sensores infravermelhos
  for (int i = 0; i < 5; i++) {
    pinMode(IR_PINS[i], INPUT);
  }

  // Configuracao dos botoes com pull-up interno
  // Pull-up mantem o pino em HIGH quando botao nao pressionado
  pinMode(BUTTON_START_PIN, INPUT_PULLUP);
  pinMode(BUTTON_STOP_PIN, INPUT_PULLUP);

  // Configuracao dos LEDs de status
  pinMode(LED_IDLE_PIN, OUTPUT);
  pinMode(LED_RUN_1_PIN, OUTPUT);
  pinMode(LED_RUN_2_PIN, OUTPUT);
  
  // Garante que todos os LEDs iniciam apagados
  digitalWrite(LED_IDLE_PIN, LOW);
  digitalWrite(LED_RUN_1_PIN, LOW);
  digitalWrite(LED_RUN_2_PIN, LOW);
  
  Serial.println("Pinos configurados");
}

/**
 * Habilita os drivers de motor BTS7960
 * Os pinos EN (enable) devem estar em HIGH para os drivers funcionarem
 */
void enableMotorDrivers() {
  digitalWrite(M1_EN_R_PIN, HIGH);
  digitalWrite(M1_EN_L_PIN, HIGH);
  digitalWrite(M2_EN_R_PIN, HIGH);
  digitalWrite(M2_EN_L_PIN, HIGH);
  Serial.println("Drivers de motor habilitados");
}

// =============================================================================
// SECAO 6: FUNCOES DE CONTROLE DE MOTOR
// =============================================================================

/**
 * Define a velocidade e direcao de ambos os motores
 * 
 * @param leftSpeed Velocidade do motor esquerdo (-255 a +255)
 *                  Positivo = frente, Negativo = re, Zero = parado
 * @param rightSpeed Velocidade do motor direito (-255 a +255)
 * 
 * A funcao automaticamente limita os valores ao intervalo valido
 * e controla os sinais PWM apropriados para cada direcao
 */
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  // Limita velocidades ao intervalo valido para prevenir overflow
  leftSpeed = constrain(leftSpeed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);
  rightSpeed = constrain(rightSpeed, MIN_MOTOR_SPEED, MAX_MOTOR_SPEED);

  // Controle do motor esquerdo
  if (leftSpeed >= 0) {
    // Rotacao para frente: RPWM ativo, LPWM desligado
    analogWrite(M2_RPWM_PIN, leftSpeed);
    analogWrite(M2_LPWM_PIN, 0);
  } else {
    // Rotacao para tras: LPWM ativo, RPWM desligado
    // Usa valor absoluto do numero negativo
    analogWrite(M2_RPWM_PIN, 0);
    analogWrite(M2_LPWM_PIN, -leftSpeed);
  }

  // Controle do motor direito (mesma logica)
  if (rightSpeed >= 0) {
    analogWrite(M1_RPWM_PIN, rightSpeed);
    analogWrite(M1_LPWM_PIN, 0);
  } else {
    analogWrite(M1_RPWM_PIN, 0);
    analogWrite(M1_LPWM_PIN, -rightSpeed);
  }
}

/**
 * Para ambos os motores imediatamente
 * Envia comando de velocidade zero para os dois motores
 */
void stopMotors() {
  setMotorSpeeds(0, 0);
}

/**
 * Inicia sequencia de parada com estabilizacao
 * Para os motores e marca inicio do periodo de estabilizacao
 */
void initiateStabilization() {
  stopMotors();
  isStabilizing = true;
  stabilizationStartTime = millis();
}

/**
 * Verifica se o periodo de estabilizacao foi concluido
 * 
 * @return true se ja estabilizou ou nao estava estabilizando
 *         false se ainda esta no periodo de estabilizacao
 */
bool isStabilized() {
  if (!isStabilizing) {
    return true;
  }
  
  if (millis() - stabilizationStartTime >= STABILIZATION_DELAY) {
    isStabilizing = false;
    return true;
  }
  
  return false;
}

// =============================================================================
// SECAO 7: FUNCOES DE LEITURA DE SENSORES
// =============================================================================

/**
 * Le a distancia do sensor ultrassonico HC-SR04
 * Realiza multiplas leituras e calcula a media para maior precisao
 * 
 * @return Distancia em centimetros ate o obstaculo mais proximo
 *         Retorna 999 em caso de erro ou timeout
 */
long readUltrasonicDistance() {
  long sum = 0;
  int validReadings = 0;

  // Realiza multiplas leituras para filtrar ruido
  for (int i = 0; i < ULTRASONIC_READINGS; i++) {
    // Limpa o pino TRIG
    digitalWrite(ULTRA_TRIG_PIN, LOW);
    delayMicroseconds(2);
    
    // Envia pulso de 10us para disparar medicao
    digitalWrite(ULTRA_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(ULTRA_TRIG_PIN, LOW);
    
    // Le o tempo que o pino ECHO fica em HIGH
    // Timeout previne travamento se nao houver eco
    long duration = pulseIn(ULTRA_ECHO_PIN, HIGH, ULTRASONIC_TIMEOUT);
    
    if (duration > 0) {
      // Converte tempo em distancia: velocidade do som = 340m/s = 0.034cm/us
      // Divide por 2 pois o som faz ida e volta
      long distance = duration * 0.034 / 2;
      
      // Filtra leituras absurdas (menos de 2cm ou mais de 400cm)
      if (distance >= 2 && distance <= 400) {
        sum += distance;
        validReadings++;
      }
    }
    
    // Pequeno delay entre leituras
    if (i < ULTRASONIC_READINGS - 1) {
      delay(10);
    }
  }

  // Retorna media das leituras validas ou valor de erro
  if (validReadings > 0) {
    return sum / validReadings;
  }
  return 999;  // Codigo de erro
}

/**
 * Le os sensores de linha e calcula a posicao da linha em relacao ao AGV
 * Usa calculo de media ponderada para determinar posicionamento
 * 
 * @return Valor de -2.0 a +2.0 indicando posicao da linha
 *         -2.0 = linha totalmente a esquerda
 *          0.0 = linha centralizada
 *         +2.0 = linha totalmente a direita
 *         1000.0 = linha nao detectada (codigo de erro)
 * 
 * Sistema de pesos:
 *   Sensor 1 (esquerda):  -2
 *   Sensor 2:             -1
 *   Sensor 3 (centro):     0
 *   Sensor 4:             +1
 *   Sensor 5 (direita):   +2
 */
float readLinePosition() {
  bool irValues[5];
  int onLineCount = 0;
  float weightedSum = 0;

  // Le todos os sensores
  // LOW indica deteccao de superficie escura (linha preta)
  // HIGH indica superficie clara (chao branco)
  for (int i = 0; i < 5; i++) {
    irValues[i] = (digitalRead(IR_PINS[i]) == LOW);
    if (irValues[i]) {
      onLineCount++;
    }
  }

  // Se pelo menos um sensor detectou a linha
  if (onLineCount > 0) {
    // Calcula posicao ponderada
    // Cada sensor contribui com seu peso multiplicado por seu estado
    weightedSum = (irValues[0] * (-2.0) + 
                   irValues[1] * (-1.0) + 
                   irValues[2] * (0.0) + 
                   irValues[3] * (1.0) + 
                   irValues[4] * (2.0));
    
    // Divide pela quantidade de sensores ativos para normalizar
    return weightedSum / onLineCount;
  }

  // Nenhum sensor detectou linha
  return 1000.0;
}

// =============================================================================
// SECAO 8: FUNCOES DE INTERFACE COM USUARIO
// =============================================================================

/**
 * Le o estado do botao START com debounce em software
 * Debounce previne leituras multiplas de um unico pressionamento
 * 
 * @return true se botao foi pressionado (transicao HIGH -> LOW detectada)
 *         false caso contrario
 */
bool isStartButtonPressed() {
  bool currentState = digitalRead(BUTTON_START_PIN);
  bool pressed = false;

  // Detecta transicao de HIGH para LOW (botao pressionado)
  if (startButtonLastState == HIGH && currentState == LOW) {
    // Verifica se passou tempo suficiente desde ultimo pressionamento
    if (millis() - lastStartPress >= DEBOUNCE_DELAY) {
      pressed = true;
      lastStartPress = millis();
    }
  }

  startButtonLastState = currentState;
  return pressed;
}

/**
 * Le o estado do botao STOP com debounce em software
 * STOP usa contato NF (Normalmente Fechado) para seguranca:
 * - Nao pressionado: circuito fechado -> pino le LOW
 * - Pressionado: circuito aberto -> pino le HIGH
 * - Fio rompido: circuito aberto -> pino le HIGH (parada automatica)
 * 
 * @return true se botao foi pressionado ou fio rompido
 *         false caso contrario
 */
bool isStopButtonPressed() {
  bool currentState = digitalRead(BUTTON_STOP_PIN);
  bool pressed = false;

  // Para botao STOP (NF), HIGH indica pressionamento
  if (stopButtonLastState == LOW && currentState == HIGH) {
    if (millis() - lastStopPress >= DEBOUNCE_DELAY) {
      pressed = true;
      lastStopPress = millis();
    }
  }

  stopButtonLastState = currentState;
  return pressed;
}

/**
 * Atualiza o estado dos LEDs baseado no estado atual do sistema
 * LEDs piscam em diferentes padroes para indicar cada estado
 */
void updateLEDs() {
  unsigned long currentTime = millis();

  // Atualiza estado de piscada baseado no intervalo configurado
  if (currentTime - lastBlinkTime >= LED_BLINK_INTERVAL) {
    lastBlinkTime = currentTime;
    ledState = !ledState;
  }

  // Define padrao de LEDs para cada estado
  switch (currentState) {
    case STATE_STOPPED:
      // Todos LEDs apagados: sistema desligado
      digitalWrite(LED_IDLE_PIN, LOW);
      digitalWrite(LED_RUN_1_PIN, LOW);
      digitalWrite(LED_RUN_2_PIN, LOW);
      break;

    case STATE_IDLE:
      // LED IDLE piscando: sistema armado, aguardando inicio
      digitalWrite(LED_IDLE_PIN, ledState);
      digitalWrite(LED_RUN_1_PIN, LOW);
      digitalWrite(LED_RUN_2_PIN, LOW);
      break;

    case STATE_FOLLOWING_LINE:
    case STATE_AVOIDING_OBSTACLE:
      // LEDs RUN piscando: sistema em operacao
      digitalWrite(LED_IDLE_PIN, LOW);
      digitalWrite(LED_RUN_1_PIN, ledState);
      digitalWrite(LED_RUN_2_PIN, ledState);
      break;
  }
}

// =============================================================================
// SECAO 9: LOGICA DE CONTROLE - SEGUIDOR DE LINHA
// =============================================================================

/**
 * Implementa controlador proporcional (P) para seguir linha
 * Le sensores, calcula erro, aplica correcao e ajusta motores
 * Tambem monitora obstaculos a frente
 */
void handleFollowingLine() {
  // Verifica presenca de obstaculo
  long distance = readUltrasonicDistance();
  
  if (distance < OBSTACLE_DISTANCE_CM && distance > 0) {
    // Primeira deteccao ou continuacao de deteccao
    if (!obstacleConfirmed) {
      if (obstacleFirstDetected == 0) {
        obstacleFirstDetected = millis();
      } else if (millis() - obstacleFirstDetected >= OBSTACLE_CONFIRM_TIME) {
        // Obstaculo confirmado apos tempo de verificacao
        obstacleConfirmed = true;
        Serial.println("Obstaculo confirmado - Iniciando desvio");
        currentState = STATE_AVOIDING_OBSTACLE;
        avoidancePhase = AVOID_TURN_RIGHT;
        avoidancePhaseStartTime = millis();
        isAvoiding = true;
        initiateStabilization();
        return;
      }
    }
  } else {
    // Reset de variaveis se obstaculo nao mais detectado
    obstacleFirstDetected = 0;
    obstacleConfirmed = false;
  }

  // Le posicao da linha
  float error = readLinePosition();

  if (error == 1000.0) {
    // Linha perdida: para o veiculo
    stopMotors();
    Serial.println("AVISO: Linha perdida - AGV parado");
  } else {
    // Calcula correcao proporcional
    // Quanto maior o erro, maior a correcao aplicada
    int correction = (int)(KP * error);

    // Aplica correcao diferencial aos motores
    // Se erro positivo (linha a direita): motor direito acelera
    // Se erro negativo (linha a esquerda): motor esquerdo acelera
    int leftSpeed = BASE_SPEED - correction;
    int rightSpeed = BASE_SPEED + correction;

    setMotorSpeeds(leftSpeed, rightSpeed);
  }
}

// =============================================================================
// SECAO 10: LOGICA DE CONTROLE - DESVIO DE OBSTACULO
// =============================================================================

/**
 * Executa manobra de desvio de obstaculo em multiplas fases
 * Usa maquina de estados nao-bloqueante para permitir interrupcao
 * 
 * Sequencia de manobra:
 * 1. Girar 90 graus para direita
 * 2. Avancar contornando obstaculo
 * 3. Girar 90 graus para esquerda
 * 4. Buscar linha e retornar ao modo de seguir linha
 */
void handleAvoidingObstacle() {
  // Permite interrupcao por STOP em qualquer fase
  if (!isStabilized()) {
    return;
  }

  unsigned long elapsed = millis() - avoidancePhaseStartTime;

  switch (avoidancePhase) {
    case AVOID_TURN_RIGHT:
      // Fase 1: Giro de 90 graus para direita
      // Motor esquerdo para frente, motor direito para tras = giro no eixo
      setMotorSpeeds(BASE_SPEED, -BASE_SPEED);
      
      if (elapsed >= TURN_DURATION) {
        Serial.println("Desvio: Fase 1 concluida - Avancando");
        avoidancePhase = AVOID_MOVE_FORWARD;
        avoidancePhaseStartTime = millis();
      }
      break;

    case AVOID_MOVE_FORWARD:
      // Fase 2: Avanco lateral ao obstaculo
      setMotorSpeeds(BASE_SPEED, BASE_SPEED);
      
      // Monitora se ainda ha obstaculo durante avanco
      if (readUltrasonicDistance() < OBSTACLE_DISTANCE_CM / 2) {
        // Obstaculo muito proximo, pode ser colisao iminente
        Serial.println("ALERTA: Obstaculo muito proximo durante desvio");
      }
      
      if (elapsed >= FORWARD_DURATION) {
        Serial.println("Desvio: Fase 2 concluida - Girando esquerda");
        avoidancePhase = AVOID_TURN_LEFT;
        avoidancePhaseStartTime = millis();
      }
      break;

    case AVOID_TURN_LEFT:
      // Fase 3: Giro de 90 graus para esquerda
      // Retorna a direcao original da trajetoria
      setMotorSpeeds(-BASE_SPEED, BASE_SPEED);
      
      if (elapsed >= TURN_DURATION) {
        Serial.println("Desvio: Fase 3 concluida - Buscando linha");
        avoidancePhase = AVOID_FIND_LINE;
        avoidancePhaseStartTime = millis();
      }
      break;

    case AVOID_FIND_LINE:
      // Fase 4: Busca e retorno a linha
      setMotorSpeeds(BASE_SPEED, BASE_SPEED);
      
      // Verifica se linha foi encontrada
      if (readLinePosition() != 1000.0) {
        Serial.println("Desvio concluido - Linha encontrada");
        currentState = STATE_FOLLOWING_LINE;
        isAvoiding = false;
        obstacleFirstDetected = 0;
        obstacleConfirmed = false;
        return;
      }
      
      // Timeout de seguranca: se nao encontrar linha em tempo maximo
      if (elapsed >= LINE_SEARCH_TIMEOUT) {
        Serial.println("ERRO: Timeout na busca de linha - Parando sistema");
        currentState = STATE_STOPPED;
        stopMotors();
        isAvoiding = false;
        return;
      }
      break;
  }
}

// =============================================================================
// SECAO 11: GERENCIAMENTO DE ESTADOS
// =============================================================================

/**
 * Processa comandos dos botoes e gerencia transicoes de estado
 * Implementa logica de seguranca com sistema de armar em dois estagios
 */
void processButtonCommands() {
  // STOP tem prioridade absoluta
  // Pode parar o sistema em qualquer estado
  if (isStopButtonPressed()) {
    if (currentState != STATE_STOPPED) {
      Serial.println("Comando STOP recebido - Sistema desarmado");
      currentState = STATE_STOPPED;
      initiateStabilization();
      isAvoiding = false;
      obstacleFirstDetected = 0;
      obstacleConfirmed = false;
    }
    return;  // Nao processa START se STOP foi pressionado
  }

  // Processa botao START baseado no estado atual
  if (isStartButtonPressed()) {
    switch (currentState) {
      case STATE_STOPPED:
        // Primeiro estagio: armar sistema
        Serial.println("Sistema armado - Pressione START novamente para iniciar");
        currentState = STATE_IDLE;
        break;

      case STATE_IDLE:
        // Segundo estagio: iniciar operacao
        Serial.println("Iniciando operacao - Seguindo linha");
        currentState = STATE_FOLLOWING_LINE;
        break;

      case STATE_FOLLOWING_LINE:
      case STATE_AVOIDING_OBSTACLE:
        // START nao tem efeito durante operacao
        break;
    }
  }
}

// =============================================================================
// SECAO 12: LOOP PRINCIPAL
// =============================================================================

/**
 * Loop principal do sistema
 * Executa continuamente verificando botoes, executando logica de estado
 * e atualizando interface visual
 * 
 * Ordem de execucao:
 * 1. Processa comandos de botoes (incluindo verificacao de STOP)
 * 2. Verifica se sistema esta em estabilizacao
 * 3. Executa logica do estado atual
 * 4. Atualiza LEDs de status
 * 5. Delay curto para controle de frequencia
 */
void loop() {
  // Processa entrada do usuario
  processButtonCommands();

  // Se estiver estabilizando, aguarda antes de executar outras acoes
  if (isStabilizing && !isStabilized()) {
    updateLEDs();
    delay(10);
    return;
  }

  // Executa logica baseada no estado atual
  switch (currentState) {
    case STATE_STOPPED:
      // Sistema desligado, nao executa nenhuma acao
      break;

    case STATE_IDLE:
      // Sistema armado, aguardando comando de inicio
      break;

    case STATE_FOLLOWING_LINE:
      // Modo operacional: seguir linha
      handleFollowingLine();
      break;

    case STATE_AVOIDING_OBSTACLE:
      // Modo operacional: desviar de obstaculo
      handleAvoidingObstacle();
      break;
  }

  // Atualiza indicadores visuais
  updateLEDs();

  // Delay para controlar frequencia do loop
  // ~100Hz de taxa de atualizacao
  delay(10);
}