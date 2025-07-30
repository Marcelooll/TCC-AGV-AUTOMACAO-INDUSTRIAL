// Definições de hardware para o AGV

// --- Alimentação ---
// ESP32 alimentado por um step down buck converter LM2596
// OUT conectado ao GND do ESP
// IN conectado ao 3V3 do ESP

// --- Sensores ---
// Sensor IR de 5 canais
const int irPin1 = 23; // OUT 1
const int irPin2 = 22; // OUT 2
const int irPin3 = 21; // OUT 3
const int irPin4 = 19; // OUT 4
const int irPin5 = 18; // OUT 5

// Sensor ultrassônico HC-SR04
const int echoPin = 2;  // Echo
const int trigPin = 15; // Trig

// --- Motores (via pontes H) ---
// Primeira ponte H (roda direita)
const int motorR_IN1 = 13; // Pin 1 (IN1)
const int motorR_IN2 = 12; // Pin 5 (IN2)

// Segunda ponte H (roda esquerda)
const int motorL_IN3 = 35; // Pin 2 (IN3)
const int motorL_IN4 = 34; // Pin 3 (IN4)

// --- Botões ---
const int startButton = 27; // Botão normalmente aberto (NA)
const int stopButton = 25;  // Botão normalmente fechado (NF)

// Variáveis para o sensor ultrassônico
long duration;
int distance;

// Estado do robô
bool isRunning = false;

void setup() {
  Serial.begin(115200);

  // Configuração dos pinos dos sensores
  pinMode(irPin1, INPUT);
  pinMode(irPin2, INPUT);
  pinMode(irPin3, INPUT);
  pinMode(irPin4, INPUT);
  pinMode(irPin5, INPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  // Configuração dos pinos dos motores
  pinMode(motorR_IN1, OUTPUT);
  pinMode(motorR_IN2, OUTPUT);
  pinMode(motorL_IN3, OUTPUT);
  pinMode(motorL_IN4, OUTPUT);

  // Configuração dos pinos dos botões
  pinMode(startButton, INPUT_PULLUP); // Usa resistor de pull-up interno
  pinMode(stopButton, INPUT_PULLUP);  // Usa resistor de pull-up interno

  // Mensagem de inicialização
  Serial.println("AGV inicializado. Pressione o botão START para começar.");
  stopMotors();
}

void loop() {
  // Botão de emergência (NF) - para o robô se o circuito for aberto
  if (digitalRead(stopButton) == HIGH) {
    isRunning = false;
    stopMotors();
    Serial.println("Botão de emergência acionado. AGV parado.");
    // Trava a execução enquanto o botão de emergência estiver pressionado
    while(digitalRead(stopButton) == HIGH);
    return;
  }

  // Botão de início (NA) - Inicia ou para o robô
  if (digitalRead(startButton) == LOW) {
    delay(50); // Debounce
    if (digitalRead(startButton) == LOW) {
        isRunning = !isRunning;
        if (isRunning) {
            Serial.println("Iniciando AGV...");
        } else {
            Serial.println("AGV pausado.");
            stopMotors();
        }
        while(digitalRead(startButton) == LOW); // Aguarda soltar o botão
    }
  }

  if (isRunning) {
    // 1. Verificar obstáculos com o sensor ultrassônico
    checkObstacle();
    if (distance <= 10) { // Se obstáculo a 10cm ou menos
      Serial.print("Obstáculo detectado a ");
      Serial.print(distance);
      Serial.println(" cm. Desviando do obstáculo...");
      avoidObstacle(); // Chama a rotina de desvio
    } else {
      // 2. Seguir a linha com os sensores IR
      followLine();
    }
  }
}

void checkObstacle() {
  // Limpa o trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  // Define o trigPin no estado HIGH por 10 micro segundos
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  // Lê o echoPin, retorna o tempo de viagem da onda sonora em micro segundos
  duration = pulseIn(echoPin, HIGH);
  // Calcula a distância
  distance = duration * 0.034 / 2;
}

void followLine() {
  bool s1 = digitalRead(irPin1); // Mais à esquerda
  bool s2 = digitalRead(irPin2);
  bool s3 = digitalRead(irPin3); // Centro
  bool s4 = digitalRead(irPin4);
  bool s5 = digitalRead(irPin5); // Mais à direita

  // Lógica para seguir a linha (assumindo que a linha é preta e o fundo é branco)
  // Sensores retornam LOW (0) na linha preta e HIGH (1) no fundo branco.

  if (!s1 && !s2 && s3 && !s4 && !s5) {
    // Em frente (apenas o sensor central detecta a linha)
    moveForward();
  } else if (!s1 && s2 && s3 && s4 && s5) {
    // Curva suave à direita
    turnRight();
  } else if (s1 && s2 && s3 && s4 && !s5) {
    // Curva acentuada à direita
    turnRight();
  } else if (s1 && s2 && s3 && !s4 && !s5) {
    // Curva acentuada à direita
    turnRight();
  } else if (s1 && s2 && !s3 && !s4 && s5) {
    // Curva à direita
    turnRight();
  } else if (s1 && s2 && s3 && s4 && !s5) {
    // Curva à direita
    turnRight();
  } else if (s1 && s2 && s3 && !s4 && s5) {
    // Curva à direita
    turnRight();
  } else if (s1 && s2 && !s3 && s4 && s5) {
    // Curva suave à esquerda
    turnLeft();
  } else if (!s1 && !s2 && s3 && s4 && s5) {
    // Curva acentuada à esquerda
    turnLeft();
  } else if (!s1 && s2 && s3 && s4 && s5) {
    // Curva acentuada à esquerda
    turnLeft();
  } else if (s1 && !s2 && !s3 && s4 && s5) {
    // Curva à esquerda
    turnLeft();
  } else if (!s1 && s2 && s3 && s4 && s5) {
    // Curva à esquerda
    turnLeft();
  } else if (s1 && !s2 && s3 && s4 && s5) {
    // Curva à esquerda
    turnLeft();
  }
  // Se todos os sensores estiverem fora da linha, pode ser uma parada ou um cruzamento.
  // Por enquanto, vamos parar.
  else if (s1 && s2 && s3 && s4 && s5) {
    stopMotors();
  }
}


// --- Funções de Movimento ---
void moveForward() {
  // Roda Direita
  digitalWrite(motorR_IN1, HIGH);
  digitalWrite(motorR_IN2, LOW);
  // Roda Esquerda
  digitalWrite(motorL_IN3, HIGH);
  digitalWrite(motorL_IN4, LOW);
}

void moveBackward() {
  // Roda Direita
  digitalWrite(motorR_IN1, LOW);
  digitalWrite(motorR_IN2, HIGH);
  // Roda Esquerda
  digitalWrite(motorL_IN3, LOW);
  digitalWrite(motorL_IN4, HIGH);
}

void turnRight() {
  // Roda Direita (para trás ou mais devagar)
  digitalWrite(motorR_IN1, LOW);
  digitalWrite(motorR_IN2, HIGH);
  // Roda Esquerda (para frente)
  digitalWrite(motorL_IN3, HIGH);
  digitalWrite(motorL_IN4, LOW);
}

void turnLeft() {
  // Roda Direita (para frente)
  digitalWrite(motorR_IN1, HIGH);
  digitalWrite(motorR_IN2, LOW);
  // Roda Esquerda (para trás ou mais devagar)
  digitalWrite(motorL_IN3, LOW);
  digitalWrite(motorL_IN4, HIGH);
}

void stopMotors() {
  // Roda Direita
  digitalWrite(motorR_IN1, LOW);
  digitalWrite(motorR_IN2, LOW);
  // Roda Esquerda
  digitalWrite(motorL_IN3, LOW);
  digitalWrite(motorL_IN4, LOW);
}

void avoidObstacle() {
  // 1. Para o robô
  stopMotors();
  delay(500);

  // 2. Vira para a direita por 500ms (ajuste o tempo conforme necessário)
  turnRight();
  delay(500);
  stopMotors();
  delay(200);

  // 3. Anda para a frente por 1000ms para passar pelo obstáculo
  moveForward();
  delay(1000);
  stopMotors();
  delay(200);

  // 4. Vira para a esquerda por 500ms para realinhar
  turnLeft();
  delay(500);
  stopMotors();
  delay(200);

  // 5. Continua em frente
  moveForward();
  delay(1000);

  // Depois de desviar, o robô tentará encontrar a linha novamente
  // A lógica em followLine() precisa ser robusta para isso.
}
