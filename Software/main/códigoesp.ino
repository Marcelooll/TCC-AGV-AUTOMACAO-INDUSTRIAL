//codigo do esp32

// ------------------------- ULTRASSÔNICO ---------------------
#define TRIG 5
#define ECHO 18

// ------------------------- BOTÕES ----------------------------
#define BTN_NA 32
#define BTN_NF 33

// ------------------------- LEDS ------------------------------
#define LED_VERDE 21
#define LED_VERMELHO 22

// ------------------------- SENSORES IR -----------------------
#define IR1 13
#define IR2 15
#define IR3 19
#define IR4 23
#define IR5 4

// ======================================================================
// VARIÁVEIS
// ======================================================================
bool sistemaAtivo = false;
bool ultimoEstadoBtnNA = HIGH;
bool ultimoEstadoBtnNF = HIGH;

// ======================================================================
// FUNÇÕES
// ======================================================================

float lerUltrassom() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);

  long duracao = pulseIn(ECHO, HIGH, 25000);
  if (duracao == 0) return 999;

  return duracao * 0.0343 / 2;
}

int lerLinha() {
  int v1 = !digitalRead(IR1);
  int v2 = !digitalRead(IR2);
  int v3 = !digitalRead(IR3);
  int v4 = !digitalRead(IR4);
  int v5 = !digitalRead(IR5);

  if (v3 == 1) return 0;
  if (v2 == 1) return -1;
  if (v1 == 1) return -2;
  if (v4 == 1) return 1;
  if (v5 == 1) return 2;

  return 0;
}

// ======================================================================
// SETUP
// ======================================================================
void setup() {
  Serial.begin(115200);   // Debug no PC
  Serial2.begin(9600, SERIAL_8N1, 16, 17);  // RX=16, TX=17 para Arduino

  // LEDs
  pinMode(LED_VERDE, OUTPUT);
  pinMode(LED_VERMELHO, OUTPUT);

  // IR
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);

  // Ultrassom
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  // Botões
  pinMode(BTN_NA, INPUT_PULLUP);
  pinMode(BTN_NF, INPUT_PULLUP);

  digitalWrite(LED_VERDE, HIGH);
  digitalWrite(LED_VERMELHO, LOW);

  Serial.println("ESP32 pronto!");
}

// ======================================================================
// LOOP
// ======================================================================
void loop() {
  bool estadoAtualBtnNA = digitalRead(BTN_NA);
  bool estadoAtualBtnNF = digitalRead(BTN_NF);

  // Botão NA → Ligar
  if (estadoAtualBtnNA == LOW && ultimoEstadoBtnNA == HIGH) {
    sistemaAtivo = true;
    digitalWrite(LED_VERDE, LOW);
    digitalWrite(LED_VERMELHO, HIGH);
    Serial.println("Sistema ATIVADO");
    delay(50);
  }

  // Botão NF → Desligar
  if (estadoAtualBtnNF == LOW && ultimoEstadoBtnNF == HIGH) {
    sistemaAtivo = false;
    digitalWrite(LED_VERDE, HIGH);
    digitalWrite(LED_VERMELHO, LOW);
    Serial2.println("P");  // Envia comando PARAR
    Serial.println("Sistema DESATIVADO");
    delay(50);
  }

  ultimoEstadoBtnNA = estadoAtualBtnNA;
  ultimoEstadoBtnNF = estadoAtualBtnNF;

  if (!sistemaAtivo) return;

  // Lê sensores
  int erro = lerLinha();
  float dist = lerUltrassom();

  // Envia para Arduino: "E,erro,distancia"
  Serial2.print("E,");
  Serial2.print(erro);
  Serial2.print(",");
  Serial2.println((int)dist);

  // Debug
  Serial.printf("Erro: %d | Dist: %.1f cm\n", erro, dist);

  delay(50);
}