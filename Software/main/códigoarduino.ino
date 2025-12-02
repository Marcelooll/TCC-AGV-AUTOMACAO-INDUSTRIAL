//codigo do arduino
// ------------------------- MOTORES (BTS7960) -------------------------
#define MOTOR_DIR_RPWM 5
#define MOTOR_DIR_LPWM 6
#define MOTOR_DIR_EN 7

#define MOTOR_ESQ_RPWM 9
#define MOTOR_ESQ_LPWM 10
#define MOTOR_ESQ_EN 8

// ======================================================================
// VARIÁVEIS PID
// ======================================================================
unsigned long lastPIDTime = 0;
float lastError = 0;
float integral = 0;

float kp = 25;
float ki = 0.5;
float kd = 4;

int baseSpeed = 200;

// ======================================================================
// FUNÇÕES
// ======================================================================

float pidCompute(float error) {
  unsigned long now = millis();
  float dt = (now - lastPIDTime);
  if (dt < 1) dt = 1;
  dt = dt / 1000.0;

  float P = error * kp;

  integral += error * ki * dt;
  integral = constrain(integral, -255, 255);

  float D = kd * (error - lastError) / dt;

  lastError = error;
  lastPIDTime = now;

  return P + integral + D;
}

void mover(int pwmDir, int pwmEsq) {
  pwmDir = constrain(pwmDir, 0, 255);
  pwmEsq = constrain(pwmEsq, 0, 255);

  digitalWrite(MOTOR_DIR_EN, HIGH);
  analogWrite(MOTOR_DIR_RPWM, pwmDir);
  analogWrite(MOTOR_DIR_LPWM, 0);

  digitalWrite(MOTOR_ESQ_EN, HIGH);
  analogWrite(MOTOR_ESQ_RPWM, pwmEsq);
  analogWrite(MOTOR_ESQ_LPWM, 0);
}

void pararTudo() {
  digitalWrite(MOTOR_DIR_EN, LOW);
  digitalWrite(MOTOR_ESQ_EN, LOW);
  analogWrite(MOTOR_DIR_RPWM, 0);
  analogWrite(MOTOR_DIR_LPWM, 0);
  analogWrite(MOTOR_ESQ_RPWM, 0);
  analogWrite(MOTOR_ESQ_LPWM, 0);
}

// ======================================================================
// SETUP
// ======================================================================
void setup() {
  Serial.begin(9600);  // Comunicação com ESP32

  pinMode(MOTOR_DIR_RPWM, OUTPUT);
  pinMode(MOTOR_DIR_LPWM, OUTPUT);
  pinMode(MOTOR_DIR_EN, OUTPUT);
  pinMode(MOTOR_ESQ_RPWM, OUTPUT);
  pinMode(MOTOR_ESQ_LPWM, OUTPUT);
  pinMode(MOTOR_ESQ_EN, OUTPUT);

  pararTudo();
}

// ======================================================================
// LOOP
// ======================================================================
void loop() {
  if (Serial.available()) {
    String dados = Serial.readStringUntil('\n');

    // Comando PARAR
    if (dados.startsWith("P")) {
      pararTudo();
      return;
    }

    // Comando com dados: "E,erro,distancia"
    if (dados.startsWith("E,")) {
      int primeiraVirgula = dados.indexOf(',');
      int segundaVirgula = dados.indexOf(',', primeiraVirgula + 1);

      int erro = dados.substring(primeiraVirgula + 1, segundaVirgula).toInt();
      int dist = dados.substring(segundaVirgula + 1).toInt();

      // Obstáculo
      if (dist < 10) {
        pararTudo();
        return;
      }

      // PID e movimento
      float pid = pidCompute(erro);
      int pwmEsq = baseSpeed + pid;
      int pwmDir = baseSpeed - pid;

      mover(pwmDir, pwmEsq);
    }
  }
}