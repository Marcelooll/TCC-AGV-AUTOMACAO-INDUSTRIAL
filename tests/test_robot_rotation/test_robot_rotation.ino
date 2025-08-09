/**
 * @file test_led_directional_safe.ino
 * @brief Código de teste para LEDs direcionais, com pinos seguros para ESP32.
 * 
 * Objetivo: Este código serve para testar o acionamento de dois LEDs 
 * que indicam a direção de movimento de um robô (Esquerda e Direita).
 * Ele foi revisado para usar pinos (GPIOs) que são seguros para uso geral no ESP32,
 * evitando problemas durante a inicialização do microcontrolador.
 * 
 * Como funciona:
 * - Usamos pinos de entrada com resistores de PULL-UP internos. Isso significa que o pino já está em nível lógico ALTO (HIGH). Para ativá-lo, você deve conectá-lo ao GND (Terra).
 * - Se o pino de entrada para "direita" for conectado ao GND, o LED da direita piscará.
 * - Se o pino de entrada para "esquerda" for conectado ao GND, o LED da esquerda piscará.
 * - Se nenhum dos pinos de entrada for aterrado, ambos os LEDs permanecerão apagados.
 * 
 * Montagem do circuito:
 * - Conecte um LED ao pino `LED_DIREITA_PIN`. O lado maior do LED (anodo) vai no pino, e o menor (catodo) vai no GND (terra) através de um resistor de 220 ohms.
 * - Conecte outro LED ao pino `LED_ESQUERDA_PIN` da mesma forma.
 * - Conecte um botão ou um fio ao pino `ENTRADA_DIREITA_PIN`. O outro lado do botão/fio deve ser conectado ao GND (Terra).
 * - Conecte outro botão ou um fio ao pino `ENTRADA_ESQUERDA_PIN` da mesma forma.
 */

// --- Pinos de Configuração (Seguros para ESP32) ---

// Pinos de SAÍDA (OUTPUT) para os LEDs
// GPIOs 22 e 23 são seguros para uso como saídas.
const int LED_DIREITA_PIN = 22;  // LED que indica movimento para a direita
const int LED_ESQUERDA_PIN = 23; // LED que indica movimento para a esquerda

// Pinos de ENTRADA (INPUT) para os comandos
// GPIOs 16 e 17 são seguros para uso como entradas.
const int ENTRADA_DIREITA_PIN = 16; // Pino que recebe o comando "direita"
const int ENTRADA_ESQUERDA_PIN = 17; // Pino que recebe o comando "esquerda"

// --- Variáveis Globais ---
int comandoDireita = 0;
int comandoEsquerda = 0;

/**
 * @brief Função de configuração (setup).
 */
void setup() {
  Serial.begin(9600);
  Serial.println(">>> Teste de LEDs Direcionais (Pinos Seguros) <<<");
  Serial.println("Aterre os pinos de entrada para ver os LEDs piscarem.");
  Serial.println("---------------------------------------------------");

  // Configura os pinos dos LEDs como SAÍDA (OUTPUT)
  pinMode(LED_DIREITA_PIN, OUTPUT);
  pinMode(LED_ESQUERDA_PIN, OUTPUT);

  // Configura os pinos de comando como ENTRADA com PULL-UP.
  // O pino fica em estado ALTO (HIGH) por padrão. O comando é ativado
  // quando o pino é conectado ao GND, fazendo a leitura ser BAIXA (LOW).
  pinMode(ENTRADA_DIREITA_PIN, INPUT_PULLUP);
  pinMode(ENTRADA_ESQUERDA_PIN, INPUT_PULLUP);
}

/**
 * @brief Função de loop principal.
 */
void loop() {
  // Lê o estado dos pinos de entrada.
  // A leitura será LOW se o pino estiver aterrado (botão pressionado).
  comandoDireita = digitalRead(ENTRADA_DIREITA_PIN);
  comandoEsquerda = digitalRead(ENTRADA_ESQUERDA_PIN);

  // --- Lógica de Controle dos LEDs ---

  // Verifica se o comando para ir para a DIREITA foi recebido (pino em LOW)
  if (comandoDireita == LOW) {
    Serial.println("Comando: DIREITA -> Pisca LED da direita");
    
    digitalWrite(LED_DIREITA_PIN, HIGH);
    delay(250);
    digitalWrite(LED_DIREITA_PIN, LOW);

    // Garante que o LED da esquerda esteja apagado
    digitalWrite(LED_ESQUERDA_PIN, LOW);
  }
  // Verifica se o comando para ir para a ESQUERDA foi recebido (pino em LOW)
  else if (comandoEsquerda == LOW) {
    Serial.println("Comando: ESQUERDA -> Pisca LED da esquerda");

    digitalWrite(LED_ESQUERDA_PIN, HIGH);
    delay(250);
    digitalWrite(LED_ESQUERDA_PIN, LOW);

    // Garante que o LED da direita esteja apagado
    digitalWrite(LED_DIREITA_PIN, LOW);
  }
  // Se nenhum comando for recebido
  else {
    // Garante que ambos os LEDs estejam apagados
    digitalWrite(LED_DIREITA_PIN, LOW);
    digitalWrite(LED_ESQUERDA_PIN, LOW);
  }
   delay(100); // Pequeno delay para estabilidade
}