# Guia de Testes de LEDs Direcionais

Este guia explica como preparar e executar os testes de LEDs direcionais para o projeto do AGV.

## Visão Geral

Os códigos de teste localizados nas pastas `test_motor_bts7960` e `test_robot_rotation` foram modificados para realizar um teste simples e visual com LEDs. O objetivo é verificar se o microcontrolador ESP32 consegue ler comandos de entrada e controlar saídas de forma correta.

**Nota:** Ambos os arquivos (`test_motor_bts7960.ino` e `test_robot_rotation.ino`) contêm o **mesmo código** de teste de LED.

## Hardware Necessário

- 1x Placa de desenvolvimento ESP32
- 2x LEDs (qualquer cor)
- 2x Resistores de 220Ω (Ohms)
- 2x Botões de pressão (push-buttons) ou fios para simular o comando
- 1x Protoboard (placa de ensaio)
- Fios de jumper

## Montagem do Circuito

Para que o teste funcione, você precisa montar o seguinte circuito na protoboard:

### 1. Conexão dos LEDs (Saídas)

Os LEDs indicam a direção "Direita" e "Esquerda".

- **LED da Direita:**
    1. Conecte o pino mais longo do LED (anodo) ao pino **GPIO 22** do ESP32.
    2. Conecte o pino mais curto do LED (catodo) a uma perna do resistor de 220Ω.
    3. Conecte a outra perna do resistor ao `GND` (Terra) do ESP32.

- **LED da Esquerda:**
    1. Conecte o pino mais longo do LED (anodo) ao pino **GPIO 23** do ESP32.
    2. Conecte o pino mais curto do LED (catodo) a uma perna do resistor de 220Ω.
    3. Conecte a outra perna do resistor ao `GND` (Terra) do ESP32.

### 2. Conexão dos Botões (Entradas)

Os botões simulam os comandos "ir para a direita" e "ir para a esquerda".

- **Botão de Comando "Direita":**
    1. Conecte uma perna do botão ao pino **GPIO 16** do ESP32.
    2. Conecte a outra perna do botão ao `GND` (Terra) do ESP32.

- **Botão de Comando "Esquerda":**
    1. Conecte uma perna do botão ao pino **GPIO 17** do ESP32.
    2. Conecte a outra perna do botão ao `GND` (Terra) do ESP32.

*Se não tiver botões, você pode usar um fio jumper. Basta conectar o pino de entrada (16 ou 17) diretamente no `GND` para ativar o comando.*

## Como Executar o Teste

1.  **Carregue o Código:** Abra a Arduino IDE, carregue um dos arquivos de teste (`test_motor_bts7960.ino` ou `test_robot_rotation.ino`) para o seu ESP32.
2.  **Abra o Monitor Serial:** Na Arduino IDE, vá em `Ferramentas > Monitor Serial` e certifique-se de que a velocidade (baud rate) está configurada para **9600**. Você verá a mensagem `>>> Teste de LEDs Direcionais (Pinos Seguros) <<<`.
3.  **Teste o Comando "Direita":**
    *   Pressione e segure o botão conectado ao pino **16**.
    *   **Resultado esperado:** O LED conectado ao pino **22** deve começar a piscar. No Monitor Serial, a mensagem `Comando: DIREITA -> Pisca LED da direita` aparecerá repetidamente.
4.  **Teste o Comando "Esquerda":**
    *   Pressione e segure o botão conectado ao pino **17**.
    *   **Resultado esperado:** O LED conectado ao pino **23** deve começar a piscar. No Monitor Serial, a mensagem `Comando: ESQUERDA -> Pisca LED da esquerda` aparecerá repetidamente.
5.  **Sem Comandos:**
    *   Quando nenhum botão estiver pressionado, ambos os LEDs devem permanecer apagados e nenhuma nova mensagem aparecerá no Monitor Serial.

Se o comportamento observado for o descrito acima, o teste foi um sucesso!

---

# Guia de Teste de Rotação Automática

Este guia explica como executar o teste de rotação automática do robô.

## Visão Geral

O código localizado em `test_robot_rotation_auto/test_robot_rotation_auto.ino` executa uma sequência pré-programada de movimentos de rotação para verificar o funcionamento dos dois motores simultaneamente, com feedback visual de LEDs. **Este teste não precisa de botões.**

## Hardware Necessário

- 1x Placa de desenvolvimento ESP32
- 2x LEDs (qualquer cor)
- 2x Resistores de 220Ω (Ohms)
- **2x Drivers de Motor Ponte H (ex: BTS7960)**
- **2x Motores DC**
- 1x Protoboard
- Fios de jumper
- Fonte de alimentação externa para os motores

## Montagem do Circuito

Este teste requer a montagem completa do sistema de motorização do robô.

### 1. Conexão dos LEDs

- **LED da Direita (Rotação Horária):** Conecte ao **GPIO 22** (seguindo o mesmo método do teste anterior, com resistor ao `GND`).
- **LED da Esquerda (Rotação Anti-horária):** Conecte ao **GPIO 23** (seguindo o mesmo método do teste anterior, com resistor ao `GND`).

### 2. Conexão dos Motores e Drivers

Conecte os drivers de motor e os motores ao ESP32 conforme os pinos definidos no topo do arquivo `test_robot_rotation_auto.ino`.

- **Motor 1 (Direito):**
  - `M1_R_EN_PIN`: GPIO 21
  - `M1_L_EN_PIN`: GPIO 19
  - `M1_RPWM_PIN`: GPIO 18
  - `M1_LPWM_PIN`: GPIO 5
- **Motor 2 (Esquerdo):**
  - `M2_R_EN_PIN`: GPIO 17
  - `M2_L_EN_PIN`: GPIO 16
  - `M2_RPWM_PIN`: GPIO 4
  - `M2_LPWM_PIN`: GPIO 2

**Importante:** Certifique-se de que os drivers de motor estão sendo alimentados por uma fonte externa apropriada, e que os terras (`GND`) do ESP32 e da fonte dos motores estão conectados.

## Como Executar o Teste

1.  **Carregue o Código:** Abra o arquivo `test_robot_rotation_auto/test_robot_rotation_auto.ino` na Arduino IDE e carregue-o para o seu ESP32.
2.  **Abra o Monitor Serial:** Configure o Monitor Serial para a velocidade **115200**.
3.  **Observe a Sequência:** Assim que o código começar a rodar, o robô iniciará a sequência automática:
    *   **Giro Horário:** O robô girará para a direita por 3 segundos. O LED da direita ficará aceso.
    *   **Pausa:** O robô parará por 2 segundos. Ambos os LEDs se apagarão.
    *   **Giro Anti-Horário:** O robô girará para a esquerda por 3 segundos. O LED da esquerda ficará aceso.
    *   **Pausa:** O robô parará por 2 segundos.
    *   O ciclo se repetirá indefinidamente.

Se o robô e os LEDs seguirem essa sequência, o teste foi um sucesso.