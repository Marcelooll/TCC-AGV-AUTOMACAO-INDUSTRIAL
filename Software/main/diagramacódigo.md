# Robô Seguidor de Linha com ESP32 e Arduino

## Visão Geral

Este projeto implementa um robô seguidor de linha que utiliza dois microcontroladores:

* **ESP32** : Gerencia sensores e lógica de controle
* **Arduino** : Controla os motores com PID

```mermaid
graph TB
    subgraph ESP32["ESP32 - Controle Principal"]
        BTN_NA["Botão NA<br/>Ligar"]
        BTN_NF["Botão NF<br/>Desligar"]
        LED_V["LED Verde"]
        LED_R["LED Vermelho"]
        IR["5 Sensores IR<br/>Pins: 13,15,19,23,4"]
        ULTRA["Sensor Ultrassônico<br/>TRIG: 5, ECHO: 18"]
        ESP_LOOP["Loop ESP32"]
        ESP_FUNC["Funções:<br/>lerLinha<br/>lerUltrassom"]
    end

    subgraph ARDUINO["Arduino - Controle de Motores"]
        SERIAL_RX["Serial RX"]
        ARDUINO_LOOP["Loop Arduino"]
        PID["Controlador PID<br/>Kp=25 Ki=0.5 Kd=4<br/>baseSpeed=200"]
        MOTOR_CTRL["Controle de Motores"]
        MOTOR_D["Motor Direito<br/>BTS7960<br/>RPWM:5 LPWM:6 EN:7"]
        MOTOR_E["Motor Esquerdo<br/>BTS7960<br/>RPWM:9 LPWM:10 EN:8"]
    end

    subgraph COMUNICACAO["Comunicação Serial 9600 baud"]
        UART["UART<br/>ESP32 TX17 → Arduino RX"]
        CMD_E["Comando E<br/>E,erro,distancia"]
        CMD_P["Comando P<br/>Parar motores"]
    end

    BTN_NA -->|Pressionar| ESP_LOOP
    BTN_NF -->|Pressionar| ESP_LOOP
    ESP_LOOP -->|Ativar| LED_R
    ESP_LOOP -->|Desativar| LED_V

    IR --> ESP_FUNC
    ULTRA --> ESP_FUNC
    ESP_FUNC -->|Calcular erro| ESP_LOOP

    ESP_LOOP --> UART
    UART --> CMD_E
    UART --> CMD_P

    CMD_E --> SERIAL_RX
    CMD_P --> SERIAL_RX

    SERIAL_RX --> ARDUINO_LOOP
    ARDUINO_LOOP -->|Parsear dados| PID
    ARDUINO_LOOP -->|Obstáculo<10cm| MOTOR_CTRL

    PID -->|Ajustar PWM| MOTOR_CTRL
    MOTOR_CTRL --> MOTOR_D
    MOTOR_CTRL --> MOTOR_E

    style ESP32 fill:#e1f5ff
    style ARDUINO fill:#ffe1e1
    style COMUNICACAO fill:#fff9e1
    style PID fill:#ffd700
    style MOTOR_CTRL fill:#90ee90
```
