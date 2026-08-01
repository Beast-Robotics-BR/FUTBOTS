# CARRINHO ROBÓTICO COM CONTROLE POR GESTOS PARA ROBÔS JOGADORES EM ARENAS DE COMPETIÇÃO: UMA APLICAÇÃO EM FUTEBOL DE ROBÔS

Este repositório reúne os materiais de **hardware** e **software** desenvolvidos para um carrinho controlado remotamente por gestos, com comunicação via **Bluetooth Low Energy (BLE)**, projetado para atuar como robô jogador em **arenas de futebol de robôs**.

---

## ⚠️ Novas Funcionalidades Em Desenvolvimento

### 🎮 Novo Controle Remoto (estilo Gamepad)
- Prezamos pela experiência de uso e, por isso, iniciamos o desenvolvimento de um controle remoto no estilo *gamepad*
- O formato mais clássico facilita a operação em diferentes situações e para diferentes públicos
- Display **OLED** integrado ao controle, capaz de fornecer informações úteis durante a operação
- Longa duração de bateria continua sendo prioridade no novo modelo

---

## Visão Geral

O projeto consiste em um **kit robótico de baixo custo e fácil operação**. Pensamos também na facilidade de reparo e, por isso, optamos por uma abordagem que se beneficia do uso de **software livre (código aberto)** e **manufatura aditiva**. Nosso kit é reparável com poucas ferramentas, além de ter sido projetado para ser resistente e de qualidade.

### Funcionalidades Principais:
- Controle remoto **gestual**, por meio de um sensor inercial (IMU), embarcado no próprio controle
- Controle alternativo via **celular**, tanto para operação normal quanto para testes e ajustes
- Comunicação sem fio via **BLE**, com vinculação por endereço MAC para conexões estáveis
- Múltiplos **níveis de intensidade de PWM**, selecionáveis durante a operação
- **Gatilhos físicos** e **botão multifunção** para ações rápidas de controle
- **Programação por blocos**, através de um web app, permitindo customizar o comportamento do carrinho
- Painel de depuração via **Web Bluetooth**, acessível pelo navegador

---

## 🛠 Hardware

### Componentes Principais
- **ESP32-S3 Super Mini** — controladora central, embarcada no controle remoto
- **Arduino Nano** — controlador embarcado no carrinho
- **Módulo IMU (MPU-6050)** — leitura de inclinação (pitch/roll) para o controle gestual
- **Display OLED SSD1306** (interface SPI, 7 pinos) — feedback visual no controle
- **Gatilhos físicos** e **botão multifunção**
- Comunicação via **Bluetooth Low Energy (BLE)**, compatível com **Web Bluetooth**

---

## 🔧 Arquitetura de Comunicação

- Comunicação **BLE** entre o controle (ESP32-S3) e o carrinho (Arduino Nano)
- Uso do **Nordic UART Service** para garantir compatibilidade com **Web Bluetooth**
- **Vinculação por MAC address**, salva em memória não volátil (NVS), evitando reconexões indesejadas com outros dispositivos
- **Modo CONFIG**: o ESP32 passa a anunciar com UUIDs proprietários de 128 bits, permitindo o envio e o salvamento de perfis de PWM na memória do dispositivo
- **Painel de depuração Web Bluetooth (HTML)**, com múltiplos modos de conexão, D-pad virtual, monitor serial e contadores de pacotes TX/RX

---

## 🎮 Sistema de Controle Gestual

- Leitura de **pitch/roll** via MPU-6050, traduzida em comandos de direção do carrinho
- **Três níveis de intensidade de PWM**, ajustáveis conforme a situação de jogo
- **Gatilhos físicos** (GPIO) para funções adicionais de controle
- **Botão multifunção**:
  - 1 clique → recalibração do sensor
  - 2 cliques → bloqueia/desbloqueia o controle
  - Pressão longa (5s) → entra no **modo CONFIG**

---

## ⚙️ Notas Técnicas

- Durante o desenvolvimento, identificamos que módulos baratos vendidos como "HC-05" eram, na verdade, clones com chip Jieli, operando com serviço/característica BLE próprios — o que exigiu ajustes no mapeamento de UUIDs
- A arquitetura de comunicação BLE passou por múltiplas revisões (filtros de escaneamento, arquitetura de callbacks) até garantir estabilidade de conexão

---

## 🎯 Aplicações

- Robô jogador em **arenas de futebol de robôs** e competições similares
- Ensino de **robótica e automação** com foco em controle gestual
- Base para **prototipagem** de sistemas de controle remoto sem fio de baixo custo

---

## Muito obrigado pela atenção !!!

---
