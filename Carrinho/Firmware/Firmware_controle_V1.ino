/*
  ============================================================
  CARRINHO - ESBOÇO DE TESTE (Arduino Nano + TB66A6FNG + HM-10)
  Adaptado ao protocolo real do controle ESP32-S3 (v2.4)
  ============================================================

  PINAGEM (esquemático "PLACA 1 - Concept3 v28"):
  --------------------------------------------------------
  Motor 1 / roda ESQUERDA (canal A do TB66A6FNG):
    AIN1 -> A4
    AIN2 -> A3
    PWMA -> D3

  Motor 2 / roda DIREITA (canal B do TB66A6FNG):
    BIN1 -> 13   (!) pino analog-only no Nano, ver aviso abaixo
    BIN2 -> A6   (!) pino analog-only no Nano, ver aviso abaixo
    PWMB -> D5

  STBY do driver: fixo em +5V (sempre habilitado, sem controle por pino)

  LEDs:
    Verde (individual) -> D6, D9, D10
    Vermelho (todos)   -> D4
    Azul (todos)       -> D2

  HM-10 BLE:
    D7 -> leitura do pino "state" do módulo
    Comunicação com o ESP32: hardware Serial (D0/D1) do Nano, ligado
    ao HM-10 via divisor resistivo. O HM-10 atua como bridge BLE<->UART
    transparente: tudo que o ESP32 escreve na característica 0xFFE1
    chega cru (sem alteração) na serial do Nano.

  --------------------------------------------------------
  AVISOS IMPORTANTES:

  1) 13 e A6 são pinos ANALOG-INPUT ONLY no ATmega328 (Nano) e
     fisicamente não têm buffer de saída digital. O controle do
     Motor 2/roda direita (principalmente a troca de sentido)
     precisa ser validado na placa de ensaio.

  2) D0/D1 são compartilhados com a serial USB. Com o HM-10
     fisicamente ligado ao RX/TX, os prints de debug via Monitor
     Serial vão se misturar com os dados recebidos do controle.

  --------------------------------------------------------
  PROTOCOLO REAL (extraído do firmware do controle ESP32, v2.4):

  O ESP32 envia, a cada 10ms (enquanto conectado e em MODE_NORMAL),
  uma escrita BLE na característica 0xFFE1 com o conteúdo:

      "<velocidade>,<rotacao>"

  Exemplos reais: "VF2,R1"  "VB1,0"  "V0,L3"  "V0,0"

  Onde:
    velocidade = "VF1"/"VF2"/"VF3" (frente, níveis 1-3)
               = "VB1"/"VB2"/"VB3" (ré, níveis 1-3)
               = "V0"              (sem propulsão)
    rotacao    = "R1"/"R2"/"R3"    (giro à direita, níveis 1-3)
               = "L1"/"L2"/"L3"    (giro à esquerda, níveis 1-3)
               = "0"               (sem rotação)

  IMPORTANTE: o pacote NÃO tem terminador (sem '\n'). Como o ESP
  reenvia a cada 10ms, o Nano recebe um fluxo contínuo tipo:
  "VF2,R1VF2,R1VF2,R1V0,0V0,0..." sem separador entre um pacote e
  o próximo. Por isso o parser abaixo não usa readStringUntil('\n')
  -- ele é uma máquina de estados que usa o 'V' como marcador de
  início de cada pacote nova, e a vírgula como separador interno
  entre velocidade e rotação.

  Além disso, o protocolo permite velocidade e rotação SIMULTÂNEAS
  (ex: "VF2,R1" = andar pra frente enquanto curva à direita), então
  este esboço faz uma mixagem simples tipo "tank drive": a rotação
  soma/subtrai um valor de PWM em cima da velocidade base de cada
  roda, em vez de tratar frente/ré e giro como mutuamente exclusivos.
  ============================================================
*/

// ---------- Pinagem ----------
// Motor 1 / roda esquerda (canal A)
const int AIN2 = A4;
const int AIN1 = A3;
const int PWMA = 3;

// Motor 2 / roda direita (canal B)
const int BIN2 = A2;
const int BIN1 = A1;
const int PWMB = 5;

// LEDs
const int LED_VERDE_1 = 6;
const int LED_VERDE_2 = 9;
const int LED_VERDE_3 = 10;
const int LED_VERMELHO = 4;
const int LED_AZUL = 2;


// BLE
const int BLE_STATE = 7;

// ---------- Tabela de PWM por nível (1-3) ----------
// Espelha os valores DEFAULT_PWM_MIN/MID/MAX que já existem no
// firmware do controle (hoje não usados lá, pois o comentário do
// ESP diz que o PWM "é definido no Nano/interface do carrinho").
// Ajustar aqui conforme a resposta real dos motores na bancada.
const int PWM_NIVEL[4] = {0, 25, 50, 130}; // índice 0 não usado

// ---------- Falha de segurança ----------
// Se não chegar nenhum comando válido por esse tempo, para os motores.
// Importante porque o ESP só transmite continuamente enquanto conectado
// e em MODE_NORMAL -- se desconectar, entrar em LOCK ou perder alcance,
// o carrinho não pode continuar andando com o último comando recebido.
const unsigned long TIMEOUT_FAILSAFE_MS = 300;
unsigned long ultimoComandoValido = 0;

// ---------- Parser (máquina de estados) ----------
enum EstadoParser { AGUARDANDO_V, LENDO_VELOCIDADE, LENDO_ROTACAO };
EstadoParser estadoParser = AGUARDANDO_V;
String tokenVelocidade = "";
String tokenRotacao = "";

void setup() {
  // Motores como saída
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);

  // LEDs como saída
  pinMode(LED_VERDE_1, OUTPUT);
  pinMode(LED_VERDE_2, OUTPUT);
  pinMode(LED_VERDE_3, OUTPUT);
  pinMode(LED_VERMELHO, OUTPUT);
  pinMode(LED_AZUL, OUTPUT);

  // Pino de status do BLE
  pinMode(BLE_STATE, INPUT);

  pararMotores();
  apagarLeds();

  Serial.begin(38400); // usado tanto para debug via USB quanto para o link com o HM-10
  Serial.println(F("Carrinho pronto. Aguardando comandos do controle..."));

  
  
}

void loop() {
  while (Serial.available() > 0) {
    processarByte((char)Serial.read());
  }

  // Falha de segurança: para os motores se ficar tempo demais sem
  // receber um comando valido (ex: controle desconectou/saiu de alcance)
  if (millis() - ultimoComandoValido > TIMEOUT_FAILSAFE_MS) {
    pararMotores();
  }

  // Pisca o LED azul enquanto o BLE não estiver conectado, como
  // indicativo visual de status durante o teste na placa de ensaio
  if (digitalRead(BLE_STATE) == HIGH) {
    digitalWrite(LED_AZUL, (millis() / 500) % 2);
  } else {
    digitalWrite(LED_AZUL, HIGH);
  }
}

// Máquina de estados que reconstrói pacotes "VELOCIDADE,ROTACAO" a
// partir de um fluxo continuo sem terminador, usando o 'V' como
// marcador de início de cada novo pacote.
void processarByte(char c) {
  switch (estadoParser) {
    case AGUARDANDO_V:
      if (c == 'V') {
        tokenVelocidade = "V";
        estadoParser = LENDO_VELOCIDADE;
      }
      break;

    case LENDO_VELOCIDADE:
      if (c == ',') {
        tokenRotacao = "";
        estadoParser = LENDO_ROTACAO;
      } else {
        tokenVelocidade += c;
      }
      break;

    case LENDO_ROTACAO:
      if (c == 'V') {
        // Chegou o 'V' do proximo pacote -> o pacote atual esta completo
        aplicarComando(tokenVelocidade, tokenRotacao);
        tokenVelocidade = "V";
        estadoParser = LENDO_VELOCIDADE;
      } else {
        tokenRotacao += c;
      }
      break;
  }
}

void aplicarComando(String vel, String rot) {
  int nivelVel = 0;   // negativo = re, positivo = frente
  int nivelRot = 0;   // negativo = esquerda, positivo = direita

  // --- Velocidade ---
  if (vel == "V0" || vel == "VF0") {
    nivelVel = 0;
  } else if (vel.startsWith("VF") && vel.length() == 3) {
    nivelVel = vel.charAt(2) - '0';           // +1, +2, +3
  } else if (vel.startsWith("VB") && vel.length() == 3) {
    nivelVel = -(vel.charAt(2) - '0');        // -1, -2, -3
  } else {
    return; // token de velocidade desconhecido/corrompido, ignora o pacote
  }

  // --- Rotação ---
  if (rot == "0" || rot == "R0" || rot == "L0" || rot == "") {
    nivelRot = 0;
  } else if (rot.startsWith("L") && rot.length() == 2) {
    nivelRot = rot.charAt(1) - '0';           // +1, +2, +3
  } else if (rot.startsWith("R") && rot.length() == 2) {
    nivelRot = -(rot.charAt(1) - '0');        // -1, -2, -3
  } else {
    return; // token de rotacao desconhecido/corrompido, ignora o pacote
  }

  if (abs(nivelVel) > 3 || abs(nivelRot) > 3) return; // fora de faixa, descarta

  ultimoComandoValido = millis();
  moverComMixagem(nivelVel, nivelRot);
}

// Mixagem tipo "tank drive": a rotacao soma/subtrai PWM em cima da
// velocidade base de cada roda. ATENCAO: convencao de sinal (R = roda
// direita mais lenta / esquerda mais rapida) e uma suposicao -- se o
// carrinho girar pro lado errado no teste, inverter o sinal de rotPWM.
void moverComMixagem(int nivelVel, int nivelRot) {
  int velPWM = sinal(nivelVel) * PWM_NIVEL[abs(nivelVel)];
  int rotPWM = sinal(nivelRot) * PWM_NIVEL[abs(nivelRot)];

  int pwmEsquerda = constrain(velPWM + rotPWM, -255, 255);
  int pwmDireita  = constrain(velPWM - rotPWM, -255, 255);

  acionarMotor(AIN1, AIN2, PWMA, pwmEsquerda);
  acionarMotor(BIN1, BIN2, PWMB, pwmDireita);
}

int sinal(int v) {
  if (v > 0) return 1;
  if (v < 0) return -1;
  return 0;
}

void acionarMotor(int pinoIN1, int pinoIN2, int pinoPWM, int valor) {
  if (valor > 0) {
    digitalWrite(pinoIN1, HIGH);
    digitalWrite(pinoIN2, LOW);
    analogWrite(pinoPWM, valor);
  } else if (valor < 0) {
    digitalWrite(pinoIN1, LOW);
    digitalWrite(pinoIN2, HIGH);
    analogWrite(pinoPWM, -valor);
  } else {
    digitalWrite(pinoIN1, LOW);
    digitalWrite(pinoIN2, LOW);
    analogWrite(pinoPWM, 0);
  }
}

void pararMotores() {
  acionarMotor(AIN1, AIN2, PWMA, 0);
  acionarMotor(BIN1, BIN2, PWMB, 0);
}

// ---------- Funções de LED ----------
void apagarLeds() {
  digitalWrite(LED_VERDE_1, HIGH);
  digitalWrite(LED_VERDE_2, HIGH);
  digitalWrite(LED_VERDE_3, HIGH);
  digitalWrite(LED_VERMELHO, HIGH);
  digitalWrite(LED_AZUL, HIGH);
}
