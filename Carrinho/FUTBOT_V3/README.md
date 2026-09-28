# RoverLab V3 Modular

Firmware para Arduino Nano + TB6612FNG + HM-10 com controle manual, perfil PWM, EEPROM e programação por blocos.

## Arquivos

- `FUTBOT_V3.ino`: aplicação principal, protocolo, motores, EEPROM e executor de blocos.
- `Config.h`: pinagem, parâmetros e seleção de polaridade.
- `Leds.h`: estados e padrões dos LEDs.

Abra o arquivo `FUTBOT_V3.ino` pelo Arduino IDE mantendo os três arquivos na mesma pasta.

## Escolha do tipo de LED

Edite somente esta linha em `Config.h`:

```cpp
const bool LED_ACTIVE_HIGH = true;
```

Use:

```cpp
true  // catodo comum: HIGH acende
false // anodo comum: LOW acende
```

Se o seu módulo LED tiver transistores ou uma placa intermediária que inverta o sinal, use o valor que corresponde ao nível lógico que efetivamente acende o LED.

## Estados dos LEDs

- Azul fixo: BLE conectado.
- Azul piscando: BLE desconectado.
- Vermelho pulsando: janela de configuração desbloqueada.
- Vermelho fixo: reservado para falha crítica.
- Verdes fixos em quantidade 1, 2 ou 3: nível de movimento selecionado.
- Verde alternado: upload de programa em andamento.
- Verde correndo: programa por blocos em execução.

Os pinos permanecem:

| LED | Pino |
|---|---:|
| Verde 1 | D6 |
| Verde 2 | D9 |
| Verde 3 | D10 |
| Vermelho | D4 |
| Azul | D2 |

## Configuração indispensável

O perfil de velocidade continua disponível no protocolo V3:

```text
@C,UNLOCK*CRC8;
@C,PROFILE,45,85,135*CRC8;
@C,GET*CRC8;
```

A alteração só é aceita dentro da janela de 5 segundos após `UNLOCK`. O perfil é validado e gravado em dois slots alternados da EEPROM.

A programação por blocos continua usando:

```text
@P,BEGIN,TAMANHO,CRC16*CRC8;
@P,DATA,OFFSET,HEX*CRC8;
@P,COMMIT*CRC8;
@P,START*CRC8;
@P,STOP*CRC8;
```

## Teste inicial

1. Selecione `LED_ACTIVE_HIGH` conforme o carrinho.
2. Grave o firmware com as rodas suspensas.
3. Abra o terminal em 38400 baud.
4. Confira o azul: ele deve piscar desconectado e ficar fixo conectado.
5. Envie `@C,UNLOCK*3B;`. O vermelho deve pulsar por 5 segundos.
6. Envie o perfil de teste e observe os LEDs verdes.
7. Teste o programa por blocos e mantenha `@P,STOP*0A;` acessível.

## Watchdog e execução de blocos

O watchdog de 300 ms protege somente o modo manual. Durante `P,START`, o executor mantém o programa ativo até a duração de cada passo terminar; assim, `01 01 32` permanece avançando por 5 segundos. A troca entre passos ocorre na expiração do passo, sem pausa artificial. `P,STOP` e o frame enquadrado `STOP` continuam interrompendo imediatamente o programa e os motores.
