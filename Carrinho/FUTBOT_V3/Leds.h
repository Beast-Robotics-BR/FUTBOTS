#pragma once

#include <Arduino.h>
#include "Config.h"

namespace RoverLeds {

inline uint8_t activeLevel() { return LED_ACTIVE_HIGH ? HIGH : LOW; }
inline uint8_t inactiveLevel() { return LED_ACTIVE_HIGH ? LOW : HIGH; }

inline void write(uint8_t pin, bool on) {
  digitalWrite(pin, on ? activeLevel() : inactiveLevel());
}

inline void allOff() {
  write(LED_GREEN_1, false);
  write(LED_GREEN_2, false);
  write(LED_GREEN_3, false);
  write(LED_RED, false);
  write(LED_BLUE, false);
}

inline void begin() {
  pinMode(LED_GREEN_1, OUTPUT);
  pinMode(LED_GREEN_2, OUTPUT);
  pinMode(LED_GREEN_3, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  allOff();
}

inline void showLevel(uint8_t level) {
  write(LED_GREEN_1, level >= 1);
  write(LED_GREEN_2, level >= 2);
  write(LED_GREEN_3, level >= 3);
}

inline void update(bool connected, bool unlocked, bool running, bool uploading, bool error, uint8_t level) {
  const uint32_t now = millis();
  allOff();

  // Vermelho: erro fixo; modo de configuração: pulso lento.
  if (error) write(LED_RED, true);
  else if (unlocked) write(LED_RED, (now / 250) % 2 == 0);

  // Azul: conectado fixo; desconectado pisca lentamente.
  if (connected) write(LED_BLUE, true);
  else write(LED_BLUE, (now / 500) % 2 == 0);

  // Verde: nível de potência em repouso; sequência de movimento em execução.
  if (running) {
    uint8_t phase = (now / 180) % 3;
    write(LED_GREEN_1, phase == 0);
    write(LED_GREEN_2, phase == 1);
    write(LED_GREEN_3, phase == 2);
  } else if (uploading) {
    write(LED_GREEN_1, (now / 160) % 2 == 0);
    write(LED_GREEN_2, (now / 160) % 2 == 0);
  } else showLevel(level);
}

} // namespace RoverLeds
