#pragma once

#include <Arduino.h>

// ============================================================
// CONFIGURAÇÃO PRINCIPAL DO ROBÔ
// Edite principalmente LED_ACTIVE_HIGH ao trocar de carrinho.
// true  = LED com catodo comum: HIGH acende
// false = LED com anodo comum: LOW acende
// ============================================================
const bool LED_ACTIVE_HIGH = false;
const uint8_t BLE_CONNECTED_LEVEL = HIGH;

// Motores: Arduino Nano + TB6612FNG
const uint8_t MOTOR_LEFT_IN1 = A3;
const uint8_t MOTOR_LEFT_IN2 = A4;
const uint8_t MOTOR_LEFT_PWM = 3;
const uint8_t MOTOR_RIGHT_IN1 = A1;
const uint8_t MOTOR_RIGHT_IN2 = A2;
const uint8_t MOTOR_RIGHT_PWM = 5;

// HM-10
const uint8_t BLE_STATE_PIN = 7;
const uint32_t BLE_BAUD = 38400;

// LEDs do carrinho
const uint8_t LED_GREEN_1 = 6;
const uint8_t LED_GREEN_2 = 9;
const uint8_t LED_GREEN_3 = 10;
const uint8_t LED_RED = 4;
const uint8_t LED_BLUE = 2;

// Ajustes de comportamento
const uint32_t MOTOR_FAILSAFE_MS = 300;
const uint32_t CONFIG_UNLOCK_MS = 5000;
const uint8_t DEFAULT_PWM_MIN = 45;
const uint8_t DEFAULT_PWM_MED = 85;
const uint8_t DEFAULT_PWM_MAX = 135;
