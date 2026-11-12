#ifndef TEST_ARDUINO_H
#define TEST_ARDUINO_H
#include <stdint.h>
#include <stddef.h>
#define A4 18
#define A5 19
#ifndef INTERNAL
#define INTERNAL 3 /* Mirrors Arduino AVR analogReference macro. */
#endif
#define HIGH 1
#define LOW 0
#define INPUT_PULLUP 2
#define OUTPUT 1
void pinMode(uint8_t, uint8_t);
void digitalWrite(uint8_t, uint8_t);
int digitalRead(uint8_t);
uint32_t micros(void);
uint32_t millis(void);
void delay(uint32_t);
void delayMicroseconds(uint32_t);
#endif
