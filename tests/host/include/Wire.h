#ifndef TEST_WIRE_H
#define TEST_WIRE_H
#include <stdint.h>
#include <stddef.h>
class TwoWire {
public:
  void begin();
  void setClock(uint32_t);
  void setWireTimeout(uint32_t, bool);
  void clearWireTimeoutFlag();
  bool getWireTimeoutFlag();
  void beginTransmission(uint8_t);
  size_t write(uint8_t);
  uint8_t endTransmission(bool stop=true);
  uint8_t requestFrom(uint8_t, uint8_t);
  int available();
  int read();
};
extern TwoWire Wire;
#endif
