#ifndef SLIMEVR_LEDMGR_H_
#define SLIMEVR_LEDMGR_H_

#include <Arduino.h>

namespace LEDMGR {
    void On(uint8_t pin);
    void Off(uint8_t pin);
    void Blink(uint8_t pin, unsigned long time, uint8_t direction = LOW);
    void Pattern(uint8_t pin, unsigned long timeon, unsigned long timeoff, int times, uint8_t direction = LOW);
}

#endif // SLIMEVR_LEDMGR_H_