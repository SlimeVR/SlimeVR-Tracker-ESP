#include "ledmgr.h"

namespace LEDMGR
{
    /*!
    *  @brief  Turn a LED on
    *  @param  pin
    *          LED pin
    */
    void On(uint8_t pin)
    {
#if ENABLE_LEDS
        digitalWrite(pin, LOW);
#endif
    }

    /*!
    *  @brief  Turn a LED off
    *  @param  pin
    *          LED pin
    */
    void Off(uint8_t pin)
    {
        digitalWrite(pin, HIGH);
    }

    /*!
    *  @brief  Blink a LED for [time]ms
    *  @param  pin
    *          LED pin
    *  @param  time
    *          Amount of ms to turn the LED on
    *  @param  direction
    *          Direction turn the LED on, usually LOW
    */
    void Blink(uint8_t pin, unsigned long time, uint8_t direction)
    {
        digitalWrite(pin, direction);
        delay(time);
        digitalWrite(pin, direction ^ 1);
    }

    /*!
    *  @brief  Show a pattern on a LED
    *  @param  pin
    *          LED pin
    *  @param  timeon
    *          Amount of ms to turn the LED on
    *  @param  timeoff
    *          Amount of ms to turn the LED off
    *  @param  times
    *          Amount of times to display the pattern
    *  @param  direction
    *          Direction turn the LED on, usually LOW
    */
    void Pattern(uint8_t pin, unsigned long timeon, unsigned long timeoff, int times, uint8_t direction)
    {
        for (int i = 0; i < times; i++)
        {
            digitalWrite(pin, direction);
            delay(timeon);
            digitalWrite(pin, direction ^ 1);
            delay(timeoff);
        }
    }
}