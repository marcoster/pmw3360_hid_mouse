#ifndef BBUTTON_H_
#define BBUTTON_H_

#include "Arduino.h"

class Button
{
    public:
        Button(int pin, void (*isr)(void), void (*onChange)(bool is_pressed, void *user_context), void *change_user_context, void (*onDebounce)());
        void enable();
        bool isPressed();
        void onInterrupt();
        void onDebounce();

    private:
        int _pin;
        bool _is_pressed;
        void (*_onChange)(bool is_pressed, void *user_context);
        void (*_isr)();
        void *_change_user_context;
        IntervalTimer _debounce_timer;
        bool _is_stable;
        void (*_debounce_isr)();

};

#endif
