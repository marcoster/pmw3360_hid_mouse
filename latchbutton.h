#ifndef LATCHBUTTON_H_
#define LATCHBUTTON_H_

#include "Arduino.h"

class LatchButton
{
public:
    LatchButton(int pin_no, int pin_nc, void (*isr)(void), void(*onChange)(bool is_pressed, void *user_context), void *change_user_context);
    void enable();
    bool isPressed();
    void onInterrupt();


private:
    int _pin_no;
    int _pin_nc;
    bool _is_pressed;
    void (*_onChange)(bool is_pressed, void *user_context);
    void (*_isr)();
    void *_change_user_context;
};

#endif

