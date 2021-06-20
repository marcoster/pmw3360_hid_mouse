#include "button.h"

#define DEBOUNCE_TIME   5000    ///< 5000 microseconds = 5 milliseconds

Button::Button(int pin, void (*isr)(void), void (*onChange)(bool is_pressed, void *user_context), void *change_user_context, void (*debounce_isr)())
{
    this->_pin = pin;
    this->_is_pressed = false;
    this->_onChange = onChange;
    this->_isr = isr;
    this->_change_user_context = change_user_context;
    this->_is_stable = true;
    this->_debounce_isr = debounce_isr;
}

void Button::enable()
{
    pinMode(this->_pin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(this->_pin), Button::_isr, CHANGE);
}

bool Button::isPressed()
{
    return this->_is_pressed;
}

void Button::onInterrupt()
{
    bool prev = this->_is_pressed;
    bool state = !digitalRead(this->_pin);

    if(prev == state) {
        return;
    }

    if(!this->_is_stable) {
        return;
    }
    this->_is_stable = false;
    this->_debounce_timer.begin(this->_debounce_isr, DEBOUNCE_TIME);
    this->_is_pressed = state;

    if(this->_onChange != NULL) {
        this->_onChange(this->_is_pressed, this->_change_user_context);
    }
}

void Button::onDebounce()
{
    this->_is_stable = true;
    this->_debounce_timer.end();

}
