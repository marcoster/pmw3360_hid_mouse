#include "latchbutton.h"


LatchButton::LatchButton(int pin_no, int pin_nc, void (*isr)(void), void(*onChange)(bool is_pressed, void *user_context), void *change_user_context)
{
    this->_pin_no = pin_no;
    this->_pin_nc = pin_nc;
    this->_is_pressed = false;
    this->_onChange = onChange;
    this->_isr = isr;
    this->_change_user_context = change_user_context;
}

void LatchButton::enable()
{
    pinMode(this->_pin_no, INPUT_PULLUP);
    pinMode(this->_pin_nc, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(this->_pin_no), LatchButton::_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(this->_pin_nc), LatchButton::_isr, CHANGE);
}

bool LatchButton::isPressed()
{
    return this->_is_pressed;
}

void LatchButton::onInterrupt()
{
    int no, nc;
    bool prev = this->_is_pressed;

    no = digitalRead(this->_pin_no);
    nc = digitalRead(this->_pin_nc);

	if(no && !nc) {
		this->_is_pressed = false;
	}

	if(!no && nc) {
		this->_is_pressed = true;
	}

    if(prev != this->_is_pressed) {
        if(this->_onChange != NULL) {
            this->_onChange(this->_is_pressed, this->_change_user_context);
        }
    }
}

