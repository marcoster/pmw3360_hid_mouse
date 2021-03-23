#include <PMW3360.h>
#include <RotaryEncoder.h>
#include <Mouse.h>

// WARNING: This example works only in Native USB supporting boards (e.g., Micro, Leonardo, etc.)

/* 
# PIN CONNECTION
  * MI = MISO
  * MO = MOSI
  * SS = Slave Select / Chip Select
  * SC = SPI Clock
  * MT = Motion (active low interrupt line)
  * RS = Reset
  * GD = Ground
  * VI = Voltage in up to +5.5V 

  # PMW3360 Module
Module   Arduino
  RS --- (NONE)
  GD --- GND
  MT --- (NONE)
  SS --- Pin_10   (use this pin to initialize a PMW3360 instance)
  SC --- SCK 
  MO --- MOSI
  MI --- MISO
  VI --- 5V

  # Button switches
  Button 1: Common pin - GND / Normally Open pin - Arduino Pin_2
  Button 2: Common pin - GND / Normally Open pin - Arduino Pin_3

# PMW3360_DATA struct format and description
  - PMW3360_DATA.isMotion      : bool, True if a motion is detected. 
  - PMW3360_DATA.isOnSurface   : bool, True when a chip is on a surface 
  - PMW3360_DATA.dx, data.dy   : integer, displacement on x/y directions.
  - PMW3360_DATA.SQUAL         : byte, Surface Quality register, max 0x80
                               * Number of features on the surface = SQUAL * 8
  - PMW3360_DATA.rawDataSum    : byte, It reports the upper byte of an 18‐bit counter 
                               which sums all 1296 raw data in the current frame;
                               * Avg value = Raw_Data_Sum * 1024 / 1296
  - PMW3360_DATA.maxRawData    : byte, Max/Min raw data value in current frame, max=127
    PMW3360_DATA.minRawData
  - PMW3360_DATA.shutter       : unsigned int, shutter is adjusted to keep the average
                               raw data values within normal operating ranges.
 */

// User define values
#define PMW_SS          10          // Slave Select pin. Connect this to SS on the module.
#define PMW_MOT         17
#define NUMBTN          5        // number of buttons attached
#define BTN_LEFT_NO     9          // left button pin
#define BTN_LEFT_NC     8
#define BTN_RIGHT_NO    18          // right button pin
#define BTN_RIGHT_NC    19
#define BTN_MIDDLE_NO   20
#define BTN_MIDDLE_NC   21
#define BTN_FORWARD_NO  5
#define BTN_FORWARD_NC  4
#define BTN_BACK_NO     3
#define BTN_BACK_NC     2
#define ENCODER_A       6
#define ENCODER_B       7

#define PMW_USE_INTERRUPT 0
#define DEBOUNCE          10

// buttons
int btn_pins[NUMBTN] = { BTN_LEFT_NO, BTN_RIGHT_NO, BTN_MIDDLE_NO, BTN_FORWARD_NO, BTN_BACK_NO };
char btn_keys[NUMBTN] = { MOUSE_LEFT, MOUSE_RIGHT, MOUSE_MIDDLE, MOUSE_FORWARD, MOUSE_BACK };
bool btn_state[NUMBTN] = { false, false };
uint8_t btn_buffers[NUMBTN] = { 0xFF, 0xFF };
unsigned long lastButtonCheck = 0;

// sensor
PMW3360 sensor;
uint16_t xmax = 0;
uint16_t ymax = 0;

// encoder
RotaryEncoder encoder(ENCODER_A, ENCODER_B, RotaryEncoder::LatchMode::TWO03);
volatile bool encoderEvent = false;
#if PMW_USE_INTERRUPT == 1
volatile bool motion = false;
#endif


void setup()
{
    Serial.begin(9600);  
    //while(!Serial);  // remove for immediate operation

    // PMW Initialization
    if(sensor.begin(PMW_SS)) {
        Serial.println("Sensor initialization successed");
    } else {
        Serial.println("Sensor initialization failed");
    }
    sensor.setCPI(2000);

#if PMW_USE_INTERRUPT == 1
    pinMode(PMW_MOT, INPUT_PULLUP);
    //attachInterrupt(digitalPinToInterrupt(MOT), motionDetected, FALLING);
    //if(digitalRead(MOT) == LOW) {
    //  motion = true;
    //}
#endif

    attachInterrupt(ENCODER_A, checkPosition, CHANGE);
    attachInterrupt(ENCODER_B, checkPosition, CHANGE);
    Mouse.begin();
    buttons_init();
}


void loop()
{
    check_buttons_state();

#if PMW_USE_INTERRUPT == 1
    if(motion) {
#endif
        PMW3360_DATA data = sensor.readBurst(PMW3360_BURST_DATA_MIN_SIZE);
        if(data.isOnSurface && data.isMotion) {
            int mdx = constrain(data.dx, -127, 127);
            int mdy = constrain(data.dy, -127, 127);
          
            Mouse.move(mdx, mdy, 0);
            if(abs(data.dx) > xmax) {
                xmax = abs(data.dx);
            }
            if(abs(data.dy) > ymax) {
                ymax = abs(data.dy);
            }
            //Serial.printf("%d / %d\n", data.dx, data.dy);
            //Serial.printf("%d / %d\n", xmax, ymax);
        }
#if PMW_USE_INTERRUPT == 1
        motion = false;
        sei();
    }
#endif


    if(encoderEvent == true) {
        static int pos = 0;
        int newPos = encoder.getPosition();
        encoderEvent = false;
        //Serial.printf("enc pos/dir: %d/%d\n", pos, dir);
        Mouse.move(0, 0, (pos - newPos));
        pos = newPos;
    }
}


void checkPosition()
{
    encoder.tick();
    encoderEvent = true;
}

#if PMW_USE_INTERRUPT == 1
void motionDetected()
{
    motion = true;
}
#endif


void buttons_init()
{
    for(int i = 0; i < NUMBTN; i++) {
        pinMode(btn_pins[i], INPUT_PULLUP);
    }
}


void check_buttons_state() 
{
    unsigned long elapsed = micros() - lastButtonCheck;
  
    if(elapsed < (DEBOUNCE * 1000UL / 8)) {
        return;
    }
  
    lastButtonCheck = micros();
    
    // Fast Debounce (works with 0 latency most of the time)
    for(int i=0;i < NUMBTN ; i++) {
        int state = digitalRead(btn_pins[i]);
        btn_buffers[i] = btn_buffers[i] << 1 | state; 

        if(!btn_state[i] && btn_buffers[i] == 0xFE) {  // button pressed for the first time
            Mouse.press(btn_keys[i]);
            btn_state[i] = true;
        } else if( (btn_state[i] && btn_buffers[i] == 0x01) || // button released after stabilized press
                   (btn_state[i] && btn_buffers[i] == 0xFF) ) { // force release when consequent off state (for the DEBOUNCE time) is detected 
            Mouse.release(btn_keys[i]);
            btn_state[i] = false;
        }
    }
}
