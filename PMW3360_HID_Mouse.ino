#include <PMW3360.h>
#include <RotaryEncoder.h>
#include <Mouse.h>
#include "latchbutton.h"

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
//---- DEFINITIONS ----//
// User define values
#define PMW_SS          10          // Slave Select pin. Connect this to SS on the module.
#define PMW_MOT         17

#define NUM_BUTTONS        5        // number of buttons attached
#define BUTTON_LEFT_NO     9          // left button pin
#define BUTTON_LEFT_NC     8
#define BUTTON_RIGHT_NO    18          // right button pin
#define BUTTON_RIGHT_NC    19
#define BUTTON_MIDDLE_NO   20
#define BUTTON_MIDDLE_NC   21
#define BUTTON_FORWARD_NO  5
#define BUTTON_FORWARD_NC  4
#define BUTTON_BACK_NO     3
#define BUTTON_BACK_NC     2
#define BUTTON_DPI_NO      16
#define BUTTON_DPI_NC      15

#define BUTTON_LIFTOFF  22

#define ENCODER_A       6
#define ENCODER_B       7

#define NUM_LEDS        3
#define LED_RED         14
#define LED_GREEN       1
#define LED_BLUE        0

#define PMW_USE_INTERRUPT 0
#define NUM_CPI_SETTINGS  4


//---- FUNCTION PROTOTYPES ----//
void button_dpi_on_change(bool is_pressed, void *user_context);
void button_mouse_on_change(bool is_pressed, void *user_context);
void cpi_update();

//---- STRUCTURES ----//
struct cpi_setting {
    uint16_t cpi;
    int8_t red;
    int8_t green;
    int8_t blue;
};

//---- GLOBALS ----//
static int  m_led_pins[NUM_LEDS] = { LED_RED, LED_GREEN, LED_BLUE };
static char m_button_keys[NUM_BUTTONS] = { MOUSE_LEFT, MOUSE_RIGHT, MOUSE_MIDDLE, MOUSE_FORWARD, MOUSE_BACK };
static struct cpi_setting m_cpi_settings[NUM_CPI_SETTINGS] = {
    { .cpi = 500,   .red = 0, .green = 1, .blue = 0 },
    { .cpi = 1000,  .red = 0, .green = 0, .blue = 1 },
    { .cpi = 2000,  .red = 1, .green = 0, .blue = 0 },
    { .cpi = 4000,  .red = 0, .green = 1, .blue = 1 }
};
static uint8_t m_cpi_setting_cur = 2;
static bool m_cpi_setting_changed = false;

// latchbuttons
static LatchButton m_button_left   (BUTTON_LEFT_NO,    BUTTON_LEFT_NC,    +[](){ m_button_left.onInterrupt(); },    button_mouse_on_change, &m_button_keys[0]);
static LatchButton m_button_right  (BUTTON_RIGHT_NO,   BUTTON_RIGHT_NC,   +[](){ m_button_right.onInterrupt(); },   button_mouse_on_change, &m_button_keys[1]);
static LatchButton m_button_middle (BUTTON_MIDDLE_NO,  BUTTON_MIDDLE_NC,  +[](){ m_button_middle.onInterrupt(); },  button_mouse_on_change, &m_button_keys[2]);
static LatchButton m_button_forward(BUTTON_FORWARD_NO, BUTTON_FORWARD_NC, +[](){ m_button_forward.onInterrupt(); }, button_mouse_on_change, &m_button_keys[3]);
static LatchButton m_button_back   (BUTTON_BACK_NO,    BUTTON_BACK_NC,    +[](){ m_button_back.onInterrupt(); },    button_mouse_on_change, &m_button_keys[4]);
static LatchButton m_button_dpi    (BUTTON_DPI_NO,     BUTTON_DPI_NC,     +[](){ m_button_dpi.onInterrupt(); },     button_dpi_on_change,   NULL);

// sensor
PMW3360 m_pmw3360_sensor;
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
    if(m_pmw3360_sensor.begin(PMW_SS)) {
        Serial.println("Sensor initialization successed");
    } else {
        Serial.println("Sensor initialization failed");
    }
    m_pmw3360_sensor.setCPI(2000);

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
    leds_init();
    m_button_left.enable();
    m_button_right.enable();
    m_button_middle.enable();
    m_button_forward.enable();
    m_button_back.enable();
    m_button_dpi.enable();

    m_cpi_setting_changed = true;
}


void loop()
{
    //check_buttons_state();

#if PMW_USE_INTERRUPT == 1
    if(motion) {
#endif
        PMW3360_DATA data = m_pmw3360_sensor.readBurst(PMW3360_BURST_DATA_MIN_SIZE);
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

    if(m_cpi_setting_changed == true) {
        cpi_update();
        m_cpi_setting_changed = false;
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


void leds_init()
{
    for(int i = 0; i < NUM_LEDS; i++) {
        pinMode(m_led_pins[i], OUTPUT);
        digitalWrite(m_led_pins[i], 1);
    }
}


void button_dpi_on_change(bool is_pressed, void *user_context)
{
    if(is_pressed) {
        return;
    }
    m_cpi_setting_cur++;
    if(m_cpi_setting_cur >= NUM_CPI_SETTINGS) {
        m_cpi_setting_cur = 0;
    }
    m_cpi_setting_changed = true;
}

void cpi_update()
{
    struct cpi_setting *cpi = &m_cpi_settings[m_cpi_setting_cur];
    m_pmw3360_sensor.setCPI(cpi->cpi);
    digitalWrite(LED_RED,   !cpi->red);
    digitalWrite(LED_GREEN, !cpi->green);
    digitalWrite(LED_BLUE,  !cpi->blue);
}

void button_mouse_on_change(bool is_pressed, void *user_context)
{
    char key = *((char *)user_context);
    if(is_pressed) {
        Mouse.press(key);
    } else {
        Mouse.release(key);
    }
}
