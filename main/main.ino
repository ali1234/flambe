#include <Servo.h>

#include <PinChangeInterrupt.h>
#include <PinChangeInterruptSettings.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptBoards.h>

#include <avr/wdt.h>


typedef enum {Off = 0, Idle = 1, On = 2} State;
typedef enum {Low = 0, Medium = 1, High = 2, NoSignal = 3} Input;


#define C_PWM_IN (10)
#define R1_PWM_OUT (9)
#define R2_PWM_OUT (6)
#define R3_PWM_OUT (7)
#define R4_PWM_OUT (9)


// trigger time bands in microseconds
// less than 500 and greater than 2500 are ignored
unsigned long trigger_bands[4] {500, 1300, 1700, 2500};

#define HYSTERESIS (100)             // hysteresis threshold in microseconds
#define TIMEOUT (WDTO_500MS)         // signal timeout
#define BAUD (57600)                 // debug baud rate


Servo relay1, relay2, relay3, relay4;
State state = Off;
Input input = Low;

// temporary variables
unsigned long rise_time = 0;         // time when input signal went high


void setup() {
  
    Serial.begin(BAUD);
    Serial.print("Reset\n");

    relay1.attach(R1_PWM_OUT);
    relay2.attach(R1_PWM_OUT);
    relay3.attach(R1_PWM_OUT);
    relay4.attach(R1_PWM_OUT);

    relay1.write(0);
    relay2.write(0);
    relay3.write(0);
    relay4.write(0);

    pinMode(C_PWM_IN, INPUT_PULLUP);
    attachPCINT(digitalPinToPinChangeInterrupt(C_PWM_IN), rising, RISING);
  
    wdt_enable(TIMEOUT);

}


// trigger input monitoring

// rising edge interrupt
void rising(void)
{
    wdt_reset(); // tell the watchdog the signal is present
    rise_time = micros(); // record the current time
    
    // set up to catch falling edge
    attachPCINT(digitalPinToPinChangeInterrupt(C_PWM_IN), falling, FALLING);
}


// falling edge interrupt
void falling(void)
{ 
    unsigned long pulse_len = micros() - rise_time; // calculate length of pulse
    if (
        (input == NoSignal) or
        (pulse_len < (trigger_bands[input] - HYSTERESIS)) or
        (pulse_len > (trigger_bands[input+1] + HYSTERESIS))
      ) {
        for (unsigned char i=0; i<3; i++) {
            // find which trigger band the pulse fits in to
            if ((pulse_len > trigger_bands[i]) && (pulse_len < trigger_bands[i+1])) {
                input = i;
                break;
            }
            // if we reach here, the input signal does not fit in any pulse band. treat as no signal.
            input = NoSignal;
        }
    }
    
    // set up to catch next rising edge
    attachPCINT(digitalPinToPinChangeInterrupt(C_PWM_IN), rising, RISING);
}


void loop() {
    // Update state for input:
    switch(input) {
        case Low:
            Serial.print("Input: Low ");
            state = Off;
            break;
        case Medium:
            Serial.print("Input: Medium ");
            if (state != Off) state = Idle;
            break;
        case High:
            Serial.print("Input: High ");
            state = On;
            break;
        case NoSignal:
            Serial.print("Input: NoSignal ");
            state = Off;
            break;
    }
    
    // Update relays for state:
    switch(state) {
        case Off:
            Serial.print("State: Off\n");
            relay1.write(0);
            relay2.write(0);
            relay3.write(0);
            relay4.write(0);
            break;
        case Medium:
            Serial.print("State: Idle\n");
            relay1.write(180);
            relay2.write(0);
            relay3.write(180);
            relay4.write(0);
            break;
        case High:
            Serial.print("State: On\n");
            relay1.write(180);
            relay2.write(180);
            relay3.write(180);
            relay4.write(180);
            break;
        default:
            Serial.print("State: Error\n");
            relay1.write(0);
            relay2.write(0);
            relay3.write(0);
            relay4.write(0);
            break;
    }
    delay(100); // delay 100ms
}
