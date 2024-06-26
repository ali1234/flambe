// rev 54.2

#include <Servo.h>

#include <PinChangeInterrupt.h>
#include <PinChangeInterruptSettings.h>
#include <PinChangeInterruptPins.h>
#include <PinChangeInterruptBoards.h>


typedef enum {Off = 0, Idle = 1, On = 2, Error = 3} State;
char *state_names[] = {"Off", "Idle", "On", "Error"};

typedef enum {Low = 0, Medium = 1, High = 2, NoSignal = 3} Input;
char *input_names[] = {"Low", "Medium", "High", "NoSignal"};


#define C_PWM_IN (10)
#define R1_PWM_OUT (9)
#define R2_PWM_OUT (6)
#define R3_PWM_OUT (5)
#define FAN_PWM_OUT (3)

#define R_OFF (45)
#define R_ON (135)
#define R_MID (90) // not used outside of testing

#define F_OFF (45)
#define F_MIN (80)
#define F_MAX (180)
#define F_THRESH (1900) // minimum input level to enable fan


// trigger time bands in microseconds
// less than 500 and greater than 2500 are considered no signal
unsigned long trigger_bands[4] {500, 1300, 1700, 2500};

#define HYSTERESIS (100)     // hysteresis threshold in microseconds
#define ON_REPEATS (3)       // must receive this many High inputs to turn On
#define ERR_REPEATS (3)      // must have this many NoSignal inputs to enter Error state
#define TIMEOUT (100)        // signal timeout in milliseconds
#define BAUD (57600)         // debug baud rate


Servo relay1, relay2, relay3, fan;
volatile Input v_input = NoSignal;
volatile unsigned long v_pulse_len = 0;

// temporary variables
volatile unsigned long v_rise_time_micros = 0;  // time when input signal went high in microseconds
volatile unsigned long v_rise_time_millis = 0;  // time when input signal went high in milliseconds
volatile unsigned long v_repeat_count = 0;

void set_relays(unsigned char a, unsigned char b, unsigned char c) {
    relay1.write(a);
    relay2.write(b);
    relay3.write(c);
}


void setup() {
    cli(); // disable interrupt
  
    Serial.begin(BAUD);
    Serial.print("Reset\n");

    relay1.attach(R1_PWM_OUT);
    relay2.attach(R2_PWM_OUT);
    relay3.attach(R3_PWM_OUT);
    fan.attach(FAN_PWM_OUT);

    set_relays(R_OFF, R_OFF, R_OFF);
    fan.write(F_OFF);

    pinMode(C_PWM_IN, INPUT_PULLUP);
    attachPCINT(digitalPinToPinChangeInterrupt(C_PWM_IN), rising, RISING);
   
    sei(); // enable interrupt
}


// trigger input monitoring

// rising edge interrupt
void rising(void)
{
    v_rise_time_micros = micros(); // record the current time
    v_rise_time_millis = millis();
    
    // set up to catch falling edge
    attachPCINT(digitalPinToPinChangeInterrupt(C_PWM_IN), falling, FALLING);
}


// falling edge interrupt
void falling(void)
{ 
    unsigned long pulse_len = micros() - v_rise_time_micros; // calculate length of pulse
    v_pulse_len = pulse_len;
    if (
        (pulse_len > (trigger_bands[v_input] - HYSTERESIS)) &&
        (pulse_len < (trigger_bands[v_input+1] + HYSTERESIS))
    ) {
        v_repeat_count++;      
    } else {
        for (unsigned char i=0; i<3; i++) {
            // find which trigger band the pulse fits in to
            if ((pulse_len > trigger_bands[i]) && (pulse_len < trigger_bands[i+1])) {
                v_input = i;
                break;
            }
            // if we reach here, the input signal does not fit in any pulse band. treat as no signal.
            v_input = NoSignal;
        }
        v_repeat_count = 1;
    }
    
    // set up to catch next rising edge
    attachPCINT(digitalPinToPinChangeInterrupt(C_PWM_IN), rising, RISING);
}


void flambe(Input input, unsigned long pulse_len) {
    // Start in Error state so that we can't immediately go full on at power up.
    static State state = Error;
  
    // Update state for input:
    switch(input) {
        case Low:
            state = Off;
            break;
        case Medium:
            if (state == Error) state = Off;
            else if (state != Off) state = Idle;
            break;
        case High:
            if (state != Error && v_repeat_count >= ON_REPEATS) state = On;
            break;
        case NoSignal:
            if (v_repeat_count >= ERR_REPEATS) state = Error;
            break;
        default:
            state = Error;
            break;
    }

    Serial.print(", State: ");
    Serial.print(state_names[state]);
    
    // Update relays for state:
    switch(state) {
        case Idle:
            set_relays(R_ON, R_OFF, R_ON);
            break;
        case On:
            set_relays(R_ON, R_ON, R_ON);
            break;
        default: // handles Off and Error
            set_relays(R_OFF, R_OFF, R_OFF);
            break;
    }

    Serial.print(", Fan: ");

    // Update fan for state.
    // For the fan to be On we must at least be in On state
    // (ie trigger High.)
    if (state == On && pulse_len > F_THRESH) {
        unsigned long scaled = F_MIN + ((F_MAX - F_MIN) * (pulse_len - F_THRESH) * 1.0 / (trigger_bands[3] - F_THRESH));
        fan.write(scaled);
        Serial.print(scaled);
    } else {
        fan.write(F_OFF);
        Serial.print(F_OFF);
    }
}


void passthrough(Input input, unsigned long pulse_len) {
    // Mirror input on the output:
    switch(input) {
        case Medium:
            set_relays(R_MID, R_MID, R_MID);
            break;
        case High:
            set_relays(R_ON, R_ON, R_ON);
            break;
        default: // handles Low and NoSignal
            set_relays(R_OFF, R_OFF, R_OFF);
            break;            
    }
}


void loop() {
    Input input;
    unsigned long pulse_len;

    cli();
    if ((millis() - v_rise_time_millis) > TIMEOUT) {
        if (v_input == NoSignal) v_repeat_count++;
        else {
            v_repeat_count = 1;
            v_input = NoSignal;
        }
    }
    input = v_input;
    pulse_len = v_pulse_len;
    sei();

    Serial.print("Input: ");
    Serial.print(input_names[input]);
    Serial.print(", Repeats: ");
    Serial.print(v_repeat_count);
    
    //passthrough(input, pulse_len);
    flambe(input, pulse_len);

    Serial.print("\n");

    delay(100);
}
