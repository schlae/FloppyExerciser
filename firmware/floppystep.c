// Fuses E:F7, H:D9, L:E2
#define F_CPU (8000000UL)
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>

#define ANODE1 5
#define ANODE2 4
#define C_A 3 // PB
#define C_D 2 // PB
#define C_E 1 // PB
#define C_F 0 // PB

#define C_C 3 // PC
#define C_G 2 // PC
#define C_DP 1 // PC
#define C_B 0 // PC

#define PIN_DIR 0 // PD
#define PIN_STEP 1 // PD

// Averaging filter for removing ADC noise
#define FILT_BITS (5)    // Number of fixed point fractional bits
#define AVG_FILT(input, output)  output = (output - (output >> FILT_BITS) + (input))
#define GET_AVG(v) ((v + _BV(FILT_BITS) / 2) >> FILT_BITS) // Rounds up

//   AAA       DDD
//  F   B     C   E
//   GGG       GGG
//  E   C     B   F
//   DDD       AAA
// 0000 0000 -> CG.B ADEF
// 0 = 1001 1111
// 1 = 1001 0000
// 2 = 0101 1110
// 3 = 1101 1100
// 4 = 1101 0001
// 5 = 1100 1101
// 6 = 1100 1111
// 7 = 1001 1000
// 8 = 1101 1111
// 9 = 1101 1101
// A = 1101 1011
// B = 1100 0111
// C = 0000 1111
// D = 1101 0110
// E = 0100 1111
// F = 0100 1011

uint8_t digit_table[] = { 0x9F, 0x90, 0x5E, 0xDC,
                          0xD1, 0xCD, 0xCF, 0x98,
                          0xDF, 0xDD, 0xDB, 0xC7,
                          0x0F, 0xD6, 0x4F, 0x4B };

uint8_t dpoints = 0;

uint8_t adc_chan = 0;

// FIXME all these variables
uint16_t val_duty = 0;
uint16_t val_freq = 0;
volatile uint16_t val_vref = 0;
uint8_t val_range = 0;

volatile uint16_t val_actual_freq = 0;
volatile uint16_t val_actual_duty = 0;
volatile uint16_t val_actual_duty_old = 0;
uint8_t disp_freq = 0;
uint8_t disp_duty = 0;

// Wheel tracking state
volatile uint8_t wheel_val;
uint8_t wheel_state_old = 0;
volatile uint16_t blink_counter = 0;

uint8_t sel_track = 0;
uint8_t cur_track = 0;

// Puts a digit on the 7-segment display
void load_digit(uint8_t val, uint8_t anode)
{
    uint8_t d = ~digit_table[val]; // Invert bits since LED on when 0
    uint8_t dpoint;

    dpoint = (dpoints >> (anode - 1)) & 1;

    PORTB = d & 0x0F;
    PORTC = ((anode << 4) | (d >> 4)) & ~(dpoint << 1);
}

// Hacky binary to BCD converter
uint8_t bin2bcd(uint8_t val)
{
    char c[4];
    itoa(val, c, 10);
    if (val < 10) {
        return c[0] - 0x30;
    } else {
        return (c[1] - 0x30) | ((c[0] - 0x30) << 4);
    }
}

// Set the ADC channel mux
void adc_set_chan(uint8_t chan)
{
    ADMUX = (ADMUX & 0xF0) | (chan & 0xf);
    adc_chan = chan;
}

// Convert all ADC channels and filter results
ISR(ADC_vect)
{
    switch (adc_chan) {
        case 0x6: // Duty cycle
            AVG_FILT(255 - ADCH, val_duty);
            adc_set_chan(0x7);
            val_actual_duty = (GET_AVG(val_duty));
            if (val_actual_duty != val_actual_duty_old) {
                val_actual_duty_old = val_actual_duty;
            }
            break;
        case 0x7: // Frequency
            AVG_FILT(255 - ADCH, val_freq);
            adc_set_chan(0xe);
            // Ensure there is no 0, since 0 Hz makes no sense
            val_actual_freq = ((GET_AVG(val_freq) * 99L) >> 8) + 1;
            break;
        default:
        case 0xe: // Internal bandgap reference voltage
            AVG_FILT(ADCH, val_vref);
            adc_set_chan(0x6);
            break;
    }
    _delay_us(100);
    ADCSRA |= _BV(ADSC);
}

void wheel_increment ()
{
    blink_counter = 0;
    wheel_val++;
    if (wheel_val > 99) wheel_val = 99;
}

void wheel_decrement()
{
    blink_counter = 0;
    wheel_val--;
    if (wheel_val == 255) wheel_val = 0;
}

// Interrupt on change
ISR(PCINT3_vect)
{
    uint8_t wheel_state = PINE & 0x3;
    uint8_t st = wheel_state | (wheel_state_old << 4);
    // Compare to wheel_state_old, ignore B input due to detent.
    if (wheel_state != wheel_state_old) {
        // Present state, next state
        // 00 -> 01 clockwise
        // 10 -> 11 counterclockwise
        // 11 -> 10 clockwise
        // 01 -> 00 counterclockwise
        if ((st == 0x01) || (st == 0x32)) {
            wheel_increment();
        }
        if ((st == 0x23) || (st == 0x10)) {
            wheel_decrement();
        }
        wheel_state_old = wheel_state;
    }
}

// Timer interrupt for pulsing stepper
ISR(TIMER3_COMPA_vect)
{
        if (sel_track > cur_track) {
            // Pulse increment, step in
            PORTD |= _BV(PIN_DIR);
            PORTD |= _BV(PIN_STEP);
            _delay_us(100);
            PORTD &= ~_BV(PIN_STEP);
            cur_track++;
        }

        if (sel_track < cur_track) {
            // Pulse decrement, step out
            PORTD &= ~_BV(PIN_DIR);
            PORTD |= _BV(PIN_STEP);
            _delay_us(100);
            PORTD &= ~_BV(PIN_STEP);
            cur_track--;
        }
}

int main (void)
{
    uint8_t displayval;
    uint8_t counter = 0;
    bool blink = false;

    // Set up IO ports
    DDRD = 0xFB; // Track0 sensor input, all others are outputs
    PORTD = 0x00;
    DDRB = 0x0F; // Output for LED cathode drivers
    DDRC = 0x3F; // Output for LED cathode and anode drivers
    DDRE = 0x00; // Wheel inputs, ADC inputs

    // Setup interrupt on change
    PCMSK3 = _BV(PCINT25) | _BV(PCINT24);
    PCICR = _BV(PCIE3);

    // Setup timer
    TCCR3A = 0;
    TCCR3B = _BV(WGM32) | 5; // Divide by 1024.
    OCR3A = 50; // FIXME
    TIMSK3 = _BV(OCIE3A); // Interrupt on compare match

    // Setup ADC
    ADMUX = _BV(REFS0) | _BV(ADLAR); // Left adjust results, Vref=VCC.
    ADCSRA = _BV(ADEN) | _BV(ADIE) | 7; // Division factor of 128
    adc_set_chan(6);
    sei();
    ADCSRA |= _BV(ADSC);

    dpoints = 1;

    while (1) {
        counter++;

        // Selecting a new track
        if (wheel_val != sel_track) {
            if (!blink) {
                blink = true;
                blink_counter = 0;
            }
            displayval = bin2bcd(wheel_val);
        } else {
            blink = false;
            displayval = bin2bcd(sel_track);
        }

        // Check for selection timeout
        if (blink) {
            blink_counter++;
            if (blink_counter > 1000) {
                blink = false;
                wheel_val = sel_track;
            }
        }

        if (!(PINB & _BV(6)) & blink) {
            blink = false;
            sel_track = wheel_val;
        }

        

        // Display
        if (!blink || (counter & 0x40)) {
            load_digit(displayval & 0x0F, 1);
            _delay_ms(1);
            load_digit(displayval >> 4, 2);
            _delay_ms(1);
        } else {
            load_digit(0, 0);
            _delay_ms(2);
        }
    }
    return 0;
}
