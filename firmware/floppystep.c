// Fuses E:F7, H:D9, L:E2
#define F_CPU (8000000UL)
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>

#include "segment_font.h"

#define ANODE1 5
#define ANODE2 4
#define C_A 3 // PB
#define C_D 2 // PB
#define C_E 1 // PB
#define C_F 0 // PB

#define PIN_MENU_BUTTON 7 // PB
#define PIN_ENC_BUTTON 6 // PB

#define C_C 3 // PC
#define C_G 2 // PC
#define C_DP 1 // PC
#define C_B 0 // PC

#define PIN_DIR 0 // PD
#define PIN_STEP 1 // PD
#define PIN_TRACK0 2 // PD
#define PIN_TG43 3 // PD

// Averaging filter for removing ADC noise
#define FILT_BITS (5)    // Number of fixed point fractional bits
#define AVG_FILT(input, output)  output = (output - (output >> FILT_BITS) + (input))
#define GET_AVG(v) ((v + _BV(FILT_BITS) / 2) >> FILT_BITS) // Rounds up

#define STEP_MODE_WHOLE 0
#define STEP_MODE_DOUBLE 1
#define STEP_MODE_HALF 2
#define STEP_MODE_15 3 // Step-and-a half mode

uint8_t step_mode = 0;

volatile uint8_t dpoints = 0;
volatile char displaychars[2];
volatile bool blink = false;

uint8_t adc_chan = 0;

uint16_t val_pw = 0;
uint16_t val_rate = 0;
volatile uint16_t val_vref = 0;
uint8_t val_range = 0;

volatile uint16_t val_actual_rate = 0;
volatile uint16_t val_actual_pw = 0;
volatile uint16_t val_actual_pw_old = 0;
uint8_t disp_freq = 0;
uint8_t disp_duty = 0;

// Wheel tracking state
volatile int16_t wheel_val;
uint8_t wheel_state_old = 0;
volatile uint16_t blink_counter = 0;

volatile int16_t sel_track = 0;
volatile int16_t cur_track = 0;

// Puts a digit on the 7-segment display
void load_digit(uint8_t val, uint8_t anode)
{
//   AAA       DDD
//  F   B     C   E
//   GGG       GGG
//  E   C     B   F
//   DDD       AAA
//              26x1 0345
// 0000 0000 -> CG.B ADEF

    uint8_t d2 = ~segment_font_table[val]; // Invert bits since LED on when 0
    uint8_t d;
    uint8_t dpoint;

    // Remap segment font to our GPIO mapping
    d = (d2 & 0x3) << 3; // Bits 0 and 1
    d |= (d2 & _BV(2)) << 5; // Bit 2
    d |= (d2 & _BV(3)) >> 1; // Bit 3
    d |= (d2 & _BV(4)) >> 3; // Bit 4
    d |= (d2 & _BV(5)) >> 5; // Bit 5
    d |= (d2 & _BV(6)); // Bit 6
    d |= _BV(5); // DP off by default

    dpoint = (dpoints >> (anode - 1)) & 1;

    PORTB = d & 0x0F;
    PORTC = ((anode << 4) | (d >> 4)) & ~(dpoint << 1);
}

// Timer interrupt for display
ISR(TIMER0_COMPA_vect)
{

    static uint8_t counter = 0;

    counter++;

    // Should display be blank?
    if (!blink || (counter & 0x80)) {
        // Alternate between digits
        if (PORTC & _BV(5)) {
            load_digit(displaychars[1], 1);
        } else {
            load_digit(displaychars[0], 2);
        }
    } else {
        load_digit(0, 0);
    }

}

// Display binary value
void disp_bin(int16_t val)
{
    char c[10];
    if (val > 199) val = 199;
    if (val < -9) val = -9;
    itoa(val, c, 10);
    // Right justify result
    if ((val < 10) && (val > -1)) {
        displaychars[0] = ' ';
        displaychars[1] = c[0];
        dpoints &= ~_BV(1);
    } else if (val >= 100) {
        displaychars[0] = c[1];
        displaychars[1] = c[2];
        dpoints |= _BV(1); // This is our hundreds digit
    } else {
        displaychars[0] = c[0];
        displaychars[1] = c[1];
        dpoints &= ~_BV(1);
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
        case 0x6: // Pulse width
            AVG_FILT(255 - ADCH, val_pw);
            adc_set_chan(0x7);
            val_actual_pw = (GET_AVG(val_pw));
            if (val_actual_pw != val_actual_pw_old) {
                val_actual_pw_old = val_actual_pw;
            }
            break;
        case 0x7: // Pulse rate
            AVG_FILT(255 - ADCH, val_rate);
            adc_set_chan(0xe);
            val_actual_rate = (GET_AVG(val_rate));
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

void wheel_increment()
{
    blink_counter = 0;
    wheel_val++;
    if (wheel_val > 199) wheel_val = 199;
}

void wheel_decrement()
{
    blink_counter = 0;
    wheel_val--;
    if (wheel_val < -9) wheel_val = -9;
}

// Interrupt on change for encoder
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

// Time that step pulse is active
void pulse_delay()
{
    int count;
    for (count = 0; count < val_actual_pw; count++) {
        _delay_us(1);
    }
}

uint8_t a2_step_table_single[] = {1, 2, 4, 8, 1, 2, 4, 8};
uint8_t a2_step_table_half[] = {1, 3, 2, 6, 4, 12, 8, 9};

// Sets manual stepper pins to specified position
void set_step(uint8_t pos)
{
    PORTD &= 0x0F; // MS nybble is our stepper drive
    if ((step_mode == STEP_MODE_HALF) || (step_mode == STEP_MODE_15)) {
        PORTD |= (a2_step_table_half[pos & 0x7]) << 4;
    } else {
        PORTD |= (a2_step_table_single[pos & 0x7]) << 4;
    }
}

// Timer interrupt for pulsing stepper
ISR(TIMER3_COMPA_vect)
{
    int16_t sel_track_phy;
    int16_t track43 = 43;

    if (step_mode == STEP_MODE_DOUBLE) {
        sel_track_phy = sel_track * 2;
        track43 = 43 * 2;
    } else if (step_mode == STEP_MODE_15) {
        sel_track_phy = sel_track * 3; // Three half-steps
        track43 = 43 * 3 / 2;
    } else {
        sel_track_phy = sel_track;
    }

    if (sel_track_phy > cur_track) {
        // Pulse increment, step in
        cur_track++;
        PORTD |= _BV(PIN_DIR);
        _delay_us(10);
        PORTD |= _BV(PIN_STEP);
        set_step(cur_track);
        pulse_delay();
        PORTD &= ~_BV(PIN_STEP);
        _delay_us(10);
        PORTD &= ~_BV(PIN_DIR);
    }

    if (sel_track_phy < cur_track) {
        // Pulse decrement, step out
        cur_track--;
        PORTD &= ~_BV(PIN_DIR);
        _delay_us(10);
        PORTD |= _BV(PIN_STEP);
        set_step(cur_track);
        pulse_delay();
        PORTD &= ~_BV(PIN_STEP);
    }

    // Update step timer from ADC

    // Nominal value of 63 = 8ms, which is a good typical step time
    // Min is 0 >> 2 + 1 = 1 (0.1ms)
    // Max is 127 + 1 = 128 (16ms)
    // Middle scale is 127 >> 2 + 1 = 64

    // Timer is 0.128ms per bit
    OCR3A = (val_actual_rate >> 2) + 1;

    // Update TG43 signal
    if (cur_track >= track43) {
        PORTD |= _BV(PIN_TG43);
    } else {
        PORTD &= ~_BV(PIN_TG43);
    }
}

// Check debounced state of menu button
bool menu_button_check()
{
    static uint8_t debounce = 0;
    if (!(PINB & _BV(PIN_MENU_BUTTON))) {
        if (debounce < 10) {
            debounce++;
            if (debounce == 10) {
                return true;
            }
        }
    } else {
        debounce = 0;
    }
    return false;
}

// Single step, double step, half step, 1.5x step
char step_menu_table[][2] = {"SS", "DS", "HS", "15"};

// Menu picker UI
int8_t select_menu(char menu_array[][2], uint8_t max_items)
{
    int16_t old_wheel = wheel_val;
    int8_t menu_selection = 0;
    int8_t picked;
    while (1) {
        menu_selection = wheel_val;
        if (menu_selection >= max_items) {
            menu_selection = max_items - 1;
            wheel_val = max_items - 1;
        }

        if (menu_selection < 0) {
            menu_selection = 0;
            wheel_val = 0;
        }

        // Hitting menu button again exits the menu
        if (menu_button_check()) {
            picked = -1;
            break;
        }

        // Show the menu option
        displaychars[0] = menu_array[menu_selection][0];
        displaychars[1] = menu_array[menu_selection][1];

        // When encoder button is pushed, select the item
        if (!(PINB & _BV(PIN_ENC_BUTTON))) {
            picked = menu_selection;
            break;
        }
        _delay_ms(1);
    }
    wheel_val = old_wheel;
    return picked;
}

// Menu on powerup selects type of step
void step_select_menu()
{
    int8_t val = select_menu(step_menu_table, 4);
    if (val != -1) step_mode = val;
}

// Alignment mode
void align_mode(uint8_t max_track)
{
    int16_t old_wheel = wheel_val;
    int16_t prev_wheel = wheel_val;
    wheel_val = 0;
    sel_track = 0; // Go to track 0

    while (1)
    {
        // Check for exit
        if (menu_button_check()) {
            wheel_val = old_wheel;
            return;
        }

        // Limit wheel position
        if (wheel_val > 2) {
            wheel_val = 2;
        }
        if (wheel_val < 0) {
            wheel_val = 0;
        }

        // Check for change
        if (wheel_val != prev_wheel) {
            prev_wheel = wheel_val;

            switch (wheel_val) {
                case 0:
                default:
                    sel_track = 0;
                    break;
                case 1:
                    sel_track = max_track >> 1;
                    break;
                case 2:
                    sel_track = max_track;
                    break;
            }
        }

        disp_bin(cur_track);
        _delay_ms(1);
    }
}

// Go find track 0
void homing_mode()
{
    sel_track = -80;
    _delay_ms(10);
    while (1) {
        if (cur_track == sel_track) {
            // Bumped against hard stop? Regardless, we're done.
            cur_track = 0;
            sel_track = 0;
            wheel_val = 0;
            return;
        }

        // Stop early if we hit track0 signal
        if (!(PIND & _BV(PIN_TRACK0))) {
            // Track0 pin has asserted
            cli();
            sel_track = 0;
            cur_track = 0;
            wheel_val = 0;
            sei();
            return;
        }
    }
}


// Options are Home, Align 80 track, Align 40 track, Align 35 track
char main_menu_table [][2] = {"HO", "A8", "A7", "A4", "35"};

// Main menu
void main_menu()
{
    int8_t val;
    val = select_menu(main_menu_table, 5);

    switch(val) {
        default:
        case -1:
            break;
        case 0:
            homing_mode();
            break;
        case 1:
            align_mode(79);
            break;
        case 2:
            align_mode(76);
            break;
        case 3:
            align_mode(39);
            break;
        case 4:
            align_mode(34);
            break;
    }
}

int main (void)
{

    // Set up IO ports
    DDRD = 0xFB; // Track0 sensor input, all others are outputs
    PORTD = 0x04; // Pullup on Track0 input
    DDRB = 0x0F; // Output for LED cathode drivers
    DDRC = 0x3F; // Output for LED cathode and anode drivers
    DDRE = 0x00; // Wheel inputs, ADC inputs

    // Setup interrupt on change
    PCMSK3 = _BV(PCINT25) | _BV(PCINT24); // For encoder
    PCICR = _BV(PCIE3);

    // Setup display timer
    TCCR0A = _BV(WGM01); // CTC mode
    TCCR0B = 4; // Divide by 256
    OCR0A = 32; // 1ms timer
    TIMSK0 = _BV(OCIE0A); // Interrupt on match

    // Setup step timer
    TCCR3A = 0;
    TCCR3B = _BV(WGM32) | 5; // Divide by 1024.
    OCR3A = 63;
    TIMSK3 = _BV(OCIE3A); // Interrupt on compare match

    // Setup ADC
    ADMUX = _BV(REFS0) | _BV(ADLAR); // Left adjust results, Vref=VCC.
    ADCSRA = _BV(ADEN) | _BV(ADIE) | 7; // Division factor of 128
    adc_set_chan(6);
    sei();
    ADCSRA |= _BV(ADSC);

    // Greet string
    dpoints = 0;
    displaychars[0] = 'H';
    displaychars[1] = 'I';

    _delay_ms(700);

    step_select_menu();

    set_step(0); // Set manual stepper control to phase 0

    while (1) {

        // Selecting a new track
        if (wheel_val != sel_track) {
            if (!blink) {
                blink = true;
                blink_counter = 0;
            }
            disp_bin(wheel_val);
        } else {
            blink = false;
            disp_bin(sel_track);
        }

        // Check for selection timeout
        if (blink) {
            blink_counter++;
            if (blink_counter > 2000) {
                blink = false;
                wheel_val = sel_track;
            }
        }

        // Check for wheel button
        if (!(PINB & _BV(PIN_ENC_BUTTON)) & blink) {
            blink = false;
            sel_track = wheel_val;
        }

        // Check for menu button
        if (menu_button_check()) {
            main_menu();
        }

        _delay_ms(1);

    }
    return 0;
}
