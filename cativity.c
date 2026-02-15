/*
 * CatIVity - CAT protocol (Icom CI-V) based remote VFO tuner
 *
 * Copyright (c) 2026 Helmut Sipos, YO6ASM <yo6asm@gmail.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdint.h>

// States of the indicator LED
#define LED_OFF                      0
#define LED_ON                       1

// UART default baudate
#define UART_BAUDRATE                19200

// CI-V protocol elements
#define CIV_PREAMBLE                 0xFE
#define CIV_TRANSCEIVER_ADDRESS      0x70
#define CIV_CONTROLLER_ADDRESS       0xE0
#define CIV_END_OF_MESSAGE           0xFD
#define CIV_GET_ACTIVE_VFO_FREQUENCY 0x03
#define CIV_SET_ACTIVE_VFO_FREQUENCY 0x05

// Supported VFO frequency range 0.5 - 30.0 MHz
#define VFO_FRQUENCY_MIN             500000
#define VFO_FRQUENCY_MAX             30000000

// RX buffer
volatile uint8_t gUartRXBuffer[64];
// Counter indicating how many bytes are in the RX buffer
volatile uint8_t gUartRXBytes = 0;

// The VFO frequency
volatile uint32_t gVFOFrequency = 0;
// Frequency increment/decrement step
volatile uint8_t gVFOFrequencyStep = 0;
// VFO synchronized with tranceiver indicator flag
volatile uint8_t gVFOFrequencyValid = 0;

// Number of pulses per time unit (200ms) received from the rotary encoder
volatile uint16_t gEncoderPulses = 0;

// VFO frequency tuning increment/decrement steps
const uint8_t gTuningSteps[] = {1, 1, 2, 3, 4, 5, 8, 16, 64, 255};
#define TUNING_STEPS_IDX_MAX ((sizeof(gTuningSteps)/sizeof(gTuningSteps[0])) - 1)

// External INT0 interrupt service routine
ISR(INT0_vect)
{
    // Rotation RIGHT?
    if (bit_is_set(PIND, PIND3)) {
        gEncoderPulses++;
        gVFOFrequency += gVFOFrequencyStep;
    }
}

// External INT1 interrupt service routine
ISR(INT1_vect)
{
    // Rotation LEFT?
    if (bit_is_set(PIND, PIND2)) {
        gEncoderPulses++;
        gVFOFrequency -= gVFOFrequencyStep;
    }
}

// Interrupt on change PCINT8..14 service routine
ISR(PCINT1_vect)
{
    if (bit_is_set(PINC, PINC5)) {
        // Port C pin 5 -> Sync to transceiver
        // Request reading the current VFO set on the transceiver
        gVFOFrequencyValid = 0;
    }
    else if (bit_is_set(PINC, PINC4)) {
        // TBD
    }
    else if (bit_is_set(PINC, PINC3)) {
        // TBD
    }
    else if (bit_is_set(PINC, PINC2)) {
        // TBD
    }
    else if (bit_is_set(PINC, PINC1)) {
        // TBD
    }
    else if (bit_is_set(PINC, PINC0)) {
        // TBD
    }
}

// UART RX interrupt service routine
ISR(USART_RX_vect)
{
    // Copy the received byte inito the buffer if there is space left
    // Increment the counter indicating how many bytes are in the buffer
    if (gUartRXBytes < sizeof(gUartRXBuffer)) {
        gUartRXBuffer[gUartRXBytes] = UDR0;
        gUartRXBytes++;
    }
}

// Timer1 overflow interrupt service routine
ISR(TIMER1_OVF_vect)
{
    // Setup for new timer overflow interrupt after 200ms
    TCNT1H = 0x9E;
    TCNT1L = 0x58;

    uint32_t idx = gEncoderPulses / 60;
    if (idx > TUNING_STEPS_IDX_MAX) {
        idx = TUNING_STEPS_IDX_MAX;
    }

    gVFOFrequencyStep = gTuningSteps[idx];

    gEncoderPulses = 0;
}

// Set red LED to ON or OFF
void led_red(uint8_t state)
{
    if (state == LED_ON) {
        PORTD |= _BV(PORTD4);
    }
    else {
        PORTD &= ~_BV(PORTD4);
    }
}

// Send an array of bytes through the UART
void uart_send(const uint8_t *data, uint8_t length)
{
    for (int i = 0; i < length; ++i) {
        // Wait for empty transmit buffer
        while (bit_is_clear(UCSR0A, UDRE0)) {
        }

        // Put tx data into the buffer. It will then be send by the hardware
        UDR0 = data[i];
    }
}

// Convert frequency Hz -> 5-byte BCD LSB-first
void frequency_to_bcd(uint32_t frequency, uint8_t *out)
{
    uint8_t decimals[10];

    // Get the decimal for each position
    for (int i = 0; i < 10; ++i) {
        decimals[i] = frequency % 10;
        frequency /= 10;
    }

    // Pack the decimals into the byte array
    out[0] = (decimals[1] << 4) | decimals[0];
    out[1] = (decimals[3] << 4) | decimals[2];
    out[2] = (decimals[5] << 4) | decimals[4];
    out[3] = (decimals[7] << 4) | decimals[6];
    out[4] = (decimals[9] << 4) | decimals[8];
}

// Convert 5-byte BCD LSB-first -> frequency Hz. Return 0 if invalid (any nibble >9)
uint32_t bcd_to_frequency(volatile const uint8_t *const data)
{
    uint8_t decimals[10];

    // Unpack the decimals from the byte array
    decimals[0] = data[0] & 0x0F;
    decimals[1] = (data[0] >> 4) & 0x0F;
    decimals[2] = data[1] & 0x0F;
    decimals[3] = (data[1] >> 4) & 0x0F;
    decimals[4] = data[2] & 0x0F;
    decimals[5] = (data[2] >> 4) & 0x0F;
    decimals[6] = data[3] & 0x0F;
    decimals[7] = (data[3] >> 4) & 0x0F;
    decimals[8] = data[4] & 0x0F;
    decimals[9] = (data[4] >> 4) & 0x0F;

    // Check for plausible value
    for (int i = 0; i < 10; ++i) {
        if (decimals[i] > 9) {
            return 0;
        }
    }

    uint32_t frequency = 0;
    uint32_t mul = 1;

    // Assemble the frequency value from the decimals
    for (int i = 0; i < 10; ++i) {
        frequency += decimals[i] * mul;
        mul *= 10;
    }

    return frequency;
}

// Retrieve the currently set VFO frequency from the transceiver
uint32_t get_current_vfo_frequency(void)
{
    uint8_t timeout_counter = 0x00;
    uint32_t frequency = 0x00;

    // See CI-V protocol specification for message details
    const uint8_t cmdGetActiveVFOFrequency[] = {
        CIV_PREAMBLE,
        CIV_PREAMBLE,
        CIV_TRANSCEIVER_ADDRESS,
        CIV_CONTROLLER_ADDRESS,
        CIV_GET_ACTIVE_VFO_FREQUENCY,
        CIV_END_OF_MESSAGE
    };

    // Request the current frequency from the transceiver
    // Xiegu G90 is echoing back the received command and THEN sending the response
    // therefore we need to wait for both messages.
    gUartRXBytes = 0;
    uart_send(cmdGetActiveVFOFrequency, sizeof(cmdGetActiveVFOFrequency) / sizeof(cmdGetActiveVFOFrequency[0]));
    timeout_counter = 50;
    while ((gUartRXBytes < 17) && --timeout_counter) {
        _delay_ms(1);
    }

    if (timeout_counter == 0) {
        // RX Timeout
        led_red(LED_ON);
        _delay_ms(1000);
        led_red(LED_OFF);
        _delay_ms(1000);
    }
    else {
        // Check message for plausibility
        if (       (gUartRXBuffer[6] == CIV_PREAMBLE)
                && (gUartRXBuffer[7] == CIV_PREAMBLE)
                && (gUartRXBuffer[8] == CIV_CONTROLLER_ADDRESS)
                && (gUartRXBuffer[9] == CIV_TRANSCEIVER_ADDRESS)
                && (gUartRXBuffer[10] == CIV_GET_ACTIVE_VFO_FREQUENCY)
                && (gUartRXBuffer[16] == CIV_END_OF_MESSAGE)
           ) {
            // decode the frequency from the received response
            frequency = bcd_to_frequency(&gUartRXBuffer[11]);
        }
        else {
            // Malformed message
            led_red(LED_ON);
            _delay_ms(3000);
            led_red(LED_OFF);
            _delay_ms(3000);
        }
    }

    return frequency;
}

// Initialize the MCU
void init_hardware(void)
{
    // Initialize Ports

    // Port B all inputs
    DDRB  = 0b00000000;
    // Pulls-ups for all inputs
    PORTB = 0b11111111;

    // Port C all inputs
    DDRC  = 0b00000000;
    // Pulls-ups for all inputs
    PORTC = 0b11111111;

    // Port D - all inputs except UART TX (PD1) and LED (PD4)
    DDRD  = 0b00010010;
    // Enable internal pull-ups for all but UART TX and LED
    PORTD = 0b11101101;

    // Initialize external interrupts

    // External interrupts 0,1: Select falling edge trigger
    EICRA = 0b00001010;
    // Enable External Interrupts
    EIMSK = 0b00000011;

    // Initialize interrupt on change

    // Port C Pin 5 .. 0, PCINT13 .. PCINT8 is used for various user defined pushbuttons
    // PCIE1: Pin Change Interrupt Enable 1
    PCICR =  0b00000010;
    // Enable interrupt on pin change on PINC5..0
    PCMSK1 = 0b00111111;

    // Initialize UART

    // Set format N81 and baud rate
    UBRR0 = (uint8_t)((F_CPU / UART_BAUDRATE + 8) / 16 - 1);
    // Enable receiver and transmitter as well as receive interrupt
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);

    // Initialize Timer1

    TCCR1A = 0x00;
    // 1:64 prescaler
    TCCR1B = 0x03;
    TCCR1C = 0x00;
    // Enable overflow interrupt
    TIMSK1 = 0x01;
    // Timer overflows after 200ms
    TCNT1H = 0x9E;
    TCNT1L = 0x58;
}

int main(void)
{
    uint8_t timeout_counter = 0x00;

    uint32_t last_vfo_frequency = 0x00;

    // See CI-V protocol specification for message details
    uint8_t cmdSetActiveVFOFrequency[] = {
        CIV_PREAMBLE,
        CIV_PREAMBLE,
        CIV_TRANSCEIVER_ADDRESS,
        CIV_CONTROLLER_ADDRESS,
        CIV_SET_ACTIVE_VFO_FREQUENCY,
        0x00, 0x00, 0x00, 0x00, 0x00, // Frequency -> BCD coded
        CIV_END_OF_MESSAGE
    };

    // Disable interrupts while setting things up
    cli();

    // Initialize the microcontroller
    init_hardware();

    // Enable interrupts
    sei();

    // Forever
    for (;;) {

        // If not done so yet, retrieve the current frequency from the transceiver
        if (!gVFOFrequencyValid) {

            if (!((gVFOFrequency = get_current_vfo_frequency()))) {
                continue;
            }

            gVFOFrequencyValid = 1;
        }

        // Clamp the frequency to the supported range
        if (gVFOFrequency < VFO_FRQUENCY_MIN)
        {
            gVFOFrequency = VFO_FRQUENCY_MIN;
        }
        else if (gVFOFrequency > VFO_FRQUENCY_MAX)
        {
            gVFOFrequency = VFO_FRQUENCY_MAX;
        }

        // Tune in steps of 10 Hz. Ignore the 'ones'
        gVFOFrequency -= gVFOFrequency % 10;

        // Do nothing while the frequency didn't change
        if (gVFOFrequency == last_vfo_frequency) {
            continue;
        }

        // Update last values
        last_vfo_frequency = gVFOFrequency;

        // Encode the frequency in BCD as per spec.
        frequency_to_bcd(gVFOFrequency, &cmdSetActiveVFOFrequency[5]);

        // Send the current frequency to the transceiver
        // then read (and ignore) the response
        gUartRXBytes = 0;
        uart_send(cmdSetActiveVFOFrequency, sizeof(cmdSetActiveVFOFrequency) / sizeof(cmdSetActiveVFOFrequency[0]));
        timeout_counter = 50;
        while ((gUartRXBytes < 17) && --timeout_counter) {
            _delay_ms(1);
        }
    }
}

