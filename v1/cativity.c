/*
 * CatIVity - CAT protocol (Icom CI-V) based VFO tuner
 *
 * Copyright (c) 2025 Helmut Sipos, YO6ASM <yo6asm@gmail.com>
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

// Xiegu G90's CI-V protocol baudrate
#define UART_BAUDRATE                19200

#define ADC_CHANNEL_POT_LEFT         0
#define ADC_CHANNEL_POT_RIGHT        1

#define LED_RED                      PD2
#define LED_OFF                      0
#define LED_ON                       1

#define CIV_PREAMBLE                 0xFE

// Xiegu G90 default address (also responds to 0x88 etc.)
#define CIV_TRANSCEIVER_ADDRESS      0x70
#define CIV_CONTROLLER_ADDRESS       0xE0
#define CIV_END_OF_MESSAGE           0xFD

#define CIV_GET_ACTIVE_VFO_FREQUENCY 0x03
#define CIV_SET_ACTIVE_VFO_FREQUENCY 0x05

// See CI-V protocol specification for message details
const uint8_t cmdGetActiveVFOFrequency[] = {
    CIV_PREAMBLE,
    CIV_PREAMBLE,
    CIV_TRANSCEIVER_ADDRESS,
    CIV_CONTROLLER_ADDRESS,
    CIV_GET_ACTIVE_VFO_FREQUENCY,
    CIV_END_OF_MESSAGE
};

// RX buffer
volatile uint8_t uart_rx_buffer[64];
// Counter indicating how many bytes are in the RX buffer
volatile uint8_t uart_rx_bytes = 0x00;

// Digitized voltage set by the voltage divider right potentiometer
volatile uint8_t right_pot_value = 0x00;

// Digitized voltage set by the voltage divider right potentiometer
volatile uint8_t left_pot_value = 0x00;

// UART service routine
ISR(USART_RX_vect)
{
    // Copy the received byte inito the buffer if there is space left
    // Increment the counter indicating how many bytes are in the buffer
    if (uart_rx_bytes < sizeof(uart_rx_buffer)) {
        uart_rx_buffer[uart_rx_bytes] = UDR0;
        uart_rx_bytes++;
    }
}

// ADC service routine
ISR(ADC_vect)
{
    uint8_t adc_l = ADCL;
    uint8_t adc_h = ADCH;
    // The ADC conversion is a 10 bit value -> values range from 0...1024
    // Diving by  10 puts it nicely into our desired range of 0..99
    uint16_t adc_val = ((adc_h << 8) | adc_l) / 10;

    // Clamp the value to 0..100
    if (adc_val > 100) {
        adc_val = 100;
    }

    // Alternatively read the value set by pot right and left
    if (ADMUX & ADC_CHANNEL_POT_RIGHT) {
        right_pot_value = (right_pot_value + adc_val) / 2;
        ADMUX = 0x40 | ADC_CHANNEL_POT_LEFT;
    }
    else {
        left_pot_value = (left_pot_value + adc_val) / 2;
        ADMUX = 0x40 | ADC_CHANNEL_POT_RIGHT;
    }
}

// TIMER 1 service routine
ISR(TIMER1_OVF_vect)
{
    // Another interrupt will occur after ~50ms due to timer overflow
    // TIMER1 overflow triggers the ADC
    TCNT1L = 0xB0;
    TCNT1H = 0x3C;
}

// Set red led to ON or OFF
void led_red(uint8_t state)
{
    if (state == LED_ON) {
        PORTD |= _BV(LED_RED);
    }
    else {
        PORTD &= ~_BV(LED_RED);
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

int main(void)
{
    uint8_t oneHzTenHz = 0x00;
    uint8_t hundredHzOneKHz = 0x00;
    uint8_t tenKHzHundredKHz = 0x00;
    uint8_t oneMHzTenMHz = 0x00;

    uint8_t last_right_pot_value = 0x00;
    uint8_t last_left_pot_value = 0x00;

    uint8_t timeout_counter = 0x00;

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

    // Initialize Ports
    // Port B all inputs
    DDRB  = 0x00;
    // Pulls-ups for inputs
    PORTB = 0xFF;

    // Port C all inputs C0 and C1 -> ADC
    DDRC  = 0x00;
    // Enable pull-ups for uppermost 2 bits
    PORTC = 0xF0;

    // Port D - all inputs except UART TX and LED (PD1 and PD2)
    DDRD  = 0x06;
    // Enable internal pull-ups for all but RX and TX
    PORTD = 0xF9;

    // Initialize Timer1
    TCCR1A = 0x00;
    // 1:8 prescaler
    TCCR1B = 0x02;
    TCCR1C = 0x00;
    // Enable TC1.ovf interrupt
    TIMSK1 = 0x01;
    // Interrupt after 50ms due to timer overflow
    TCNT1L = 0xB0;
    TCNT1H = 0x3C;

    // Initialize UART set format N81, enable RX, TX and RX interrupt
    UBRR0 = (uint8_t)((F_CPU / UART_BAUDRATE + 8) / 16 - 1);
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0); // Enable RX, TX, and RX interrupt

    // Initialize ADC
    // AVCC with external capacitor at AREF pin, align right, chanel 0
    ADMUX = 0x40;

    // Enable ADC, auto trigger, enable interrupt, prescaler = 64 -> result in 125KHz sampling rate
    ADCSRA = 0xEE;

    // ADC TIMER1 overflow mode. Free running mode would mean interrupts every 200us
    ADCSRB = 0x06;
    // Disable analog comp
    ACSR   = 0x80;

    // Disable ditital input for the analog input channels
    DIDR0 |= (1 << ADC1D) | (1 << ADC0D);

    // Enable interrupts
    sei();

    // Forever
    for (;;) {

        // Turn the red LED off and delay 300 ms
        led_red(LED_OFF);
        _delay_ms(300);

        // Do nothing while the potentiometers have not moved
        if ((right_pot_value == last_right_pot_value)
            &&
            (left_pot_value == last_left_pot_value)
           ) {
            continue;
        }

        // Update last values
        last_right_pot_value = right_pot_value;
        last_left_pot_value = left_pot_value;

        // Request the current frequency from the radio
        uart_rx_bytes = 0;
        uart_send(cmdGetActiveVFOFrequency, sizeof(cmdGetActiveVFOFrequency));

        // Xiegu G90 is echoing back the received command and THEN sending the response
        // therefore we need to wait for both messages.
        timeout_counter = 50;
        while ((uart_rx_bytes < 17) && --timeout_counter) {
            _delay_ms(1);
        }

        // If we run into the timeout (i.e. no response from the tranceiver)
        // keep the red LED on for another second then try again
        if (timeout_counter == 0) {
            led_red(LED_ON);
            _delay_ms(1000);
            continue;
        }

        // Check message for plausibility
        if (    (uart_rx_buffer[6] == CIV_PREAMBLE)
             && (uart_rx_buffer[7] == CIV_PREAMBLE)
             && (uart_rx_buffer[8] == CIV_CONTROLLER_ADDRESS)
             && (uart_rx_buffer[9] == CIV_TRANSCEIVER_ADDRESS)
             && (uart_rx_buffer[10] == CIV_GET_ACTIVE_VFO_FREQUENCY)
             && (uart_rx_buffer[16] == CIV_END_OF_MESSAGE)
            ) {
            tenKHzHundredKHz = uart_rx_buffer[13];
            oneMHzTenMHz = uart_rx_buffer[14];
        }
        else {
            // Malformed message -> keep the red led on for 3 seconds then try again
            led_red(LED_ON);
            _delay_ms(3000);
            continue;
        }

        // Flash the red led
        led_red(LED_ON);

        // Translate the potentiometer positions into frequency values
        oneHzTenHz = ((right_pot_value % 10) << 4) & 0xF0;
        hundredHzOneKHz = (((left_pot_value % 10) << 4) | ((right_pot_value / 10) & 0x0F));
        tenKHzHundredKHz &= 0xF0;
        tenKHzHundredKHz |= ((left_pot_value / 10) & 0x0F);

        // Put the values into the set frequency message array
        cmdSetActiveVFOFrequency[5] = oneHzTenHz;
        cmdSetActiveVFOFrequency[6] = hundredHzOneKHz;
        cmdSetActiveVFOFrequency[7] = tenKHzHundredKHz;
        cmdSetActiveVFOFrequency[8] = oneMHzTenMHz;

        // Set the current frequency on the radio
        uart_rx_bytes = 0;
        uart_send(cmdSetActiveVFOFrequency, sizeof(cmdSetActiveVFOFrequency));

        // Read (and ignore) the response to the set active vfo frequency command
        timeout_counter = 50;
        while ((uart_rx_bytes < 17) && --timeout_counter) {
            _delay_ms(1);
        }
    }
}

