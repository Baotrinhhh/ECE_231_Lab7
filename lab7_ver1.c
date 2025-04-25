/* 
  lab7_ver1.c
  Description:
    This program reads an analog value from an ADC, converts it into a temperature value,
    and displays the result via UART and on an OLED. A momentary pushbutton connected 
    to PD3 selects between Fahrenheit and Celsius conversion modes. Additionally, an LED 
    on PD4 is lit when the temperature exceeds 90 units.

  Author: Bao Trinh
  Date: 04/19/25
*/

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include "i2c.h"
#include "SSD1306.h"
#include "my_adc_lib.h"
#include "my_uart_lib.h"

#define BUTTON PD3     // Button input on PD3 (Arduino Uno pin 3)
#define REDLED PD4     // LED output on PD4 (Arduino Uno pin 4)
#define MYDELAY 100    // Debounce delay in msec
#define BUFFER_SIZE 16  // Buffer size for temperature string
#define TOO_HOT 80.0f

int main(void)
{
    char buf[BUFFER_SIZE];
    uint16_t dout;
    float temp;
    int int_temp, frac_temp;
    unsigned char button, deg;
    const char *too_hot;

    // Configure LED pin as output and enable pull-up on the button pin.
    DDRD |= (1 << REDLED);
    PORTD |= (1 << BUTTON);

    OLED_Init();
    OLED_Clear();
    adc_init();
    uart_init();

    while (1)
    {
        // Read button state and debounce
        button = (PIND & (1 << BUTTON));
        _delay_ms(MYDELAY);

        // Read ADC value and apply offset correction.
        dout = get_adc() - 100;

        // Convert ADC reading to temperature value based on selected mode.
        if (button != 0)
        {
            // Fahrenheit conversion: temperature = ADC value + 32
            deg  = 'F';
            temp = dout * 0.8784f + 32.0f;
        }
        else
        {
            // Celsius conversion: temperature = (ADC value * conversion factor)
            deg  = 'C';
            temp = dout * 0.4883f;  // Assign converted value to temperature
        }

        // Separate the temperature into integer and fractional parts.
        int_temp  = (int)temp;
        frac_temp = (int)(((temp - int_temp) * 10) + 0.5f); // Round to nearest tenth

        // Check if the temperature exceeds threshold and update the LED status.
        if (temp >= TOO_HOT)
        {
            PORTD |= (1 << REDLED);
            too_hot = " TOO_HOT";
        }
        else
        {
            PORTD &= ~(1 << REDLED);
            too_hot = "";
        }

        // Format the temperature string and send via UART.
        snprintf(buf, sizeof(buf), "%d.%01d%c", int_temp, frac_temp, deg);
        send_string(buf);
        send_string(too_hot);
        uart_send(13); // TX carriage return
        uart_send(10); // TX new line
        
        // Display the temperature string on the OLED.s
        OLED_Clear();
        OLED_GoToLine(1);
        OLED_DisplayString(buf);
        _delay_ms(1000);
    }
}

/**** End of File *****/