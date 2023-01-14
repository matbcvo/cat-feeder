#define F_CPU 16000000UL // Define CPU frequency 16MHz
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define LCD_DATA_PIN_0 PD0 // LCD_DB0 on Port D (PD0 - LCD_DB0)
#define LCD_DATA_PIN_1 PD1 // LCD_DB1 on Port D (PD1 - LCD_DB1)
#define LCD_DATA_PIN_2 PD2 // LCD_DB2 on Port D (PD2 - LCD_DB2)
#define LCD_DATA_PIN_3 PD3 // LCD_DB3 on Port D (PD3 - LCD_DB3)
#define LCD_DATA_PIN_4 PD4 // LCD_DB4 on Port D (PD4 - LCD_DB4)
#define LCD_DATA_PIN_5 PD5 // LCD_DB5 on Port D (PD5 - LCD_DB5)
#define LCD_DATA_PIN_6 PB4 // LCD_DB6 on Port B (PB4 - LCD_DB6)
#define LCD_DATA_PIN_7 PB7 // LCD_DB7 on Port B (PB7 - LCD_DB7)
#define LCD_EN PF4 // LCD Enable pin
#define LCD_RS PF0 // LCD Register Select pin
#define LCD_RW PF1 // LCD Read/Write data pin
#define LCD_CMD_2_LINES_8_BIT_MODE 0b00111100 // 2 lines, 8-bit mode
#define LCD_CMD_DISPLAY_ON_CURSOR_OFF 0b00001100 // Turn display ON, cursor OFF
#define LCD_CMD_CLEAR_DISPLAY 0b00000001 // Clear LCD display
#define LCD_CMD_CURSOR_INCREMENT 0b00000110 // Cursor increment
#define LCD_CMD_CURSOR_LOCATION 0b10000000 // addr location 0 + cursor 0th pos
/*
32 - 13% duty cycle
64 - 25% duty cycle
128 - 50% duty cycle
192 - 75% duty cycle
*/
#define DEFAULT_MOTOR_DUTY_CYCLE 16

void LCD_Init(); // Initialize LCD
void LCD_EnablePulse(); // Enable Pulse, latch data into register
void LCD_SendCommand(char command); // Send command to LCD
void LCD_ClearDisplay(); // Clear LCD display
void LCD_DisplayChar(uint8_t data); // Display char on LCD
void LCD_DisplayString(uint8_t *data); // Display string on LCD
void LCD_GoTo(uint8_t row, uint8_t column); // Move cursor to X, Y
unsigned char reverse(unsigned char b); // Get bits in reversed order
void TimerCounter4_Init();

volatile uint16_t motor_duty_cycle = DEFAULT_MOTOR_DUTY_CYCLE;

int main(void) {
	// Remove CLKDIV8
	CLKPR = 0x80;
	CLKPR = 0x00;
	// Disable JTAG
	MCUCR = (1<<JTD);
	MCUCR = (1<<JTD);
	
        DDRC |= (1<<PC7); // Motor PWM signal output
	
	LCD_Init(); // Initialize LCD
	TimerCounter4_Init(); // Initialize Timer/Counter4
	while (1) {
		OCR4A = motor_duty_cycle;
		if ((PINE & (1 << PE6))) { // Turn motor driving off
                        // DDRC = (0<<PC6);
                        motor_duty_cycle = 0;
                        TCCR4B = (0 << CS40); // Disable timer
                }
                else if ((PINF & (1 << PF7))) { // Turn motor driving on
                        // DDRC = (1<<PC6);
                        motor_duty_cycle = DEFAULT_MOTOR_DUTY_CYCLE;
                        TCCR4B = (1 << CS40); // Enable timer
                }

		LCD_ClearDisplay(); // Clear LCD display
		LCD_DisplayString((uint8_t *)"Cat Feeder");
	}
}

void LCD_Init() {
	// First 5 LCD data pins are on Port D
	DDRD = (1<<LCD_DATA_PIN_0)|(1<<LCD_DATA_PIN_1)|(1<<LCD_DATA_PIN_2)|(1<<LCD_DATA_PIN_3)|(1<<LCD_DATA_PIN_4)|(1<<LCD_DATA_PIN_5); // Set LCD Data pins DDR output
	// Last 2 LCD data pins are on Port B
	DDRB = (1<<LCD_DATA_PIN_6)|(1<<LCD_DATA_PIN_7); // Set LCD Data pins DDR output
	
	DDRF = (1<<LCD_EN)|(1<<LCD_RS)|(1<<LCD_RW); // Set EN & RS & RW DDR output
	_delay_ms(40); // LCD power ON delay >40ms
	LCD_SendCommand(LCD_CMD_2_LINES_8_BIT_MODE); // 2 lines, 8-bit mode
	LCD_SendCommand(LCD_CMD_DISPLAY_ON_CURSOR_OFF); // Display ON, Cursor OFF
	LCD_SendCommand(LCD_CMD_CURSOR_INCREMENT); // Cursor increment
	LCD_SendCommand(LCD_CMD_CLEAR_DISPLAY); // Clear display
	LCD_SendCommand(LCD_CMD_CURSOR_LOCATION); // Cursor home
}

void LCD_EnablePulse() {
	PORTF |= (1<<LCD_EN); // Set Enable bit
	_delay_ms(1); // Wait for 1ms
	PORTF &= ~(1<<LCD_EN); // Clear Enable bit
}

void LCD_SendCommand(char command) {
	// command = reverse(command); // Command bits in reverse order
	// First 6 LCD data pins are on Port D
	if ((command >> 0) & 1) {
		PORTD |= (1<<LCD_DATA_PIN_0); // Set bit 1
	} else {
		PORTD &= ~(1<<LCD_DATA_PIN_0); // Set bit 0
	}
	if ((command >> 1) & 1) {
		PORTD |= (1<<LCD_DATA_PIN_1); // Set bit 1
	} else {
		PORTD &= ~(1<<LCD_DATA_PIN_1); // Set bit 0
	}
	if ((command >> 2) & 1) {
		PORTD |= (1<<LCD_DATA_PIN_2); // Set bit 1
	} else {
		PORTD &= ~(1<<LCD_DATA_PIN_2); // Set bit 0
	}
	if ((command >> 3) & 1) {
		PORTD |= (1<<LCD_DATA_PIN_3); // Set bit 1
	} else {
		PORTD &= ~(1<<LCD_DATA_PIN_3); // Set bit 0
	}
	if ((command >> 4) & 1) {
		PORTD |= (1<<LCD_DATA_PIN_4); // Set bit 1
	} else {
		PORTD &= ~(1<<LCD_DATA_PIN_4); // Set bit 0
	}
	if ((command >> 5) & 1) {
		PORTD |= (1<<LCD_DATA_PIN_5); // Set bit 1
	} else {
		PORTD &= ~(1<<LCD_DATA_PIN_5); // Set bit 0
	}
	// Last 2 LCD data pins are on Port B
	if ((command >> 6) & 1) {
		PORTB |= (1<<LCD_DATA_PIN_6); // Set bit 1
	} else {
		PORTB &= ~(1<<LCD_DATA_PIN_6); // Set bit 0
	}
	if ((command >> 7) & 1) {
		PORTB |= (1<<LCD_DATA_PIN_7); // Set bit 1
	} else {
		PORTB &= ~(1<<LCD_DATA_PIN_7); // Set bit 0
	}
	
	PORTF &= ~(1<<LCD_RS); // Register Select = 0; Command Register
	PORTF &= ~(1<<LCD_RW); // RW = 0; Write data to the LCD
	LCD_EnablePulse(); // Enable pulse to latch data to register
	_delay_ms(2);
}

void LCD_ClearDisplay() {
	LCD_SendCommand(LCD_CMD_CLEAR_DISPLAY); // Clear display
	_delay_ms(2);
	LCD_SendCommand(LCD_CMD_CURSOR_LOCATION); // Cursor home
	_delay_ms(2);
}

void LCD_DisplayChar(uint8_t data) {
	// data = reverse(data); // Data bits in reverse order
	// First 6 LCD data pins are on Port D
	if ((data >> 0) & 1) {
		PORTD |= (1<<LCD_DATA_PIN_0); // Set bit 1
	} else {
		PORTD &= ~(1<<LCD_DATA_PIN_0); // Set bit 0
	}
	if ((data >> 1) & 1) {
		PORTD |= (1<<LCD_DATA_PIN_1); // Set bit 1
	} else {
		PORTD &= ~(1<<LCD_DATA_PIN_1); // Set bit 0
	}
	if ((data >> 2) & 1) {
		PORTD |= (1<<LCD_DATA_PIN_2); // Set bit 1
	} else {
		PORTD &= ~(1<<LCD_DATA_PIN_2); // Set bit 0
	}
	if ((data >> 3) & 1) {
		PORTD |= (1<<LCD_DATA_PIN_3); // Set bit 1
	} else {
		PORTD &= ~(1<<LCD_DATA_PIN_3); // Set bit 0
	}
	if ((data >> 4) & 1) {
		PORTD |= (1<<LCD_DATA_PIN_4); // Set bit 1
	} else {
		PORTD &= ~(1<<LCD_DATA_PIN_4); // Set bit 0
	}
	if ((data >> 5) & 1) {
		PORTD |= (1<<LCD_DATA_PIN_5); // Set bit 1
	} else {
		PORTD &= ~(1<<LCD_DATA_PIN_5); // Set bit 0
	}
	// Last 2 LCD data pins are on Port B
	if ((data >> 6) & 1) {
		PORTB |= (1<<LCD_DATA_PIN_6); // Set bit 1
	} else {
		PORTB &= ~(1<<LCD_DATA_PIN_6); // Set bit 0
	}
	if ((data >> 7) & 1) {
		PORTB |= (1<<LCD_DATA_PIN_7); // Set bit 1
	} else {
		PORTB &= ~(1<<LCD_DATA_PIN_7); // Set bit 0
	}
	
	PORTF |= (1<<LCD_RS); // Register Select = 1; Data Register
	PORTF &= ~(1<<LCD_RW);	// RW = 0; Write data to the LCD
	LCD_EnablePulse(); // Enable pulse to latch data to register
	_delay_ms(2);
}

void LCD_DisplayString(uint8_t *data) {
	while(*data != 0) {
		LCD_DisplayChar(*data);
		data++;
	}
}

void LCD_GoTo(uint8_t row, uint8_t column)
{
	uint8_t addr = 0;
	row = row-1;
	column = column-1;
	switch(row)	{
		case 0:
		addr = column;
		break;
		case 1:
		addr = column + 0x40;
		break;
	}
	LCD_SendCommand(addr | LCD_CMD_CURSOR_LOCATION); // OR bitwise
}

unsigned char reverse(unsigned char b) {
	// Source: https://stackoverflow.com/a/2602885/4364420
	// First the left four bits are swapped with the right four bits.
	// Then all adjacent pairs are swapped and then all adjacent single bits.
	// This results in a reversed order.
	b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
	b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
	b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
	return b;
}

void TimerCounter1_Init() {
        // Timer 10-bit Fast PWM operation mode
        // Compare Output Mode (Clear on match, set TOP), Waveform Generation Mode (Fast PWM) 
        TCCR4A = (0<<COM4A1)|(1<<COM4A0)|(1<<PWM4A);
        // Waveform Generation Mode (Fast PWM), No prescaling
        TCCR4D = (0 << WGM40);
        TCCR4B = (1<<CS40);
}

