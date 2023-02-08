#define F_CPU 16000000UL // Define CPU frequency 16MHz

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/atomic.h>

typedef unsigned long millis_t;

/* Small % duty cycle = Slow motor speed
32 - 13% duty cycle
64 - 25% duty cycle
128 - 50% duty cycle
192 - 75% duty cycle
*/
#define DEFAULT_MOTOR_DUTY_CYCLE 16
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
#define BUTTON_4_PIN PE6
#define BUTTON_3_PIN PF5
#define BUTTON_2_PIN PF6
#define BUTTON_1_PIN PF7
#define HX711_SCK_DDR DDRB
#define HX711_SCK_PORT PORTB
#define HX711_SCK_PIN PB5
#define HX711_DT_DDR DDRB
#define HX711_DT_PORT PORTB
#define HX711_DT_INPUT PINB
#define HX711_DT_PIN PB6
#define HX711_GAIN_CHANNEL_A_128 1
#define HX711_GAIN_CHANNEL_A_64	3
#define HX711_GAIN_CHANNEL_B_32	2
#define HX711_GAIN_DEFAULT HX711_GAIN_CHANNEL_A_128
#define HX711_SCALEDEFAULT 10000 // Defines scale
#define HX711_OFFSETDEFAULT 8000000 // Defines offset
#define HX711_READTIMES 5 // Set how many time to read
#define HX711_USEAVERAGEONREAD 1 // Set if use average for read
#define HX711_CALIBRATIONREADTIMES 5 // Calibration average times read
#define HX711_ATOMICMODEENABLED	1 // Enable the atomic mode on shift in
#define EEPROM_HX711_OFFSET 64
#define EEPROM_HX711_SCALE 80
#define MENU_MODE_FOOD_WEIGHT 1
#define MENU_MODE_BUZZER 2
#define MENU_MODE_FEED 3
#define MENU_MODE_FEEDINGS_PER_DAY 4
#define MENU_MODE_CHANGE_FOOD_AMOUNT 5
#define MENU_MODE_TIMER 6
#define MOTOR_EN PC6
#define MOTOR_PWM PC7
#define SEC_IN_uSEC 1000000
#define NOTE_FREQ_A 493.88 // music frequency in Hz
#define NOTE_FREQ_C 261.53
#define NOTE_FREQ_F 349.23
#define NOTE_FREQ_AS 466.16
#define NOTE_FREQ_G 392
#define NOTE_FREQ_D 293.66
#define NOTE_FREQ_E 329.63
#define BUZZER_PIN PB0
#define MILLIS_CLOCKSEL (_BV(CS01)|_BV(CS00))
#define MILLIS_PRESCALER 64
#define MILLIS_REG_TCCRA TCCR0A
#define MILLIS_REG_TCCRB TCCR0B
#define MILLIS_REG_TIMSK TIMSK0
#define MILLIS_REG_OCR	OCR0A
#define MILLIS_BIT_WGM	WGM01
#define MILLIS_BIT_OCIE OCIE0A
#define MILLIS_ISR_VECT TIMER0_COMPA_vect

uint8_t motor_duty_cycle = DEFAULT_MOTOR_DUTY_CYCLE;
uint8_t buzzer_activated = 1;
uint8_t HX711_calibration_process = 0;
uint8_t HX711_gain = 0; // actual gain
double HX711_scale = 0; // actual scale
int32_t HX711_offset = 0; // actual offset
uint8_t food_amount = 2;
uint8_t feedings_per_day = 1;
volatile millis_t milliseconds;
uint8_t time_seconds = 0;
uint8_t time_minutes = 0;
uint8_t time_hours = 0;

uint8_t isAnyButtonPressedDown();
uint8_t isFirstButtonPressedDown();
uint8_t isSecondButtonPressedDown();
uint8_t isThirdButtonPressedDown();
uint8_t isLastButtonPressedDown();
void updateMenu(uint8_t menu_mode);
void LCD_Init(); // Initialize LCD
void LCD_EnablePulse(); // Enable Pulse, latch data into register
void LCD_SendCommand(char command); // Send command to LCD
void LCD_ClearDisplay(); // Clear LCD display
void LCD_DisplayChar(uint8_t data); // Display char on LCD
void LCD_DisplayString(uint8_t *data); // Display string on LCD
void LCD_GoTo(uint8_t row, uint8_t column); // Move cursor to X, Y
unsigned char reverse(unsigned char b); // Get bits in reversed order
void TimerCounter4_Init();
void HX711_Init(uint8_t gain, double scale, int32_t offset);
int32_t HX711_Read();
int32_t hx711_ReadAverage(uint8_t times);
double HX711_Readwithtare();
double HX711_GetWeight();
void HX711_SetGain(uint16_t gain);
uint16_t HX711_GetGain();
void HX711_SetScale(double scale);
double HX711_GetScale();
void HX711_SetOffset(int32_t offset);
int32_t HX711_GetOffset();
void HX711_TareToZero();
void HX711_PowerDown();
void HX711_PowerUp();
void HX711_Calibrate1SetOffset();
void HX711_Calibrate2SetScale(double weight);
void feeding_cycle();
void play_note_as(); // musical notes function calling
void play_note_c();
void play_note_f();
void play_note_a();
void play_note_D();
void play_note_E();
void play_note_G();
void feeding_music(); // feeding tune function calling
void millis_init(void);
millis_t millis_get(void); // Get milliseconds
void millis_reset(void); // Reset milliseconds count to 0

int main(void) {
	// Remove CLKDIV8
	CLKPR = 0x80;
	CLKPR = 0x00;
	// Disable JTAG
	MCUCR = (1<<JTD);
	MCUCR = (1<<JTD);
	
	// Define motor PWM signal as output
	DDRC |= (1<<MOTOR_PWM);
	// Disable motor enable pin for now (setting as input)
	DDRC = (0<<MOTOR_EN);
	
	// Define button as input
	DDRF &= (~(1<<BUTTON_1_PIN));
	DDRF &=	(~(1<<BUTTON_2_PIN));
	DDRF &= (~(1<<BUTTON_3_PIN));
	DDRE &= (~(1<<BUTTON_4_PIN));
	
	LCD_Init(); // Initialize LCD
	TimerCounter4_Init(); // Initialize Timer/Counter4
	millis_init(); // Enable milliseconds timer
	
	LCD_ClearDisplay(); // Clear LCD display
	LCD_GoTo(1, 1);
	LCD_DisplayString((uint8_t *)"LCD init-d");
	
	HX711_Init(HX711_gain, HX711_scale, HX711_offset); // Initialize HX711
	
	LCD_GoTo(2, 1);
	LCD_DisplayString((uint8_t *)"HX711 init-d");
	
	_delay_ms(1000);
	
	// first and last buttons are pressed down - start calibrate process
	if ((PINF & (1 << BUTTON_1_PIN)) && (PINE && (1 << BUTTON_4_PIN))) {
		HX711_calibration_process = 1;
		
		LCD_ClearDisplay();
		LCD_GoTo(1, 1);
		LCD_DisplayString((uint8_t *)"Starting calib.");
		LCD_GoTo(2, 1);
		LCD_DisplayString((uint8_t *)"in 5 seconds...");
		
		_delay_ms(5000);
		
		LCD_ClearDisplay();
		LCD_GoTo(1, 1);
		LCD_DisplayString((uint8_t *)"Setting tare");
		LCD_GoTo(2, 1);
		LCD_DisplayString((uint8_t *)"in 5 seconds...");
		
		_delay_ms(5000);
	
		LCD_ClearDisplay();
		LCD_DisplayString((uint8_t *)"Doing tare");
		
		HX711_Calibrate1SetOffset(); // Set offset		
	
		LCD_ClearDisplay();
		LCD_DisplayString((uint8_t *)"Tare done");
		LCD_GoTo(2,1);
		LCD_DisplayString((uint8_t *)"Now put 500g");
		
		_delay_ms(5000);
		
		LCD_ClearDisplay();
		LCD_DisplayString((uint8_t *)"Doing scale");
		
		double known_mass = 500.0; // 500g
		HX711_Calibrate2SetScale(known_mass);
	
		LCD_ClearDisplay();
		LCD_DisplayString((uint8_t *)"Scale set");
		HX711_calibration_process = 0;
	} else {
		// Read offset from EEPROM
		double eeprom_hx711_offset_value = 0;
		eeprom_hx711_offset_value = eeprom_read_dword((uint32_t *)EEPROM_HX711_OFFSET);
		HX711_SetOffset((int32_t)eeprom_hx711_offset_value);
		// Read scale from EEPROM
		double eeprom_hx711_scale_value = 0;
		eeprom_hx711_scale_value = eeprom_read_dword((uint32_t *)EEPROM_HX711_SCALE);
		HX711_SetScale(eeprom_hx711_scale_value);
	}
	
	uint8_t menu_mode = 0;
	updateMenu(menu_mode);

	while (1) {
		if (millis_get() >= 1000) {
			time_seconds += 1;
			millis_reset();
			updateMenu(menu_mode);
		}
		if (time_seconds >= 60) {
			time_minutes += 1;
			time_seconds = 0;
		}
		if (time_minutes >= 60) {
			time_hours += 1;
			time_minutes = 0;
		}
		if (time_hours >= 24) {
			time_hours = 0;
		}
		
		if (time_minutes == 0 && time_seconds == 0) {
			switch(feedings_per_day) {
				case 1: // one feeding per day
					if (time_hours == 12) {
						feeding_cycle();
					}
				break;
				case 2: // two feedings per day
					if (time_hours == 12 || time_hours == 18) {
						feeding_cycle();
					}
				break;
				case 3:
					if (time_hours == 8 || time_hours == 12 || time_hours == 16) {
						feeding_cycle();
					}
				break;
				case 4:
					if (time_hours == 12 || time_hours == 14 || time_hours == 16 || time_hours == 18) {
						feeding_cycle();
					}
				break;
				case 5:
					if (time_hours == 12 || time_hours == 14 || time_hours == 16 || time_hours == 18 || time_hours == 20) {
						feeding_cycle();
					}
				break;
			}	
		}
		
		if (isLastButtonPressedDown()) {
			while(!isLastButtonPressedDown()); // Until not pressed
			menu_mode++;
			if (menu_mode > 6) {
				menu_mode = 0;
			}
			updateMenu(menu_mode);
			_delay_ms(500);
		}
		else if (isFirstButtonPressedDown()) {
			while(!isFirstButtonPressedDown());
			menu_mode--;
			if (menu_mode > 6) {
				menu_mode = 6;
			}
			updateMenu(menu_mode);
			_delay_ms(500);
		}
		else if (isThirdButtonPressedDown()) {
			while(!isThirdButtonPressedDown());
			if (menu_mode == MENU_MODE_BUZZER) {
				LCD_GoTo(2, 1);
				if (buzzer_activated == 1) {
					buzzer_activated = 0;
					LCD_DisplayString((uint8_t *)"Buzzer disabled ");
					_delay_ms(500);
				}
				else if (buzzer_activated == 0) {
					buzzer_activated = 1;
					play_note_a();
					LCD_DisplayString((uint8_t *)"Buzzer enabled  ");
					_delay_ms(500);
				}
			}
			else if (menu_mode == MENU_MODE_FEED) {
				LCD_GoTo(2, 1);
				LCD_DisplayString((uint8_t *)"Feeding now...  ");
				feeding_cycle();
				if (buzzer_activated == 1) {
					feeding_music();
				}
			}
			else if (menu_mode == MENU_MODE_FEEDINGS_PER_DAY) {
				if (feedings_per_day < 5) {
					feedings_per_day++;
				}
			}
			else if (menu_mode == MENU_MODE_CHANGE_FOOD_AMOUNT) {
				if (food_amount < 5) {
					food_amount++;
				}
			}
			updateMenu(menu_mode);
			_delay_ms(500);
		}
		else if (isSecondButtonPressedDown()) {
			while(!isSecondButtonPressedDown());
			if (menu_mode == MENU_MODE_FEEDINGS_PER_DAY) {
				if (feedings_per_day > 1) {
					feedings_per_day--;
				}
			}
			else if (menu_mode == MENU_MODE_CHANGE_FOOD_AMOUNT) {
				if (food_amount > 1) {
					food_amount--;
				}
			}
			updateMenu(menu_mode);
			_delay_ms(500);
		}
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

void TimerCounter4_Init() {
	// Timer 10-bit Fast PWM operation mode
	// Compare Output Mode (Clear on match, set TOP), Waveform Generation Mode (Fast PWM)
	TCCR4A = (0<<COM4A1)|(1<<COM4A0)|(1<<PWM4A);
	// Waveform Generation Mode (Fast PWM)
	TCCR4D = (0 << WGM40);
	// No prescaling
	TCCR4B = (1<<CS40);
}

int32_t HX711_Read() { // Read raw value
	uint32_t count = 0;
	uint8_t i = 0;
	
	while (HX711_DT_INPUT & (1<<HX711_DT_PIN)); // Wait for the chip to became ready

#if HX711_ATOMICMODEENABLED == 1
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
#endif
	for(i = 0; i < 24; i++) { // Read data with a 24 shift
		HX711_SCK_PORT |= (1<<HX711_SCK_PIN);
		// asm volatile("nop");
		_delay_us(1);
		count=count<<1;
		HX711_SCK_PORT &= ~(1<<HX711_SCK_PIN);
		// asm volatile("nop");
		_delay_us(1);
		if(HX711_DT_INPUT & (1<<HX711_DT_PIN)) {
			count++;
		}
	}
	count ^= 0x800000;
	for (i=0; i < HX711_gain; i++) { // Set the channel and the gain
		HX711_SCK_PORT |= (1<<HX711_SCK_PIN);
		// asm volatile("nop");
		_delay_us(1);
		HX711_SCK_PORT &= ~(1<<HX711_SCK_PIN);
	}
#if HX711_ATOMICMODEENABLED == 1
	}
#endif
	return count;
}

int32_t HX711_ReadAverage(uint8_t times) { // Read raw value using average
	int32_t sum = 0;
	uint8_t i = 0;
	for (i=0; i < times; i++) {
		sum += HX711_Read();
	}
	return (int32_t)(sum/times);
}

double HX711_GetValue() { // Perform a read excluding tare
#if HX711_USEAVERAGEONREAD == 1
	return (double)HX711_ReadAverage(HX711_READTIMES)-(double)HX711_offset;
#else
	return (double)HX711_Read()-(double)HX711_offset;
#endif
}

double HX711_GetWeight() { // Return the weight
	return HX711_GetValue()/HX711_scale;
}

void HX711_SetGain(uint16_t gain) { // Set the gain
	if (gain == HX711_GAIN_CHANNEL_A_128) {
		HX711_gain = 1;
	}
	else if (gain == HX711_GAIN_CHANNEL_A_64) {
		HX711_gain = 3;
	}
	else if (gain == HX711_GAIN_CHANNEL_B_32) {
		HX711_gain = 2;
	}
	else {
		HX711_gain = 1;
	}
	
	HX711_SCK_PORT &= ~(1<<HX711_SCK_PIN);
	HX711_Read();
}

uint16_t HX711_GetGain() { // Return the actual gain
	return HX711_gain;
}

void HX711_SetScale(double scale) { // Set the scale to use
	HX711_scale = scale;
}

double HX711_GetScale() { // Return the actual scale
	return HX711_scale;
}

void HX711_SetOffset(int32_t offset) { // Set the offset raw value
	HX711_offset = offset;
}

int32_t HX711_GetOffset() { // Return the actual offset
	return HX711_offset;
}

void HX711_TareToZero() { // Set tare to zero
#if HX711_USEAVERAGEONREAD == 1
	double sum = HX711_ReadAverage(HX711_READTIMES);
#else
	double sum = HX711_Read();
#endif
	HX711_SetOffset(sum);
}

void HX711_Calibrate1SetOffset() {
	// Calibration step 1 of 2, set the offset for tare zero
	int32_t new_offset = HX711_ReadAverage(HX711_CALIBRATIONREADTIMES);
	HX711_SetOffset(new_offset);
	if (HX711_calibration_process) eeprom_write_dword((uint32_t *)EEPROM_HX711_OFFSET, (double)new_offset); // Save to EEPROM
}

void HX711_Calibrate2SetScale(double weight) {
	// Calibration step 2 of 2, set the scale
	double new_scale = (HX711_ReadAverage(HX711_CALIBRATIONREADTIMES)-HX711_offset)/weight;
	HX711_SetScale(new_scale);
	if (HX711_calibration_process) eeprom_write_dword((uint32_t *)EEPROM_HX711_SCALE, new_scale); // Save to EEPROM
}

void HX711_Init(uint8_t gain, double scale, int32_t offset) {
	// Set SCK as output
	HX711_SCK_DDR |= (1<<HX711_SCK_PIN);
	HX711_SCK_PORT &= ~(1<<HX711_SCK_PIN);
	HX711_DT_DDR &=~ (1<<HX711_DT_PIN); // Set DT as input
	HX711_SetGain(gain); // Set gain
	HX711_SetScale(scale); // Set scale
	HX711_SetOffset(offset); // Set offset
}

uint8_t isAnyButtonPressedDown() {
	if (PINF & (1 << BUTTON_1_PIN) || (PINF & (1 << BUTTON_2_PIN)) || (PINF & (1 << BUTTON_3_PIN)) || (PINE & (1 << BUTTON_4_PIN))) {
		return 1;
	}
	return 0;
}

uint8_t isFirstButtonPressedDown() {
	if (PINF & (1 << BUTTON_1_PIN)) {
		return 1;
	}
	return 0;
}

uint8_t isSecondButtonPressedDown() {
	if (PINF & (1 << BUTTON_2_PIN)) {
		return 1;
	}
	return 0;
}

uint8_t isThirdButtonPressedDown() {
	if (PINF & (1 << BUTTON_3_PIN)) {
		return 1;
	}
	return 0;
}

uint8_t isLastButtonPressedDown() {
	if (PINE & (1 << BUTTON_4_PIN)) {
		return 1;
	}
	return 0;
}

void updateMenu(uint8_t menu_mode) {
	LCD_ClearDisplay();
	LCD_GoTo(1, 1);
	if (menu_mode == 0) {
		double weight = HX711_GetWeight();
		if (weight <= 30.0) { // equal or smaller than 30g
			LCD_GoTo(1, 1);
			LCD_DisplayString((uint8_t *)"Low food!");
			LCD_GoTo(2, 1);
			LCD_DisplayString((uint8_t *)"Please refill!");
		} else {
			LCD_DisplayString((uint8_t *)"   CAT FEEDER");
			LCD_GoTo(2, 1);
			LCD_DisplayString((uint8_t *)"(1)Prev  Next(4)");
		}
	}
	else if (menu_mode == MENU_MODE_FOOD_WEIGHT) {
		LCD_DisplayString((uint8_t *)"Food container");
		LCD_GoTo(2, 1);
		double weight = HX711_GetWeight();
		if (weight < 0.0d) {
			LCD_DisplayString((uint8_t *)"Weight: 0g");
		} else {
			char buffer[16];
			dtostrf(weight, 3, 0, buffer);
			LCD_DisplayString((uint8_t *)"Weight: ");
			LCD_DisplayString((uint8_t *)buffer);
			LCD_DisplayString((uint8_t *)"g");	
		}
	}
	else if (menu_mode == MENU_MODE_BUZZER) {
		if (buzzer_activated == 1) {
			LCD_DisplayString((uint8_t *)"Buzzer [ON]/OFF");
			LCD_GoTo(2, 1);
			LCD_DisplayString((uint8_t *)"    (3) Turn OFF");
		} else {
			LCD_DisplayString((uint8_t *)"Buzzer ON/[OFF]");
			LCD_GoTo(2, 1);
			LCD_DisplayString((uint8_t *)"    (3) Turn ON");
		}
	}
	else if (menu_mode == MENU_MODE_FEED) {
		LCD_DisplayString((uint8_t *)"Feed now?");
		LCD_GoTo(2, 1);
		LCD_DisplayString((uint8_t *)"       (3) Yes");
	}
	else if (menu_mode == MENU_MODE_FEEDINGS_PER_DAY) {
		LCD_DisplayString((uint8_t *)"Feedings/day: ");
		char buffer[16];
		itoa(feedings_per_day, buffer, 10);
		LCD_DisplayString((uint8_t *)buffer);
		LCD_GoTo(2, 1);
		if (feedings_per_day <= 1) {
			LCD_DisplayString((uint8_t *)"        (3) +");
		}
		else if (feedings_per_day >= 5) {
			LCD_DisplayString((uint8_t *)"  (2) -");
		}
		else {
			LCD_DisplayString((uint8_t *)"  (2) - (3) +");
		}
	}
	else if (menu_mode == MENU_MODE_CHANGE_FOOD_AMOUNT) {
		LCD_DisplayString((uint8_t *)"Food amount: ");
		char buffer[16];
		itoa(food_amount, buffer, 10);
		LCD_DisplayString((uint8_t *)buffer);
		LCD_GoTo(2, 1);
		if (food_amount <= 1) {
			LCD_DisplayString((uint8_t *)"        (3) +");
		}
		else if (food_amount >= 5) {
			LCD_DisplayString((uint8_t *)"  (2) -");
		}
		else {
			LCD_DisplayString((uint8_t *)"  (2) - (3) +");	
		}
	}
	else if (menu_mode == MENU_MODE_TIMER) {
		char buffer[16];
		sprintf(buffer, "Timer: %02d:%02d:%02d", time_hours, time_minutes, time_seconds);
		LCD_GoTo(1,1);
		LCD_DisplayString((uint8_t *)buffer);
	}
}

void feeding_cycle() {
	DDRC = (1<<MOTOR_EN);
	motor_duty_cycle = DEFAULT_MOTOR_DUTY_CYCLE;
	OCR4A = motor_duty_cycle;
	TCCR4B = (1 << CS40); // Enable timer
	for (int i=0; i < food_amount; i++) {
		_delay_ms(250); // Should deliver the food by 1 cycle
	}
	DDRC = (0<<MOTOR_EN);
	motor_duty_cycle = 0;
	OCR4A = 0;
	TCCR4B = (0 << CS40); // Disable timer
}

void play_note_a() {
	double note_frequency = NOTE_FREQ_A;
	double note_period = (1 / note_frequency) * SEC_IN_uSEC;
	double total_duration = 0;
	while(total_duration <= (SEC_IN_uSEC/4)) {
		total_duration += note_period;
		PORTB = (1 << BUZZER_PIN);
		_delay_us(note_period/2);
		PORTB = (0 << BUZZER_PIN);
		_delay_us(note_period/2);
	}
}

void play_note_as() {
	double note_frequency = NOTE_FREQ_AS;
	double note_period = (1 / note_frequency) * SEC_IN_uSEC;
	double total_duration = 0;
	while(total_duration <= (SEC_IN_uSEC/4)) {
		total_duration += note_period;
		PORTB = (1 << BUZZER_PIN);
		_delay_ms(note_period/2000);
		PORTB = (0 << BUZZER_PIN);
		_delay_ms(note_period/2000);
	}
}

void play_note_f() {
	double note_frequency = NOTE_FREQ_F;
	double note_period = (1 / note_frequency) * SEC_IN_uSEC;
	double total_duration = 0;
	while(total_duration <= (SEC_IN_uSEC/8)) {
		total_duration += note_period;
		PORTB = (1 << BUZZER_PIN);
		_delay_us(note_period/2);
		PORTB = (0 << BUZZER_PIN);
		_delay_us(note_period/2);
	}
}

void play_note_c() {
	double note_frequency = NOTE_FREQ_C;
	double note_period = (1 / note_frequency) * SEC_IN_uSEC;
	double total_duration = 0;
	while(total_duration <= (SEC_IN_uSEC/4)) {
		total_duration += note_period;
		PORTB = (1 << BUZZER_PIN);
		_delay_us(note_period/2);
		PORTB = (0 << BUZZER_PIN);
		_delay_us(note_period/2);
	}
}

void play_note_G() {
	double note_frequency = NOTE_FREQ_G;
	double note_period = (1 / note_frequency) * SEC_IN_uSEC;
	double total_duration = 0;
	while(total_duration <= (SEC_IN_uSEC/2)) {
		total_duration += note_period;
		PORTB = (1 << BUZZER_PIN);
		_delay_us(note_period/2);
		PORTB = (0 << BUZZER_PIN);
		_delay_us(note_period/2);
	}
}

void play_note_D() {
	double note_frequency = NOTE_FREQ_D;
	double note_period = (1 / note_frequency) * SEC_IN_uSEC;
	double total_duration = 0;
	while(total_duration <= (SEC_IN_uSEC/4)) {
		total_duration += note_period;
		PORTB = (1 << BUZZER_PIN);
		_delay_us(note_period/2);
		PORTB = (0 << BUZZER_PIN);
		_delay_us(note_period/2);
	}
}

void play_note_E() {
	double note_frequency = NOTE_FREQ_E;
	double note_period = (1 / note_frequency) * SEC_IN_uSEC;
	double total_duration = 0;
	while(total_duration <= (SEC_IN_uSEC/8)) {
		total_duration += note_period;
		PORTB = (1 << BUZZER_PIN);
		_delay_us(note_period/2);
		PORTB = (0 << BUZZER_PIN);
		_delay_us(note_period/2);
	}
}

void feeding_music(){
	//NOTES TO PLAY When feeding time
	play_note_G();
	_delay_ms(100);
	play_note_E();
	_delay_ms(100);
	play_note_G();
	_delay_ms(1000);
	play_note_G();
	_delay_ms(100);
	play_note_E();
	_delay_ms(100);
	play_note_G();
	_delay_ms(1000);
	play_note_a();
	_delay_ms(100);
	play_note_G();
	_delay_ms(100);
	play_note_f();
	_delay_ms(100);
	play_note_E();
	_delay_ms(100);
	play_note_D();
	_delay_ms(100);
	play_note_E();
	_delay_ms(100);
	play_note_f();
	_delay_ms(100);
	play_note_D();
	_delay_ms(100);
	play_note_E();
	_delay_ms(500);
	play_note_G();
	_delay_ms(100);
	play_note_a();
	_delay_ms(100);
	play_note_G();
}

void millis_init() {
	// Timer settings
	MILLIS_REG_TCCRA = _BV(MILLIS_BIT_WGM);
	MILLIS_REG_TCCRB = MILLIS_CLOCKSEL;
	MILLIS_REG_TIMSK = _BV(MILLIS_BIT_OCIE);
	MILLIS_REG_OCR = ((F_CPU / MILLIS_PRESCALER) / 1000);
}


millis_t millis_get() { // Get current milliseconds
	millis_t ms;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		ms = milliseconds;
	}
	return ms;
}

void millis_reset() { // Reset milliseconds count to 0
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		milliseconds = 0;
	}
}

ISR(MILLIS_ISR_VECT)
{
	++milliseconds;
}
