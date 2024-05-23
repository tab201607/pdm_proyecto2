/******************************************************************************
; Universidad del Valle de Guatemala
; 1E2023: Programacion de Microcontroladores
; main.c
; Autor: Jacob Tabush
; Proyecto: Proyecto 2
; Hardware: ATMEGA328P
; Creado: 8/05/2024
; Ultima modificacion: 22/05/2024
*******************************************************************************

#define F_CPU 1000000
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include "PWM.h"

void initADC(void);
void convertADC(char channel);
void moveServos(uint16_t servo1, uint16_t servo2, uint16_t servo3, uint16_t servo4);

const uint8_t timer2reset = 255;
const uint8_t debouncetimerrestart = 255;

uint8_t debouncetimer = 0;

bool eepromactive = 0;
bool adafruitactive = 0; 
uint8_t eepromselect = 0;

// Grabamos los valores que medimos en los ADCs en estos variables
uint8_t ADCResult1 = 0;
uint8_t ADCResult2 = 0;
uint8_t ADCResult3 = 0;
uint8_t ADCResult4 = 0;

uint8_t ADCChannel = 6; //Canal ADC seleccionado

void setup(void)
{
	cli();
	
	// Control prescaler
	CLKPR = (0b10000000); // habilitamos cambios del prescale
	CLKPR = (0x03); // Colocamos prescaler de 8

	//Inicializamos botones (C0 - C3 para control de EEPROM)
	PCICR = (0b00000010);
	PCMSK1 = (1<<PCINT8) | (1<<PCINT9) | (1<<PCINT10) | (1<<PCINT11); //Interrupts en C0 - C3
	PORTC = (1<<PORTC0) | (1<<PORTC1) | (1<<PORTC2) | (1<<PORTC3); //Pullup
	DDRC = 0;

	// Inicializacion PWM
	initPWM1FastTopAB(reset, no_invertido, 8, 2499); //activamos el timer 1 en modo pwm, utilizando OCR1A y OCR1B
	initPWM0FastA(reset, no_invertido, 256); // Activamos timer 0 en modo pwm, utilizando OCR0A y OCR0B
	initPWM0FastB(no_reset, no_invertido, 256);
	initPWM2FastB(reset, no_invertido, 256);
	
	//Inicializamos Timer2 para debounce y "PWM"
	/*TCCR2A = 0;
	TCCR2B = 0;
	TCCR2A |= 
	TCCR2B |= 0b00000110; //Prescaler 256
	TIMSK2 = 0x01; //Overflow y compare A interrupt
	TCNT2 = timer2reset; 
	//OCR2A = 128;
	//DDRB |= (1<<DDB2);
	DDRD |= 0b10010100; // D2 y D4 - luces mostrando eeprom seleccionado
	//D7 - luz mostrando que se esta desplegando luz 
	
	initADC();
	
	sei();
}

int main(void)
{
	setup();
	
	
	while (1) //Ejecutamos los ADCs de los 3 channels
	{
		
		//Bloque principal
		
		for (int i = 4; i < 8; i++) {
		ADCChannel = i; //iniciamos el ADC convirtiendo el canal 4
		convertADC(i);
		_delay_ms(300);
		}

	}
}

//////////////////////////////////////////////////////
// Funciones ADC
//////////////////////////////////////////////////////

void initADC(void) //Funcion para inicializar el ADC
{
	ADMUX = 0;
	
	ADMUX |= (1<<REFS0); //conectamos a AVcc
	ADMUX &= ~(1<<REFS1);
	
	ADMUX |= (1<<ADLAR); // Justificado a la izquierda
	
	ADCSRA |= (1<<ADEN); //Encendemos el ADC
	ADCSRA |= (1<<ADIE); // Encendemos el interrupt
	ADCSRA |= (0b00000100); //Prescaler de 16
}

void convertADC(char channel) //Funcion para leer info ADC
{	ADMUX &= ~(0x0F); // Borramos los ultimos 4 bits de ADMUX
	
	if (channel > 8) {channel = 8;} //valor maximo es 8
	
	if (channel < 6) {DIDR0 = channel;}
	
	ADMUX |= channel; // seleccionamos el canal correcto
	ADCSRA |= (1<<ADSC); // iniciamos el ADC
}

//////////////////////////////////////////////////////
// Funciones ISR
//////////////////////////////////////////////////////

ISR(ADC_vect){
	
	 //Los siguientes toman los valores de los ADCs y los graban en sus ADCs respectivos 
	if (ADCChannel == 4) {
		ADCResult1 = ADCH;}
	else if (ADCChannel == 5) {
		ADCResult2 = ADCH;
	}
	else if (ADCChannel == 6) {
		ADCResult3 = ADCH;
	}
	else if (ADCChannel == 7) {
		ADCResult4 = ADCH;
	} 
	
	if (!(eepromactive | adafruitactive)) {moveServos(ADCResult1, ADCResult2, ADCResult3, ADCResult4);}

	// transformamos el resultado de ADC para usarlo para timer1
	
	
	ADCSRA |= (1<<ADIF);
	return;
}

// ISR de timer2 para debounce
ISR(TIMER2_OVF_vect){

	//PORTB |= (1<<PORTB2); //Encendemos PORTB3 
	
	if (debouncetimer > 0) {
		debouncetimer--;
	}
	
	TCNT2 = timer2reset;
	TIFR2 |= (1 << TOV2);
	return;
}

ISR(TIMER2_COMPA_vect) {
	PORTB &= ~(1<<PORTB2); //Apagamos PORTB3
	TIFR2 |= (1<<OCIE2A);
}

// ISR de botones para EEPROM
ISR(PCINT2_vect){
	
	if (debouncetimer != 0) {return;}
		debouncetimer = 0xFF; //encendemos debounce
	
	if ((PINC & (0b00000001)) == 0) { //C0 - play

	}
	else if ((PINC & (0b00000010)) == 0) { //C1 - record

	}
	else if ((PINC & (0b00000100)) == 0) { //C2 - next eeprom
		eepromselect++;
		if (eepromselect > 3) {eepromselect = 0;}
	
		//Desplegamos luz eeprom	
		PORTD &= 0b11110101;
		PORTD |= ((eepromselect & 0x01) << 1);
		PORTD |= ((eepromselect & 0x02) << 2);
	}
	
	else if ((PINC & (0b00001000)) == 0) { //C3 - back eeprom
		eepromselect--;
		if (eepromselect < 3) {eepromselect = 3;}
		
		//Desplegamos modo eeprom
		PORTD &= 0b11110101;
		PORTD |= ((eepromselect & 0x01) << 1);
		PORTD |= ((eepromselect & 0x02) << 2);
	}
	
	return;
	
}


//Este funcion sirve para mover los 4 servos
void moveServos(uint16_t servo1, uint16_t servo2, uint16_t servo3, uint16_t servo4) { 
	uint16_t ADCRT1 = servo1 + 65;
	updateDutyCycle1A(ADCRT1);	
	
	uint16_t ADCRT2 = servo2 + 65;
	updateDutyCycle1B(ADCRT2);
	
	uint8_t ADCRT3 = servo3/16 + 4;
	updateDutyCycle0A(ADCRT3);
	
	uint8_t ADCRT4 = servo4/16 + 4;
	updateDutyCycle0B(ADCRT4);
	
}
*/
	
	
	
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
	
	
	
/******************************************************************************
; Universidad del Valle de Guatemala
; 1E2023: Programacion de Microcontroladores
; main.c
; Autor: Jacob Tabush
; Proyecto: Proyecto 2
; Hardware: ATMEGA328P
; Creado: 8/05/2024
; Ultima modificacion: 22/05/2024
*******************************************************************************/

/*#define F_CPU 1000000
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include "PWM.h"

void initADC(void);
void convertADC(char channel);
void moveServos(uint16_t servo1, uint16_t servo2, uint16_t servo3, uint16_t servo4);

const uint8_t timer2reset = 255;
const uint8_t debouncetimerrestart = 255;

uint8_t debouncetimer = 0;

bool eepromactive = 0;
bool adafruitactive = 0; 
uint8_t eepromselect = 0;

// Grabamos los valores que medimos en los ADCs en estos variables
uint8_t ADCResult1 = 0;
uint8_t ADCResult2 = 0;
uint8_t ADCResult3 = 0;
uint8_t ADCResult4 = 0;

uint8_t ADCChannel = 6; //Canal ADC seleccionado

void setup(void)
{
	cli();
	
	// Control prescaler
	CLKPR = (0b10000000); // habilitamos cambios del prescale
	CLKPR = (0x03); // Colocamos prescaler de 8

	//Inicializamos botones (C0 - C3 para control de EEPROM)
	PCICR = (0b00000010);
	PCMSK1 = (1<<PCINT8) | (1<<PCINT9) | (1<<PCINT10) | (1<<PCINT11); //Interrupts en C0 - C3
	PORTC = (1<<PORTC0) | (1<<PORTC1) | (1<<PORTC2) | (1<<PORTC3); //Pullup
	DDRC = 0;

	// Inicializacion PWM
	initPWM0FastA(reset, no_invertido, 256); // Activamos timer 0 en modo pwm, utilizando OCR0A y OCR0B
	initPWM0FastB(no_reset, no_invertido, 256);
	initPWM2FastA(reset, no_invertido, 128);
	initPWM2FastB(no_reset, no_invertido, 128);
	
	//Inicializamos Timer2 para debounce y "PWM"
	/*TCCR2A = 0;
	TCCR2B = 0;
	TCCR2A |= 
	TCCR2B |= 0b00000110; //Prescaler 256
	TIMSK2 = 0x01; //Overflow y compare A interrupt
	TCNT2 = timer2reset; 
	//OCR2A = 128;
	//DDRB |= (1<<DDB2);

	DDRD |= 0b10010100; // D2 y D4 - luces mostrando eeprom seleccionado
	//D7 - luz mostrando que se esta desplegando luz 
	
	initADC();
	
	sei();
}

int main(void)
{
	setup();
	
	
	while (1) //Ejecutamos los ADCs de los 3 channels
	{
		
		//Bloque principal
		
		for (int i = 4; i < 8; i++) {
		ADCChannel = i; //iniciamos el ADC convirtiendo el canal 4
		convertADC(i);
		_delay_ms(300);
		}

	}
}

//////////////////////////////////////////////////////
// Funciones ADC
//////////////////////////////////////////////////////

void initADC(void) //Funcion para inicializar el ADC
{
	ADMUX = 0;
	
	ADMUX |= (1<<REFS0); //conectamos a AVcc
	ADMUX &= ~(1<<REFS1);
	
	ADMUX |= (1<<ADLAR); // Justificado a la izquierda
	
	ADCSRA |= (1<<ADEN); //Encendemos el ADC
	ADCSRA |= (1<<ADIE); // Encendemos el interrupt
	ADCSRA |= (0b00000100); //Prescaler de 16
}

void convertADC(char channel) //Funcion para leer info ADC
{	ADMUX &= ~(0x0F); // Borramos los ultimos 4 bits de ADMUX
	
	if (channel > 8) {channel = 8;} //valor maximo es 8
	
	if (channel < 6) {DIDR0 = channel;}
	
	ADMUX |= channel; // seleccionamos el canal correcto
	ADCSRA |= (1<<ADSC); // iniciamos el ADC
}

//////////////////////////////////////////////////////
// Funciones ISR
//////////////////////////////////////////////////////

ISR(ADC_vect){
	
	 //Los siguientes toman los valores de los ADCs y los graban en sus ADCs respectivos 
	if (ADCChannel == 4) {
		ADCResult1 = ADCH;}
	else if (ADCChannel == 5) {
		ADCResult2 = ADCH;
	}
	else if (ADCChannel == 6) {
		ADCResult3 = ADCH;
	}
	else if (ADCChannel == 7) {
		ADCResult4 = ADCH;
	} 
	
	if (!(eepromactive | adafruitactive)) {moveServos(ADCResult1, ADCResult2, ADCResult3, ADCResult4);}

	// transformamos el resultado de ADC para usarlo para timer1
	
	
	ADCSRA |= (1<<ADIF);
	return;
}

// ISR de timer2 para debounce
ISR(TIMER2_OVF_vect){

	//PORTB |= (1<<PORTB2); //Encendemos PORTB3 
	
	if (debouncetimer > 0) {
		debouncetimer--;
	}
	
	TCNT2 = timer2reset;
	TIFR2 |= (1 << TOV2);
	return;
}

ISR(TIMER2_COMPA_vect) {
	PORTB &= ~(1<<PORTB2); //Apagamos PORTB3
	TIFR2 |= (1<<OCIE2A);
}

// ISR de botones para EEPROM
ISR(PCINT2_vect){
	
	if (debouncetimer != 0) {return;}
		debouncetimer = 0xFF; //encendemos debounce
	
	if ((PINC & (0b00000001)) == 0) { //C0 - play

	}
	else if ((PINC & (0b00000010)) == 0) { //C1 - record

	}
	else if ((PINC & (0b00000100)) == 0) { //C2 - next eeprom
		eepromselect++;
		if (eepromselect > 3) {eepromselect = 0;}
	
		//Desplegamos luz eeprom	
		PORTD &= 0b11110101;
		PORTD |= ((eepromselect & 0x01) << 1);
		PORTD |= ((eepromselect & 0x02) << 2);
	}
	
	else if ((PINC & (0b00001000)) == 0) { //C3 - back eeprom
		eepromselect--;
		if (eepromselect < 3) {eepromselect = 3;}
		
		//Desplegamos modo eeprom
		PORTD &= 0b11110101;
		PORTD |= ((eepromselect & 0x01) << 1);
		PORTD |= ((eepromselect & 0x02) << 2);
	}
	
	return;
	
}


//Este funcion sirve para mover los 4 servos
void moveServos(uint16_t servo1, uint16_t servo2, uint16_t servo3, uint16_t servo4) { 
	uint16_t ADCRT1 = servo1/8 + 7;
	updateDutyCycle2A(ADCRT1);	
	
	uint16_t ADCRT2 = servo2/8 + 7;
	updateDutyCycle2B(ADCRT2);
	
	uint8_t ADCRT3 = servo3/16 + 4;
	updateDutyCycle0A(ADCRT3);
	
	uint8_t ADCRT4 = servo4/16 + 4;
	updateDutyCycle0B(ADCRT4);
	
}*/