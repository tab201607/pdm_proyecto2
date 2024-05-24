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

#define F_CPU 2000000
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

void initUART9600(void);
void writeUART(char senddata);
void writeString(char* senddata);
void sendnumberasstring(char datain);

const uint8_t timer2reset = 255;
const uint16_t debouncetimerrestart = 8;

uint8_t debouncetimer = 0;
uint8_t debouncetimer2 = 0;
uint16_t senddatatimer = 0;
const uint16_t sendatatimerrestart = 8;

bool eepromactive = 0;
bool adafruitactive = 0; 
uint8_t eepromselect = 0;

uint8_t recordlight = 0;
const uint8_t recordlightrestart = 5;

// Grabamos los valores que medimos en los ADCs en estos variables
uint8_t ADCResult1 = 0;
uint8_t ADCResult2 = 0;
uint8_t ADCResult3 = 0;
uint8_t ADCResult4 = 0;

uint8_t ADCChannel = 6; //Canal ADC seleccionado

char basestring[] = "BASE\n";
char hombrostring[] = "HOMBRO\n";
char codostring[] = "CODO\n";
char pinzastring[] = "PINZA\n";

void setup(void)
{
	cli();
	
	// Control prescaler
	CLKPR = (0b10000000); // habilitamos cambios del prescale
	CLKPR = (0x03); // Colocamos prescaler de 8

	//Inicializamos botones (C0 - C3 para control de EEPROM)
	PCICR = (0b00000011);
	PCMSK1 = (1<<PCINT8) | (1<<PCINT9) | (1<<PCINT10) | (1<<PCINT11); //Interrupts en C0 - C3
	PCMSK0 = (1<<PCINT5);
	PORTC = (1<<PORTC0) | (1<<PORTC1) | (1<<PORTC2) | (1<<PORTC3); //Pullup
	PORTB |= (1<<PORTB5);
	DDRC = 0;

	// Inicializacion PWM
	initPWM0FastA(reset, no_invertido, 256); // Activamos timer 0 en modo pwm, utilizando OCR0A y OCR0B
	initPWM0FastB(no_reset, no_invertido, 256);
	initPWM2FastA(reset, no_invertido, 128);
	initPWM2FastB(no_reset, no_invertido, 128);
	
	//Inicializamos Timer2 para debounce y "PWM"
	TCCR1A = 0;
	TCCR1B = 0b00000001; //Prescaler 256
	TIMSK1 = 0x01; //Overflow interrupt
	TCNT1 = 0xFFFF - timer2reset; 

	DDRD |= 0b10010100; // D2 y D4 - luces mostrando eeprom seleccionado
	//D7 - luz eeprom activo */
	DDRB |= (1<<DDB4); // Luz para mostrar que adafruit esta activo
	
	initADC();
	
	initUART9600();
	
	sei();
}

int main(void)
{
	setup();
	
	
	while (1) //Ejecutamos los ADCs de los 3 channels
	{
		
		//Bloque principal
		
		if (!eepromactive) {//Si el eeprom esta desactivado revisamos datos del ADC
		for (int i = 4; i < 8; i++) {
		ADCChannel = i; //iniciamos el ADC convirtiendo el canal 4
		convertADC(i);
		_delay_ms(300);
		}
		
		if (senddatatimer == 0) {
		//Mandar valor base
		uint16_t angleresult1 = ADCResult1 * 6 / 17; //0-255 -> 0-90
		writeString(basestring);
		sendnumberasstring(angleresult1);
		
		//Mandar valor codo
		uint16_t angleresult2 = ADCResult2 * 6/17; //0-255 -> 0-90
		writeString(codostring);
		sendnumberasstring(angleresult2);
		
		//Mandar valor hombro
		uint16_t angleresult3 = ADCResult3 * 4/17 + 60; //0-255 -> 60-120
		writeString(hombrostring);
		sendnumberasstring(angleresult3);
		
		//Mandar valor pinza
		uint16_t angleresult4 = ADCResult4 * 5/17; //0-255 -> 0-75
		writeString(pinzastring);
		sendnumberasstring(angleresult4);
		
		senddatatimer = sendatatimerrestart;
		}
		else {senddatatimer--;}
		
		}
		else if (eepromactive) { //Si el eeprom esta activado 
			PORTD |= (1<<PORTD7);
			uint8_t eepromdata[4];
			uint8_t eepromaddress = eepromselect * 4;
			
			eeprom_read_block(eepromdata, eepromaddress, 4);
			moveServos(eepromdata[0], eepromdata[1], eepromdata[2], eepromdata[3]);
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
	cli();
	 //Los siguientes toman los valores de los ADCs y los graban en sus ADCs respectivos 
	if (ADCChannel == 4) {
		ADCResult1 = ADCH;}
	else if (ADCChannel == 6) {
		ADCResult2 = ADCH;
	}
	else if (ADCChannel == 5) {
		ADCResult3 = ADCH;
	}
	else if (ADCChannel == 7) {
		ADCResult4 = ADCH;
	} 
	
	if (!(eepromactive | adafruitactive)) {moveServos(ADCResult1, ADCResult2, ADCResult3, ADCResult4);}

	// transformamos el resultado de ADC para usarlo para timer1
	
	sei();
	ADCSRA |= (1<<ADIF);
	
	return;
}

// ISR de timer2 para debounce
ISR(TIMER1_OVF_vect){
	
	if (debouncetimer > 0) {
		debouncetimer--; //Decrementamos el timer de debounce
	}
	
	if (debouncetimer2 > 0) {
		debouncetimer2--; //Decrementamos el timer de debounce
	}
	
	if (~eepromactive){
	if (recordlight > 0) {
		recordlight--; // Para tener un efecto de un luz momentaneo al momento de apachar grabar
	}
	else {PORTD &= ~(1<<PORTD7);
	}
	}
	
	TCNT1 = 0xFF - timer2reset;
	//TIFR1 |= (1 << TOV1);
}

ISR(PCINT0_vect){ //Apagar y encender modo adafruit
	if (debouncetimer2 != 0) {return;}
	else {debouncetimer2 = 10; //encendemos debounce
	}
	
	if (adafruitactive) {PORTB &= ~(1<<PORTB4);
		adafruitactive = 0;}
	else {PORTB |= (1<<PORTB4);
		adafruitactive = 1;}
}

// ISR de botones para EEPROM
ISR(PCINT1_vect){
	
	if (debouncetimer != 0) {return;}
		else {debouncetimer = debouncetimerrestart; //encendemos debounce
		}
		
	if ((PINC & (0b00000001)) == 0) { //C0 - next eeprom
		eepromselect++;
		if (eepromselect > 0b00000011) {eepromselect = 0;}
		
		//Desplegamos luz eeprom
		PORTD &= 0b11101011;
		PORTD |= ((eepromselect & 0x01) << 2);
		PORTD |= ((eepromselect & 0x02) << 3);
	}
	else if ((((PINC & (0b00000010)) == 0) & ~(eepromactive))) { //C1 - record
		//encendemos un luz para un segundo para mostrar que grabamos el valor
		recordlight = recordlightrestart;
		PORTD |= (1<<PORTD7);
		
		//Grabamos el valor a eeprom
		uint8_t eepromdata[4] = {ADCResult1, ADCResult2, ADCResult3, ADCResult4};
		uint8_t eepromaddress = eepromselect * 4;
		eeprom_write_block(eepromdata, eepromaddress, 4);
	}
	
	if ((PINC & (0b00000100)) == 0) { //C2 - play
		
		if (eepromactive) {PORTD &= ~(1<<PORTD7);
		eepromactive = 0;}
			else {PORTD |= (1<<PORTD7);
			eepromactive = 1;}
	}
	
}


//Este funcion sirve para mover los 4 servos
void moveServos(uint16_t servo1, uint16_t servo2, uint16_t servo3, uint16_t servo4) { 
	
	uint16_t ADCRT1 = servo1/16 + 7; 
	updateDutyCycle2A(ADCRT1);	//Base 0 - 90 grados
	
	uint16_t ADCRT2 = servo2/16 + 7; 
	updateDutyCycle2B(ADCRT2); //Codo 0-90
	
	uint8_t ADCRT3 = servo3/48 + 9;
	updateDutyCycle0A(ADCRT3); //Hombro 60 - 120 grados
	
	uint8_t ADCRT4 = servo4/38 + 4;
	updateDutyCycle0B(ADCRT4); //Pinza 0 - 75 grados
	
}

//Funciones UART

void initUART9600(void) {
	DDRD |= (1<<DDD1); // arreglamos d0 y d1
	
	UCSR0A = 0;
	UCSR0A |= (1<<U2X0); // double speed
	
	UCSR0B = 0;
	UCSR0B |= (1<<RXCIE0)|(1<<RXEN0)|(1<<TXEN0); //RX con interrupt y TX encendido
	
	UCSR0C = 0;
	UCSR0C |= (1<< UCSZ01)|(1<<UCSZ00); //8 bits sin paridad con 1 bit de stop
	
	UBRR0 = 25; //Baud rate de 9600 para prescaler de 2MHz
	
}



void writeUART(char senddata){ // Funcion para mandar un caracter por UART
	while(!(UCSR0A & (1<<UDRE0) )); // Revisamos si el buffer esta lleno
	UDR0 = senddata; //Colocamos el valor a mandar
}

void writeString(char* senddata){ //Funcion para mandar varios datos
	for (uint8_t i = 0; senddata[i] != '\0'; i++) { //Seguimos mandando hasta que nos topamos con un caracter vacio
		writeUART(senddata[i]);
	}
}

void sendnumberasstring(char datain){
	writeUART(datain/100 + 0x30); // desplegar cienes
	writeUART((datain % 100)/10 + 0x30); // desplegar decenas
	writeUART((datain % 100) % 10 + 0x30); //desplegar unidades
	writeUART(10); //newline
}