/******************************************************************************
; Universidad del Valle de Guatemala
; 1E2023: Programacion de Microcontroladores
; PWM.c
; Autor: Jacob Tabush
; Proyecto: Libreria PWM
; Hardware: ATMEGA328P
; Creado: 15/04/2024
; Ultima modificacion: 15/04/2024
*******************************************************************************/

#include "PWM.h"

int fakePWMtop;
int fakePWMduty;
int fakePWMcounter;
int fakePWMtimertop;

// Inicializar PWM0

void initPWM0FastA(uint8_t resetorno, uint8_t inverted, uint16_t prescaler){
	//Poner a PD6 como salida
	DDRD |= (1<<DDD6);
	
	if (resetorno) { //Para evitar eliminar la configuracion de una al modificar la otra
	TCCR0A = 0;
	TCCR0B = 0;}
	
	TCCR0A |= (1<<COM0A1); //Encendemos OC0A
	// Configurando OC0A como invertido o no invertido
	if (inverted) {TCCR0A |= (1<<COM0A0);}

	// Modo PWM Fast TOP = 0xFF
	TCCR0A |= (1<<WGM01)|(1<<WGM00);
	
	//Prescaler
	
	switch(prescaler) {
		case 1: TCCR0B |= (1<<CS00);
		break;
		
		case 8: TCCR0B |= (1<<CS01);
		break;
		
		case 64: TCCR0B |= (1<<CS01)|(1<<CS00);
		break;
		
		case 256: TCCR0B |= (1<<CS02);
		break;
		
		case 1024: TCCR0B |= (1<<CS02)|(1<<CS00);
		break;
		
		default: TCCR0B |= (1<<CS02)|(1<<CS00); //1024 de default
	}
	
	
	};

void initPWM0FastB(uint8_t resetorno, uint8_t inverted, uint16_t prescaler){
	//Poner a PD6 como salida
	DDRD |= (1<<DDD5);
	
	if (resetorno) {
	TCCR0A = 0;
	TCCR0B = 0;}
	
	TCCR0A |= (1<<COM0B1); //Encendemos OC0A
	// Configurando OC0A como invertido o no invertido
	if (inverted) {TCCR0A |= (1<<COM0B0);}

	// Modo PWM Fast TOP = 0xFF
	TCCR0A |= (1<<WGM01)|(1<<WGM00);
	
	//Prescaler
	
	switch(prescaler) {
		case 1: TCCR0B |= (1<<CS00);
		break;
		
		case 8: TCCR0B |= (1<<CS01);
		break;
		
		case 64: TCCR0B |= (1<<CS01)|(1<<CS00);
		break;
		
		case 256: TCCR0B |= (1<<CS02);
		break;
		
		case 1024: TCCR0B |= (1<<CS02)|(1<<CS00);
		break;
		
		default: TCCR0B |= (1<<CS02)|(1<<CS00); //1024 de default
	}
	
	
};

// Inicializar PWM1

void initPWM1FastTopA(uint8_t resetorno, uint8_t inverted, uint16_t prescaler, uint16_t top){
	//Poner a PB1 como salida
	DDRB |= (1<<DDB1);
	
	if (resetorno) { //Para evitar eliminar la configuracion de una al modificar la otra
		TCCR1A = 0;
	TCCR1B = 0;}
	
	TCCR1C = 0;
	
	TCCR1A |= (1<<COM1A1); //Encendemos OC0A
	// Configurando OC0A como invertido o no invertido
	if (inverted) {TCCR1A |= (1<<COM1A0);}

	// Modo PWM Fast 10 bit TOP = ICR1
	TCCR1A |= (1<<WGM13)|(1<<WGM12)|(1<<WGM11);
	
	//Prescaler
	
	switch(prescaler) {
		case 1: TCCR1B |= (1<<CS10);
		break;
		
		case 8: TCCR1B |= (1<<CS11);
		break;
		
		case 64: TCCR1B |= (1<<CS11)|(1<<CS10);
		break;
		
		case 256: TCCR1B |= (1<<CS12);
		break;
		
		case 1024: TCCR1B |= (1<<CS12)|(1<<CS10);
		break;
		
		default: TCCR1B |= (1<<CS12)|(1<<CS10); //1024 de default
	}
	
	ICR1 = top;
};


// controlar pwm con A y B
void initPWM1FastTopAB(uint8_t resetorno, uint8_t inverted, uint16_t prescaler, uint16_t top){
	//Poner a PB1 como salida
	DDRB |= (1<<DDB1) | (1<<DDB2);
	
	if (resetorno) { //Para evitar eliminar la configuracion de una al modificar la otra
		TCCR1A = 0;
	TCCR1B = 0;}
	
	TCCR1C = 0;
	
	TCCR1A |= ((1<<COM1A1) | (1<<COM1B1)); //Encendemos OC0A
	// Configurando OC0A como invertido o no invertido
	if (inverted) {TCCR1A |= ((1<<COM1A0) | (1<<COM1B0));}

	// Modo PWM Fast 10 bit TOP = ICR1
	TCCR1A |= (1<<WGM13)|(1<<WGM12)|(1<<WGM11);
	
	//Prescaler
	
	switch(prescaler) {
		case 1: TCCR1B |= (1<<CS10);
		break;
		
		case 8: TCCR1B |= (1<<CS11);
		break;
		
		case 64: TCCR1B |= (1<<CS11)|(1<<CS10);
		break;
		
		case 256: TCCR1B |= (1<<CS12);
		break;
		
		case 1024: TCCR1B |= (1<<CS12)|(1<<CS10);
		break;
		
		default: TCCR1B |= (1<<CS12)|(1<<CS10); //1024 de default
	}
	
	ICR1 = top;
	
	}


// Inicializar PWM2

void initPWM2FastA(uint8_t resetorno, uint8_t inverted, uint16_t prescaler) {
	//Poner a PD3 como salida
	DDRB |= (1<<DDB3);
	
	if (resetorno) { //Para evitar eliminar la configuracion de una al modificar la otra
		TCCR2A = 0;
	TCCR2B = 0;}
	
	TCCR2A |= (1<<COM2A1); //Encendemos OC2A
	// Configurando OC0A como invertido o no invertido
	if (inverted) {TCCR2A |= (1<<COM2A0);}

	// Modo PWM Fast TOP = 0xFF
	TCCR2A |= (1<<WGM21)|(1<<WGM20);
	
	//Prescaler
	
	switch(prescaler) {
		case 1: TCCR2B |= (1<<CS00);
		break;
		
		case 8: TCCR2B |= (1<<CS01);
		break;
		
		case 32: TCCR2B |= (1<<CS21)|(1<<CS20);
		break;
		
		case 64: TCCR2B |= (1<<CS22);
		break;
		
		case 128: TCCR2B |= (1<<CS22)|(1<<CS20);
		break;
		
		case 256: TCCR2B |= (1<<CS22)|(1<<CS21);
		break;
		
		case 1024: TCCR2B |= (1<<CS02)|(1<<CS21)|(1<<CS00);
		break;
		
		default: TCCR2B |= (1<<CS02)|(1<<CS21)|(1<<CS00); //1024 de default
	}
	
	
}

void initPWM2FastB(uint8_t resetorno, uint8_t inverted, uint16_t prescaler){
	//Poner a PD3 como salida
	DDRD |= (1<<DDD3);
	
	if (resetorno) { //Para evitar eliminar la configuracion de una al modificar la otra
		TCCR2A = 0;
	TCCR2B = 0;}
	
	TCCR2A |= (1<<COM2B1); //Encendemos OC0A
	// Configurando OC0A como invertido o no invertido
	if (inverted) {TCCR2A |= (1<<COM2B0);}

	// Modo PWM Fast TOP = 0xFF
	TCCR2A |= (1<<WGM21)|(1<<WGM20);
	
	//Prescaler
	
	switch(prescaler) {
		case 1: TCCR2B |= (1<<CS00);
		break;
		
		case 8: TCCR2B |= (1<<CS01);
		break;
		
		case 32: TCCR2B |= (1<<CS21)|(1<<CS20);
		break;
		
		case 64: TCCR2B |= (1<<CS22);
		break;
		
		case 128: TCCR2B |= (1<<CS22)|(1<<CS20);
		break;
		
		case 256: TCCR2B |= (1<<CS22)|(1<<CS21);
		break;
		
		case 1024: TCCR2B |= (1<<CS02)|(1<<CS21)|(1<<CS00);
		break;
		
		default: TCCR2B |= (1<<CS02)|(1<<CS21)|(1<<CS00); //1024 de default
	}
	
	
}

void initPWM2FastTop(uint8_t inverted, uint16_t prescaler, uint8_t top){
	//Poner a PD3 como salida
	DDRD |= (1<<DDD3);
	
		TCCR2A = 0;
	TCCR2B = 0;
	
	TCCR2A |= (1<<COM2B1); //Encendemos OC2B
	// Configurando OC0A como invertido o no invertido
	if (inverted) {TCCR2A |= (1<<COM2B0);}

	// Modo PWM Fast TOP = OCRA
	TCCR2A |= (1<<WGM22)|(1<<WGM21)|(1<<WGM20);
	
	//Prescaler
	
	switch(prescaler) {
		case 1: TCCR2B |= (1<<CS00);
		break;
		
		case 8: TCCR2B |= (1<<CS01);
		break;
		
		case 32: TCCR2B |= (1<<CS21)|(1<<CS20);
		break;
		
		case 64: TCCR2B |= (1<<CS22);
		break;
		
		case 128: TCCR2B |= (1<<CS22)|(1<<CS20);
		break;
		
		case 256: TCCR2B |= (1<<CS22)|(1<<CS21);
		break;
		
		case 1024: TCCR2B |= (1<<CS02)|(1<<CS21)|(1<<CS00);
		break;
		
		default: TCCR2B |= (1<<CS02)|(1<<CS21)|(1<<CS00); //1024 de default
	}
	
	OCR2A = top;
	
	
}

// Inicializa u PWM pero solo utilizando funciones del timer0 y no del pwm en si 

void initPWM0Fake(uint8_t duty, uint8_t top, uint8_t timertop, uint16_t prescaler){
	//Poner a PD6 como salida
	DDRB |= (1<<DDB5);
	
	TCCR0A = 0;
	TCCR0B = 0;
	
	fakePWMcounter = 0; // aumentamos cada vez que termina el timer
	fakePWMduty = duty; // apagamos el pin cuando llega a duty
	fakePWMtimertop = timertop; //valor tcnt
	fakePWMtop = top; // valor maximo del counterexterno
	
	TIMSK0 = 0x01; // habilitamos interrupt en TIMR0 overflow
	
	TCNT0 = 255 - fakePWMtimertop; 
	
	//Prescaler
	
	switch(prescaler) { // colocamos valores de prescaler
		case 1: TCCR0B |= (1<<CS00);
		break;
		
		case 8: TCCR0B |= (1<<CS01);
		break;
		
		case 64: TCCR0B |= (1<<CS01)|(1<<CS00);
		break;
		
		case 256: TCCR0B |= (1<<CS02);
		break;
		
		case 1024: TCCR0B |= (1<<CS02)|(1<<CS00);
		break;
		
		default: TCCR0B |= (1<<CS02)|(1<<CS00); //1024 de default
	}
	
}

// a diferencia de la pasada en esta simplemente cambiamos cuando llegamos a OCR0A

void initPWM0FakeV2(uint8_t duty, uint8_t top, uint16_t prescaler){
	//Poner a PD6 como salida
	DDRB |= (1<<DDB5);
	
	TCCR0A = 0;
	TCCR0B = 0;
	
	TIMSK0 = 0x03; // habilitamos interrupt en TIMR0 overflow y OCR0A compare match
	
	TCNT0 = 255 - top;
	OCR0A = duty;
	
	
	
	//Prescaler
	
	switch(prescaler) { // colocamos valores de prescaler
		case 1: TCCR0B |= (1<<CS00);
		break;
		
		case 8: TCCR0B |= (1<<CS01);
		break;
		
		case 64: TCCR0B |= (1<<CS01)|(1<<CS00);
		break;
		
		case 256: TCCR0B |= (1<<CS02);
		break;
		
		case 1024: TCCR0B |= (1<<CS02)|(1<<CS00);
		break;
		
		default: TCCR0B |= (1<<CS02)|(1<<CS00); //1024 de default
	}
	
}

//update duty cycles

void updateDutyCycle0A(uint8_t duty) {
	OCR0A = duty;}

void updateDutyCycle0B(uint8_t duty) {
	OCR0B = duty;}
	
void updateDutyCycle1A(uint16_t duty) {
	OCR1A = duty;
}

void updateDutyCycle1B(uint16_t duty) {
	OCR1B = duty;
}


void updateDutyCycle2A(uint8_t duty) {
	OCR2A = duty;
}

void updateDutyCycle2B(uint8_t duty) {
	OCR2B = duty;
}

void updateDutyCycleFake(uint8_t duty) {
	fakePWMduty = duty;
}