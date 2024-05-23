#include "PWM.h"  // Inclusión del archivo de cabecera PWM.h

// Función para inicializar el PWM
void PWM_init(void){
	// Configuración de los pines PB1 (OC1A) y PB2 (OC1B) como salidas de PWM
	DDRB |= (1 << PB1) | (1 << PB2);
	
	// Reset del contador del Timer/Counter 1
	TCNT1 = 0;
	
	// Establecimiento del valor de TOP para el modo Fast PWM
	ICR1 = 39999;  // TOP
	
	// Configuración del modo de operación del Timer/Counter 1: Fast PWM TOP ICR1
	TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (0 << COM1A0);  // Salidas en estado bajo en la coincidencia de comparación
	TCCR1A |= (1 << WGM11) | (0 << WGM10);  // Fast PWM TOP ICR1
	TCCR1B = (1 << WGM13) | (1 << WGM12);  // Fast PWM TOP ICR1
	
	// Configuración del prescalador del Timer/Counter 1 (prescaler 8)
	TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10);  // Prescaler 8
}

// Función para escribir en el canal A del PWM
void servo_writeA(float adc_Value){
	// Mapeo del valor del ADC al rango adecuado para controlar el servo
	OCR1A = map(adc_Value, 0, 1024, 1000, 4800);
}

// Función para escribir en el canal B del PWM
void servo_writeB(float adc_Value){
	// Mapeo del valor del ADC al rango adecuado para controlar el servo
	OCR1B = map(adc_Value, 0, 1024, 1000, 4800);
}

// Función para mapear un valor de entrada a un rango de salida
float map(float x, float in_min, float in_max, float out_min, float out_max){
	return ((x - in_min) * (out_max - out_min) / (in_max - in_min)) + out_min;
}