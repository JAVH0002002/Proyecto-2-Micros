#include "PWM0.h"  // Inclusi�n del archivo de cabecera PWM0.h

// Funci�n para inicializar el PWM0
void PWM0_init(void)
{
	// Modo Fast PWM
	TCCR0B &= ~(1<<WGM02); // Configuraci�n del modo de onda: Fast PWM (WGM02=0)
	TCCR0A |= (1<<WGM01);  // WGM01=1
	TCCR0A |= (1<<WGM00);  // WGM00=1
	
	// Prescalador 64
	TCCR0B &= ~(1<<CS02);  // CS02=0
	TCCR0B |= (1<<CS01);   // CS01=1
	TCCR0B |= (1<<CS00);   // CS00=1
}

// Funci�n para establecer el ciclo de trabajo del PWM0 en la salida A
void PWM0_dca(uint8_t dc, uint8_t modo)
{
	if(modo == 1)
	{
		TCCR0A |= (1<<COM0A1);  // Configura la salida A para no inversi�n
		TCCR0A &= ~(1<<COM0A0);
	}
	else
	{
		TCCR0A |= (1<<COM0A1);  // Configura la salida A para inversi�n
		TCCR0A |= (1<<COM0A0);
	}
	OCR0A = (dc * 255) / 100;  // Establece el valor de comparaci�n para el ciclo de trabajo
}

// Funci�n para establecer el ciclo de trabajo del PWM0 en la salida B
void PWM0_dcb(uint8_t dc, uint8_t modo)
{
	if(modo == 1)
	{
		TCCR0A |= (1<<COM0B1);  // Configura la salida B para no inversi�n
		TCCR0A &= ~(1<<COM0B0);
	}
	else
	{
		TCCR0A |= (1<<COM0B1);  // Configura la salida B para inversi�n
		TCCR0A |= (1<<COM0B0);
	}
	OCR0B = (dc * 254) ;  // Establece el valor de comparaci�n para el ciclo de trabajo
	
}