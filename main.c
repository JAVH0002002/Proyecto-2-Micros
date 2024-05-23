#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include "PWM.h"
#include "PWM0.h"

#define F_CPU 16000000

#define EEPROM_ADDR_SERVO1 0x03 //  direcci�n EEPROM para los datos del bot�n A
#define EEPROM_ADDR_SERVO2 0x07
#define EEPROM_ADDR_SERVO3 0x0B
#define EEPROM_ADDR_SERVO4 0x0F

#define EEPROM_ADDR_SERVO1_B 0x13 //  direcci�n EEPROM para los datos del bot�n B
#define EEPROM_ADDR_SERVO2_B 0x17
#define EEPROM_ADDR_SERVO3_B 0x1B
#define EEPROM_ADDR_SERVO4_B 0x1F

#define EEPROM_ADDR_SERVO1_C 0x23 //  direcci�n EEPROM para los datos del bot�n C
#define EEPROM_ADDR_SERVO2_C 0x27
#define EEPROM_ADDR_SERVO3_C 0x2B
#define EEPROM_ADDR_SERVO4_C 0x2F

#define EEPROM_ADDR_SERVO1_D 0x33 //  direcci�n EEPROM para los datos del bot�n D
#define EEPROM_ADDR_SERVO2_D 0x37
#define EEPROM_ADDR_SERVO3_D 0x3B
#define EEPROM_ADDR_SERVO4_D 0x3F

float ValorADC1 = 0;
float ValorADC2 = 0;
float ValorADC3 = 0;
float ValorADC4 = 0;
float dutyCycle = 0;
float dutyCycle2 = 0;

void ADC_init(void);
uint16_t adcRead(uint8_t);

void saveServoPositions(void);	//  funci�n para el bot�n A
void saveServoPositionsB(void); //  funci�n para el bot�n B
void saveServoPositionsC(void); //  funci�n para el bot�n C
void saveServoPositionsD(void); //  funci�n para el bot�n D
void loadServoPositions(void);  //  funci�n para cargar los datos del bot�n A
void loadServoPositionsB(void); //  funci�n para cargar los datos del bot�n B
void loadServoPositionsC(void); //  funci�n para cargar los datos del bot�n C
void loadServoPositionsD(void); //  funci�n para cargar los datos del bot�n D

int main(void)
{
	DDRD = 0xFF; // Configura el puerto D como salida
	DDRD &= ~(1 << PD2) & ~(1 << PD4); // Configura PD2 y PD4 como entrada (botones)
	PORTD |= (1 << PD2) | (1 << PD4); // Habilita la resistencia de pull-up en PD2 y PD4

	DDRB &= ~(1 << PB0) & ~(1 << PB3) & ~(1 << PB4) & ~(1 << PB5); // Configura PB0, PB3, PB4 y PB5 como entrada (botones)
	PORTB |= (1 << PB0) | (1 << PB3) | (1 << PB4) | (1 << PB5); // Habilita la resistencia de pull-up en PB0, PB3, PB4 y PB5

	ADC_init();
	PWM_init();
	PWM0_init();

	uint8_t boton_presionado = 0;

	while (1)
	{
		// Verificar si el bot�n est� presionado
		if (!(PIND & (1 << PD2))) // Si el bot�n PD2 est� presionado
		{
			_delay_ms(20); // Debouncing
			if (!(PIND & (1 << PD2))) // Verifica nuevamente si el bot�n sigue presionado
			{
				boton_presionado = (boton_presionado + 1) % 3; // Cambia el estado del bot�n
				while (!(PIND & (1 << PD2))); // Espera a que el bot�n se libere
			}
		}

		switch (boton_presionado)
		{
			
			case 0:
			// MODO MANUAL
			ValorADC1 = adcRead(0);
			servo_writeA(ValorADC1);
			_delay_ms(10);

			ValorADC2 = adcRead(1);
			servo_writeB(ValorADC2);
			_delay_ms(10);

			ValorADC3 = adcRead(2);
			dutyCycle = map(ValorADC3, 0, 1023, 0, 100);

			ValorADC4 = adcRead(3);
			dutyCycle2 = map(ValorADC4, 0, 1023, 25, 100);

			PWM0_dcb(dutyCycle, NO_INVERTING);
			PWM0_dca(dutyCycle2, NO_INVERTING);

			PORTD |= (1 << PD3); // Enciende el LED en PD3
			
			PORTD &= ~(1 << PD7); // Apaga el LED en PD7
			
			// Guardar posiciones de los servos
			if (!(PIND & (1 << PD4))) // Si el bot�n PD4 est� presionado
			{
				_delay_ms(20); // Debouncing
				if (!(PIND & (1 << PD4))) // Verifica nuevamente si el bot�n sigue presionado
				{
					if (!(PINB & (1 << PB0))) saveServoPositions();
					else if (!(PINB & (1 << PB3))) saveServoPositionsB();
					else if (!(PINB & (1 << PB4))) saveServoPositionsC();
					else if (!(PINB & (1 << PB5))) saveServoPositionsD();
					while (!(PIND & (1 << PD4))); // Espera a que el bot�n se libere
				}
			}
			break;
			case 1:
			// MODO EEPROM
			PORTD &= ~(1 << PD3); // Apaga el LED en PD3
			PORTD &= ~(1 << PD7); // Apaga el LED en PD7
			

			if (!(PINB & (1 << PB0))) // Si el bot�n PB0 est� presionado
			{
				_delay_ms(20); // Debouncing
				if (!(PINB & (1 << PB0))) // Verifica nuevamente si el bot�n sigue presionado
				{
					loadServoPositions();
					while (!(PINB & (1 << PB0))); // Espera a que el bot�n se libere
				}
			}
			else if (!(PINB & (1 << PB3)))
			{
				_delay_ms(20); // Debouncing
				if (!(PINB & (1 << PB3))) // Verifica nuevamente si el bot�n sigue presionado
				{
					loadServoPositionsB();
					while (!(PINB & (1 << PB3))); // Espera a que el bot�n se libere
				}
			}
			else if (!(PINB & (1 << PB4)))
			{
				_delay_ms(20); // Debouncing
				if (!(PINB & (1 << PB4))) // Verifica nuevamente si el bot�n sigue presionado
				{
					loadServoPositionsC();
					while (!(PINB & (1 << PB4))); // Espera a que el bot�n se libere
				}
			}
			else if (!(PINB & (1 << PB5)))
			{
				_delay_ms(20); // Debouncing
				if (!(PINB & (1 << PB5))) // Verifica nuevamente si el bot�n sigue presionado
				{
					loadServoPositionsD();
					while (!(PINB & (1 << PB5))); // Espera a que el bot�n se libere
				}
			}
			break;
			case 2:
			// MODO UART
			PORTD |= (1 << PD7); // Enciende el LED en PD3
			break;
		}
	}
}

void ADC_init(void)
{
	ADMUX |= (1 << REFS0); // Referencia de voltaje en VCC
	ADMUX &= ~(1 << REFS1);
	ADMUX &= ~(1 << ADLAR); // Resoluci�n de 10 bits
	ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Preescalador 128 para 125KHz
	ADCSRA |= (1 << ADEN); // Habilitaci�n del ADC
}

uint16_t adcRead(uint8_t canal)
{
	ADMUX = (ADMUX & 0xF0) | canal; // Selecci�n del canal
	ADCSRA |= (1 << ADSC);          // Inicia la conversi�n
	while ((ADCSRA) & (1 << ADSC)); // Espera hasta que la conversi�n finalice
	return (ADC);
}

void saveServoPositions(void)
{
	eeprom_write_float((float *)EEPROM_ADDR_SERVO1, ValorADC1);
	eeprom_write_float((float *)EEPROM_ADDR_SERVO2, ValorADC2);
	eeprom_write_float((float *)EEPROM_ADDR_SERVO3, ValorADC3);
	eeprom_write_float((float *)EEPROM_ADDR_SERVO4, ValorADC4);
}

void saveServoPositionsB(void)
{
	eeprom_write_float((float *)EEPROM_ADDR_SERVO1_B, ValorADC1);
	eeprom_write_float((float *)EEPROM_ADDR_SERVO2_B, ValorADC2);
	eeprom_write_float((float *)EEPROM_ADDR_SERVO3_B, ValorADC3);
	eeprom_write_float((float *)EEPROM_ADDR_SERVO4_B, ValorADC4);
}

void saveServoPositionsC(void)
{
	eeprom_write_float((float *)EEPROM_ADDR_SERVO1_C, ValorADC1);
	eeprom_write_float((float *)EEPROM_ADDR_SERVO2_C, ValorADC2);
	eeprom_write_float((float *)EEPROM_ADDR_SERVO3_C, ValorADC3);
	eeprom_write_float((float *)EEPROM_ADDR_SERVO4_C, ValorADC4);
}

void saveServoPositionsD(void)
{
	eeprom_write_float((float *)EEPROM_ADDR_SERVO1_D, ValorADC1);
	eeprom_write_float((float *)EEPROM_ADDR_SERVO2_D, ValorADC2);
	eeprom_write_float((float *)EEPROM_ADDR_SERVO3_D, ValorADC3);
	eeprom_write_float((float *)EEPROM_ADDR_SERVO4_D, ValorADC4);
}

void loadServoPositions(void)
{
	ValorADC1 = eeprom_read_float((float *)EEPROM_ADDR_SERVO1);
	ValorADC2 = eeprom_read_float((float *)EEPROM_ADDR_SERVO2);
	ValorADC3 = eeprom_read_float((float *)EEPROM_ADDR_SERVO3);
	ValorADC4 = eeprom_read_float((float *)EEPROM_ADDR_SERVO4);

	servo_writeA(ValorADC1);
	_delay_ms(10);
	servo_writeB(ValorADC2);
	_delay_ms(10);
	dutyCycle = map(ValorADC3, 0, 1023, 0, 100);
	dutyCycle2 = map(ValorADC4, 0, 1023, 25, 100);
	PWM0_dcb(dutyCycle, NO_INVERTING);
	PWM0_dca(dutyCycle2, NO_INVERTING);
}

void loadServoPositionsB(void)
{
	ValorADC1 = eeprom_read_float((float *)EEPROM_ADDR_SERVO1_B);
	ValorADC2 = eeprom_read_float((float *)EEPROM_ADDR_SERVO2_B);
	ValorADC3 = eeprom_read_float((float *)EEPROM_ADDR_SERVO3_B);
	ValorADC4 = eeprom_read_float((float *)EEPROM_ADDR_SERVO4_B);

	servo_writeA(ValorADC1);
	_delay_ms(10);
	servo_writeB(ValorADC2);
	_delay_ms(10);
	dutyCycle = map(ValorADC3, 0, 1023, 0, 100);
	dutyCycle2 = map(ValorADC4, 0, 1023, 25, 100);
	PWM0_dcb(dutyCycle, NO_INVERTING);
	PWM0_dca(dutyCycle2, NO_INVERTING);
}

void loadServoPositionsC(void)
{
	ValorADC1 = eeprom_read_float((float *)EEPROM_ADDR_SERVO1_C);
	ValorADC2 = eeprom_read_float((float *)EEPROM_ADDR_SERVO2_C);
	ValorADC3 = eeprom_read_float((float *)EEPROM_ADDR_SERVO3_C);
	ValorADC4 = eeprom_read_float((float *)EEPROM_ADDR_SERVO4_C);

	servo_writeA(ValorADC1);
	_delay_ms(10);
	servo_writeB(ValorADC2);
	_delay_ms(10);
	dutyCycle = map(ValorADC3, 0, 1023, 0, 100);
	dutyCycle2 = map(ValorADC4, 0, 1023, 25, 100);
	PWM0_dcb(dutyCycle, NO_INVERTING);
	PWM0_dca(dutyCycle2, NO_INVERTING);
}

void loadServoPositionsD(void)
{
	ValorADC1 = eeprom_read_float((float *)EEPROM_ADDR_SERVO1_D);
	ValorADC2 = eeprom_read_float((float *)EEPROM_ADDR_SERVO2_D);
	ValorADC3 = eeprom_read_float((float *)EEPROM_ADDR_SERVO3_D);
	ValorADC4 = eeprom_read_float((float *)EEPROM_ADDR_SERVO4_D);

	servo_writeA(ValorADC1);
	_delay_ms(10);
	servo_writeB(ValorADC2);
	_delay_ms(10);
	dutyCycle = map(ValorADC3, 0, 1023, 0, 100);
	dutyCycle2 = map(ValorADC4, 0, 1023, 25, 100);
	PWM0_dcb(dutyCycle, NO_INVERTING);
	PWM0_dca(dutyCycle2, NO_INVERTING);
}