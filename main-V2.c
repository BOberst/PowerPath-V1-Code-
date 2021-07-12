// before this test is started the coulomb counter needs to be written the value 0xEFFF and it accounts for a discharge capacity of 2800mAh.

#define F_CPU 1000000UL
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
unsigned char CMSB = 0;				// Accumulated charge MSB
unsigned char CLSB = 0;				// Accumulated charge LSB
unsigned char MSBVOLT = 0;			// Voltage MSB
unsigned char LSBVOLT = 0;			// Voltage LSB
unsigned char MSBAMP = 0;			// Current MSB
unsigned char LSBAMP = 0;			// Current LSB
unsigned char C1LSB = 0;			// Middle Cell Voltage LSB
unsigned char C1MSB = 0;			// Middle Cell Voltage MSB
unsigned short VOLT = 0;			// Volt 16 bit
unsigned short CHARGE = 0;			// Charge 16 bit
unsigned short AMP = 0;				// Current 16 bit
unsigned short C1VOLT = 0;			// Cell 1 Voltage 


// double check this I2C communication again

void i2c_init(void)
{
	TWSR = 0X00;
	TWBR = 18;     // 50kHz Master Frequency
	TWCR = 0X44;   // enables the TWI module and enables acknowledge bit
}

void i2c_start(void)
{
	TWCR = (1 << TWINT)  | (1 << TWSTA) | (1 << TWEN);
	while ((TWCR & (1 << TWINT)) == 0);
}

void i2c_write(unsigned char data)
{
	TWDR = data;
	TWCR = (1<< TWINT)|(1<<TWEN);
	while  ((TWCR & (1 <<TWINT)) == 0);
}

unsigned char i2c_read(unsigned char isLast)
{
	
	TWCR = (1<< TWINT)|(1<<TWEN);
	while ((TWCR & (1 <<TWINT)) ==0);
	return TWDR;
}

void i2c_stop()
{
	TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
}

int main (void)
{
	
	TCCR1A = 0X00;				//NORMAL MODE PRESCALING 1024
	TCCR1B = 0X04;				//NORMAL MODE PRESCALING 1024
	TCNT1H = (-65530)>>8;		// THE HIGH BYTE
	TCNT1L = (-62530)&0XFF;		//OVERFLOW AFTER THIS MANY CLOCK CYCLES
	TIMSK1 = (1<<TOIE1);
	EICRA = 0x03; 		// enables a rising edge interrupt on INT0/PD2
	EIMSK = (1<<INT0);	// enable external interrupt
	sei ();						//ENABLE INTERRUPTS
	



	
	// transmit 0xE0 to control register address 0x01  --> automatic mode with 256 prescaler

	i2c_init();					// INITALIZE TWI FOR MASTER MODE
	i2c_start();				// TRANSMIT START CONDITION
	i2c_write(0b11001000);		//TRANSMIT SLA 1100100 + R(0)
	i2c_write(0x01);			//write to the control register
	i2c_write(0xD8);			//send D8 which makes automatic measurements and sets prescaler to 64.
	i2c_stop();
	
	DDRD = 0x24;				// bidirectional switch and interrupt 0 
	PORTD = 0xFF;
	DDRB = 0xFF;
	PORTB = 0x00;
	DDRC = 0xFF;				// LEDS can be turned on and off
	ADCSRA = 0x87;				//enable ADC and select clock /128
	ADMUX = 0x47;				// AVCC (5V) is the voltage reference, ADC7, right justified

	while(1)

	{
		
		// this is for the MSB of the accumulated charge register
		
		_delay_ms(2000);
		_delay_ms(2000);
		_delay_ms(2000);
		_delay_ms(2000);


		i2c_init();			// INITALIZE TWI FOR MASTER MODE
		i2c_start();			// TRANSMIT START CONDITION
		i2c_write(0b11001000);		//TRANSMIT SLA 1100100 + R(0)
		i2c_write(0x02);		//TRANSMIT WHAT ADDRESS TO READ FROM
		i2c_start();			// TRANSMIT START CONDITION
		i2c_write(0b11001001);		//TRANSMIT SLA 1100100 + R(1)
		CMSB = i2c_read(1);		// READ ONLY ONE BYTE OF DATA
		i2c_stop();
	
		//this is for the LSB of the accumulated charge register
	
		i2c_init();			// INITALIZE TWI FOR MASTER MODE
		i2c_start();			// TRANSMIT START CONDITION
		i2c_write(0b11001000);		//TRANSMIT SLA 1100100 + R(0)
		i2c_write(0x03);		//TRANSMIT WHAT ADDRESS TO READ FROM
		i2c_start();			// TRANSMIT START CONDITION
		i2c_write(0b11001001);		//TRANSMIT SLA 1100100 + R(1)
		CLSB = i2c_read(1);		// READ ONLY ONE BYTE OF DATA
		i2c_stop();
		
		// combine the CLSB and the CMSB
		
		CHARGE = CLSB | (0xFF00 & (CMSB << 8));	// adds on MSB
		
		
		// this is for the MSB of the voltage register
		
		i2c_init();			// INITALIZE TWI FOR MASTER MODE
		i2c_start();			// TRANSMIT START CONDITION
		i2c_write(0b11001000);		//TRANSMIT SLA 1100100 + R(0)
		i2c_write(0x08);		    //TRANSMIT WHAT ADDRESS TO READ FROM
		i2c_start();			   // TRANSMIT START CONDITION
		i2c_write(0b11001001);		//TRANSMIT SLA 1100100 + R(1)
		MSBVOLT = i2c_read(1);		// READ ONLY ONE BYTE OF DATA
		i2c_stop();
		
		// this is for the LSB of the voltage register
		
		i2c_init();			// INITALIZE TWI FOR MASTER MODE
		i2c_start();			// TRANSMIT START CONDITION
		i2c_write(0b11001000);		//TRANSMIT SLA 1100100 + R(0)
		i2c_write(0x09);		    //TRANSMIT WHAT ADDRESS TO READ FROM
		i2c_start();			   // TRANSMIT START CONDITION
		i2c_write(0b11001001);		//TRANSMIT SLA 1100100 + R(1)
		LSBVOLT = i2c_read(1);		// READ ONLY ONE BYTE OF DATA
		i2c_stop();

		// combine the CLSB and CMSB

		VOLT = (LSBVOLT | (0xFF00 & (MSBVOLT << 8)));	// adds on MSB
		
		//ADC conversion of the first cell's voltage put it in C1Volt for later use 
		
		ADCSRA |= (1<<ADSC);		  // start conversion
		while((ADCSRA&(1<ADIF)) ==0);	 // wait for conversion to finish
		ADCSRA |= (1<<ADIF);		// set the ADC interrupt flag
		C1LSB = ADCL;			// give the low byte to PortD
		C1MSB = ADCH;			 // give the high byte to PortD
		
		C1VOLT = (C1LSB | (0xFF00 & (C1MSB << 8)));	// adds on MSB
		
		// this is the voltage for the middle cell 


		if (PORTD && 0x20 == 0x20)				// charge and discharge are turned on
		{
			//do nothing if this is the case
		}
		// check if it is greater than 20% charged and needs to discharge
		else if (PORTD == 0x00 && CMSB >= 0x7C)			// PB2 == 0 && 20%<CMSB
		{
				PORTD = 0x20;
				_delay_ms(200);
			
				i2c_init();			// INITALIZE TWI FOR MASTER MODE
				i2c_start();			// TRANSMIT START CONDITION
				i2c_write(0b11001000);		//TRANSMIT SLA 1100100 + R(0)
				i2c_write(0x0E);		    //TRANSMIT WHAT ADDRESS TO READ FROM
				i2c_start();			   // TRANSMIT START CONDITION
				i2c_write(0b11001001);		//TRANSMIT SLA 1100100 + R(1)
				MSBAMP = i2c_read(1);		// READ ONLY ONE BYTE OF DATA
				i2c_stop();
			
				i2c_init();			// INITALIZE TWI FOR MASTER MODE
				i2c_start();			// TRANSMIT START CONDITION
				i2c_write(0b11001000);		//TRANSMIT SLA 1100100 + R(0)
				i2c_write(0x0F);		    //TRANSMIT WHAT ADDRESS TO READ FROM
				i2c_start();			   // TRANSMIT START CONDITION
				i2c_write(0b11001001);		//TRANSMIT SLA 1100100 + R(1)
				LSBAMP = i2c_read(1);		// READ ONLY ONE BYTE OF DATA
				i2c_stop();
			
			
				// combine the CLSB and CMSB

				AMP = LSBAMP | (0xFF00 & (MSBAMP << 8));	// adds on MSB
			
				// this code recognizes the current is discharge
			
			
						if (AMP < 0x7800) // if the current is discharge
						{
						PORTD = 0x04;
						}
						else
						{
						PORTD = 0x00;
						}
			

		}
		// check if it is less than 60% charged and needs to charge
		else if (PORTD == 0x00 &&  CMSB > 0xC9)		// PB2 == 0 && 0 CMSB < 75%
		{
				PORTD = 0x20;
				_delay_ms(200);
					
				i2c_init();			// INITALIZE TWI FOR MASTER MODE
				i2c_start();			// TRANSMIT START CONDITION
				i2c_write(0b11001000);		//TRANSMIT SLA 1100100 + R(0)
				i2c_write(0x0E);		    //TRANSMIT WHAT ADDRESS TO READ FROM
				i2c_start();			   // TRANSMIT START CONDITION
				i2c_write(0b11001001);		//TRANSMIT SLA 1100100 + R(1)
				MSBAMP = i2c_read(1);		// READ ONLY ONE BYTE OF DATA
				i2c_stop();
					
				i2c_init();			// INITALIZE TWI FOR MASTER MODE
				i2c_start();			// TRANSMIT START CONDITION
				i2c_write(0b11001000);		//TRANSMIT SLA 1100100 + R(0)
				i2c_write(0x0F);		    //TRANSMIT WHAT ADDRESS TO READ FROM
				i2c_start();			   // TRANSMIT START CONDITION
				i2c_write(0b11001001);		//TRANSMIT SLA 1100100 + R(1)
				LSBAMP = i2c_read(1);		// READ ONLY ONE BYTE OF DATA
				i2c_stop();
					
					
				// combine the CLSB and CMSB

				AMP = LSBAMP | (0xFF00 & (MSBAMP << 8));	// adds on MSB
					
					
				// this code recognizes the current is charge
						if (AMP > 0x8500) // if the current is charge
						{
							PORTD = 0x04;
						}
						else
						{
							PORTD = 0x00;
						}
			
		}
		else
		{

			// if the other conditions are not met do nothing
		}

	}
	
}	// this last parenthesis ends int main 








// this is the interrupt service routine that cuts of charging if it is out of limits


ISR (TIMER1_OVF_vect)				//ISR FOR TIMER1 COMPARE MATCH A
{
	TCNT1H = (-65530)>>8;     // THE HIGH BYTE
	TCNT1L = (-62530)&0XFF;	  //OVERFLOW AFTER THIS MANY CLOCK CYCLES
	if (CHARGE < 0x5500 || VOLT < 0x3B00 || C1VOLT < 0x01F4)			// 0x3B --> 2.73 Volt/Cell    --> 5.45 volts/battery
	{
		PORTD = 0x00;
	}

	else if (CHARGE > 0xEFFF || VOLT > 0x5900 || C1VOLT > 0x035C )		// 0x5A --> 4.14 Volt/Cell   --> 8.27 volt/battery
	{
		PORTD = 0x00;
	}
	else
	{
		
		PORTD = 0x20;
	}
	
	// this is for state of charge

	if (CHARGE < 0xF000 & CHARGE >= 0xEA00)        // 61440 to 59904   -->33000 units in total 11.78 units per mAh
	{
		PORTC = 0x0F;					   // 4 LEDS are lit
	}
	else if (CHARGE < 0xEA00 & CHARGE >= 0xC900)   // 59904 to 52992
	{
		PORTC = 0X07;					   // 3 LEDS are lit
	}
	else if (CHARGE < 0xC900 & CHARGE >= 0xA200)   // 44800 to 36608
	{
		PORTC = 0x03;					   // 2 LEDS are lit
	}
	else if (CHARGE < 0xA200 & CHARGE >= 0x7C00)   // 36608 to 31232
	{
		PORTC = 0X01;						// 1 LED is lit
	}
	else if (CHARGE < 0x5900)					// 31232
	{
		PORTC = 0x00;					// 0 LEDs are lit
	}
	else
	{
		PORTC = 0x0A;					// fault condition is detected --> output alternating LEDS
	}


	// this ends state of charge
}


ISR (INT0_vect)
{
// measure current 

	i2c_init();			// INITALIZE TWI FOR MASTER MODE
	i2c_start();			// TRANSMIT START CONDITION
	i2c_write(0b11001000);		//TRANSMIT SLA 1100100 + R(0)
	i2c_write(0x0E);		    //TRANSMIT WHAT ADDRESS TO READ FROM
	i2c_start();			   // TRANSMIT START CONDITION
	i2c_write(0b11001001);		//TRANSMIT SLA 1100100 + R(1)
	MSBAMP = i2c_read(1);		// READ ONLY ONE BYTE OF DATA
	i2c_stop();
	
	i2c_init();			// INITALIZE TWI FOR MASTER MODE
	i2c_start();			// TRANSMIT START CONDITION
	i2c_write(0b11001000);		//TRANSMIT SLA 1100100 + R(0)
	i2c_write(0x0F);		    //TRANSMIT WHAT ADDRESS TO READ FROM
	i2c_start();			   // TRANSMIT START CONDITION
	i2c_write(0b11001001);		//TRANSMIT SLA 1100100 + R(1)
	LSBAMP = i2c_read(1);		// READ ONLY ONE BYTE OF DATA
	i2c_stop();
	
	
	// combine the CLSB and CMSB

	AMP = LSBAMP | (0xFF00 & (MSBAMP << 8));	// adds on MSB
	
	// add in an iff statement that shuts off bidirectional switch here 
	
}