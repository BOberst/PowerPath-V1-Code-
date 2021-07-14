/*
Beginning of Battery Management System Code for 2 Cell BMS with 3500mAh cells
Need to initialize to internal clock of 1 MhZ. 
Declare Standard Library and interrupt
This contains a while loop that goes forever with two interrupts
Coded by Brendan Oberst working for MaxPower Inc 
*/

#define F_CPU 1000000UL
#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>

/*
Declare global variables that will be used throughout the while loop
LSB and MSB are (datatype char) least and most significant bits from the LTC29433 registers
They are then combined into (datatype short)
*/ 


// Global Variable Declaration
// Characters are 8 bits 1 byte
unsigned char CMSB = 0;				// Accumulated charge MSB
unsigned char CLSB = 0;				// Accumulated charge LSB
unsigned char MSBVOLT = 0;			// Voltage MSB
unsigned char LSBVOLT = 0;			// Voltage LSB
unsigned char MSBAMP = 0;			// Current MSB
unsigned char LSBAMP = 0;			// Current LSB
unsigned char C1LSB = 0;			// Middle Cell Voltage LSB
unsigned char C1MSB = 0;			// Middle Cell Voltage MSB

//SHORTS are 16 bits 2 bytes
unsigned short VOLT = 0;			// Volt 16 bit
unsigned short CHARGE = 0;			// Charge 16 bit
unsigned short AMP = 0;				// Current 16 bit
unsigned short C1VOLT = 0;			// Cell 1 Voltage
unsigned short C1VOLTL = 0; 	     // Long Cell 1 Voltage 16 bits


// this is I2C communication functions
// this is used so many times throughout the code to read registers throughout the LTC 2943

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

// This is the end of the I2C code. 
// the main void loop happens at this point. 

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
	



	
	// transmit 0xD8 to control register address 0x01  --> automatic mode with 256 prescaler

	i2c_init();					// INITALIZE TWI FOR MASTER MODE
	i2c_start();				// TRANSMIT START CONDITION
	i2c_write(0b11001000);		//TRANSMIT SLA 1100100 + R(0)
	i2c_write(0x01);			//write to the control register
	i2c_write(0xD8);			//send D8 which makes automatic measurements and sets prescaler to 64.
	i2c_stop();
	
	
	//enable ports
	
	DDRD = 0xFF;				// bidirectional switch and interrupt 0
	PORTD = 0x20;				// make port D all 1s
	DDRC = 0xFF;				// LEDS can be turned on and off
	
	// before entering while loop need to turn ADC on for middle cell voltage reading at ADC 7 
	
	ADCSRA = 0x87;				//enable ADC and select clock /128
	ADMUX = 0x47;				// AVCC (5V) is the voltage reference, ADC7, right justified


// enter while loop
	while(1)

	{
		
		// the delay is to keep the MCU focused on nothing for 8 seconds so it does not spend all its time checking the gates. 
		
		_delay_ms(2000);
		_delay_ms(2000);
		_delay_ms(2000);
		_delay_ms(2000);
	
	//
	//
	//
	//
	// beginning of the charge most signficant bit and least significant bit read operation 
	// CMSB = 0x02 == accumulated charge most significant bit 

		i2c_init();			// INITALIZE TWI FOR MASTER MODE
		i2c_start();			// TRANSMIT START CONDITION
		i2c_write(0b11001000);		//TRANSMIT SLA 1100100 + R(0)
		i2c_write(0x02);		//TRANSMIT WHAT ADDRESS TO READ FROM
		i2c_start();			// TRANSMIT START CONDITION
		i2c_write(0b11001001);		//TRANSMIT SLA 1100100 + R(1)
		CMSB = i2c_read(1);		// READ ONLY ONE BYTE OF DATA
		i2c_stop();
		
	// CLSB = 0x03 == accumulated charge least significant bit 
		
		i2c_init();			// INITALIZE TWI FOR MASTER MODE
		i2c_start();			// TRANSMIT START CONDITION
		i2c_write(0b11001000);		//TRANSMIT SLA 1100100 + R(0)
		i2c_write(0x03);		//TRANSMIT WHAT ADDRESS TO READ FROM
		i2c_start();			// TRANSMIT START CONDITION
		i2c_write(0b11001001);		//TRANSMIT SLA 1100100 + R(1)
		CLSB = i2c_read(1);		// READ ONLY ONE BYTE OF DATA
		i2c_stop();
		
	// CHARGE == CMSB + CLSB
		
		CHARGE = CLSB | (0xFF00 & (CMSB << 8));	// combine them into one 2 byte variable called charge 
	
	// end of the charge most significant bit and least significant bit read operation 
		
	//
	//
	//	
	//
	
	
	
	//
	//
	//
	//
	// beginning of the voltage register most significant bit and least significant bit read operation 	
	// MSBVOLT = 0x08 == voltage register most significant bit 
		
		i2c_init();			// INITALIZE TWI FOR MASTER MODE
		i2c_start();			// TRANSMIT START CONDITION
		i2c_write(0b11001000);		//TRANSMIT SLA 1100100 + R(0)
		i2c_write(0x08);		    //TRANSMIT WHAT ADDRESS TO READ FROM
		i2c_start();			   // TRANSMIT START CONDITION
		i2c_write(0b11001001);		//TRANSMIT SLA 1100100 + R(1)
		MSBVOLT = i2c_read(1);		// READ ONLY ONE BYTE OF DATA
		i2c_stop();
		
	// LSBVOLT = 0x09 == voltage register with least significant bit 
		
		i2c_init();			// INITALIZE TWI FOR MASTER MODE
		i2c_start();			// TRANSMIT START CONDITION
		i2c_write(0b11001000);		//TRANSMIT SLA 1100100 + R(0)
		i2c_write(0x09);		    //TRANSMIT WHAT ADDRESS TO READ FROM
		i2c_start();			   // TRANSMIT START CONDITION
		i2c_write(0b11001001);		//TRANSMIT SLA 1100100 + R(1)
		LSBVOLT = i2c_read(1);		// READ ONLY ONE BYTE OF DATA
		i2c_stop();

	// VOLTAGE == MSBVOLT + LSBVOLT

		VOLT = (LSBVOLT | (0xFF00 & (MSBVOLT << 8)));	// adds on MSB
	
	// end of volt register most significant bit and least significant bit read operation
	//
	//
	//
	//
	
		
		
	//
	//
	//	
	//ADC conversion of the first cell's voltage put it in C1Volt for later use
		
		ADCSRA |= (1<<ADSC);		  // start conversion
		while((ADCSRA&(1<ADIF)) ==0);	 // wait for conversion to finish
		ADCSRA |= (1<<ADIF);		// set the ADC interrupt flag
		C1LSB = ADCL;			// give the low byte to PortD
		C1MSB = ADCH;			 // give the high byte to PortD
		
		C1VOLT = (C1LSB | (0xFF00 & (C1MSB << 8)));	// adds on MSB
		
		// this is the voltage for the middle cell
		//
		//
		//

		 // main if and ifelse statement starts here
		 // checks if bidirection switch is on
		 // checks it it is off
		if (PORTD && 0x20 == 0x20)				// charge and discharge are turned on
		{
			
				_delay_ms(500);			// delay for half a second 
				//
				//
				//
				//
				// MSBAMP = 0x0E == current register MSB
				i2c_init();			// INITALIZE TWI FOR MASTER MODE
				i2c_start();			// TRANSMIT START CONDITION
				i2c_write(0b11001000);		//TRANSMIT SLA 1100100 + R(0)
				i2c_write(0x0E);		    //TRANSMIT WHAT ADDRESS TO READ FROM
				i2c_start();			   // TRANSMIT START CONDITION
				i2c_write(0b11001001);		//TRANSMIT SLA 1100100 + R(1)
				MSBAMP = i2c_read(1);		// READ ONLY ONE BYTE OF DATA
				i2c_stop();
				
				// LSBAMP = 0x0F -- current register LSB
				
				i2c_init();			// INITALIZE TWI FOR MASTER MODE
				i2c_start();			// TRANSMIT START CONDITION
				i2c_write(0b11001000);		//TRANSMIT SLA 1100100 + R(0)
				i2c_write(0x0F);		    //TRANSMIT WHAT ADDRESS TO READ FROM
				i2c_start();			   // TRANSMIT START CONDITION
				i2c_write(0b11001001);		//TRANSMIT SLA 1100100 + R(1)
				LSBAMP = i2c_read(1);		// READ ONLY ONE BYTE OF DATA
				i2c_stop();
				
				
				// AMP == MSBAMP + LSBAMP

				AMP = LSBAMP | (0xFF00 & (MSBAMP << 8));	// adds on MSB
				
				//end of charge most significant bit and least significant bit read operation
				
				//
				//
				//
				//
			
			
			
					if (AMP > 0x8010) // if charging current is greater than 10mA
					{
				
						//
						//
						//
						//
						// beginning of the voltage register most significant bit and least significant bit read operation
						// MSBVOLT = 0x08 == voltage register most significant bit
						
						i2c_init();			// INITALIZE TWI FOR MASTER MODE
						i2c_start();			// TRANSMIT START CONDITION
						i2c_write(0b11001000);		//TRANSMIT SLA 1100100 + R(0)
						i2c_write(0x08);		    //TRANSMIT WHAT ADDRESS TO READ FROM
						i2c_start();			   // TRANSMIT START CONDITION
						i2c_write(0b11001001);		//TRANSMIT SLA 1100100 + R(1)
						MSBVOLT = i2c_read(1);		// READ ONLY ONE BYTE OF DATA
						i2c_stop();
						
						// LSBVOLT = 0x09 == voltage register with least significant bit
						
						i2c_init();			// INITALIZE TWI FOR MASTER MODE
						i2c_start();			// TRANSMIT START CONDITION
						i2c_write(0b11001000);		//TRANSMIT SLA 1100100 + R(0)
						i2c_write(0x09);		    //TRANSMIT WHAT ADDRESS TO READ FROM
						i2c_start();			   // TRANSMIT START CONDITION
						i2c_write(0b11001001);		//TRANSMIT SLA 1100100 + R(1)
						LSBVOLT = i2c_read(1);		// READ ONLY ONE BYTE OF DATA
						i2c_stop();

						// VOLTAGE == MSBVOLT + LSBVOLT

						VOLT = (LSBVOLT | (0xFF00 & (MSBVOLT << 8)));	// adds on MSB
						
						// end of volt register most significant bit and least significant bit read operation
						//
						//
						//
						//
						
						ADCSRA |= (1<<ADSC);		  // start conversion
						while((ADCSRA&(1<ADIF)) ==0);	 // wait for conversion to finish
						ADCSRA |= (1<<ADIF);		// set the ADC interrupt flag
						C1LSB = ADCL;			// give the low byte to PortD
						C1MSB = ADCH;			 // give the high byte to PortD
				
						C1VOLT = (C1LSB | (0xFF00 & (C1MSB << 8)));	// adds on MSB
				
						C1VOLTL = C1VOLT << 6; // this shifts the C1 Volt left 6 digits which makes it 16 bits
						C1VOLTL = C1VOLT | 0x003F; // this & it with 6 0s.
				
						// C1VOLTL and VOLT are now both 16 bit shorts
				
						C1VOLTL = C1VOLTL * (5/23.6); // it is now scaled to the same scale of 23.6 volts
						VOLT = VOLT - C1VOLTL; // subtract the bottom cell to get only the top cell voltage
						// now VOLT and C1VOLTL should be apples to apples
						// need to calculate accuracy of two data types here
						// it should be OK for now.
						if (VOLT > C1VOLTL)
						{
						
									if (VOLT - C1VOLTL > 166)
									{
									// turn on the gate to lower cell 2 voltage PD6
									PORTD |= (1<<PD6)
									// delay for ten seconds 
									_delay_ms(2000);
									_delay_ms(2000);
									_delay_ms(2000);
									_delay_ms(2000);
									_delay_ms(2000);
									//end of delay
									PORTD |= ~(1<<PD6)
									_delay_ms(2000);
									}
									else
									{
									// do not balance
									}
						}
						else if (C1VOLTL > VOLT)
						{
									if (C1VOLTL - VOLT > 166)
									{
										// turn on the gate to lower cell 1 voltage PD7
										PORTD |= (1<<PD7)
										// delay for ten seconds
										_delay_ms(2000);
										_delay_ms(2000);
										_delay_ms(2000);
										_delay_ms(2000);
										_delay_ms(2000);
										//end of delay
										PORTD |= ~(1<<PD7)
										_delay_ms(2000);
									}
									else
									{
										// do not balance
									}
						}
						else if (C1VOLT = VOLT)
						{
						// do not balance
						}
						else
						{
						// do not balance
						}					
		// check if it is greater than 20% charged and needs to discharge
		else if (PORTD == 0x00 && CMSB >= 0x7C)			// PB2 == 0 && 20%<CMSB
		{
			PORTD = 0x20;
			_delay_ms(200);
			
			
			//
			//
			//
			//
			// MSBAMP = 0x0E == current register MSB
			i2c_init();			// INITALIZE TWI FOR MASTER MODE
			i2c_start();			// TRANSMIT START CONDITION
			i2c_write(0b11001000);		//TRANSMIT SLA 1100100 + R(0)
			i2c_write(0x0E);		    //TRANSMIT WHAT ADDRESS TO READ FROM
			i2c_start();			   // TRANSMIT START CONDITION
			i2c_write(0b11001001);		//TRANSMIT SLA 1100100 + R(1)
			MSBAMP = i2c_read(1);		// READ ONLY ONE BYTE OF DATA
			i2c_stop();
			
			// LSBAMP = 0x0F -- current register LSB
			
			i2c_init();			// INITALIZE TWI FOR MASTER MODE
			i2c_start();			// TRANSMIT START CONDITION
			i2c_write(0b11001000);		//TRANSMIT SLA 1100100 + R(0)
			i2c_write(0x0F);		    //TRANSMIT WHAT ADDRESS TO READ FROM
			i2c_start();			   // TRANSMIT START CONDITION
			i2c_write(0b11001001);		//TRANSMIT SLA 1100100 + R(1)
			LSBAMP = i2c_read(1);		// READ ONLY ONE BYTE OF DATA
			i2c_stop();
			
			
			// AMP == MSBAMP + LSBAMP

			AMP = LSBAMP | (0xFF00 & (MSBAMP << 8));	// adds on MSB
			
			//end of charge most significant bit and least significant bit read operation 
			
			//
			//
			//
			//			
						if (AMP < 0x7800) // if the current is discharge
							{
								PORTD = 0x04;	// keep the gate open
							}
						else
							{
								PORTD = 0x00;	// turn the gate off
							}
			

		}
		// check if it is less than 60% charged and needs to charge
		else if (PORTD == 0x00 &&  CMSB > 0xC9)		// PB2 == 0 && 0 CMSB < 75%
		{
			PORTD = 0x20;
			_delay_ms(200);
			
			//
			//
			//
			//
			// MSBAMP = 0x0E == current register MSB
			i2c_init();			// INITALIZE TWI FOR MASTER MODE
			i2c_start();			// TRANSMIT START CONDITION
			i2c_write(0b11001000);		//TRANSMIT SLA 1100100 + R(0)
			i2c_write(0x0E);		    //TRANSMIT WHAT ADDRESS TO READ FROM
			i2c_start();			   // TRANSMIT START CONDITION
			i2c_write(0b11001001);		//TRANSMIT SLA 1100100 + R(1)
			MSBAMP = i2c_read(1);		// READ ONLY ONE BYTE OF DATA
			i2c_stop();
		
			// LSBAMP = 0x0F -- current register LSB
		
			i2c_init();			// INITALIZE TWI FOR MASTER MODE
			i2c_start();			// TRANSMIT START CONDITION
			i2c_write(0b11001000);		//TRANSMIT SLA 1100100 + R(0)
			i2c_write(0x0F);		    //TRANSMIT WHAT ADDRESS TO READ FROM
			i2c_start();			   // TRANSMIT START CONDITION
			i2c_write(0b11001001);		//TRANSMIT SLA 1100100 + R(1)
			LSBAMP = i2c_read(1);		// READ ONLY ONE BYTE OF DATA
			i2c_stop();
		
		
			// AMP == MSBAMP + LSBAMP

			AMP = LSBAMP | (0xFF00 & (MSBAMP << 8));	// adds on MSB
		
			//end of charge most significant bit and least significant bit read operation
		
			//
			//
			//
			//
				
			
						if (AMP > 0x8500) // if the current is charge
						{
							PORTD = 0x04;	// keep the gate on
						}
						else
						{
							PORTD = 0x00;	// turn the gate off
						}
		else
		{
		}
						// if the other conditions are not met do nothing
		}

	}					// this ends the while loop
	
}						// this last parenthesis ends int main

//
//
//
//
//
//
//
//
// This is the ISR section of the code
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



// this is the ISR that is activated if the pin of the coulomb counter goes low
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
	
	// add in an if statement that shuts off bidirectional switch here
	PORTD = 0x00;			This will shut down ALCC pin 
	if (AMP < 0x6978 || AMP > 0x9664) // if the current is greater than 3.5 amps 
	{
		PORTB = 0x0A;	// If short circuit happened display through the LEDS to let user know. 
		_delay_ms(1000);
		_delay_ms(1000);
		PORTB = 0x00;
	}
	else
	{
		// do nothing 
	}
	
	
	
}