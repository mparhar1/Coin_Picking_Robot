#include <XC.h>
#include <sys/attribs.h>
#include <stdio.h>
#include <stdlib.h>
 
// Configuration Bits (somehow XC32 takes care of this)
#pragma config FNOSC = FRCPLL       // Internal Fast RC oscillator (8 MHz) w/ PLL
#pragma config FPLLIDIV = DIV_2     // Divide FRC before PLL (now 4 MHz)
#pragma config FPLLMUL = MUL_20     // PLL Multiply (now 80 MHz)
#pragma config FPLLODIV = DIV_2     // Divide After PLL (now 40 MHz) 
#pragma config FWDTEN = OFF         // Watchdog Timer Disabled
#pragma config FPBDIV = DIV_1       // PBCLK = SYCLK
#pragma config FSOSCEN = OFF        // Turn off secondary oscillator on A4 and B4

// Defines
#define SYSCLK 40000000L
#define FREQ 100000L // We need the ISR for timer 1 every 10 us
#define Baud2BRG(desired_baud)( (SYSCLK / (16*desired_baud))-1)

volatile int ISR_pwm1=150, ISR_pwm2=150, ISR_cnt=0;

// The Interrupt Service Routine for timer 1 is used to generate one or more standard
// hobby servo signals.  The servo signal has a fixed period of 20ms and a pulse width
// between 0.6ms and 2.4ms.
void __ISR(_TIMER_1_VECTOR, IPL5SOFT) Timer1_Handler(void)
{
	IFS0CLR=_IFS0_T1IF_MASK; // Clear timer 1 interrupt flag, bit 4 of IFS0

	ISR_cnt++;
	if(ISR_cnt==ISR_pwm1)
	{
		LATAbits.LATA3 = 0;
	}
	if(ISR_cnt==ISR_pwm2)
	{
		LATBbits.LATB4 = 0;
	}
	if(ISR_cnt>=2000)
	{
		ISR_cnt=0; // 2000 * 10us=20ms
		LATAbits.LATA3 = 1;
		LATBbits.LATB4 = 1;
	}
}

void SetupTimer1 (void)
{
	// Explanation here: https://www.youtube.com/watch?v=bu6TTZHnMPY
	__builtin_disable_interrupts();
	PR1 =(SYSCLK/FREQ)-1; // since SYSCLK/FREQ = PS*(PR1+1)
	TMR1 = 0;
	T1CONbits.TCKPS = 0; // 3=1:256 prescale value, 2=1:64 prescale value, 1=1:8 prescale value, 0=1:1 prescale value
	T1CONbits.TCS = 0; // Clock source
	T1CONbits.ON = 1;
	IPC1bits.T1IP = 5;
	IPC1bits.T1IS = 0;
	IFS0bits.T1IF = 0;
	IEC0bits.T1IE = 1;
	
	INTCONbits.MVEC = 1; //Int multi-vector
	__builtin_enable_interrupts();
}

// Use the core timer to wait for 1 ms.
void wait_1ms(void)
{
    unsigned int ui;
    _CP0_SET_COUNT(0); // resets the core timer count

    // get the core timer count
    while ( _CP0_GET_COUNT() < (SYSCLK/(2*1000)) );
}

void waitms(int len)
{
	while(len--) wait_1ms();
}

#define PIN_PERIOD (PORTB&(1<<5))

// GetPeriod() seems to work fine for frequencies between 200Hz and 700kHz.
long int GetPeriod (int n)
{
	int i;
	unsigned int saved_TCNT1a, saved_TCNT1b;
	
    _CP0_SET_COUNT(0); // resets the core timer count
	while (PIN_PERIOD!=0) // Wait for square wave to be 0
	{
		if(_CP0_GET_COUNT() > (SYSCLK/4)) return 0;
	}

    _CP0_SET_COUNT(0); // resets the core timer count
	while (PIN_PERIOD==0) // Wait for square wave to be 1
	{
		if(_CP0_GET_COUNT() > (SYSCLK/4)) return 0;
	}
	
    _CP0_SET_COUNT(0); // resets the core timer count
	for(i=0; i<n; i++) // Measure the time of 'n' periods
	{
		while (PIN_PERIOD!=0) // Wait for square wave to be 0
		{
			if(_CP0_GET_COUNT() > (SYSCLK/4)) return 0;
		}
		while (PIN_PERIOD==0) // Wait for square wave to be 1
		{
			if(_CP0_GET_COUNT() > (SYSCLK/4)) return 0;
		}
	}

	return  _CP0_GET_COUNT();
}
 
void UART2Configure(int baud_rate)
{
    // Peripheral Pin Select
    U2RXRbits.U2RXR = 4;    //SET RX to RB8
    RPB9Rbits.RPB9R = 2;    //SET RB9 to TX

    U2MODE = 0;         // disable autobaud, TX and RX enabled only, 8N1, idle=HIGH
    U2STA = 0x1400;     // enable TX and RX
    U2BRG = Baud2BRG(baud_rate); // U2BRG = (FPb / (16*baud)) - 1
    
    U2MODESET = 0x8000;     // enable UART2
}

void uart_puts(char * s)
{
	while(*s)
	{
		putchar(*s);
		s++;
	}
}

char HexDigit[]="0123456789ABCDEF";
void PrintNumber(long int val, int Base, int digits)
{ 
	int j;
	#define NBITS 32
	char buff[NBITS+1];
	buff[NBITS]=0;

	j=NBITS-1;
	while ( (val>0) | (digits>0) )
	{
		buff[j--]=HexDigit[val%Base];
		val/=Base;
		if(digits!=0) digits--;
	}
	uart_puts(&buff[j+1]);
}

// Good information about ADC in PIC32 found here:
// http://umassamherstm5.org/tech-tutorials/pic32-tutorials/pic32mx220-tutorials/adc
void ADCConf(void)
{
    AD1CON1CLR = 0x8000;    // disable ADC before configuration
    AD1CON1 = 0x00E0;       // internal counter ends sampling and starts conversion (auto-convert), manual sample
    AD1CON2 = 0;            // AD1CON2<15:13> set voltage reference to pins AVSS/AVDD
    AD1CON3 = 0x0f01;       // TAD = 4*TPB, acquisition time = 15*TAD 
    AD1CON1SET=0x8000;      // Enable ADC
}

int ADCRead(char analogPIN)
{
    AD1CHS = analogPIN << 16;    // AD1CHS<16:19> controls which analog pin goes to the ADC
 
    AD1CON1bits.SAMP = 1;        // Begin sampling
    while(AD1CON1bits.SAMP);     // wait until acquisition is done
    while(!AD1CON1bits.DONE);    // wait until conversion done
 
    return ADC1BUF0;             // result stored in ADC1BUF0
}

void ConfigurePins(void)
{
    // Configure pins as analog inputs
    ANSELBbits.ANSB2 = 1;   // set RB2 (AN4, pin 6 of DIP28) as analog pin
    TRISBbits.TRISB2 = 1;   // set RB2 as an input
    ANSELBbits.ANSB3 = 1;   // set RB3 (AN5, pin 7 of DIP28) as analog pin
    TRISBbits.TRISB3 = 1;   // set RB3 as an input
    
	// Configure digital input pin to measure signal period
	ANSELB &= ~(1<<5); // Set RB5 as a digital I/O (pin 14 of DIP28)
    TRISB |= (1<<5);   // configure pin RB5 as input
    CNPUB |= (1<<5);   // Enable pull-up resistor for RB5
    
    // Configure output pins
	TRISAbits.TRISA0 = 0; // pin  2 of DIP28
	TRISAbits.TRISA1 = 0; // pin  3 of DIP28
	TRISBbits.TRISB0 = 0; // pin  4 of DIP28
	TRISBbits.TRISB1 = 0; // pin  5 of DIP28
	TRISAbits.TRISA2 = 0; // pin  9 of DIP28
	TRISAbits.TRISA3 = 0; // pin 10 of DIP28
	TRISBbits.TRISB4 = 0; // pin 11 of DIP28
	INTCONbits.MVEC = 1;
}

// In order to keep this as nimble as possible, avoid
// using floating point or printf() on any of its forms!
void main(void)
{
	volatile unsigned long t=0;
    int adcval;
    long int v;
	unsigned long int count, f;
	unsigned char LED_toggle=0;

	/* Constants for the Voltage of the Perimeter Detectors */
	int per_detected_1 = ;
	int per_detected_2 = 0.100;

	/* Variables to Store Measured Voltages & Periods */
	int adcval4;
	int adcval5;
	int count_Period;

	/* Constant for the Frequency of the Metal Detector */
	int coin_detected = 60250;

	/* Counter for the # of Coins Picked Up */
	count_Coins = 0;

	CFGCON = 0;
  
    UART2Configure(115200);  // Configure UART2 for a baud rate of 115200
    ConfigurePins();
    SetupTimer1();
  
    ADCConf(); // Configure ADC
    
    waitms(500); // Give PuTTY time to start

	while(1) {
		/* Initialize Pins & Direct Robot to Drive Straight */
		LATAbits.LATA0 = 1; /* Left Side Forward */
		LATAbits.LATA1 = 0; /* Left Side Reverse */
		LATBbits.LATB0 = 1; /* Right Side Forward */
		LATBbits.LATB1 = 0; /* Right Side Reverse */
		LATAbits.LATA2 = 0; /* Electro-Magnet Off */

		/* Get Voltages of Perimeter Detectors - Pins 4 & 5 */
		adcval4 = ADCRead(4);
		adcval5 = ADCRead(5);

		/* Check if Perimeter is Detected */
		if ( adcval4 > per_detected_1 || adcval5 > per_detected_2 ) {
			/* Reverse a Little Bit to Give Robot Room from Perimeter */
			LATAbits.LATA0 = 0; /* Left Side Forward */
			LATAbits.LATA1 = 1; /* Left Side Reverse */
			LATBbits.LATB0 = 0; /* Right Side Forward */
			LATBbits.LATB1 = 1; /* Right Side Reverse */

			/* Allow Robot to Reverse for 25ms */
			waitms(25);

			/* Pivot the Robot's Direction */
			LATAbits.LATA0 = 1; /* Left Side Forward */
			LATAbits.LATA1 = 0; /* Left Side Reverse */
			LATBbits.LATB0 = 0; /* Right Side Forward */
			LATBbits.LATB1 = 1; /* Right Side Reverse */

			/* Allow Robot to Pivot for 100ms */
			waitms(100);
		}

		/* Get Frequency of Metal Detector */
		count_Period = GetPeriod(100);
		if(count>0)
		{
			f=((SYSCLK/2L)*100L)/count;
		}

		/* Check if Metal Coin is Detected */
		if ( f > coin_detected) {
			/* Make the Robot Stop in Position */
			LATAbits.LATA0 = 1; /* Left Side Forward */
			LATAbits.LATA1 = 0; /* Left Side Reverse */
			LATBbits.LATB0 = 0; /* Right Side Forward */
			LATBbits.LATB1 = 1; /* Right Side Reverse */

			/* Bend the Servo-Arm Down into a Position to Pick-Up Coin */


			/* Turn On the Magnet to Pull the Coin */
			LATAbits.LATA2 = 1; /* Electro-Magnet On */

			/* Bend the Servo-Arm into a Position to Drop-Off the Coin */


			/* Turn Off the Magnet to Release the Coin */
			LATAbits.LATA2 = 0; /* Electro-Magnet Off */

			/* Coin Pick-Up Successful - Increment the Counter for # of Coins Collected */
			count_Coins++;

			/* If All 20 Coins are Picked Up, Terminate Program */
			if ( count_Coins == 20 ) {
				exit();
			}
		}
	}

	// uart_puts("\x1b[2J\x1b[1;1H"); // Clear screen using ANSI escape sequence.
	// uart_puts("\r\nPIC32 multi I/O example.\r\n");
	// uart_puts("Measures the voltage at channels 4 and 5 (pins 6 and 7 of DIP28 package)\r\n");
	// uart_puts("Measures period on RB5 (pin 14 of DIP28 package)\r\n");
	// uart_puts("Toggles RA0, RA1, RB0, RB1, RA2 (pins 2, 3, 4, 5, 9, of DIP28 package)\r\n");
	// uart_puts("Generates Servo PWM signals at RA3, RB4 (pins 10, 11 of DIP28 package)\r\n\r\n");
	// while(1)
	// {
    // 	adcval = ADCRead(4); // note that we call pin AN4 (RB2) by it's analog number
	// 	uart_puts("ADC[4]=0x");
	// 	PrintNumber(adcval, 16, 3);
	// 	uart_puts(", V=");
	// 	v=(adcval*3290L)/1023L; // 3.290 is VDD
	// 	PrintNumber(v/1000, 10, 1);
	// 	uart_puts(".");
	// 	PrintNumber(v%1000, 10, 3);
	// 	uart_puts("V ");

	// 	adcval=ADCRead(5);
	// 	uart_puts("ADC[5]=0x");
	// 	PrintNumber(adcval, 16, 3);
	// 	uart_puts(", V=");
	// 	v=(adcval*3290L)/1023L; // 3.290 is VDD
	// 	PrintNumber(v/1000, 10, 1);
	// 	uart_puts(".");
	// 	PrintNumber(v%1000, 10, 3);
	// 	uart_puts("V ");

	// 	count=GetPeriod(100);
	// 	if(count>0)
	// 	{
	// 		f=((SYSCLK/2L)*100L)/count;
	// 		uart_puts("f=");
	// 		PrintNumber(f, 10, 7);
	// 		uart_puts("Hz, count=");
	// 		PrintNumber(count, 10, 6);
	// 		uart_puts("          \r");
	// 	}
	// 	else
	// 	{
	// 		uart_puts("NO SIGNAL                     \r");
	// 	}

	// 	// Now toggle the pins on/off to see if they are working.
	// 	// First turn all off:
	// 	LATAbits.LATA0 = 0;	
	// 	LATAbits.LATA1 = 0;			
	// 	LATBbits.LATB0 = 0;			
	// 	LATBbits.LATB1 = 0;		
	// 	LATAbits.LATA2 = 0;			

	// 	// Now turn on one of the outputs per loop cycle to check
	// 	switch (LED_toggle++)
	// 	{
	// 		case 0:
	// 			LATAbits.LATA0 = 1;
	// 			break;
	// 		case 1:
	// 			LATAbits.LATA1 = 1;
	// 			break;
	// 		case 2:
	// 			LATBbits.LATB0 = 1;
	// 			break;
	// 		case 3:
	// 			LATBbits.LATB1 = 1;
	// 			break;
	// 		case 4:
	// 			LATAbits.LATA2 = 1;
	// 			break;
	// 		default:
	// 			break;
	// 	}
	// 	if(LED_toggle>4) LED_toggle=0;

	// 	waitms(200);
	// }
}
