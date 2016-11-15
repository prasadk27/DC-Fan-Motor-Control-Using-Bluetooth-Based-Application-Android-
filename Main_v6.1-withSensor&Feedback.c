#include <msp430.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <in430.h>
#include <intrinsics.h>
//#include <semaphore.h>

#define PERIOD050USEC 52                    // assuming ~1.048576 MHz DCO clock

#define L0DUTYCYCLE   0                     // L0 PWM duty cycle (fan off)
#define L1DUTYCYCLE   30                    // L1 PWM duty cycle
#define L2DUTYCYCLE   45                    // L2 PWM duty cycle
#define L3DUTYCYCLE   60                    // L3 PWM duty cycle
#define L4DUTYCYCLE   80                    // L4 PWM duty cycle
#define L5DUTYCYCLE   100                   // L5 PWM duty cycle (fan full speed)

//#define PERIOD5MSEC    5243                 // 5 msec worth of SMCLK tics @ DCO = 1.048576 MHz

typedef enum Status {
  ALARM, OK
} Status;

typedef enum FanLevel {
  FANSLEVEL0 = 0, FANSLEVEL1 = 1, FANSLEVEL2 = 2, FANSLEVEL3 = 3, FANSLEVEL4 = 4, FANSLEVEL5 = 5
} FanLevel;

// Port1 Pin Configuration
#define ACTIVITY    BIT4
//#define SENSOR	BIT2

// Port2 Pin Configuration
#define TACH        BIT2
#define PWM         BIT0

#define MAXLEVELSETTINGS  6

#define PULSESPERREVOLUTION  2              // # of frequency generator pulses per 1 revolution of the fan

FanLevel level = FANSLEVEL0;                // Cooling level
Status status = OK;                         // Alarm status of system whenever smoke is detected
volatile unsigned int flagTask0 = 0;		// Flags indicating status of tasks
volatile unsigned int flagTask1 = 0;
volatile unsigned int flagTask2 = 0;
volatile unsigned int flagScheduler = 0;
volatile unsigned char Rx_Data = 0;						// Store received data
char *Tx_Data;											// Store data to be transmitted, might need to be volatile
//char *string2;											// Store complete string along with RPM value
volatile unsigned int tachCount = 0;                 // Count revolutions of fan
volatile unsigned int tachRPM = 0;                   // Stores RPM of fan

volatile uint8_t  task_id; 					// has the current running task

unsigned int count = 0;
unsigned int count0 = 0;
unsigned int count1 = 0;					// For debug
unsigned int count2 = 0;
unsigned int count3 = 0;
unsigned int countADC = 0;

											// Duty cycles for each level
unsigned int dutyCycles[ MAXLEVELSETTINGS] =
  { L0DUTYCYCLE, L1DUTYCYCLE, L2DUTYCYCLE,
    L3DUTYCYCLE, L4DUTYCYCLE, L5DUTYCYCLE};

void Sys_init();
void ADC_init(void);
void USCI_A0_init(void);
void PWM_init(void);
void task0(void);
void task1(void);
void task2(void);
void scheduler(void);

int main(void) {
	Sys_init();								// Initialize the system
	ADC_init();								// Initialize ADC
	USCI_A0_init();							// Initialize UART mode
	PWM_init();								// Initialize PWM
	flagTask1 = 1;							// Set initial level and speed
	Tx_Data = "Level 0 ";
	//flagTask2 = 1;							// Update readings and display
	ADC12CTL0 |= ADC12SC;                   		// Start sampling/conversion
	task_id = 0;
	task0();

	_EINT();								// Enable Global Interrupts

	while(1)
	{
		__bis_SR_register(LPM0_bits + GIE);       // Enter LPM0, interrupts enabled
		__no_operation();							// No Operation
	}
}

void scheduler(void)
{
		if (flagScheduler == 1)						// Check if scheduler invoked/flagged
		{
			if (task_id == 0)
			{
				task0();
			}
			else if (task_id == 1)
			{
				task1();
			}
			else if (task_id == 2)
			{
				task2();
			}

			flagScheduler = 0;
			count3++;									// for debug
		}
}

void Sys_init()
{
	volatile unsigned int  i;                 // Volatile var used for delay
	WDTCTL = WDTPW | WDTHOLD;				  // Stop watchdog timer
	//FLL_CTL0 |= XCAP18PF;                     // Set load cap for xtal
	//for(i = 0; i < 10000; i++);               // Delay for FLL to lock
	P3SEL = BIT3 + BIT4;                      // P3.3,4 = USCI_A0 TXD/RXD
	P1SEL &= ~(ACTIVITY);                     // Set I/O function for alarm
	P1DIR |= (ACTIVITY);                      // Set output direction for alarm
	P1OUT &= ~(ACTIVITY);						// Set as low
	//P1SEL &= ~(SENSOR);								// Set I/O function for sensor input
	//P1DIR &= ~(SENSOR);								// Set input direction for sensor input
	P2SEL &= ~(TACH);                         // Set I/O function
	P2DIR &= ~(TACH);                         // Set input direction
	P2IE  |= (TACH);                          // Enable Port 2 interrupt
	P2IES |= (TACH);                          // Generate ints falling edge
	TA0CCTL0 = CCIE;						// CCR0 interrupt enabled
	TA0CCR0 = 12500;						// Count value to generate timer intervals every 100ms
	TA0CTL = TASSEL_2 + MC_1 + ID_3;         // SMCLK, upmode, clear TAR
	//WDTCTL = WDT_MDLY_32;                   // Disable the Watchdog for 32ms
	WDTCTL = WDTPW + WDTSSEL1 + WDTTMSEL + WDTCNTCL + WDTIS_7;		// Set WDT in interval timer mode, ~1.95 ms intervals
	SFRIE1 |= WDTIE;                          // Enable Watchdog interrupt (modify for system security)
}

void USCI_A0_init(void)
{
	UCA0CTL1 |= UCSWRST;                      // **Put state machine in reset**
	UCA0CTL1 |= UCSSEL_2;                     // SMCLK
	UCA0BR0 = 109;                            // 1MHz 9600 - Check for compatibility with HC06
	UCA0BR1 = 0;                              // 1MHz 9600
	UCA0MCTL |= UCBRS_1 + UCBRF_0;            // Modulation UCBRSx=1, UCBRFx=0
	UCA0CTL1 &= ~UCSWRST;                     // **Initialize USCI state machine**
	UCA0IE |= UCRXIE;                         // Enable USCI_A0 RX interrupt
}

void PWM_init(void)
{
  P2DIR |= PWM;                             // Configure output direction
  P2SEL |= PWM;                             // Select peripheral option
  TA1CCR0 = PERIOD050USEC;                  // Set up the PWM Period
  TA1CTL = TASSEL_2 + MC_1 + TACLR;         // Use SMCLK, count up mode, clear TAR
  TA1CCTL1 = OUTMOD_1;                      // Keep fan(s) off initially
}

void ADC_init(void)
{
	ADC12CTL0 = ADC12SHT02 + ADC12ON;         // Sampling time, ADC12 on
	ADC12CTL1 = ADC12SHP;                     // Use sampling timer
	ADC12IE = 0x01;                           // Enable interrupt
	ADC12CTL0 |= ADC12ENC;					  // Enable Conversion
	P6SEL |= 0x01;                            // P6.0 ADC option select for sensor data
}

void task0(void)
{
	//while(1)											// Task0 runs in an infinite loop
		if (flagTask0 == 1)								// If Smoke detected, sound alarm
			{
				flagTask0 = 0;
				status = ALARM;
				P1OUT |= ACTIVITY;                   	   // Set ALARM LED
				//flagTask0 = 1;
				level = FANSLEVEL0;						// may need to protect shared variable
				flagTask1 = 1;           				// Event registered for task 1 to take place
				Tx_Data = "X";							// Store emergency character to be transmitted
				//flagTask2 = 1;							// Event registered for task 2 to take place
				// Disable further interrupts here
			}
		count0++;										// for debug
	//}
}

void task1(void)
{
	flagTask1 = 0;							// Clear flag for Task1

	float dutyCycle;
	                                            // Convert duty cycle to a %
	dutyCycle = ((float)(dutyCycles[level]))/100;

	TA1CCR1 = (dutyCycle * PERIOD050USEC);  // Update the compare register
	TA1CCTL1 = OUTMOD_7;                    // Generate PWM via out mode 7

	flagTask2 = 1;							// Event registered for task 2 to take place

	count1++;								// for debug
}

void task2(void)
{
	flagTask2 = 0;							// Clear flag for Task2

	char *string1 = Tx_Data;					// Store data to be transmitted
	//strTX = malloc(strlen(string1)+strlen(string2)+1);
	//strcpy(strTX, string1);						// Copy string1 in strTX
	//strcat(strTX, string2);						// Add string2
	UCA0IE |= UCTXIE;               		// Enable USCI_A0 TX interrupt
	while (*string1 != '\0')
	{
		while (!(UCA0IFG&UCTXIFG));     	// USCI_A0 TX buffer ready?
		UCA0TXBUF = *string1;				// TX string, character by character
		string1++;
	}
	UCA0IE &= ~UCTXIE;						// Disable USCI_A0 TX interrupt

	count2++;								// for debug
}

// PORT2 Interrupt Service Routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT2_VECTOR))) Port_2 (void)
#else
#error Compiler not supported!
#endif
{
  if (P2IFG & TACH) {                           // Tachometer pulse detected ?
    tachCount++;                                // Increment tach counter
    P2IFG &= ~TACH;                             // Clear interrupt flag
  }
}

/*
// TIMER0_A1 Interrupt Service Routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A1_VECTOR
__interrupt void TIMER0_A1_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A1_VECTOR))) TIMER0_A1_ISR (void)
#else
#error Compiler not supported!
#endif
{
  LPM0_EXIT;                                    // Exit out of LPM0
  TA0CCTL1 &= ~CCIFG;                           // Clear TA0CCR1 interrupt flag
}
*/

// Timer0 A0 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER0_A0_VECTOR
__interrupt void TIMER0_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) TIMER0_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
	//P2IE &= ~(TACH);                          // Disable tachometer interrupt
	//_NOP();                                   // Wait 1 instruction cycle
	                                            // Convert pulse count to RPM
	tachRPM = (tachCount/PULSESPERREVOLUTION)*10*60;		// Would be multiplied by the time interval between consecutive refreshTach readings
	//P2IE |= (TACH);                           // Re-enable tachometer interrupt
	tachCount = 0;                            // Reset the pulse counter
	/*int displayRPM = tachRPM;						// Store current Tach RPM
	int size = log10(displayRPM) + 1;				// Compute size in terms of digits
	string2 = malloc(size);
	snprintf(string2, size, "%d", displayRPM);		// Convert RPM value to a string for transmission
	UCA0IE |= UCTXIE;               		// Enable USCI_A0 TX interrupt
	while (*string2 != '\0')
	{
		while (!(UCA0IFG&UCTXIFG));     	// USCI_A0 TX buffer ready?
		UCA0TXBUF = *string2;				// TX string, character by character
		string2++;
	}
	UCA0IE &= ~UCTXIE;						// Disable USCI_A0 TX interrupt
	*/
}

// WDT Interrupt Service Routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=WDT_VECTOR
__interrupt void WDT_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(WDT_VECTOR))) WDT_ISR (void)
#else
#error Compiler not supported!
#endif
{
	if (flagScheduler == 0)
	{
		// Scheduling algorithm
		if (flagTask0 == 1 && (flagTask1 == 0 || flagTask1 == 1) && (flagTask2 == 0 || flagTask2 == 1))
		{
			task_id = 0;
		}
		else if (flagTask1 == 1 && flagTask2 == 1 && flagTask0 == 0)
		{
			task_id = 1;
		}
		else if (flagTask1 == 1 && flagTask2 == 0 && flagTask0 == 0)
		{
			task_id = 1;
		}
		else if (flagTask1 == 0 && flagTask2 == 1 && flagTask0 == 0)
		{
			task_id = 2;
		}
		else
		{
			task_id = 0;
		}

		flagScheduler = 1;
		count++;									// for debug
		scheduler();								// Invoke scheduler
		LPM0_EXIT;                                    // Exit out of LPM0
	}
}

// USCI_A0 ISR
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(USCI_A0_VECTOR))) USCI_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
	//if (IFG2 & UCA0RXIFG) {
		Rx_Data = UCA0RXBUF;									// Store received data in variable
		switch (Rx_Data)
		  	  {
		  	  	  case 'A':									// Level 1
		  	  		level = FANSLEVEL1;						// may need to protect shared variable
		  	  		flagScheduler = 0;
		  	  		flagTask1 = 1;              			// Update PWM and speed, Event registered for task 1 to take place
		  	  		Tx_Data = "Level 1 ";
		  	  		//_EINT();								// Enable Global Interrupts
		  	  		//flagTask2 = 1;							// Event registered for task 2 to take place
		  	  		break;

		  	  	  case 'B':									// Level 2
		  	  		level = FANSLEVEL2;						// may need to protect shared variable
		  	  		flagScheduler = 0;
		  	  		flagTask1 = 1;              			// Update PWM and speed, Event registered for task 1 to take place
		  	  		Tx_Data = "Level 2 ";
		  	  		//_EINT();								// Enable Global Interrupts
		  	  		//flagTask2 = 1;							// Event registered for task 2 to take place
		  	 	  	break;

		  	  	  case 'C':									// Level 3
		  	  		level = FANSLEVEL3;						// may need to protect shared variable
		  	  		flagScheduler = 0;
		  	  		flagTask1 = 1;              			// Update PWM and speed, Event registered for task 1 to take place
		  	  		Tx_Data = "Level 3 ";
		  	  		//_EINT();								// Enable Global Interrupts
		  	  		//flagTask2 = 1;							// Event registered for task 2 to take place
			  	  	break;

		  	  	  case 'D':									// Level 4
		  	  		level = FANSLEVEL4;						// may need to protect shared variable
		  	  		flagScheduler = 0;
		  	  		flagTask1 = 1;              			// Update PWM and speed, Event registered for task 1 to take place
		  	  		Tx_Data = "Level 4 ";
		  	  		//_EINT();								// Enable Global Interrupts
		  	  		//flagTask2 = 1;							// Event registered for task 2 to take place
				  	break;

		  	  	  case 'E':									// Level 5
		  	  		level = FANSLEVEL5;						// may need to protect shared variable
		  	  		flagScheduler = 0;
		  	  		flagTask1 = 1;              			// Update PWM and speed, Event registered for task 1 to take place
		  	  		Tx_Data = "Level 5 ";
		  	  		//_EINT();								// Enable Global Interrupts
		  	  		//flagTask2 = 1;							// Event registered for task 2 to take place
		  	  		break;

		  	  	  case 'F':									// Level 0 - OFF
		  	  		level = FANSLEVEL0;						// may need to protect shared variable
		  	  		flagScheduler = 0;
		  	  		flagTask1 = 1;              			// Update PWM and speed, Event registered for task 1 to take place
		  	  		Tx_Data = "Level 0 ";
		  	  		//_EINT();								// Enable Global Interrupts
		  	  		//flagTask2 = 1;							// Event registered for task 2 to take place
		  	  		break;

		  	  	  default:
		  	  		flagScheduler = 0;
		  	  		break;
		  	  }
	//}
	//__bic_SR_register_on_exit(LPM0_bits);		// Wake-up CPU

}

//ADC12_ISR
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = ADC12_VECTOR
__interrupt void ADC12_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(ADC12_VECTOR))) ADC12_ISR (void)
#else
#error Compiler not supported!
#endif
{
  switch(__even_in_range(ADC12IV,34))
  {
  case  0: break;                           // Vector  0:  No interrupt
  case  2: break;                           // Vector  2:  ADC overflow
  case  4: break;                           // Vector  4:  ADC timing overflow
  case  6:                                  // Vector  6:  ADC12IFG0
	countADC++;
    if (ADC12MEM0 >= 0x7ff)              	// ADC12MEM = A0 > 0.5AVcc, Threshold value for firing alarm
    {
      flagTask0 = 1;
    }
    else
    {
    	ADC12CTL0 |= ADC12SC;                   		// Start sampling/conversion
    }
    //__bic_SR_register_on_exit(LPM0_bits);   // Exit active CPU
      break;
  case  8: break;                           // Vector  8:  ADC12IFG1
  case 10: break;                           // Vector 10:  ADC12IFG2
  case 12: break;                           // Vector 12:  ADC12IFG3
  case 14: break;                           // Vector 14:  ADC12IFG4
  case 16: break;                           // Vector 16:  ADC12IFG5
  case 18: break;                           // Vector 18:  ADC12IFG6
  case 20: break;                           // Vector 20:  ADC12IFG7
  case 22: break;                           // Vector 22:  ADC12IFG8
  case 24: break;                           // Vector 24:  ADC12IFG9
  case 26: break;                           // Vector 26:  ADC12IFG10
  case 28: break;                           // Vector 28:  ADC12IFG11
  case 30: break;                           // Vector 30:  ADC12IFG12
  case 32: break;                           // Vector 32:  ADC12IFG13
  case 34: break;                           // Vector 34:  ADC12IFG14
  default: break;
  }
}
