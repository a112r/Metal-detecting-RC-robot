#include <XC.h>
#include <sys/attribs.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "lcd.h"

/* Pinout for DIP28 PIC32MX130:
                                          --------
                                   MCLR -|1     28|- AVDD 
  VREF+/CVREF+/AN0/C3INC/RPA0/CTED1/RA0 -|2     27|- AVSS 
        VREF-/CVREF-/AN1/RPA1/CTED2/RA1 -|3     26|- AN9/C3INA/RPB15/SCK2/CTED6/PMCS1/RB15
   PGED1/AN2/C1IND/C2INB/C3IND/RPB0/RB0 -|4     25|- CVREFOUT/AN10/C3INB/RPB14/SCK1/CTED5/PMWR/RB14
  PGEC1/AN3/C1INC/C2INA/RPB1/CTED12/RB1 -|5     24|- AN11/RPB13/CTPLS/PMRD/RB13
   AN4/C1INB/C2IND/RPB2/SDA2/CTED13/RB2 -|6     23|- AN12/PMD0/RB12
     AN5/C1INA/C2INC/RTCC/RPB3/SCL2/RB3 -|7     22|- PGEC2/TMS/RPB11/PMD1/RB11
                                    VSS -|8     21|- PGED2/RPB10/CTED11/PMD2/RB10
                     OSC1/CLKI/RPA2/RA2 -|9     20|- VCAP
                OSC2/CLKO/RPA3/PMA0/RA3 -|10    19|- VSS
                         SOSCI/RPB4/RB4 -|11    18|- TDO/RPB9/SDA1/CTED4/PMD3/RB9
         SOSCO/RPA4/T1CK/CTED9/PMA1/RA4 -|12    17|- TCK/RPB8/SCL1/CTED10/PMD4/RB8
                                    VDD -|13    16|- TDI/RPB7/CTED3/PMD5/INT0/RB7
                    PGED3/RPB5/PMD7/RB5 -|14    15|- PGEC3/RPB6/PMD6/RB6
                                          --------
*/
 
// Configuration Bits (somehow XC32 takes care of this)
#pragma config FNOSC = FRCPLL       // Internal Fast RC oscillator (8 MHz) w/ PLL
#pragma config FPLLIDIV = DIV_2     // Divide FRC before PLL (now 4 MHz)
#pragma config FPLLMUL = MUL_20     // PLL Multiply (now 80 MHz)
#pragma config FPLLODIV = DIV_2     // Divide After PLL (now 40 MHz)
 
#pragma config FWDTEN = OFF         // Watchdog Timer Disabled
#pragma config FPBDIV = DIV_1       // PBCLK = SYCLK
#pragma config FSOSCEN = OFF

// Defines
#define SYSCLK 40000000L
#define DEF_FREQ 16000L
#define FREQ 100000L // We need the ISR for timer 2 every 10 us
#define Baud2BRG(desired_baud)( (SYSCLK / (16*desired_baud))-1)
#define Baud1BRG(desired_baud)( (SYSCLK / (16*desired_baud))-1)

volatile int ISR_pw=100, ISR_cnt=0, ISR_frc;

void __ISR(_TIMER_1_VECTOR, IPL5SOFT) Timer1_Handler(void)
{
    LATBbits.LATB6 = !LATBbits.LATB6; // Desired frequency on RB6
    IFS0CLR = _IFS0_T1IF_MASK; // Clear timer 1 interrupt flag, bit 4 of IFS0
}

void __ISR(_TIMER_2_VECTOR, IPL5SOFT) Timer2_Handler(void)
{
    IFS0CLR=_IFS0_T2IF_MASK; // Clear timer 2 interrupt flag, bit 8 of IFS0
    
    ISR_cnt++;
    if(ISR_cnt<ISR_pw)
    {
        LATBbits.LATB1 = 1;
    }
    else
    {
        LATBbits.LATB1 = 0;
    }
    if(ISR_cnt>=2000)
    {
        ISR_cnt=0; // 2000 * 10us=20ms
        ISR_frc++;
    }
}


void SetupTimer1(void)
{
    // Explanation here:
    // https://www.youtube.com/watch?v=bu6TTZHnMPY
    __builtin_disable_interrupts();
    PR1 = (SYSCLK / (DEF_FREQ * 2L)) - 1; // since SYSCLK/FREQ = PS*(PR1+1)
    TMR1 = 0;
    T1CONbits.TCKPS = 0; // Pre-scaler: 1
    T1CONbits.TCS = 0; // Clock source
    T1CONbits.ON = 1;
    IPC1bits.T1IP = 5;
    IPC1bits.T1IS = 0;
    IFS0bits.T1IF = 0;
    IEC0bits.T1IE = 1;

    INTCONbits.MVEC = 1; //Int multi-vector
    __builtin_enable_interrupts();
}

unsigned int SerialReceive(char* buffer, unsigned int max_size)
{
    unsigned int num_char = 0;

    /* Wait for and store incoming data until either a carriage return is received
     *   or the number of received characters (num_chars) exceeds max_size */
    while (num_char < max_size)
    {
        while (!U2STAbits.URXDA);   // wait until data available in RX buffer
        *buffer = U2RXREG;          // empty contents of RX buffer into *buffer pointer

        while (U2STAbits.UTXBF);    // wait while TX buffer full
        U2TXREG = *buffer;          // echo

        // insert nul character to indicate end of string
        if (*buffer == '\r')
        {
            *buffer = '\0';
            break;
        }

        buffer++;
        num_char++;
    }

    return num_char;
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


void ADCConf(void)
{
	AD1CON1CLR = 0x8000;    // disable ADC before configuration
	AD1CON1 = 0x00E0;       // internal counter ends sampling and starts conversion (auto-convert), manual sample
	AD1CON2 = 0;            // AD1CON2<15:13> set voltage reference to pins AVSS/AVDD
	AD1CON3 = 0x0f01;       // TAD = 4*TPB, acquisition time = 15*TAD 
	AD1CON1SET = 0x8000;      // Enable ADC
}

void delay_ms(int msecs)
{
    int ticks;
    ISR_frc = 0;
    ticks = msecs / 20;
    while (ISR_frc < ticks);
}


int ADCRead(char analogPIN)
{
	AD1CHS = analogPIN << 16;    // AD1CHS<16:19> controls which analog pin goes to the ADC

	AD1CON1bits.SAMP = 1;        // Begin sampling
	while (AD1CON1bits.SAMP);     // wait until acquisition is done
	while (!AD1CON1bits.DONE);    // wait until conversion done

	return ADC1BUF0;             // result stored in ADC1BUF0
}


void ConfigurePins(void)
{
	// Configure pins as analog inputs
	ANSELBbits.ANSB2 = 1;   // set RB2 (AN4, pin 6 of DIP28) as analog pin
	TRISBbits.TRISB2 = 1;   // set RB2 as an input
	//ANSELBbits.ANSB3 = 1;   // set RB3 (AN5, pin 7 of DIP28) as analog pin
	ANSELBbits.ANSB0 = 1;   // set RB0 (AN2, pin 4 of DIP28) as analog pin
	TRISBbits.TRISB0 = 1;   // set RB0 as an input

	// Configure digital input pin to measure signal period
	ANSELB &= ~(1 << 6); // Set RB6 as a digital I/O (pin 15 of DIP28)
	TRISB |= (1 << 6);   // configure pin RB6 as input
	CNPUB |= (1 << 6);   // Enable pull-up resistor for RB6

	// Configure output pins
	TRISAbits.TRISA0 = 0; // pin  2 of DIP28
	TRISAbits.TRISA1 = 0; // pin  3 of DIP28
	//TRISBbits.TRISB0 = 0; // pin  4 of DIP28
	TRISBbits.TRISB1 = 0; // pin  5 of DIP28
	TRISAbits.TRISA2 = 0; // pin  9 of DIP28
	TRISAbits.TRISA3 = 0; // pin 10 of DIP28
	TRISBbits.TRISB4 = 0; // pin 11 of DIP28
	INTCONbits.MVEC = 1;
}

// Needed to by scanf() and gets()
int _mon_getc(int canblock)
{
	char c;
	
    if (canblock)
    {
	    while( !U2STAbits.URXDA); // wait (block) until data available in RX buffer
	    c=U2RXREG;
        while( U2STAbits.UTXBF);    // wait while TX buffer full
        U2TXREG = c;          // echo
	    if(c=='\r') c='\n'; // When using PUTTY, pressing <Enter> sends '\r'.  Ctrl-J sends '\n'
		return (int)c;
    }
    else
    {
        if (U2STAbits.URXDA) // if data available in RX buffer
        {
		    c=U2RXREG;
		    if(c=='\r') c='\n';
			return (int)c;
        }
        else
        {
            return -1; // no characters to return
        }
    }
}

/////////////////////////////////////////////////////////
// UART1 functions used to communicate with the JDY40  //
/////////////////////////////////////////////////////////

// TXD1 is in pin 26
// RXD1 is in pin 24

int UART1Configure(int desired_baud)
{
	int actual_baud;

    // Peripheral Pin Select for UART1.  These are the pins that can be used for U1RX from TABLE 11-1 of '60001168J.pdf':
    // 0000 = RPA2
	// 0001 = RPB6
	// 0010 = RPA4
	// 0011 = RPB13
	// 0100 = RPB2

	// Do what the caption of FIGURE 11-2 in '60001168J.pdf' says: "For input only, PPS functionality does not have
    // priority over TRISx settings. Therefore, when configuring RPn pin for input, the corresponding bit in the
    // TRISx register must also be configured for input (set to ??."
    
    ANSELB &= ~(1<<13); // Set RB13 as a digital I/O
    TRISB |= (1<<13);   // configure pin RB13 as input
    CNPUB |= (1<<13);   // Enable pull-up resistor for RB13
    U1RXRbits.U1RXR = 3; // SET U1RX to RB13

    // These are the pins that can be used for U1TX. Check table TABLE 11-2 of '60001168J.pdf':
    // RPA0
	// RPB3
	// RPB4
	// RPB15
	// RPB7

    ANSELB &= ~(1<<15); // Set RB15 as a digital I/O
    RPB15Rbits.RPB15R = 1; // SET RB15 to U1TX
	
    U1MODE = 0;         // disable autobaud, TX and RX enabled only, 8N1, idle=HIGH
    U1STA = 0x1400;     // enable TX and RX
    U1BRG = Baud1BRG(desired_baud); // U1BRG = (FPb / (16*baud)) - 1
    // Calculate actual baud rate
    actual_baud = SYSCLK / (16 * (U1BRG+1));

    U1MODESET = 0x8000;     // enable UART1

    return actual_baud;
}

void putc1 (char c)
{
	while( U1STAbits.UTXBF);   // wait while TX buffer full
	U1TXREG = c;               // send single character to transmit buffer
}
 
int SerialTransmit1(const char *buffer)
{
    unsigned int size = strlen(buffer);
    while(size)
    {
        while( U1STAbits.UTXBF);    // wait while TX buffer full
        U1TXREG = *buffer;          // send single character to transmit buffer
        buffer++;                   // transmit next character on following loop
        size--;                     // loop until all characters sent (when size = 0)
    }
 
    while( !U1STAbits.TRMT);        // wait for last transmission to finish
 
    return 0;
}
 
unsigned int SerialReceive1(char *buffer, unsigned int max_size)
{
    unsigned int num_char = 0;
 
    while(num_char < max_size)
    {
        while( !U1STAbits.URXDA);   // wait until data available in RX buffer
        *buffer = U1RXREG;          // empty contents of RX buffer into *buffer pointer
 
        // insert nul character to indicate end of string
        if( *buffer == '\n')
        {
            *buffer = '\0';     
            break;
        }
 
        buffer++;
        num_char++;
    }
 
    return num_char;
}

void wait_100us(void)
{
    _CP0_SET_COUNT(0); // resets the core timer count

    // get the core timer count
    while ( _CP0_GET_COUNT() < ( (SYSCLK)/(2*10000) ) );
}


// Use the core timer to wait for 1 ms.
void wait_1ms(void)
{
    unsigned int ui;
    _CP0_SET_COUNT(0); // resets the core timer count

    // get the core timer count
    while ( _CP0_GET_COUNT() < (SYSCLK/(2*1000)) );
}

void delayms(int len)
{
	while(len--) wait_1ms();
}

void SendATCommand (char * s)
{
	char buff[40];
	printf("Command: %s", s);
	LATB &= ~(1<<14); // 'SET' pin of JDY40 to 0 is 'AT' mode.
	delayms(10);
	SerialTransmit1(s);
	SerialReceive1(buff, sizeof(buff)-1);
	LATB |= 1<<14; // 'SET' pin of JDY40 to 1 is normal operation mode.
	delayms(10);
	printf("Response: %s\n", buff);
}

void SetupTimer2(void)
{
    __builtin_disable_interrupts();
    PR2 = (SYSCLK / FREQ) - 1; // set period register
    TMR2 = 0; // reset timer
    T2CONbits.TCKPS = 0; // set prescaler to 1
    T2CONbits.TCS = 0; // set internal clock source (peripheral clock)
    T2CONbits.ON = 1; // turn on Timer 2
    IPC2bits.T2IP = 5; // set interrupt priority
    IPC2bits.T2IS = 0; // set subpriority
    IFS0bits.T2IF = 0; // clear interrupt flag
    IEC0bits.T2IE = 1; // enable interrupt
    INTCONbits.MVEC = 1; // enable multi-vector interrupts
    __builtin_enable_interrupts();
}

void int_to_char(int num, char result[], int start_index) {
	// Handle negative numbers
	int index = start_index;
	int i;
	if (num < 0) {
		result[index++] = '-';
		num = -num;
	}

	// Handle zero explicitly, otherwise empty string is printed
	if (num == 0) {
		result[index++] = '0';
	}

	// Convert individual digits to characters
	char temp[20]; // Temporary array to store digits
	int temp_index = 0;
	while (num != 0) {
		int digit = num % 10;
		temp[temp_index++] = '0' + digit; // Convert digit to character
		num /= 10;
	}

	// Reverse the temporary array and copy it to the result array
	for (i = temp_index - 1; i >= 0; i--) {
		result[index++] = temp[i];
	}

	// Add null terminator to indicate end of string
	result[index] = '\0';
}

void buzzer(int newF)
{
	unsigned long reload;
	
	reload = (SYSCLK / (newF * 2L))/1000 - 1;
    //SerialTransmit("\r\nFrequency set to: ");
        			//PrintNumber(SYSCLK / ((reload + 1) * 2L), 10, 1);
    T1CONbits.ON = 0;
    PR1 = reload;
    T1CONbits.ON = 1;
}

void main(void)
{
	char buff[80];
    int cont1=0, cont2=100;
    int cnt = 0;
	int joy_x, joy_y;
	int adcvalX;
	int adcvalY;
	int timeout_cnt;
	float X;
	float Y;
	volatile unsigned long t = 0;
	int frequency;
	char freqLCD[6];
	int adcval;
	long int vx, vy;
	unsigned long int count, f;
	unsigned char LED_toggle = 0;
	unsigned int rx_size;
	int newF;
	unsigned long reload;

    
	DDPCON = 0;
	CFGCON = 0;
  
    UART2Configure(115200);  // Configure UART2 for a baud rate of 115200
    UART1Configure(9600);  // Configure UART1 to communicate with JDY40 with a baud rate of 9600
 	LCD_4BIT();
	ConfigurePins();
	ADCConf(); // Configure ADC
	LCDprint("frequency(Hz):", 1, 0);

 	
	delayms(500); // Give putty time to start before we send stuff.
    printf("JDY40 test program. PIC32 behaving as Master.\r\n");
	
	//Initialize pin 5 or RB1 for square wave
	TRISBbits.TRISB6 = 0;
	LATBbits.LATB6 = 0;
	INTCONbits.MVEC = 1;
	SetupTimer1();
	SetupTimer2();
	//SetupTimer1(frequency);

	// RB14 is connected to the 'SET' pin of the JDY40.  Configure as output:
    ANSELB &= ~(1<<14); // Set RB14 as a digital I/O
    TRISB &= ~(1<<14);  // configure pin RB14 as output
	LATB |= (1<<14);    // 'SET' pin of JDY40 to 1 is normal operation mode

	// We should select an unique device ID.  The device ID can be a hex
	// number from 0x0000 to 0xFFFF.  In this case is set to 0xABBA
	SendATCommand("AT+DVID1337\r\n");  

	// To check configuration
	SendATCommand("AT+VER\r\n");
	SendATCommand("AT+BAUD\r\n");
	SendATCommand("AT+RFID\r\n");
	SendATCommand("AT+DVID\r\n");
	SendATCommand("AT+RFC\r\n");
	SendATCommand("AT+POWE\r\n");
	SendATCommand("AT+CLSS\r\n");
	
	delay_ms(500); // wait 500 ms
    
    printf("\x1b[2J\x1b[1;1H"); // Clear screen using ANSI escape sequence.
    printf("Servo signal generator for the PIC32MX130F064B.  Output is in RB6 (pin 15).\r\n");
    printf("By Jesus Calvino-Fraga (c) 2018.\r\n");
    printf("Pulse width between 60 (for 0.6ms) and 240 (for 2.4ms)\r\n");

	//SetupTimer1(4048);
	while(1)
	{
		//buzzer(1048);
	
		adcvalX = ADCRead(2); // note that we call pin AN4 (RB2) by it's analog number
		vx = (adcvalX * 3290L) / 1023L; // 3.290 is VDD
		
		if (vx <= 3290 && vx >= 2741)
		{
			joy_x = 6;
		}
		else if (vx < 2741 && vx >= 2192)
		{
			joy_x = 5;
		}
		else if (vx < 2192 && vx>1643)
		{
			joy_x = 4;
		}
		else if (vx < 481 && vx >= 0)
		{
			joy_x = 0;
		}
		else if (vx < 1443 && vx >= 962)
		{
			joy_x = 2;
		}
		else if (vx < 962 && vx >= 481)
		{
			joy_x = 1;
		}
		else if (vx <= 1643 && vx >= 1443)
		{
			joy_x = 3;
		}

		adcvalY = ADCRead(4);
		vy = (adcvalY * 3290L) / 1023L; // 3.290 is VDD
		
		if (vy <= 3290 && vy >= 2775)
		{
			joy_y = 6;
		}
		else if (vy < 2775 && vy >= 2260)
		{
			joy_y = 5;
		}

		else if (vy < 2260 && vy>1746)
		{
			joy_y = 4;
		}
		else if (vy < 515 && vy >= 0)
		{
			joy_y = 0;
		}
		else if (vy < 1546 && vy >= 1030)
		{
			joy_y = 2;
		}
		else if (vy < 1030 && vy >= 515)
		{
			joy_y = 1;
		}
		else if (vy <= 1746 && vy >= 1546)
		{
			joy_y = 3;
		}
		
		// Send a message to the slave. First send the 'attention' character which is '!':
		putc1('!');
		// Wait a bit so the slave has a chance to get ready
		delayms(10);
		// Construct a message
		//sprintf(buff, "%03d,%03d\n", cont1, cont2);
		sprintf(buff, "%d%d\n", joy_x, joy_y);
		//int_to_char(joy_x, buff, 0);
		//int_to_char(joy_y, buff, 1);
		// Send the message
		SerialTransmit1(buff);
		// Increment test counters for next message
		if(++cont1>200) cont1=0;
		if(++cont2>200) cont2=0;
		
		delayms(5);
		// Request a message from the slave
		//if(U1STAbits.URXDA) SerialReceive1(buff, sizeof(buff)-1);
		
		putc1('@');
		
		// Wait up to 10ms for the repply
		timeout_cnt=0;
		while(1)
		{
			if(U1STAbits.URXDA) break; // Something has arrived
			if(++timeout_cnt>100) break;
			wait_100us(); // 100us*100=10ms
		}
		
		if(U1STAbits.URXDA) // Something has arrived from slave
		{
			rx_size = SerialReceive1(buff, sizeof(buff)-1); // wait here until data is received
		
			freqLCD[0] = buff[0];
			freqLCD[1] = buff[1];
			freqLCD[2] = buff[2];
			freqLCD[3] = buff[3];
			freqLCD[4] = buff[4];
			freqLCD[5] = '0';
			printf("frequency is %s", freqLCD);
			
			LCDprint(freqLCD, 2, 0);
			LCDprint("      ", 2, 6);
			frequency = atoi(freqLCD);
			
    		if (rx_size > 0)
			{
    			newF = frequency;
    			//printf("%d", newF);
    			if (newF > 200000L)
    			{
       			 	//SerialTransmit("Warning: High frequencies will cause the interrupt service routine for\r\n"
        			//    "the timer to take all available processor time.  Capping to 200000Hz.\r\n");
        			newF = 200000L;
        			//buzzer(1);
    			}
    			
    			if (newF < 172600)
    			{
			        buzzer(100);
			        ISR_pw = 60;
			        
    			}
    			
    			else if (newF >= 172600 && newF < 173600) 
    			{
        			reload = (SYSCLK / (newF * 2L)+999)/1000 - 1;
        			buzzer(2);
        			ISR_pw = 100;
        		} 
        		
        		else if (newF >= 173600 && newF < 174600) 
    			{
        			buzzer(3);
        			ISR_pw = 140;
        		} 
        		
        		else if (newF >= 174600 && newF < 175600) 
    			{
        			buzzer(4);
        			ISR_pw=180;
        		} 
        		
        		else if (newF >= 175600 && newF < 176000) 
    			{
        			buzzer(5);
        			ISR_pw=240;
        		} 
        		else
				{
        			buzzer(100); 
				} 
    			
			}
			printf("Slave says: %s\r\n", buff);

		}  
		else
		{
			printf("NO RESPONSE\r\n", buff);
		}
		
		
		delayms(50);  // Set the pace: communicate about every ~50ms
	}
}
