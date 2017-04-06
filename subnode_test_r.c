/*
 * Weather Engineering Team
 * 
 * Code for sending I@C data over radio
 * code goes on subnode pic 
 */


// FBS
#pragma config BWRP = OFF               // Boot Segment Write Protect (Disabled)
#pragma config BSS = OFF                // Boot segment Protect (No boot flash segment)

// FGS
#pragma config GWRP = OFF               // General Segment Flash Write Protect (General segment may be written)
#pragma config GSS0 = OFF               // General Segment Code Protect (No Protection)

// FOSCSEL
#pragma config FNOSC = FRCPLL           // Oscillator Select (Fast RC Oscillator with Postscaler and PLL (FRCDIV+PLL))
#pragma config SOSCSRC = ANA            // SOSC Source Type (Analog Mode for use with crystal)
#pragma config LPRCSEL = HP             // LPRC Power and Accuracy (High Power/High Accuracy)
#pragma config IESO = ON                // Internal External Switch Over bit (Internal External Switchover mode enabled (Two-speed Start-up enabled))

// FOSC
#pragma config POSCMD = NONE            // Primary Oscillator Mode (Primary oscillator disabled)
#pragma config OSCIOFNC = ON            // CLKO Pin I/O Function (Port I/O enabled (CLKO disabled))
#pragma config POSCFREQ = HS            // Primary Oscillator Frequency Range (Primary Oscillator/External Clock frequency >8MHz)
#pragma config SOSCSEL = SOSCHP         // SOSC Power Selection Configuration bits (Secondary Oscillator configured for high-power operation)
#pragma config FCKSM = CSECME           // Clock Switching and Monitor Selection (Clock Switching and Fail-safe Clock Monitor Enabled)

// FWDT
#pragma config WDTPS = PS32768          // Watchdog Timer Postscale Select bits (1:32768)
#pragma config FWPSA = PR128            // WDT Prescaler bit (WDT prescaler ratio of 1:128)
#pragma config FWDTEN = SWON            // Watchdog Timer Enable bits (WDT controlled with SWDTEN bit setting)
#pragma config WINDIS = OFF             // Windowed Watchdog Timer Disable bit (Standard WDT selected (windowed WDT disabled))

// FPOR
#pragma config BOREN = BOR3             // Brown-out Reset Enable bits (Enabled in hardware; SBOREN bit disabled)
#pragma config PWRTEN = ON              // Power-up Timer Enable (PWRT enabled)
#pragma config I2C1SEL = PRI            // Alternate I2C1 Pin Mapping bit (Default SCL1/SDA1 Pins for I2C1)
#pragma config BORV = V18               // Brown-out Reset Voltage bits (Brown-out Reset at 1.8V)
#pragma config MCLRE = ON               // MCLR Pin Enable bit (RA5 input disabled; MCLR enabled)

// FICD
#pragma config ICS = PGx1               // ICD Pin Placement Select (EMUC/EMUD share PGC1/PGD1)

// http://www.microchip.com/forums/m273860.aspx

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <xc.h>
#include <stdint.h>
#include <stdbool.h>
#include <p24F16KL401.h>
#include <uart.h>

#include <stdio.h>

/*
 
 Defines
 
 */
#define _XTAL_FREQ  8000000
#define CONSOLE_IsPutReady()        (U2STAbits.TRMT)
#define TREG2        U2TXREG
#define RREG2       U2RXREG
#define TREG1       U1TXREG
#define RREG1       U1RXREG
/*
 
 Function Dec
 
 */
void RadioINIT(void);
void Radio_Put(uint8_t c);
bool Radio_IsPutReady(void);
void RadioTX(char* str);
char* RadioRX(void);

void CONSOLE_PutString(char* str);
void CONSOLE_Put(uint8_t c);
void CONSOLE_INIT(void);

void __attribute__((interrupt, shadow, no_auto_psv)) _U1RXInterrupt();

/*
 *
 * Global Var
 * 
 */
int new_data = 0;
char RX_char = 0;
uint16_t temperature = 0x652D;
uint16_t humidity = 0x40B7;
uint16_t uv = 0x0029A;
uint8_t node = 2;

void main(void)
{
    // initialize console
    //CONSOLE_INIT();
    //ConfigIntUART1(1);
    
    RadioINIT();

    char RXmsg[255];
    int count = 0;
    uint32_t c = 0;
    uint8_t i = 0;
    //RadioTX((char *)"\n\r before while(1)");
    //sprintf(RXmsg,"X %d %04x %04x %04xU", node,temperature,humidity,uv);
    
    
    while(1)
    {
        //RadioTX((char *)"Hello ");
        while(c++<0xFFFFF);
        c = 0;
        //while(IFS0bits.U1RXIF==0);    
        //RX_char = (U1RXREG & 0xFF);
        //Radio_Put(RX_char);
        new_data = 1;
        if (new_data==1)
        {   
            printf(RXmsg,"X %d %04x %04x %04xU", node,temperature,humidity,uv);
            RadioTX((char *)RXmsg);
            if (node++ > 4)
            {
                node = 2;
            }
        

            //RadioTX((char *)"\n\r in new_data");
            //while(!U1STAbits.TRMT);
           // RX_char = (U1RXREG & 0x00FF);
            //RX_char = (char)RX_char;
            ///sprintf(RXmsg,"2 652D 40B7 029EU");
            //RadioTX((char *)RXmsg);
            /*if (RXmsg[2] < '6')
            {
                
                RadioTX((char *)RXmsg);
                RXmsg[3]++;
            }
            else
            {
                
                
                sprintf(RXmsg,"X END 7FFF 652D 40B7 029EU");
                RadioTX((char *)RXmsg);
                sprintf(RXmsg,"X 2 652D 40B7 029EU");
                
            }
                
            //RadioTX((char *)"hello you sent : ");
            //RXmsg[i++] = RX_char;
            new_data = false;
            //Radio_Put(RX_char);
            /*if (RX_char =='U')
            {
                
                RXmsg[i] = 0x00;
                //RadioTX((char *)"msg : ");
                
                //RadioTX((char *)"\n\r");
                //RXmsg = 0;
                i=0;
                
            }
            else
            {
                //RadioTX((char *)"writing t str : ");
                Radio_Put(RX_char);
                RXmsg[i++] = RX_char;
            }*/
            
        }
    }
}

/*
void CONSOLE_PutString(char* str)
{
    uint8_t c;

    while( c = *str++ )
        CONSOLE_Put(c);
}

void CONSOLE_Put(uint8_t c)
{
    while( !CONSOLE_IsPutReady() );
        TREG2 = c;
        

}

void CONSOLE_INIT(void)
{
    
     //A function that initializes UART 2 for use of the Console
    
    /// for uart 2 
    U2MODEbits.UARTEN = 1;//enable UART
    U2MODEbits.USIDL = 0; // continue operation in idle mode
    U2MODEbits.STSEL = 0; // 1 stop bit
    U2MODEbits.PDSEL0 = 0;
    U2MODEbits.PDSEL1 = 0;// 8, N, 
    U2MODEbits.BRGH = 0;
    //U2MODEbits.BRGH = 1;

    U2MODEbits.RXINV = 0;// rx idle state = 1
    U2MODEbits.ABAUD = 0;// no autobaud
    U2MODEbits.LPBACK = 0; // loopback mode disabled
    U2MODEbits.WAKE = 0; // wakeup disable
    U2MODEbits.UEN0 = 0;
    U2MODEbits.UEN1 = 0;
    U2MODEbits.RTSMD = 1;// rts mode 
    U2MODEbits.IREN = 0;// irda disabled
    U2STAbits.ADDEN = 0; // address mode disabled
      
    U2STAbits.URXISEL0 = 1;
    U2STAbits.URXISEL1 = 1; // interrupt any data received
    
    U2STAbits.UTXEN = 1; // transmit enabled
    U2STAbits.UTXBRK = 0; // break disabled
    
    U2STAbits.UTXISEL0 = 1;
    U2STAbits.UTXISEL1 = 0; // interrupt when txregister becomes empty
    
    
     
    U2BRG = 51 ; // 3.340277778 //run at 9600
}*/

void RadioINIT(void)
{
    /*
     * A function that initializes UART 1 for use of the Console
     */
    TRISAbits.TRISA4 = 1;
    ANSBbits.ANSB2 = 0;
    
	U1MODEbits.UARTEN = 1;		// UART1 is Enabled
	U1MODEbits.USIDL = 0;		// Continue operation at Idlestate
	U1MODEbits.IREN = 0;		// IrDA En/Decoder is disabled
	U1MODEbits.UEN = 0b00;		
	U1MODEbits.WAKE = 1;		// Wake-up on start bit is enabled
	U1MODEbits.LPBACK = 0;		// Loop-back is disabled
	U1MODEbits.ABAUD = 0;		// auto baud is disabled
	U1MODEbits.RXINV = 0;		// No RX inversion
	U1MODEbits.BRGH = 0;		// low boud rate
	U1MODEbits.PDSEL = 0b00; 	// 8bit no parity
	U1MODEbits.STSEL = 0;		// one stop bit	
		

	U1STAbits.UTXISEL1 = 0b00;	
	//U1STA &= 0xDFFF;			// clear TXINV by bit masking
	U1STAbits.UTXBRK = 0;		// sync break tx is disabled
	U1STAbits.UTXEN = 1;		//transmit  is enabled
	U1STAbits.URXISEL = 0b00;	// interrupt flag bit is set when RXBUF is filled with 1 character
	U1STAbits.ADDEN = 0;		// address detect mode is disabled

	IFS0bits.U1RXIF = 0;		// clear interrupt flag of rx
	IEC0bits.U1RXIE = 1;		// enable rx recieved data interrupt
    
    U1BRG = 51 ; // 3.340277778 for baud of 9600
    

}

void RadioTX(char* str)
{
 
    uint8_t c;

    while( c = *str++ )
    {
        Radio_Put(c);
    }
}

void Radio_Put(uint8_t c)
{
    while( !Radio_IsPutReady() );
    TREG1 = c;

}

bool Radio_IsPutReady()
{
    //may need to check another condition as well
    // depending on if radio has interrupts
    if (!U1STAbits.TRMT)
    {
        return 0;
    }
    else 
    {
        return 1;
    }
}
  
void __attribute__((interrupt, shadow, no_auto_psv)) _U1RXInterrupt(void)
{
    //This is called when interrupt happens
    U1STAbits.OERR = 0;
   // RadioTX((char *)"\n\rwriting RX_char ");
    RX_char = (U1RXREG);
    
    new_data = 1;
    IFS0bits.U1RXIF = 0;
}
