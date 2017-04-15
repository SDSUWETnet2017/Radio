//Project: Supernode Integration
//Engineer: Alven Eusantos, Jeremey Lee, Matthew Salvino
//3/13/17

//rev 3 I2C locking up




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

uint32_t count = 0;
bool new_word = 0;
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

void ISR_INIT(void);
void TMR1_INIT(void);
void TMR3_INIT(void);
void ADC_INIT(void);

void I2C_Master_Init(const unsigned long c);
void I2C_Master_Wait();
void I2C_Master_Start();
void I2C_Master_RepeatedStart();
void I2C_Master_Stop();
void I2C_Master_Write(unsigned d);
void Write(unsigned a,unsigned b,unsigned c);
unsigned short I2C_Master_Read(unsigned short a);
void Read(unsigned q,unsigned w,unsigned e);

float MPH_Calculate(void);
uint16_t MPH_Average(uint16_t mph_calc[120], uint16_t countSpeed);
uint16_t is_gust(uint16_t mph_calc[120], uint16_t countSpeed);
uint16_t airQualityAverage(void);
void getAngle(void);
void setDirection(void);
void checkHighFlag();
void clearGlobalData(void);;

void __attribute__((__interrupt__,no_auto_psv)) _T1Interrupt(void);
void __attribute__((interrupt, shadow, no_auto_psv)) _U1RXInterrupt();
void __attribute__((interrupt, shadow, no_auto_psv)) _U2RXInterrupt();

/*
 *
 * Global Var
 * 
 */
int new_data = 0;
char RX_char = 0;
char offset_char = 0;


volatile uint16_t clock = 0;
volatile uint16_t totalPeriod = 60000;//total period in 10s of ms
volatile bool endCycle = false;

/*
 *
 *Anemometer Var
 *
 */
volatile uint16_t pulses = 0;
volatile bool calc_flag = false;

/*
 *
 *Air Quality Var
 *
 */
volatile bool ADC_SAMPLING = false;
volatile uint16_t AirQualityArray[100];
volatile uint16_t airQualitySamples = 0;

/*
 *
 *Wind Vane Var
 *
 */

volatile uint16_t N_flag = 0;
volatile uint16_t NE_flag = 0;
volatile uint16_t E_flag = 0;
volatile uint16_t SE_flag = 0;
volatile uint16_t S_flag = 0;
volatile uint16_t SW_flag = 0;
volatile uint16_t W_flag = 0;
volatile uint16_t NW_flag = 0;
volatile bool send_flag = false;
volatile uint16_t array_size = 100;
volatile uint16_t array_counter = 0;
volatile bool set_direction = false; //flag goes high once all the flags are compared
volatile uint16_t MAX = 0;
volatile uint16_t ANGLE;  
volatile uint8_t ii;

void main(void)
{
    
    // initialize console
    CONSOLE_INIT();//uart 2 goes to PI
    
    RadioINIT();//uart 1 goes to radio
    
    I2C_Master_Init(100000);
    TMR1_INIT();
    TMR3_INIT();
    ADC_INIT();
    ISR_INIT();
    char RXmsg[25];//initialize a string to hold values from radio & local sensors
    char TXmsg[2]; //confirmation
    char pairMSG[13];
    char localData[25];
    int deltaT = 0; //check if ahead or behind
    int offsetThreshold = 1000; //set to 10 seconds //comparison for how much oscillator can be off on subnode in 10s of milliseconds
    uint8_t i = 0;
    
    uint16_t MPH[100] = {0};
    uint16_t countSpeed = 0;
    uint16_t mphAverage = 0;
    uint16_t gust = 0;
    uint16_t airQuality = 0;
    //CONSOLE_PutString((char *)"START SEQ");
    //CONSOLE_PutString((char *)"X 3 7FFF 7FFF 7FFFU");
    
    bool pairFlag = 1;
    char pairNode = '2';
    while(pairFlag) //used for pairing nodes
    {
        if(clock % 100 == 0) //every 100 ms
        {
            sprintf(pairMSG, "%c", pairNode);
            RadioTX((char *)pairMSG); //send node number
            if(RX_char == pairNode) //if node number received back
            {
                sprintf(pairMSG,"Node %c Paired", pairNode);
                CONSOLE_PutString((char *)pairMSG); //alert pi
                pairNode++; //increment searching for nodes
                if(pairNode == '6') //if node 5 has paired, exit while loop
                    pairFlag = 0;
 
            }
        }  
    }
    uint8_t delayer = clock;
    while(delayer + 2 > clock);
    RadioTX((char *)"SSSS"); //start subnode operation
    clock = 0;
    
    
    while(1)
    {
        //CONSOLE_PutString((char *)"whiel 1");
        /*
        if(calc_flag == 1)
        {
           //CONSOLE_PutString((char *)"\n\rCALC FLAG ");
           MPH[countSpeed] = MPH_Calculate();
           countSpeed++;
           calc_flag = 0;
        }
        if(ADC_SAMPLING == true)
        {
           // CONSOLE_PutString((char *)"\n\rIn ADC ON ");
            AD1CON1bits.SAMP = 0; // End A/D sampling and start conversion
            ADC_SAMPLING == false;
        }
        if(send_flag == true)
        {
           // CONSOLE_PutString((char *)"\n\rSEND FLAG ");
            I2CStart();
        }
         */
        if(endCycle == true)
        {
            //CONSOLE_PutString((char *)"end cycle");
            /*
            mphAverage = MPH_Average(MPH, countSpeed);
            gust = is_gust(MPH, countSpeed);
            countSpeed = 0;
            memset(MPH, 0, sizeof MPH);
            airQuality = airQualityAverage();    */
            
            sprintf(localData,"X END %04x %02x %02x %02xU", airQuality, ANGLE, mphAverage,gust);
            CONSOLE_PutString((char *)localData);
             
            endCycle = false;
        }
        if (new_data == 1 && new_word == 1)
        {     
            //CONSOLE_PutString((char *)"new data");
            new_data = 0;

            if (RX_char == 'U')
            {
                
                RXmsg[i++] = RX_char;
                RXmsg[i] = 0x00;
                i=0; 
                
                //sending confirmation
                if(RXmsg[2] == '2')
                {
                    deltaT = (totalPeriod / 10) - clock;
                    if(deltaT >= -offsetThreshold && deltaT <= offsetThreshold) //if acceptable offset
                    {
                        sprintf(TXmsg,"%cN", RXmsg[2]);
                        RadioTX((char *)TXmsg);
                        //((char *)TXmsg);
                    }
                    else if(deltaT > 0) //if subnode is drifting too far out of bin by positive deltaT
                    {
                        sprintf(TXmsg,"%c-",RXmsg[2]);//subtract from clock
                        RadioTX((char *)TXmsg);
                        //CONSOLE_PutString((char *)TXmsg);
                    }
                    else //if subnode is drifting too far out of bin by negative deltaT
                    {
                        sprintf(TXmsg,"%c+",RXmsg[2]); //add to clock
                        RadioTX((char *)TXmsg);
                        //CONSOLE_PutString((char *)TXmsg);
                    }
                    
                }
                if(RXmsg[2] == '3')
                {
                    deltaT = ((totalPeriod / 10)*3) - clock;
                    if(deltaT >= -offsetThreshold && deltaT <= offsetThreshold) //if acceptable offset
                    {
                        sprintf(TXmsg,"%cN", RXmsg[2]);
                        RadioTX((char *)TXmsg);
                        //CONSOLE_PutString((char *)TXmsg);
                    }
                    else if(deltaT > 0) //if subnode is drifting too far out of bin by positive deltaT
                    {
                        sprintf(TXmsg,"%c-",RXmsg[2]);//subtract from clock
                        RadioTX((char *)TXmsg);
                        //CONSOLE_PutString((char *)TXmsg);
                    }
                    else //if subnode is drifting too far out of bin by negative deltaT
                    {
                        sprintf(TXmsg,"%c+",RXmsg[2]); //add to clock
                        RadioTX((char *)TXmsg);
                        //CONSOLE_PutString((char *)TXmsg);
                    }
                    
                }
                if(RXmsg[2] == '4')
                {
                    deltaT = ((totalPeriod / 10)*5) - clock;
                    if(deltaT >= -offsetThreshold && deltaT <= offsetThreshold) //if acceptable offset
                    {
                        sprintf(TXmsg,"%cN", RXmsg[2]);
                        RadioTX((char *)TXmsg);
                        //CONSOLE_PutString((char *)TXmsg);
                    }
                    else if(deltaT > 0) //if subnode is drifting too far out of bin by positive deltaT
                    {
                        sprintf(TXmsg,"%c-",RXmsg[2]);//subtract from clock
                        RadioTX((char *)TXmsg);
                        //CONSOLE_PutString((char *)TXmsg);
                    }
                    else //if subnode is drifting too far out of bin by negative deltaT
                    {
                        sprintf(TXmsg,"%c+",RXmsg[2]); //add to clock
                        RadioTX((char *)TXmsg);
                       // CONSOLE_PutString((char *)TXmsg);
                    }
                    
                }
                if(RXmsg[2] == '5')
                {
                    deltaT = ((totalPeriod / 10)*7) - clock;
                    if(deltaT >= -offsetThreshold && deltaT <= offsetThreshold) //if acceptable offset
                    {
                        sprintf(TXmsg,"%cN", RXmsg[2]);
                        RadioTX((char *)TXmsg);
                        //CONSOLE_PutString((char *)TXmsg);
                    }
                    else if(deltaT > 0) //if subnode is drifting too far out of bin by positive deltaT
                    {
                        sprintf(TXmsg,"%c-",RXmsg[2]);//subtract from clock
                        RadioTX((char *)TXmsg);
                       // CONSOLE_PutString((char *)TXmsg);
                    }
                    else //if subnode is drifting too far out of bin by negative deltaT
                    {
                        sprintf(TXmsg,"%c+",RXmsg[2]); //add to clock
                        RadioTX((char *)TXmsg);
                        //CONSOLE_PutString((char *)TXmsg);
                    }
                }

                CONSOLE_PutString((char *)RXmsg);
                new_word = 0;
 
                
            }
            else
            {
		//construct string from recieved chars
                RXmsg[i++] = RX_char;
            }
            
        }
        
        
    }
}

void TMR1_INIT(void)
{   
    T1CONbits.TCS = 1;
    T1CONbits.T1ECS = 0b10;
    T1CONbits.TCKPS = 0;
    T1CONbits.TON = 1;
    PR1 = 310;
    TMR1 = 0;
}

void TMR3_INIT(void)
{
    T3CON = 0b0000000010000101;
    TMR3 = 0;
}

void ADC_INIT(void)
{
    AD1CON1bits.FORM = 0b00; //Output of ADC is an integer: 0b---- --xx xxxx xxxx
    AD1CON1bits.SSRC = 0b000; //Clearing SAMP bit ends sampling and begins conversion
    AD1CON1bits.ADSIDL = 1; //Discontinue operation in idle mode
    AD1CON1bits.ASAM = 0; //Sampling begins when SAMP bit is set
    
    AD1CON3bits.ADRC = 0b0; //Clock derived from system clock
    AD1CON3bits.SAMC = 0b00001; //Auto sample time bits = T_AD
    AD1CON3bits.ADCS = 0b111111;
      
    AD1CHSbits.CH0NA = 0; //Negative reference voltage at VR-
    AD1CHSbits.CH0SA = 0b0000; //Positive input select for MUXA is AN0
    AD1CSSL = 0; // No inputs are scanned.
}

void ISR_INIT(void)
{
    INTCON1bits.NSTDIS = 1; //Disable nested interrupts
    IFS0bits.T1IF = 0;
    IEC0bits.T1IE = 1;
}
void CONSOLE_PutString(char* str)
{
    uint8_t c;
    uint8_t count;
    while( c = *str++ )
    {
        CONSOLE_Put(c);
    }
    
    
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
    TRISBbits.TRISB1 = 1; //enalbe input at U2RX port at RB1
    LATBbits.LATB1 = 0;
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
    U2MODEbits.WAKE = 1; // wakeup disable
    U2MODEbits.UEN0 = 0;
    U2MODEbits.UEN1 = 0;
    U2MODEbits.RTSMD = 1;// rts mode 
    U2MODEbits.IREN = 0;// irda disabled
    U2STAbits.ADDEN = 0; // address mode disabled
      
    U2STAbits.URXISEL0 = 0;
    U2STAbits.URXISEL1 = 0; // interrupt any data received
    
    U2STAbits.UTXEN = 1; // transmit enabled
    U2STAbits.UTXBRK = 0; // break disabled
    
    U2STAbits.UTXISEL0 = 1;
    U2STAbits.UTXISEL1 = 0; // interrupt when txregister becomes empty
    
    IFS1bits.U2RXIF = 0;		// clear interrupt flag of rx
    IEC1bits.U2RXIE = 1;		// enable rx recieved data interrupt
    
    U2BRG = 51 ; // 3.340277778 //run at 9600
}

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
    
    while(count++<0xFFFFF){}
    
    TRISBbits.TRISB8 = 0;
    LATBbits.LATB8 = 1;

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
  
float MPH_Calculate(void)
{
    float RPM = 0;

    RPM = pulses * 60;
    RPM /= 12;
    RPM *= 60;
    RPM *= 1.11;
    return RPM/5280; //conversion
}

uint16_t MPH_Average(uint16_t mph_calc[120], uint16_t countSpeed)
{
    int average = 0;
    int i = 0;
    for (i = 0; i < countSpeed; i++)
    {
        average += mph_calc[i];
    }
    return average/i;
}

uint16_t is_gust(uint16_t mph_calc[120], uint16_t countSpeed)
{
    unsigned int max = mph_calc[0];
    int i = 0;
    for (i = 1; i < countSpeed; i++)
    {
        if(mph_calc[i] > max)
        {
            max = mph_calc[i];
        }
    }
    return max;
}

uint16_t airQualityAverage(void)
{
    uint16_t count = 0;
    uint16_t averageAirTotal = 0;
    uint16_t average = 0;
    while (count < airQualitySamples)
    {        
            averageAirTotal += AirQualityArray[count];
            count++;            
    }
    average = averageAirTotal/count;
    if(average <= 124)
        {
            return 0x0001;
        }
    else if(average < 217 && average >= 124)
        {
            return 0x0010;
        }
        else if(average < 321 && average >= 217)
        {
            return 0x0100;
        }
        else if(average >= 321)
        {
            return 0x1000;
        }

}

void I2CStart()
{
    //if(send_flag == 1) //Enters if-statement after timer2 sets flag high (every 2 seconds)
    // {
    
   CONSOLE_PutString((char *)"\n\rI2CSTART BEGIN");
    I2C_Master_Start(); //start 
    CONSOLE_PutString((char *)"\n\rMASTER START");
    I2C_Master_Wait(); //wait
    CONSOLE_PutString((char *)"\n\rMASTER WAIT");
    Read(0x80,0xFE,0x81); //MSB angle address write device address, internal address, and device add + 1 
    
    ii = I2C_Master_Read(0); //Read the MSB i2c Value
    CONSOLE_PutString((char *)"\n\rMASTER READ");
    I2C_Master_Wait(); //causing problems
    CONSOLE_PutString((char *)"\n\rMASTER WAIT");
    I2C_Master_Stop(); //causing problems
    CONSOLE_PutString((char *)"\n\rMASTER STOP");
    //send_flag == 0;
    
    getAngle();
    
    
  }




void getAngle()
{
    if(send_flag == true)
    {
            CONSOLE_PutString((char *)"\n\rGET ANGLE");
            ANGLE = ii;  //set i2c value to ANGLE (do i need this?)
            //ANGLE =(i<<6)+(j&0x3F); //Store MSB and LSB i2c values together into one variable (i2c values)
            if(array_counter < array_size)   //array_size is set to 100 samples (set to 300 samples for 10 min)    
            {
               // i2c value is compared to 8 min and max ranges for each direction
                if(((ANGLE<0x90) && (ANGLE>=0x79)) || ((ANGLE<0x81) && (ANGLE>=0x70)))   
                {
                    N_flag++;
                }
                else if((ANGLE<0x70) && (ANGLE>=0x50))  
                {
                    NE_flag++;
                }
                else if((ANGLE<0x50) && (ANGLE>=0x30))   
                {
                    E_flag++;
                }
                else if((ANGLE<0x30) && (ANGLE>=0x10))  
                {
                    SE_flag++;
                }
                else if(((ANGLE<=0xFE) && (ANGLE>=0xF0)) || ((ANGLE<=0x10) && (ANGLE>=0x01)) || (ANGLE==0xFF))   
                {
                    S_flag++;
                }
                else if((ANGLE<0xF0) && (ANGLE>=0xD0))   
                {
                    SW_flag++;
                }
                else if((ANGLE<0xD0) && (ANGLE>=0xB0))   
                {
                    W_flag++;
                }
                else if((ANGLE<0xB0) && (ANGLE>=0x90))   
                {
                    NW_flag++; //Increments flag if ANGLE is in this area
                }
                array_counter++; //counter goes until 300 samples have been made
                send_flag = false;
            }//if(array_counter < array_size)  
            
    } //if(send_flag == 1)
    


    setDirection();
}

void setDirection()
{
    if(array_counter >= array_size) //Enter if-statement once all the samples have been set to a direction flag   
        {
            array_counter = 0; //empty array counter so it can sample again
            // Set up each flag to a char N or to hexadecimal number
            //Count which flag has the most counts
            if(N_flag > MAX) MAX = N_flag; 
            if(NE_flag > MAX) MAX = NE_flag;
            if(E_flag > MAX) MAX = E_flag;
            if(SE_flag > MAX) MAX = SE_flag;
            if(S_flag > MAX) MAX = S_flag;
            if(SW_flag > MAX) MAX = SW_flag;
            if(W_flag > MAX) MAX = W_flag;
            if(NW_flag > MAX) MAX = NW_flag;
                    
            set_direction = true; //Increments to 10 samples and then can enter 2nd if-statement
            
            checkHighFlag();
        }
    
}


void checkHighFlag()
    {
    if(set_direction == true) //10 samples so now check which flag is the highest
    {
       if(MAX == N_flag) ANGLE = 0x80; //0x80 //float = 
                    else if(MAX == NE_flag) ANGLE = 96; //0x60 
                    else if(MAX == E_flag) ANGLE = 64; //0x40
                    else if(MAX == SE_flag) ANGLE = 32; //0x20
                    else if(MAX == S_flag) ANGLE = 0xff; //0xFF //float = 255
                    else if(MAX == SW_flag) ANGLE = 223; //0xDF //float = 128;
                    else if(MAX == W_flag) ANGLE = 191; //0xBF
                    else if(MAX == NW_flag) ANGLE = 159; //0x9F
                    
                    else 
                        ANGLE = 0; //If an error, send zero
                    
        //prepareForUART(ANGLE);

        set_direction = false;
        send_flag = false;
        

        }       

}//void I2CStart()

void clearGlobalData()
{
    memset(AirQualityArray, 0, sizeof AirQualityArray);
    airQualitySamples = 0;
    set_direction = 0;
    send_flag = 0;
    ANGLE = 0; // CLEAR THE DIRECTION
    MAX = 0; //CLEAR THE MAX FLAG
    ii = 0;
    N_flag = 0;
    NE_flag = 0;
    E_flag = 0;
    SE_flag = 0;
    S_flag = 0;
    SW_flag = 0;
    W_flag = 0;
    NW_flag = 0; //CLEAR ALL FLAGS

}

void __attribute__((__interrupt__,no_auto_psv)) _T1Interrupt(void)
{
    
    if(clock % 500 == 0)
    {
        pulses = TMR3/3;
        //time[sample_count] = 4 * (sample_count + 1);
        //sample_count++;
        calc_flag = 1;
        TMR3 = 0;
        IFS0bits.AD1IF = 0;
        IEC0bits.AD1IE = 1; // Enable A/D conversion interrupt
        AD1CON1bits.ADON = 1; // Turn on A/D
        AD1CON1bits.SAMP = 1; // Start sampling the input
        ADC_SAMPLING = true;
        send_flag = true;
        if(clock >= totalPeriod) //This sets the period
        {
            endCycle = true;
            clock = 0;
        }
    }
    clock++;
    IFS0bits.T1IF = 0;
}

void __attribute__ ((__interrupt__,no_auto_psv)) _ADC1Interrupt(void)
{   
    if(IFS0bits.AD1IF == 1)
    {
    if(airQualitySamples < 100)
        {
            AirQualityArray[airQualitySamples] =  ADC1BUF0;
            airQualitySamples++;
        }   
    }
    IFS0bits.AD1IF = 0;
}

void __attribute__((interrupt, shadow, no_auto_psv)) _U1RXInterrupt(void)
{
    //This is called when interrupt happens from recieved char from radio
    U1STAbits.OERR = 0;
    RX_char = (U1RXREG);
    new_data = 1;
    
    if(RX_char == 'X' && new_word == 0)
    {
        new_word = 1;
    }
   
   // RadioTX((char *)"\n\rwriting RX_char ");
    IFS0bits.U1RXIF = 0;
    
}

void __attribute__((interrupt, shadow, no_auto_psv)) _U2RXInterrupt(void)
{
    U2STAbits.OERR = 0;
    offset_char = (U2RXREG);
    CONSOLE_PutString((char *)"In Interrupt");
    
    if(offset_char == '+') //add half a second to period of clock
    {
        clock = clock + 500;
        CONSOLE_PutString((char *)"Adding 500");
    }
        
    else if(offset_char == '-') //subtract half a second from period
    {
        clock = clock - 500;
        CONSOLE_PutString((char *)"Subtracting 500");
    }
        
    
    offset_char = 0;
    IFS1bits.U2RXIF = 0;
}

void I2C_Master_Init(const unsigned long c)
{
    SSP1CON1 =0b0000000000101000; //SSPM mater node, no collision, no overflow,  Enables the serial port and configures the SDAx and SCLx pins as the serial port pins  
    SSP1CON2 =0b0000000000000000; //General Call address// Set Acknowledgment bit
    SSP1ADD = (_XTAL_FREQ/(4*c))-1; //Setting Clock Speed   (8M/4*100K)-1 = 19
    SSP1STAT =0b0000000000000000;  //Set low 
}

void I2C_Master_Wait()
{
    while ((SSP1STAT & 0x04) || (SSP1CON2 & 0x1F)); //Transmit is in progress  // Once SSP1STAT and SSP1CON2 goes high, then proceed
}


void I2C_Master_Start()
{
    I2C_Master_Wait();                 
    SSP1CON2bits.SEN = 1;             //Initiate start condition
}


 void I2C_Master_RepeatedStart()
{
    I2C_Master_Wait();
    SSP1CON2bits.RSEN = 1;           //Initiate repeated start condition
}

void I2C_Master_Stop()
{
    I2C_Master_Wait();
    SSP1CON2bits.PEN = 1;           //Initiate stop condition
}

void I2C_Master_Write(unsigned d)
{
    I2C_Master_Wait();
    SSP1BUF = d;         //Write data to SSPBUF   
}//I2C_Master_Write

void Write(unsigned a,unsigned b,unsigned c)
{
    I2C_Master_Write(a);     //slave id + 0
    I2C_Master_Write(b);    //register address
    I2C_Master_Write(c);    //data
}

unsigned short I2C_Master_Read(unsigned short a)
{
    unsigned short angle;
  
    I2C_Master_Wait();
    SSP1CON2bits.RCEN = 1; //enable receive mode for i2c
    I2C_Master_Wait();
  
  
    angle = SSP1BUF;      //Read data from SSPBUF (binary)
    I2C_Master_Wait();
  
    SSP1CON2bits.ACKDT = (a)?0:1;    //Acknowledge bit
    SSP1CON2bits.ACKEN = 1;          //Acknowledge sequence
    
        return angle;  
}//I2C_Master_Read



void Read(unsigned q,unsigned w,unsigned e)
{
    I2C_Master_Write(q); //7 bits + write
    I2C_Master_Write(w);
    I2C_Master_RepeatedStart();
    I2C_Master_Write(e); // 7 bits + read (+1)
}
