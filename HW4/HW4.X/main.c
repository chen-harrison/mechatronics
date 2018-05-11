#include <xc.h>           // processor SFR definitions
#include <sys/attribs.h>  // __ISR macro
#include "NU32.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // disable secondary osc
#pragma config FPBDIV = DIV_1 // divide sysclk freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1048576 // use slowest wdt
#pragma config WINDIS = OFF // wdt no window mode
#pragma config FWDTEN = OFF // wdt disabled
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the sysclk clock to 48MHz from the 8MHz crystal
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiplied by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 0 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module

#define CS LATBbits.LATB7

void initSPI1(void);
unsigned char spi_io(unsigned char o);
void setVoltage(unsigned char channel, int voltage);

int main() {
    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // do your TRIS and LAT commands here
    TRISAbits.TRISA4 = 0;
    LATAbits.LATA4 = 1;
    TRISBbits.TRISB4 = 1;

    __builtin_enable_interrupts();
    
    initSPI1();
    int i = 0, j;
    
    int sine[200], triangle[200];
    int VoutA, VoutB;
    /*
    for(j=0; j<=199; j++){
        if(j <= 100){
                triangle[j] = j * 10.24;
            }
        else if(j > 100){
            triangle[j] = (200-j) * 10.24;
            }
        
        sine[j] = (cos(20*M_PI*i/100)+1)/2 * 1024;
    }
    */
    
    int square[4] = {1023,0, 1023, 0};
    
    while(1) {
	// use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
	// remember the core timer runs at half the sysclk
        if(PORTBbits.RB4 == 0){
            VoutA = 0;
            VoutB = 0;
            
            setVoltage(0,VoutA);
            setVoltage(1,VoutB);
        }
        else if(PORTBbits.RB4 == 1){
            _CP0_SET_COUNT(0);
            /*
            VoutA = sine[i];
            VoutB = triangle[i];
            
            if(i < 199){
                i++;
            }
            else if(i >= 199){
                i = 0;
            }
            */
            VoutA = square[i];
            
            if(i < 3){
                i++;
            }
            else if(i >= 3){
                i = 0;
            }
            
            setVoltage(0,VoutA);
            
            while(_CP0_GET_COUNT() < 24000){ ; }
            
            
            // setVoltage(1,VoutB);
        }
    }
    
    return 0;
}


void initSPI1(void){
    TRISBbits.TRISB7 = 0;
    CS = 1;

  // setup SPI1
    SPI1CON = 0;                // turn off the spi module and reset it
    SPI1BUF;                    // clear the rx buffer by reading from it
    SPI1BRG = 100;             // baud rate to 10 MHz [SPI4BRG = (24000000/(2*desired))-1]
    SPI1STATbits.SPIROV = 0;    // clear the overflow bit
    SPI1CONbits.CKE = 0;        // data changes when clock goes from low to high
    SPI1CONbits.MSTEN = 1;      // master operation
 
    // CFGCONbits.IOLOCK = 0;   
    RPB7Rbits.RPB7R = 0b0011;    // SS1 = RPB7 (pin 16)
    RPA1Rbits.RPA1R = 0b0011;    // SDO1 = RPA1 (pin 3)
    TRISBbits.TRISB8 = 1;
    SDI1Rbits.SDI1R = 0b0100;    // SDI1 = RPB8 (pin 17)
                                // SCK1 = RPB14 (pin 25)
    
    SPI1CONbits.ON = 1;         // turn on SPI1
    
}

unsigned char spi_io(unsigned char o){
    SPI1BUF = o;
    while(!SPI1STATbits.SPIRBF) { ; }
    return SPI1BUF;
}


void setVoltage(unsigned char a, int v){
    unsigned short t = 0; 
    t= a << 15;                         // a is at the very end of the data transfer
	t = t | 0b0111000000000000;         // removed a 0 at the end, so 16 bits
	t = t | ((v & 0b1111111111) << 2);  // add integer, shift up 2, scrap the rest
	
	CS = 0;
	spi_io(t >> 8); // add 8 zeros on left end to convert from short to char
    spi_io(t);
    CS = 1;
}