#include "i2c_master_noint.h"
#include <xc.h>           // processor SFR definitions
#include <sys/attribs.h>  // __ISR macro
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

void initExpander(void);
char getExpander(void);
void setExpander(char pin, char level);

int main() {
    char status;
    TRISAbits.TRISA4 = 0;
    LATAbits.LATA4 = 1;
    
    initExpander();
    setExpander(0,1);               // turn on LED at pin 0
    
    while(1){
        status = getExpander();
        
        if(!(status >> 7)){
            setExpander(0,1);
        }
        else if(status >> 7){
            setExpander(0,0);
        }
        
        LATAINV = 0b10000;          // inverting LED to show code is running
    }
    return 0;
}

void initExpander(void){
    ANSELBbits.ANSB2 = 0;
    ANSELBbits.ANSB3 = 0;
    i2c_master_setup();
    
    i2c_master_start();             // set input and output pins
    i2c_master_send(0b01001110);    // A0 = 1, A1 = 1, A2 = 1, write bit 0
    i2c_master_send(0x00);          // IODIR register
    i2c_master_send(0xF0);          // 0-3 = outputs, 4-7 = inputs
    i2c_master_stop();
    
    i2c_master_start();             // set outputs at pins 0-3
    i2c_master_send(0b01001110);    // write bit 0
    i2c_master_send(0x0A);          // OLAT register
    i2c_master_send(0x0F);
    i2c_master_stop();
}

char getExpander(void){
    i2c_master_start();             // read input pins
    i2c_master_send(0b01001110);    // write bit 0
    i2c_master_send(0x09);          // GPIO register
    i2c_master_restart();
    i2c_master_send(0b01001111);    // read bit 1
    char read = i2c_master_recv();  // receive from slave
    i2c_master_stop();
    
    return read;
}

void setExpander(char pin, char level){
    char write = getExpander();     // existing outputs
    char change = 1 << pin;         // 1 at the desired pin bit
    if(level){
        write = (write | change);   // set desired pin high
    }
    else if(!level){
        write = ~write;             // invert existing outputs
        write = (write | change);   // set the inverted bit to 1
        write = ~write;             // invert it again to set it to 0
    }
    
    i2c_master_start();             // read input pins
    i2c_master_send(0b01001110);    // write bit 0
    i2c_master_send(0x0A);          // OLAT register
    i2c_master_send(write);         // set the outputs to new byte
    i2c_master_stop();
}