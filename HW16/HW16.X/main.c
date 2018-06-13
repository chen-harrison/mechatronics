#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro

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

initMotorControl(void);
initEncoder(void);

void __ISR(_TIMER_4_VECTOR, IPL4SOFT) Timer4ISR(void) {
  // code for PI control goes here

  IFS0bits.T4IF = 0; // clear interrupt flag, last line
}

int main() {
    /*
    __builtin_disable_interrupts();
    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;
    __builtin_enable_interrupts();
    */

    __builtin_disable_interrupts();
    T4CONbits.TCKPS = 2; // Timer4 prescaler N=4
    PR4 = 23999; // 48000000 Hz / 500 Hz / 4 - 1 = 23999 (500Hz from 48MHz clock with 4:1 prescaler)
    TMR4 = 0; // initial TMR4 count is 0
    T4CONbits.ON = 1;
    IPC4bits.T4IP = 4; // priority for Timer 4 
    IFS0bits.T4IF = 0; // clear interrupt flag for Timer4
    IEC0bits.T4IE = 1; // enable interrupt for Timer4
    
    // do your TRIS and LAT commands here
    TRISAbits.TRISA4 = 0;   // A4 = output (LED))
    LATAbits.LATA4 = 1;     // A4 = high
    TRISBbits.TRISB4 = 1;   // B4 = input (push button)
    
    __builtin_enable_interrupts();
    
    initMotorContorl();
    initEncoder();
    
}

initMotorControl(void){
    RPA0Rbits.RPA0R = 0b0101; // A0 to OC1
    RPB13Rbits.RPB13R = 0b0101; // B13 to OC4
    T2CONbits.TCKPS = 0; // Timer2 prescaler N=1 (1:1)
    
    PR2 = 2399; // 48000000 Hz / 20000 Hz / 1 - 1 = 2399 (20kHz PWM from 48MHz clock with 1:1 prescaler)
    TMR2 = 0; // initial TMR2 count is 0
    OC1CONbits.OCM = 0b110; // PWM mode without fault pin; other OCxCON bits are defaults
    OC1RS = 0; // duty cycle
    OC1R = 0; // initialize before turning OC1 on; afterward it is read-only
    OC4CONbits.OCM = 0b110; // PWM mode without fault pin; other OCxCON bits are defaults
    OC4RS = 0; // duty cycle
    OC4R = 0; // initialize before turning OC4 on; afterward it is read-only
    T2CONbits.ON = 1; // turn on Timer2
    OC1CONbits.ON = 1; // turn on OC1
    OC4CONbits.ON = 1; // turn on OC4
}

initEncoder(void){
    T5CKbits.T5CKR = 0b0100; // B9 is read by T5CK
    T3CKbits.T3CKR = 0b0100; // B8 is read by T3CK
    
    T5CONbits.TCS = 1; // count external pulses
    PR5 = 0xFFFF; // enable counting to max value of 2^16 - 1
    TMR5 = 0; // set the timer count to zero
    T5CONbits.ON = 1; // turn Timer on and start counting
    T3CONbits.TCS = 1; // count external pulses
    PR3 = 0xFFFF; // enable counting to max value of 2^16 - 1
    TMR3 = 0; // set the timer count to zero
    T3CONbits.ON = 1; // turn Timer on and start counting
}