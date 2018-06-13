#include <stdio.h>
#include <stdlib.h>
#include <xc.h>
#include "ST7735.h"
#include "i2c_master_noint.h"

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

void initIMU(void);
void I2C_read_multiple(unsigned char address, unsigned char reg, unsigned char * data, int length);
void printLetter(char letter, unsigned short x, unsigned short y, unsigned short print, unsigned short background);


int main(void){
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
    TRISAbits.TRISA4 = 0;               // set LED pin to output
    LATAbits.LATA4 = 1;                 // turning on LED
    TRISBbits.TRISB4 = 1;               // set pushbutton pin to input

    __builtin_enable_interrupts();

    initIMU();
    unsigned char address = 0b11010110;     // device opcode with write bit 0
    unsigned char reg = 0x20;               // OUT_TEMP_L
    int length = 14;                        // array length
    unsigned char data[length];             // storage array
    signed short temperature, gyroX, gyroY, gyroZ, accelX, accelY, accelZ;
    short endX, endY;
    char statusX[20], statusY[20];
    
    LCD_init();
    LCD_clearScreen(BLACK);
    
    //WHO_AM_I check
    
    i2c_master_start();             // check WHO_AM_I
    i2c_master_send(0b11010110);    // write bit 0
    i2c_master_send(0x0F);          // WHO_AM_I register
    i2c_master_restart();
    i2c_master_send(0b11010111);    // read bit 1
    char read = i2c_master_recv();  // receive from slave
    i2c_master_stop();
    
    if(read != 0b01101001){
        LATAbits.LATA4 = 0;
        while(1){ ; }               // black LCD if not done correctly
    }
    
    int x,y,i,j = 0;
    for(x = 1; x <= 128; x++){
        for(y = 77; y <= 83; y++){
                LCD_drawPixel(x,y, BLUE);
        }
    }
    
    for(x = 61; x <= 67; x++){
        for(y = 1; y <= 160; y++){
                LCD_drawPixel(x,y, BLUE);
        }
    }
    
    while(1) {
	// use _CP0_SET_COUNT(0) and _CP0_GET_COUNT() to test the PIC timing
	// remember the core timer runs at half the sysclk
        if(PORTBbits.RB4 == 0){
            LATAbits.LATA4 = 0;
        }
        else if(PORTBbits.RB4 == 1){
            _CP0_SET_COUNT(0);
            
            I2C_read_multiple(address, reg, data, length);
            
            temperature = (data[0] << 8) | (data[1] | 0b0000000000000000);  // if not, try (data[1] | 0b0000000000000000)
            gyroX = (data[2] << 8) | (data[3] | 0b0000000000000000);
            gyroY = (data[4] << 8) | (data[5] | 0b0000000000000000);
            gyroZ = (data[6] << 8) | (data[7]);
            accelX = (data[8] << 8) | (data[9]);
            accelY = (data[10] << 8) | (data[11]);
            accelZ = (data[12] << 8) | (data[13]);
            
            sprintf(statusX,"accelX = %hi",accelX);
            sprintf(statusY,"accelY = %hi",accelY);
            
            while(statusX[j]){                     // write word out
                printLetter(statusX[j], (5 + 5*j), 5, WHITE, BLACK);
                j++;
            }
            
            j = 0;
            
            while(statusY[j]){                     // write word out
                printLetter(statusY[j], (5 + 5*j), 12, WHITE, BLACK);
                j++;
            }
            
            j = 0;
            
            /*
            endX = 64 + gyroX;
            endY = 80 + gyroY;
            
            for(x = 61; x <= 67; x++){
                for(y = 1; y <= 160; y++){
                    if((y < endY && y > 80) || (y > endY && y < 80)){
                        LCD_drawPixel(x,y, WHITE);
                    }
                    else{
                        LCD_drawPixel(x,y, BLUE);
                    }
                    
                }
            }
            
            for(y = 77; y <= 83; y++){
                for(x = 1; x <= 128; x++){
                    if((x < endX && x > 64) || (x > endX && x < 64)){
                        LCD_drawPixel(x,y, WHITE);
                    }
                    else{
                        LCD_drawPixel(x,y, BLUE);
                    }
                    LCD_drawPixel(x,y, BLUE);
                }
            }
            */
            
            while(_CP0_GET_COUNT() < 2400000){ ; }  // 20 Hz
            LATAINV = 0b10000;
        }
    }
    
    return 0;
}

void initIMU(void){
    ANSELBbits.ANSB2 = 0;       // SDA2 pin is digital
    ANSELBbits.ANSB3 = 0;       // SCL2 pin is digital
    i2c_master_setup();
    
    i2c_master_start();
    i2c_master_send(0b11010110);    // IMU address, write bit 0
    i2c_master_send(0x10);          // CTRL1_XL address
    i2c_master_send(0b10000010);    // info bit
    i2c_master_stop();
    
    i2c_master_start();
    i2c_master_send(0b11010110);    // IMU address, write bit 0
    i2c_master_send(0x11);          // CTRL2_G address
    i2c_master_send(0b10001000);    // info bit
    i2c_master_stop();
    
    i2c_master_start();
    i2c_master_send(0b11010110);    // IMU address, write bit 0
    i2c_master_send(0x12);          // CTRL3_C address
    i2c_master_send(0b00000100);    // info bit
    i2c_master_stop();
    
    i2c_master_start();
    i2c_master_send(0b11010110);    // IMU address, write bit 0
    i2c_master_send(0x13);          // CTRL4_C address
    i2c_master_send(0b10000000);    // info bit
    i2c_master_stop();   
}

void I2C_read_multiple(unsigned char address, unsigned char reg, unsigned char * data, int length){
    int i;
    
    // add the address with write bit 0, then when reading, OR it with 0b00000001
    
    i2c_master_start();
    i2c_master_send(address);           // write bit 0
    i2c_master_send(reg);               // OUT_TEMP_L
    i2c_master_restart();
    i2c_master_send(address | 0x01);    // read bit 1
    
    for(i = 0; i <= (length-1); i++){
        
        data[i] = i2c_master_recv();  // also can try *(data + 1)
        
        if(i < (length-1)){
            i2c_master_ack(0);
        }
        else{
            i2c_master_ack(1);
            i2c_master_stop();
        }
    }
}

void printLetter(char letter, unsigned short x, unsigned short y, unsigned short print, unsigned short background){ // add draw color, background color
    if(x >= 124 || y >= 156){ ; }
    else{
        char column, pixel;
        short i,j;
        
        for(i = 0; i <= 4; i++){
            char column = ASCII[letter - 32][i];
            
            for(j = 0; j <= 7; j++){
                pixel = ((column >> j) & 0x01);
                
                if(pixel){
                    LCD_drawPixel(x+i, y+j, print);
                }
                else if(!pixel){
                    LCD_drawPixel(x+i, y+j, background);
                }
            }
        }
    }
}