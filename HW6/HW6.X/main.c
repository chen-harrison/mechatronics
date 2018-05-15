#include <stdio.h>
#include <stdlib.h>
#include "ST7735.h"
#include <xc.h>

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

void printLetter(char letter, unsigned short x, unsigned short y, unsigned short print, unsigned short background);
void emptyBar(void);

int main(){
    int count, i, k, j = 0;
    float fps;
    char progress[20], speed[20];
    
    LCD_init();
    LCD_clearScreen(BLACK);
    emptyBar();
    
    while(1){
        emptyBar();     
        for(i = 1; i <= 100; i++){                  // 0 to 100 increment loop
            _CP0_SET_COUNT(0);
            
            sprintf(progress,"Hello world %d!  ",i);  // create corresponding text
            
            while(progress[j]){                     // write word out
                printLetter(progress[j], (28 + 5*j), 32, WHITE, BLACK);
                j++;
            }
            j = 0;                                  // set start point back
            
            for(k = 0; k <= 4; k++){
                LCD_drawPixel((14+i), (42+k), GREEN);
            }
            
            count = _CP0_GET_COUNT();
            fps = 24000000.0 / count;
            
            sprintf(speed,"FPS = %5.2f",fps);
            
            while(speed[j]){                        // write word out
                printLetter(speed[j], (15 + 5*j), 100, WHITE, BLACK);
                j++;
            }
            j = 0; 
            
            while(_CP0_GET_COUNT() < 2400000){ ; }
        }
    }

    return 0;
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

void emptyBar(void){
    int i,j;
    for(i = 1; i <= 100; i++){
        for(j = 0; j <= 4; j++){
                LCD_drawPixel((14+i), (42+j), WHITE);
        }
    }
}