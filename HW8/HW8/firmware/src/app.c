/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "ST7735.h"
#include "i2c_master_noint.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
*/

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
*/

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
*/

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

// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize ( void )
{
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    
    TRISAbits.TRISA4 = 0;
    LATAbits.LATA4 = 1;
    TRISBbits.TRISB4 = 1;
    
    initIMU();
    
    
    LCD_init();
    LCD_clearScreen(BLACK);
    
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
    
    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}


/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks ( void )
{

    /* Check the application's current state. */
    switch ( appData.state )
    {
        /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;
       
        
            if (appInitialized)
            {
                appData.state = APP_STATE_SERVICE_TASKS;             
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
            /*
            if(PORTBbits.RB4 == 0){
                    LATAbits.LATA4 = 0;
            }
            else if(PORTBbits.RB4 == 1){
                _CP0_SET_COUNT(0);
                while(_CP0_GET_COUNT() < 12000){ ; }
                LATAINV = 0b10000;
            }
            */
            
            unsigned char address = 0b11010110;     // device opcode with write bit 0
            unsigned char reg = 0x20;               // OUT_TEMP_L
            int length = 14;                        // array length
            unsigned char data[length];             // storage array
            signed short temperature, gyroX, gyroY, gyroZ, accelX, accelY, accelZ;
            short endX, endY;
            char statusX[20], statusY[20];
            int x,y,i,j = 0;
            
            if(PORTBbits.RB4 == 0){
                LATAbits.LATA4 = 0;
            }
            else if(PORTBbits.RB4 == 1){
                _CP0_SET_COUNT(0);
            
                I2C_read_multiple(address, reg, data, length);
            
                temperature = (data[1] << 8) | (data[0] | 0b0000000000000000);  // if not, try (data[1] | 0b0000000000000000)
                gyroX = (data[3] << 8) | (data[2] | 0b0000000000000000);
                gyroY = (data[5] << 8) | (data[4] | 0b0000000000000000);
                gyroZ = (data[7] << 8) | (data[6] | 0b0000000000000000);
                accelX = (data[9] << 8) | (data[8] | 0b0000000000000000);
                accelY = (data[11] << 8) | (data[10] | 0b0000000000000000);
                accelZ = (data[13] << 8) | (data[12] | 0b0000000000000000);
                
                /*
                sprintf(statusX,"x = %hi",accelX);
                sprintf(statusY,"y = %hi",accelY);
            
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
                */
            
                endX = 64 + (accelX/50);
                endY = 80 + (accelY/100);
            
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
                    }
                }
            
                for(x = 61; x <= 67; x++){
                    for(y = 77; y <= 83; y++){
                        LCD_drawPixel(x,y, WHITE);
                    }
                }
            
                while(_CP0_GET_COUNT() < 2400000){ ; }  // 20 Hz
                LATAINV = 0b10000;
            }
            
            break;
        }

        /* TODO: implement your application state machine.*/
        

        /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}

 

/*******************************************************************************
 End of File
 */
