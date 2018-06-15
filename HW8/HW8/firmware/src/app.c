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
