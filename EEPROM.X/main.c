/*
 * File:   EEPROM.c
 * Author: aliseifeldin
 *
 * Created on March 9, 2019, 2:49 PM
 */


#include <stdio.h>
#include <xc.h>
#include <lcd.h>
#include <math.h>
#include "configBits.h"

const char keys[] = "123A456B789C*0#D";

unsigned char keypress;
volatile bool start = false;

int time = 12;
int Canister = 2;
int balls = 5;
int State[10] = {2,2,2,2,2,2,2,2,2,2}; // Final Error 2 no info, 0 empty, 1 ball
int DistanceCanister[10] = {0,0,0,0,0,0,0,0,0,0}; // 0 No info, int distance in cm
int BallDispensed[10] = {2,2,2,2,2,2,2,2,2,2}; // 2 No info, 0 No ball dispensed, 1 Ball dispensed

int EEPROM_ReadByte(int eepromAddress);
void EEPROM_WriteByte(int eepromAddress, int eepromData);


void main(void) {
    
    LATD = 0x00;
    TRISD = 0x00;
    
    // Set all A/D ports to digital (pg. 222)
    ADCON1 = 0b00001111;
    
    // Enable RB1 (keypad data available) interrupt
    INT1IE = 1;
    
    // Initialize LCD
    initLCD();
    
    // Enable interrupts
    ei();
    

    int x;
    
    if (EEPROM_ReadByte(0) < 32 && EEPROM_ReadByte(0) > -1)
         x = EEPROM_ReadByte(0);
    else {
        EEPROM_WriteByte(0,0);
        x = 0;
    }
    if (x == 31) {
        
        EEPROM_WriteByte(0,0);
        x = 0;
        
    }

    x++;
    
    int shift = x * 33;


    EEPROM_WriteByte((shift + 1),time);
    EEPROM_WriteByte((shift + 2),Canister);
    EEPROM_WriteByte((shift + 3),balls);
    
    
    int temp;
    for (int loop = 0; loop < 10; loop++){
        temp = shift + 4 + loop;
        EEPROM_WriteByte(temp , State[loop]);
    }

    for (int loop = 0; loop < 10; loop++){
        temp = shift + 14 + loop;
        EEPROM_WriteByte(temp, DistanceCanister[loop]);
    }
    
    for (int loop = 0; loop < 10; loop++){     
        temp = shift + 24 + loop;
        EEPROM_WriteByte(temp, BallDispensed[loop]);
    }
    
    EEPROM_WriteByte(0,x);
   
    lcd_clear();
    printf("%d", x);
    lcd_set_ddram_addr(LCD_LINE2_ADDR);
    printf("%d", shift);
    lcd_set_ddram_addr(LCD_LINE3_ADDR);
    printf("%d this is life", EEPROM_ReadByte(3));
    
    __delay_ms(10000);
    
    while(1);
    

    


          
}  


void __interrupt() interruptHandler(void){
    // Interrupt on change handler for RB1
    
    if(INT1IF){
        // Notice how we keep the interrupt processing very short by simply
        // setting a "flag" which the main program loop checks
        keypress = (PORTB & 0xF0) >> 4;
        INT1IF = 0; // Clear interrupt flag bit to signify it's been handled
        start = ~start;
                  
    }
}


void EEPROM_WriteByte(int eepromAddress, int eepromData) {
    
unsigned char gie_Status;
while(EECON1bits.WR){};            // check the WR bit to see if a previous Write operation is in progress
    EEADR= eepromAddress;  // Write the address to EEADR.
    EEDATA=eepromData;    // load the 8-bit data value to be written in the EEDATA register.
    EECON1bits.EEPGD=0;
    EECON1bits.CFGS = 0;
    EECON1bits.WREN=1;               // Set the WREN bit to enable eeprom operation.
    EECON2=0x55;        // Execute the special instruction sequence
    EECON2=0xaa;          // Refer the datasheet for more info
    EECON1bits.WR=1;                 // Set the WR bit to trigger the eeprom write operation.
    EECON1bits.WREN=0;               // Disable the EepromWrite
    
} 

int EEPROM_ReadByte(int eepromAddress) {
    
    while(EECON1bits.RD || EECON1bits.WR);           // check the WR&RD bit to see if a RD/WR is in progress
    EEADR=eepromAddress;       // Write the address to EEADR.
    EECON1bits.EEPGD=0;
    EECON1bits.RD = 1;                    // Set the RD bit to trigger the eeprom read operation.
    return(EEDATA);            // Return the data read form eeprom.
    
}
