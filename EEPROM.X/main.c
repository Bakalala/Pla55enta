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
    
    
    TRISAbits.RA5 = 0;
    TRISAbits.RA3 = 0;
    
    LATAbits.LATA3 = 0;

    
    while(1) {
    if (PORTAbits.RA3){
        lcd_clear();
        printf("pressed");
        LATAbits.LATA5 = 1; // light
        __delay_ms(100);
    } else {
        lcd_clear();
        printf("No");
        LATAbits.LATA5 = 0; // light
        __delay_ms(100);
        
    }
        
    }
}
