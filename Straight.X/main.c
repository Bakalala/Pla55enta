/*
 * File:   main.c
 * Author: aliseifeldin
 *
 * Created on March 26, 2019, 9:46 PM
 */

#include <stdio.h>
#include <xc.h>
#include <lcd.h>
#include <math.h>
#include "configBits.h"

const char keys[] = "123A456B789C*0#D";

unsigned char keypress;
volatile bool start = false;


void main(void) {
    
    LATD = 0x00;
    TRISD = 0x00;
    ADCON1 = 0b00001111;
    INT1IE = 1;
    initLCD();
    ei();
    
    
    TRISBbits.TRISB2 = 0; // Left motor
    TRISBbits.TRISB3 = 0; // Right motor
    LATBbits.LATB2 = 0;
    LATBbits.LATB3 = 0;
    
    TRISAbits.RA3 = 0; // motor 
    LATAbits.LATA3 = 0;
    
    TRISCbits.RC2 = 1; // echo
    TRISCbits.RC1 = 0; // trigger
    
    TRISBbits.RB0 = 0; // Microswitch 
    LATBbits.LATB0 = 0;
    
    TRISAbits.RA5 = 0; // Light
    LATAbits.LATA5 = 0;
    
    int prev;
    int prev2;
    int front;
    int count;
    
    int x;
    
    while(1){
        
        front = frontsensor();
        
        if (start) {
     
        
            if (front > 50 && front < 80 ){
     
                
                for (x = 0; x < 6 ;x++){

                    LATBbits.LATB3 = 1 ; 

                    __delay_us(1200);
                    __delay_us(900);

                    LATBbits.LATB3 = 0 ; 

                    __delay_us(1700);
                }
                
            }
            
            while (front < 15){
                
                for (x = 0; x < 6; x++){
                
                    LATBbits.LATB2 = 1 ; 

                    __delay_us(1200);
                    LATBbits.LATB2 = 0 ;
                    __delay_us(900);

                    __delay_us(1700);
                
                }               
            }    
            
            else {
                
                LATBbits.LATB2 = 1 ; 
                LATBbits.LATB3 = 1 ; 

                __delay_us(1200);
                LATBbits.LATB2 = 0 ;
                __delay_us(900);

                LATBbits.LATB3 = 0 ; 

                __delay_us(1700);
            
            }
        }

    }
    
   
    
    return;
}

int frontsensor(void){
    
    int time = 0;
    int front;
    
    TMR1H = 0;                //Sets the Initial Value of Timer
    TMR1L = 0;                //Sets the Initial Value of Timer

    LATCbits.LATC1 = 1;                  //TRIGGER HIGH
    __delay_us(10);           //10uS Delay 
    LATCbits.LATC1 = 0;                  //TRIGGER LOW


    while(!PORTCbits.RC2);              //Waiting for Echo
    TMR1ON = 1;  
    time = 0;

    while(PORTCbits.RC2 == 1 && time < 1000)               //Waiting for Echo goes LOW
    {
        time ++;            // if sensor takes too long to run
    }

    TMR1ON = 0;               //Timer Stops

    front = (TMR1L | (TMR1H<<8)); //Reads Timer Value
    front = front/155;              //Converts Time to Distance 155 came through calibration
    
    return front;
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
