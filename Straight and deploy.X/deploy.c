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

void forward(void);
void left(void);
void right(void);
int frontsensor(void);
int backsensor(void);
float readADC(char channel);


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
    
    TRISCbits.RC7 = 0; // motor 
    LATCbits.LATC7 = 0;
    
    //front
    TRISCbits.RC2 = 1; // echo
    TRISCbits.RC1 = 0; // trigger
    
    //back
    TRISAbits.RA4 = 1; // echo
    TRISAbits.RA5 = 0; // trigger
    
    TRISBbits.RB0 = 0; // Microswitch 
    LATBbits.LATB0 = 0;
    
    TRISCbits.RC6 = 0; // Light
    LATCbits.LATC6 = 0;
    
    TRISAbits.RA2 = 1; // encoders
    TRISAbits.RA3 = 1; // encoders
    
    
    
    int prev = 0;
    int prev2 = 0;
    
    int front;
    int back;
    
    int count = 0;
    //int x;
    float circ = 8.8 * M_PI;
    
    // Set RA0-3 to analog mode (pg. 222)
    ADCON1 = 0b00001101;
    // Right justify A/D result
    ADCON2bits.ADFM = 1;
    
        
    float x = 2;
    float Distance = 0;
    int statleft = 0;
    int statright = 0;

     //Calibrate wheels
//    start = true;
//    while(start){
//        lcd_clear()
//        printf("RA2: %.3f", readADC(2));
//        lcd_set_ddram_addr(LCD_LINE2_ADDR);
//        printf("RA3: %.3f", readADC(3));
//        __delay_ms(200);
//    }
//    __delay_ms(1000);
//    start = false;
//    

    while (readADC(2) < x && readADC(2) < x){
        if (readADC(2) < x) {
            
            LATBbits.LATB2 = 1 ; 
            __delay_us(1200);
            LATBbits.LATB2 = 0 ;
            __delay_us(600);
            __delay_us(1000);
        }
        if (readADC(3) < x) {
            
            LATBbits.LATB3 = 1 ; 
            __delay_us(1200);
            __delay_us(600);
            LATBbits.LATB3 = 0 ; 
            __delay_us(1000);
        }
    }

    
    int lefty = 0;
    int righty = 0;
    int countl = 0;
    int countr = 0;
    
    lcd_clear()
    printf("Press to start");
    
    while(1) {
        
        front = frontsensor();
        back = backsensor();
        
        if (start){
            

            
            if (lefty == righty) {
                forward();
            }
            else if (righty > lefty ){
                left();
            }
            else {
                right();
            }
                
            
            if (readADC(2) > x && statright == 0){
                statright = 1;
                righty++;
                Distance += (circ / 12.0);
                lcd_clear();
                printf("Dist: %.3f", Distance);
                
            }
            else if (readADC(2) < 2 && statright == 1){
                statright = 0;

            }
            
            if (readADC(3) > x && statleft == 0){
                statleft = 1;
                lefty++;
                Distance += (circ / 12.0);
                lcd_clear();
                printf("Dist: %.3f", Distance);

            }
            else if (readADC(3) < 2 && statleft == 1){
                statleft = 0;
                

            }
            
//            if (front > 40 && front < 70 && count == 0){
//                
//                righty--;
//            }

        
            if (prev2 < 10 && prev < 10 && front < 10 && count == 0 ) {
            
                lcd_clear();
                lcd_set_ddram_addr(LCD_LINE3_ADDR);
                printf("closed");
                lcd_set_ddram_addr(LCD_LINE4_ADDR);
                printf("f:%d,p:%d,p2:%d", front, prev, prev2);
                        
                __delay_ms(3000);            

                count++;
                
            } 
            
            else if (prev2 < 30 && prev < 30 && front < 30 && count == 0) { // check if Canister is Open
                               
                LATCbits.LATC6 = 1; // light
                
                lcd_clear();
                lcd_set_ddram_addr(LCD_LINE3_ADDR);
                printf("open");
                lcd_set_ddram_addr(LCD_LINE4_ADDR);
                printf("f:%d,p:%d,p2:%d", front, prev, prev2);
                      
            
                while (!PORTBbits.RB0){          // not switch                         
            
                    LATCbits.LATC7 = 1;  // motor    
                    __delay_ms(10);
                    LATCbits.LATC7 = 0;
                    __delay_ms(55);

                
                }
                LATCbits.LATC6 = 0; // light
                LATCbits.LATC7 = 0;  //motor
            
            
                count++;
            
            }
            
           
        }
        

        lcd_clear();
        printf("Dist: %.3f", Distance);
        lcd_set_ddram_addr(LCD_LINE2_ADDR);
        printf("lefty: %d", lefty);
        lcd_set_ddram_addr(LCD_LINE3_ADDR);
        printf("right: %d", righty);
        lcd_set_ddram_addr(LCD_LINE4_ADDR);
        printf("Ultra: %d", frontsensor());

        prev2 = prev;
        prev = front;
      
        if (count == 60) {count = 0;}
        
        else if (count > 0) {count ++;}
    }
  
    return;
}

void forward(void){
    
    LATBbits.LATB2 = 1 ; 
    LATBbits.LATB3 = 1 ; 
    __delay_us(1300);
    LATBbits.LATB2 = 0 ;
    __delay_us(400);
    LATBbits.LATB3 = 0 ; 
    __delay_us(1000);
    
    return;
}

void left(void){ //B2 Faster
    
    LATBbits.LATB2 = 1 ; 
    __delay_us(1300);
    LATBbits.LATB2 = 0 ;
    __delay_us(1000);
    
    return;
}

void right(void){ // B3 Faster
    
    LATBbits.LATB3 = 1 ; 
    __delay_us(1650);
    LATBbits.LATB3 = 0 ; 
    __delay_us(1000);
    
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

int backsensor(void){
    
    int time = 0;
    int back;
    
    TMR1H = 0;                //Sets the Initial Value of Timer
    TMR1L = 0;                //Sets the Initial Value of Timer

    LATAbits.LATA5 = 1;                  //TRIGGER HIGH
    __delay_us(10);           //10uS Delay 
    LATAbits.LATA5 = 0;                  //TRIGGER LOW

    while(!PORTAbits.RA4);              //Waiting for Echo
    TMR1ON = 1;  
    //Timer Starts
    time = 0;

    while(PORTAbits.RA4 == 1 && time < 1000)  
    {
        time ++;            // if sensor takes too long to run
    }

    TMR1ON = 0;               //Timer Stops

    back = (TMR1L | (TMR1H<<8)); //Reads Timer Value
    back = back/130;              //Converts Time to Distance 155 came through calibration
    
    return back;
}

float readADC(char channel){
    ADCON0 = (channel & 0x0F) << 2; // Select ADC channel (i.e. pin)
    ADON = 1; // Enable module
    ADCON0bits.GO = 1; // Initiate sampling
    while(ADCON0bits.GO_NOT_DONE){
        continue; // Poll for acquisition completion
    }
    return ((ADRESH << 8) | ADRESL) *5/1023.0; // Return result as a 16-bit value
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
