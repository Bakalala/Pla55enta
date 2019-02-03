/**
 * @file
 * @author Tyler Gamvrelis
 * 
 * Created on August 12, 2017, 5:40 PM
 * 
 * @defgroup CharacterLCD_2
 * @brief Demonstrates more character LCD capabilities such as display shifting
 *        and moving the cursor to a specific DDRAM address
 * 
 * Precondition:
 * @pre Character LCD is in a PIC socket
 */

#include <stdio.h>
#include <xc.h>
#include <math.h>
#include "configBits.h"
#include "lcd.h"

const char keys[] = "123A456B789C*0#D";

volatile bool key_was_pressed = false;
volatile bool exit_key = false;
volatile bool start = false;


// Universal Data

int time = 30;
int Canister = 8;
int balls = 5;
int State[10] = {1,1,1,0,0,1,1,0,-1,-1}; // Final Error -1 no info, 0 empty, 1 ball
int DistanceCanister[10] = {20,30,40,59,123,212,332,400,-1,-1}; // -1 No info, int distance in cm
int BallDispensed[10] = {1,1,1,0,0,1,1,0,-1,-1}; // -1 No info, 0 No ball dispensed, 1 Ball dispensed
    
void main(void) {
    // RD2 is the character LCD RS
    // RD3 is the character LCD enable (E)
    // RD4-RD7 are character LCD data lines
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
    
    
    int state = 0; // Status of GUI screen
    int tick = 0;
    int clear = 1; // 1 to clear, 0 to not
    
    
    // Wait to start 
    
    lcd_display_control(true, false, false);
    lcd_clear();
    printf("A to start");
    lcd_set_ddram_addr(LCD_LINE3_ADDR);
    printf("A for Ali ;)");
    while (!start) {continue; }
    
    // Entry to Gui
    lcd_clear();
    printf("It's ya boy Ali!");
    __delay_ms(4000);   
    lcd_set_ddram_addr(LCD_LINE3_ADDR);
    printf("He gettin hot ");
    __delay_ms(4000);

    lcd_clear();
    printf("Hi Cull !");
    __delay_ms(4000);   
    lcd_set_ddram_addr(LCD_LINE3_ADDR);
    printf("Almost done");
    lcd_set_ddram_addr(LCD_LINE4_ADDR);
    printf("AND DONE");
    __delay_ms(4000);

    
    
    
    
    // Finish loop
    while(1){
        // Different GUI menu
        
        if (state == 0 & clear == 1) {
        lcd_clear();
        printf("Operation Time");
        lcd_set_ddram_addr(LCD_LINE3_ADDR);
        printf("%d seconds", time);
        clear = 0;
        
        }

        if (state == 1 & clear == 1) {
        
            lcd_clear();
            lcd_set_ddram_addr(LCD_LINE1_ADDR);
            printf("# of Canisters");
            lcd_set_ddram_addr(LCD_LINE3_ADDR);
            printf("%d Canisters", Canister);
            clear = 0;
        
        }
        
        if (state == 2 & clear == 1) {

            lcd_clear();
            printf("# of Balls");
            lcd_set_ddram_addr(LCD_LINE2_ADDR);
            printf("supplied"); 
            lcd_set_ddram_addr(LCD_LINE4_ADDR);
            printf("%d balls", balls);
            clear = 0;
            
        }
        
        if (state == 3 & clear == 1) {
            
            lcd_clear();
            printf("Individual Info");
            lcd_set_ddram_addr(LCD_LINE2_ADDR);
            printf("of Canister");
            lcd_set_ddram_addr(LCD_LINE4_ADDR);
            printf("Press 0 to %d ", Canister-1);
            clear = 0;
       
        }
        
        if(key_was_pressed){
            
            unsigned char keypress = (PORTB & 0xF0) >> 4;
            int miniTick = 0;
            int miniState = 0;
            int miniClear = 1;
            
            int number_pressed = (int) (keys[keypress] - '0');

            while(!exit_key) {  
            
                if (miniState == 0 & miniClear == 1) {
            
                    lcd_clear();
                    printf("Cansiter %c", keys[keypress]);
                    lcd_set_ddram_addr(LCD_LINE3_ADDR);
                    printf("Distance %d cm", DistanceCanister[number_pressed] );
                    lcd_set_ddram_addr(LCD_LINE4_ADDR);             
                    printf("* to exit");                    
                    miniClear = 0;
                }
                
                if (miniState == 1 & miniClear == 1) {
            
                    lcd_clear();
                    printf("Cansiter %c", keys[keypress]);
                    lcd_set_ddram_addr(LCD_LINE3_ADDR);
                    if (State[number_pressed] == 1)
                        printf("Canister Full");
                    else
                        printf("Canister Empty");
                    lcd_set_ddram_addr(LCD_LINE4_ADDR);                    
                    printf("* to exit");                    
                    miniClear = 0;
                }
                
                if (miniState == 2 & miniClear == 1) {
            
                    lcd_clear();
                    printf("Cansiter %c", keys[keypress]);
                    lcd_set_ddram_addr(LCD_LINE3_ADDR);
                    if (BallDispensed[number_pressed] == 1)
                        printf("Ball Added");
                    else
                        printf("No Ball Added"); 
                    lcd_set_ddram_addr(LCD_LINE4_ADDR);
                    printf("* to exit");


                    miniClear = 0;
                }
                
                if (miniTick == 3000) {
                    miniClear = 1;
                    miniState++;
                    miniState = miniState % 3; //make sure status is between 0 and 3
                    miniTick = 0;
                }
    
                miniTick++;
                __delay_ms(1);
            
            
            }
            
            key_was_pressed = false; // Clear the flag
            exit_key = false;
            
        }
        
        if (tick == 2000) {
            clear = 1;
            state++;
            state = state % 4; //make sure status is between 0 and 3
            tick = 0;
        }
    
        tick++;
        __delay_ms(1);

        
    }
}



void __interrupt() interruptHandler(void){
    // Interrupt on change handler for RB1
    if(INT1IF){
        // Notice how we keep the interrupt processing very short by simply
        // setting a "flag" which the main program loop checks

        INT1IF = 0; // Clear interrupt flag bit to signify it's been handled

        unsigned char keypress = (PORTB & 0xF0) >> 4;
        
        if (keys[keypress] == 'A') {
            
            start = true;
            return;
        }

        if (keys[keypress] == '*') {
            
            exit_key = true;
            return;
        }
        
        for ( int i = 0; i < Canister; i++ ) {
            
            if ((char)i + '0' == keys[keypress]) {
                key_was_pressed = true;
                return;
            }

        }
                   
        
    }
}
