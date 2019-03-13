/*
 * @file
 * @author Ali Seifeldin
 * 
 * Created on Jan 10, 2019, 5:40 PM

 */

#include <stdio.h>
#include <xc.h>
#include <math.h>
#include "configBits.h"
#include "lcd.h"
#include "I2C.h"


const char keys[] = "123A456B789C*0#D";

volatile bool key_was_pressed = false;
volatile bool exit_key = false;
volatile bool start = false;
unsigned char keypress;


// Universal Data

int time = 0;
int Canister = 0;
int balls = 0;
int State[10] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1}; // Final Error -1 no info, 0 empty, 1 ball
int DistanceCanister[10] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1}; // -1 No info, int distance in cm
int BallDispensed[10] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1}; // -1 No info, 0 No ball dispensed, 1 Ball dispensed
    

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
    
    // Setup pins 

    TRISCbits.RC0 = 0; // KPD
    LATCbits.LATC0 = 0; // Enable Keypad   
    
    TRISCbits.RC2 = 1; // echo
    TRISCbits.RC1 = 0; // trigger

    TRISBbits.TRISB2 = 0; // Left motor
    TRISBbits.TRISB3 = 0; // Right motor
    LATBbits.LATB2 = 0;
    LATBbits.LATB3 = 0;

    TRISBbits.RB0 = 0; // Microswitch 
    LATBbits.LATB0 = 0;

    lcd_display_control(true, false, false);
    printf("Press A");
    lcd_set_ddram_addr(LCD_LINE2_ADDR);
    printf("to start");
    lcd_set_ddram_addr(LCD_LINE4_ADDR);
    printf("A for Ali ;)");
    
    while (!start) {continue; }
    
    
    LATCbits.LATC0 = 1; // Disable Keypad
    
    TRISBbits.RB1 = 0; // motor 
    LATBbits.LATB1 = 0;

    TRISAbits.RA5 = 0; // Light
    LATAbits.LATA5 = 0;

    lcd_clear();
    
    
    int a; // distance from ultrasonic sensor
    int tick = 0; // wheel spin time
    int time = 0; // if ultrasonic takes too long


    while(1){
        
        while (tick < 100) {
            LATBbits.LATB2 = 1 ; 
            LATBbits.LATB3 = 1 ; 

            __delay_us(1235);
            LATBbits.LATB2 = 0 ;
            __delay_us(1400);

            LATBbits.LATB3 = 0 ; 

            __delay_us(1700);

        tick++;
        }
        
        tick = 0;
        
        lcd_clear();
        printf("Distance %d" , a);

        
        TMR1H = 0;                //Sets the Initial Value of Timer
        TMR1L = 0;                //Sets the Initial Value of Timer

        LATCbits.LATC1 = 1;                  //TRIGGER HIGH
        __delay_us(10);           //10uS Delay 
        LATCbits.LATC1 = 0;                  //TRIGGER LOW


        
        while(!PORTCbits.RC2);              //Waiting for Echo
        TMR1ON = 1;  
        //Timer Starts
        time = 0;
        while(PORTCbits.RC2 == 1 && time < 1000)               //Waiting for Echo goes LOW
        {
            time ++;            
        }
            
        TMR1ON = 0;               //Timer Stops
        
        a = (TMR1L | (TMR1H<<8)); //Reads Timer Value
        a = a/155;              //Converts Time to Distance 155 came through calibration
    
        if (a < 10 && a > 0) {  // a < 10 && a > 0
            
            LATAbits.LATA5 = 1; // light

            lcd_set_ddram_addr(LCD_LINE4_ADDR);
  
            while (!PORTBbits.RB0){          // not switch                         
            
                LATBbits.LATB1 = 1;  // motor    
                __delay_ms(30);
                LATBbits.LATB1 = 0;
                 __delay_ms(50);

                
            }
            LATAbits.LATA5 = 0; // light
            LATBbits.LATB1 = 0;  //motor


        }
        
    }
    
    
    
    // Actual running code
    // 1 means yes, 0 means no
    int running = 1;
    int end = 0;
    int forward = 1;
    int Distance = 0;
    int Sensor1Now;
    int Sensor1Before;
    int Sensor2;
    int tempCan;
    while(running){
        
        // Make servo go forward
        
        //DetectState
        if (Distance - DistanceCanister[Canister] > 20 && forward){
        
            if (abs(Sensor1Now - Sensor1Before) > 10 && Sensor1Now < 30){
                State[Canister] = 1;
                DistanceCanister[Canister] = Distance;
                BallDispensed[Canister] = 0;
                Canister +=1;
            }
            else if (abs(Sensor1Now - Sensor1Before) > 5 && Sensor1Now < 30 & abs(Distance-DistanceCanister[Canister]) > 30 ){
                State[Canister] = 1;
                DistanceCanister[Canister] = Distance;
                BallDispensed[Canister] = 1;
                Canister +=1;
            }
            
            else if (abs(Sensor1Now - Sensor1Before) > 5 && Sensor1Now < 30 ){
                State[Canister] = 0;
                DistanceCanister[Canister] = Distance;
                BallDispensed[Canister] = 0;
                Canister +=1;
                
            }
            else if (abs(Sensor1Now - Sensor1Before) < 1 && Sensor1Now < 30){
                State[Canister] = 0;
                DistanceCanister[Canister] = Distance;
                BallDispensed[Canister] = 0;
                Canister +=1;
                
            }
            tempCan = Canister;
        }
        
        if (!forward){
            if (abs(Sensor1Now - Sensor1Before) > 10 && Sensor1Now < 30 && !State[tempCan]){
                State[tempCan] = 1;
                BallDispensed[tempCan] = 0;
                tempCan -=1;
            }
            else if (abs(Sensor1Now - Sensor1Before) > 5 && Sensor1Now < 30 && abs(Distance-DistanceCanister[tempCan]) > 30 ){
                State[tempCan] = 1;
                BallDispensed[tempCan] = 1;
                tempCan -=1;
            }
            
            else if (abs(Sensor1Now - Sensor1Before) > 5 && Sensor1Now < 30 ){
                State[tempCan] = 0;
                BallDispensed[tempCan] = 0;
                tempCan -=1;
            }
            
            else if (abs(Sensor1Now - Sensor1Before) < 1 && Sensor1Now < 30  ){
                State[tempCan] = 0;
                BallDispensed[tempCan] = 0;
                tempCan -=1;
                
            }
            
            else {tempCan -= 1;}
        }
        

        
        // Calculate distance moved using the Gyro, increment distance 
        if (forward){
            Distance += 1 ;// Use integral of time and gyroscope acceleration to find distance once gyroscope works
        }
        else{
            Distance -= 1 ;// Use integral of time and gyroscope acceleration to find distance once gyroscope works
        }
        if (Distance > 420) {
            // make the robot turn right 
            // move forward 30 cm
            // turn right again
            // all while checking the angular velocity and making sure the are 90 degree angles. 
            
        }
                
        if (Distance < 0 && !forward){
            running = 0;
        }
    }
    
    
    
    int state = 0; // Status of GUI screen
    tick = 0;
    int clear = 1; // 1 to clear, 0 to not
    
    start = false;
    
    lcd_clear();
    printf("Press A to");
    lcd_set_ddram_addr(LCD_LINE3_ADDR);
    printf("view results");

    while (!start) {continue; }

    
    lcd_clear();
    printf("Canister Index");
    lcd_set_ddram_addr(LCD_LINE3_ADDR);
    printf("starts at 0");
    __delay_ms(4000);
    
    key_was_pressed = true;
    exit_key = true;
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
        
        else if (state == 2 & clear == 1) {

            lcd_clear();
            printf("# of Balls");
            lcd_set_ddram_addr(LCD_LINE2_ADDR);
            printf("supplied"); 
            lcd_set_ddram_addr(LCD_LINE4_ADDR);
            printf("%d balls", balls);
            clear = 0;
            
        }
        
        else if (state == 3 & clear == 1) {
            
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



void EEPROM_WriteByte(unsigned char eepromAddress, unsigned char eepromData) {
    
unsigned char gie_Status;
while(EECON1bits.WR){};            // check the WR bit to see if a previous Write operation is in progress
    EEADR=eepromAddress;  // Write the address to EEADR.
    EEDATA=eepromData;    // load the 8-bit data value to be written in the EEDATA register.
    WREN=1;               // Set the WREN bit to enable eeprom operation.
    gie_Status = GIE;     // Copy the current Interrupt state
    GIE = 0;              // Disable the interrupts
    EECON2=0x55;          // Execute the special instruction sequence
    EECON2=0xaa;          // Refer the datasheet for more info
    EECON1bits.WR=1;                 // Set the WR bit to trigger the eeprom write operation.
    GIE = gie_Status;     // Restore the interrupts
    WREN=0;               // Disable the EepromWrite
    
} 

unsigned char EEPROM_ReadByte(unsigned char eepromAddress) {
    
    while(EECON1bits.RD || EECON1bits.WR);           // check the WR&RD bit to see if a RD/WR is in progress
    EEADR=eepromAddress;       // Write the address to EEADR.
    EECON1bits.RD = 1;                    // Set the RD bit to trigger the eeprom read operation.
    return(EEDATA);            // Return the data read form eeprom.
    
}

void __interrupt() interruptHandler(void){
    // Interrupt on change handler for RB1
    
    if(INT1IF){
        // Notice how we keep the interrupt processing very short by simply
        // setting a "flag" which the main program loop checks
        keypress = (PORTB & 0xF0) >> 4;
        INT1IF = 0; // Clear interrupt flag bit to signify it's been handled
        
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