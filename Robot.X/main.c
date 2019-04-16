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
volatile bool B_was_pressed = false;
volatile bool exit_key = false;
volatile bool start = false;
unsigned char keypress;
volatile long int timer = 0;


// Universal Data

int time = 0;
int Canister = 0;
int balls = 0;
int State[10] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1}; // Final Error -1 no info, 0 empty, 1 ball
int DistanceCanister[10] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1}; // -1 No info, int distance in cm
int BallDispensed[10] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1}; // -1 No info, 0 No ball dispensed, 1 Ball dispensed



void forward(void);
void backdrive(void);
void left(void);
void backleft(void);
void right(void);
void backright(void);
int frontsensor(void);
int backsensor(void);
float readADC(char channel);
void EEPROM_WriteByte(unsigned char eepromAddress, unsigned char eepromData);
unsigned char EEPROM_ReadByte(unsigned char eepromAddress);
void EEPROM_save(void);
void EEPROM_prev(int trial);

void main(void) {
    // RD2 is the character LCD RS
    // RD3 is the character LCD enable (E)
    // RD4-RD7 are character LCD datfrontlines
    LATD = 0x00;
    TRISD = 0x00;
    
    // Set RA0-3 to analog mode (pg. 222)
    ADCON1 = 0b00001101;
    // Right justify A/D result
    ADCON2bits.ADFM = 1;
    
    // Enable RB1 (keypad datfrontavailable) interrupt
    INT1IE = 1;
    
    // Initialize LCD
    initLCD();
    
    // Enable interrupts
    ei();
    
    // Initialize I2C Master with 100 kHz clock
    I2C_Master_Init(100000);
    
    //timer0 setup
    
    // Enable timer 2
    TMR0ON = 1;
    // Set to 16 bit counter
    T08BIT = 0;
    // Count at positive edge of OSC0 (2.5MHz)
    T0CS = 0;
    T0SE = 0;
    //No prescaler
    PSA = 1;
    
     //reset timer0 registers
    TMR0L = 0b00000000;
    TMR0H = 0b00000000;
    
    
    // Setup pins 
    
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
    
    
    float x = 1.5;
    float circ = 8.8 * M_PI;

    int lefty = 0; 
    int righty = 0;
    int countl = 0;
    int countr = 0;
    int statright = 0;
    int statleft = 0;
    int dispense = 0; // whether a ball needs to be dispensed or not


    int front= 0; // distance from ultrasonic sensor
    int back = 0; // back ultrasonic sensor
    int prev = 0; // previous distance
    int prev2 = 0; // second previous distance
    
    int compare = 0; // compare if ball should dispense or not 
    
    float Distance = 0; // Distance traveled by robot

    int tick = 0;
    
    int num;
    int count = 0;
    int now = 0;
    
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
            
    lcd_display_control(true, false, false);
    unsigned char clock[7]; // Create a byte array to hold time read from RTC

    while(!start){
        
        lcd_clear();
        printf("A to start");
        lcd_set_ddram_addr(LCD_LINE2_ADDR);
        
        // Reset RTC memory pointer
        I2C_Master_Start(); // Start condition
        I2C_Master_Write(0b11010000); // 7 bit RTC address + Write
        I2C_Master_Write(0x00); // Set memory pointer to seconds
        I2C_Master_Stop(); // Stop condition

        // Read current time
        I2C_Master_Start(); // Start condition
        I2C_Master_Write(0b11010001); // 7 bit RTC address + Read
        for(unsigned char j = 0; j < 6; j++){
            clock[j] = I2C_Master_Read(ACK); // Read with ACK to continue reading
        }
        clock[6] = I2C_Master_Read(NACK); // Final Read with NACK
        I2C_Master_Stop(); // Stop condition
        
        // Print received data on LCD
        lcd_set_ddram_addr(LCD_LINE2_ADDR);
        printf("%02x/%02x/%02x", clock[6],clock[5],clock[4]); // Print date in YY/MM/DD
        lcd_set_ddram_addr(LCD_LINE3_ADDR);
        printf("%02x:%02x:%02x", clock[2],clock[1],clock[0]); // HH:MM:SS
        __delay_ms(1000);
    }
    
    lcd_clear();
    TMR0IE = 1;
    
    
    while(timer*0.0262144 < 100 && Distance < 400 && Canister < 11) {
        
        __delay_ms(10);
        front = frontsensor();
        //back = backsensor();
                   

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
            else if (readADC(2) < 1 && statright == 1){
                statright = 0;

            }
            
            if (readADC(3) > x && statleft == 0){
                statleft = 1;
                lefty++;
                Distance += (circ / 12.0);
                lcd_clear();
                printf("Dist: %.3f", Distance);

            }
            else if (readADC(3) < 1 && statleft == 1){
                statleft = 0;
                

            }

        
            if (prev2 < 17 && prev < 17 && front < 17 && count == 0 ) {
            
                lcd_clear();
                lcd_set_ddram_addr(LCD_LINE3_ADDR);
                printf("closed");
                lcd_set_ddram_addr(LCD_LINE4_ADDR);
                printf("f:%d,p:%d,p2:%d", front, prev, prev2);
                LATCbits.LATC6 = 1; // light

                __delay_ms(2000);   
                LATCbits.LATC6 = 0; // light

                State[Canister] = 0;
                DistanceCanister[Canister] = Distance;
                BallDispensed[Canister] = 0;
                Canister ++;
                compare = Distance + 30;


                count++ ;
                
                
            } 
            
            else if (prev2 < 50 && prev < 50 && front < 50 && count == 0) { // check if Canister is Open
                               
                
                lcd_clear();
                lcd_set_ddram_addr(LCD_LINE3_ADDR);
                printf("open");
                lcd_set_ddram_addr(LCD_LINE4_ADDR);
                printf("f:%d,p:%d,p2:%d", front, prev, prev2);
                LATCbits.LATC6 = 1; // light
      
            
                if (compare < Distance ){
                    
                    dispense = 1;
                                     

                    State[Canister] = 1;
                    DistanceCanister[Canister] = Distance;
                    BallDispensed[Canister] = 1;
                    Canister ++;
                    balls++;
                    compare = Distance + 30;

                }
                
                else {
                    
                    State[Canister] = 1;
                    DistanceCanister[Canister] = Distance;
                    BallDispensed[Canister] = 0;
                    Canister ++;
                    __delay_ms(2000);   
                    LATCbits.LATC6 = 0; // light

                }
           
                count++;
                
  

            }
        
            if (count == 1){  
                if (front< 10) {
                    
                    righty--;
                }
                if (front > 30) {
                    
                    lefty--;
                }
            }
        
        if (dispense == 1 && count == 15){
            dispense = 0;
            LATCbits.LATC6 = 1; // light

            while (!PORTBbits.RB0){          // not switch                         

                LATCbits.LATC7 = 1;  // motor    
                __delay_ms(10);
                LATCbits.LATC7 = 0;
                __delay_ms(50);

            }
            
            LATCbits.LATC6 = 0; // light
            LATCbits.LATC7 = 0;  //motor
        }
                   
        
        
        lcd_clear();
        printf("Dist: %.3f", Distance);
        lcd_set_ddram_addr(LCD_LINE2_ADDR);
        printf("cnt: %d", count);
        lcd_set_ddram_addr(LCD_LINE3_ADDR);
        printf("comp: %d", compare);
        lcd_set_ddram_addr(LCD_LINE4_ADDR);
        printf("Ultra: %d", frontsensor());

        prev2 = prev;
        prev = front;
        
        
        if (count == 50){count = 0;}
        else if (count > 0){count++;}
 
    }
    
    
    while(Distance > -20 && 0) {
                    
            
            if (lefty == righty) {
                backdrive();
            }
            else if (righty > lefty ){
                backright();
            }
            else {
                backleft();
            }
                
            
            if (readADC(2) > x && statright == 0){
                statright = 1;
                righty++;
                Distance -= (circ / 12.0);
                lcd_clear();
                printf("Dist: %.3f", Distance);
                
            }
            else if (readADC(2) < 1 && statright == 1){
                statright = 0;

            }
            
            if (readADC(3) > x && statleft == 0){
                statleft = 1;
                lefty++;
                Distance -= (circ / 12.0);
                lcd_clear();
                printf("Dist: %.3f", Distance);

            }
            else if (readADC(3) < 1 && statleft == 1){
                statleft = 0;
                

            }

        
        lcd_clear();
        printf("Dist: %.3f", Distance);
        lcd_set_ddram_addr(LCD_LINE2_ADDR);
        printf("lefty: %d", lefty);
        lcd_set_ddram_addr(LCD_LINE3_ADDR);
        printf("right: %d", righty);
        lcd_set_ddram_addr(LCD_LINE4_ADDR);
        printf("Ultra: %d", frontsensor());

      
    }
    
    

    TMR0IE = 0;
    //display time. frequency of count = (2^18)/2 500 000, so use that factor to correct count into seconds
    time = timer*0.0262144;
    
    
    EEPROM_save();
    
    
    
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
    
//    key_was_pressed = true;
//    exit_key = true;
    // Finish loop
    info:
    state = 0;
    clear = 0;
    tick = 0;
    B_was_pressed = false;
    key_was_pressed = true;
    exit_key = true;
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
        
        else if (state == 4 & clear == 1) {
            
            lcd_clear();
            printf("View previous");
            lcd_set_ddram_addr(LCD_LINE2_ADDR);
            printf("trials");
            lcd_set_ddram_addr(LCD_LINE4_ADDR);
            printf("Press B ", Canister-1);
            clear = 0;
       
        }
        
        
        if(B_was_pressed) {
            
            lcd_clear();
            printf("Pick previous");
            lcd_set_ddram_addr(LCD_LINE2_ADDR);
            printf("trials");
            lcd_set_ddram_addr(LCD_LINE3_ADDR);
            printf("0 is Current");
            lcd_set_ddram_addr(LCD_LINE4_ADDR);
            printf("0 to 4 ");

            while (!key_was_pressed){continue;}
            
            int number_pressed = (int) (keys[keypress] - '0');
            
            EEPROM_prev(number_pressed);
            goto info;

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
                    printf("# to exit");                    
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
                    printf("# to exit");                    
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
                    printf("# to exit");


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
            state = state % 5; //make sure status is between 0 and 4
            tick = 0;
        }
    
        tick++;
        __delay_ms(1);

        
    }
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
    __delay_us(1100);
    LATBbits.LATB2 = 0 ;
    __delay_us(1000);
    
    return;
}

void right(void){ // B3 Faster
    
    LATBbits.LATB3 = 1 ; 
    __delay_us(1750);
    LATBbits.LATB3 = 0 ; 
    __delay_us(1000);
    
    return;
}

void backdrive(void){
    
    LATBbits.LATB2 = 1 ; 
    LATBbits.LATB3 = 1 ; 
    __delay_us(1200);
    LATBbits.LATB3 = 0 ;
    __delay_us(400);
    LATBbits.LATB2 = 0 ; 
    __delay_us(1000);
    
    return;
}


void backleft(void){ //B2 Faster
    
    LATBbits.LATB3 = 1 ; 
    __delay_us(1100);
    LATBbits.LATB3 = 0 ;
    __delay_us(1000);
    
    return;
}

void backright(void){ // B3 Faster
    
    LATBbits.LATB2 = 1 ; 
    __delay_us(1750);
    LATBbits.LATB2 = 0 ; 
    __delay_us(1000);
    
    return;
}

int frontsensor(void){
    
    int time= 0;
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


void EEPROM_WriteByte(unsigned char eepromAddress, unsigned char eepromData) {
    
unsigned char gie_Status;
while(EECON1bits.WR){};            // check the WR bit to see if frontprevious Write operation is in progress
    EEADR=eepromAddress;  // Write the address to EEADR.
    EEDATA=eepromData;    // load the 8-bit datfrontvalue to be written in the EEDATA register.
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
    
    while(EECON1bits.RD || EECON1bits.WR);           // check the WR&RD bit to see if frontRD/WR is in progress
    EEADR=eepromAddress;       // Write the address to EEADR.
    EECON1bits.RD = 1;                    // Set the RD bit to trigger the eeprom read operation.
    return(EEDATA);            // Return the datfrontread form eeprom.
    
}

void EEPROM_save(void){
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
    
    EEPROM_WriteByte(0,x+1);

}

void EEPROM_prev(int trial ){
    int x;
    if (EEPROM_ReadByte(0) < 32 && EEPROM_ReadByte(0) > -1)
         x = EEPROM_ReadByte(0);
    else {
        EEPROM_WriteByte(0,0);
        x = 0;
    }

    
    x = x - trial;
    if (x < 0){
        x = x + 31;
    }
        

    int shift = x * 33;


    time = EEPROM_ReadByte(shift + 1);
    Canister = EEPROM_ReadByte(shift + 2);
    balls = EEPROM_ReadByte(shift + 3);
    
    
    int temp;
    for (int loop = 0; loop < 10; loop++){
        temp = shift + 4 + loop;
        State[loop] = EEPROM_ReadByte(temp);
    }

    for (int loop = 0; loop < 10; loop++){
        temp = shift + 14 + loop;
        DistanceCanister[loop] = EEPROM_ReadByte(temp);
    }
    
    for (int loop = 0; loop < 10; loop++){     
        temp = shift + 24 + loop;
        BallDispensed[loop] = EEPROM_ReadByte(temp);
    }
}


void __interrupt() interruptHandler(void){
    // Interrupt on change handler for RB1
    keypress = (PORTB & 0xF0) >> 4;
    
    //Interrupt on TMROH overflow
    if(TMR0IF){
        timer++; //increment count by 1
        TMR0IF = 0; // Clear interrupt flag bit to signify it's been handled
    }
    
    if(INT1IF){
        // Notice how we keep the interrupt processing very short by simply
        // setting front"flag" which the main program loop checks
        INT1IF = 0; // Clear interrupt flag bit to signify it's been handled
        
        if (keys[keypress] == 'A') {
            
            start = true;

            return;
        }

        if (keys[keypress] == '#') {
            
            exit_key = true;
            return;
        }
        
        if (keys[keypress] == 'B') {
            
            B_was_pressed = true;
            return;
        }
        
        for ( int i = 0; i < 10; i++ ) {
            
            if ((char)i + '0' == keys[keypress]) {
                key_was_pressed = true;
                return;
            }

        }
                   
        
    }
}