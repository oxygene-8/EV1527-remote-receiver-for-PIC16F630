////////////////////////////////////////
// File:   main.c                     //
// Author:                            //
//                                    //
// Created on April 28, 2020, 9:37 AM //
////////////////////////////////////////

#pragma config FOSC = INTRCIO   // Oscillator Selection bits (INTOSC oscillator: I/O function on RA4/OSC2/CLKOUT pin, I/O function on RA5/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RA3/MCLR pin function select (RA3/MCLR pin function is digital I/O, MCLR internally tied to VDD)
#pragma config BOREN = OFF      // Brown-out Detect Enable bit (BOD disabled)
#pragma config CP = OFF         // Code Protection bit (Program Memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)

#include <xc.h>
#include <stdint.h>

#define _XTAL_FREQ  4000000 //Mandatory for __delay_ms() function
#define RF1PIN  PORTAbits.RA5 //RF receiver 1
#define RF2PIN  PORTAbits.RA4 //RF receiver 2
#define LRNKEY  PORTAbits.RA1 //Learn button
#define LEDPIN  PORTAbits.RA0 //LED
#define CHPORT  PORTC //Pins 0 to 3 will be assigned to channels 1 to 4
#define MODKEY  PORTAbits.RA2 //External interrupt for EEPROM manipulation
#define TLRNC   1//Data cycle time tolerance at timer 1 frequency for internal oscillator inaccuracy
#define STORE   1//Number of LED blinks when a remote succesfully learned
#define LEARNED 2//Number of LED blinks when remote already exists in EEPROM
#define FULL    3//Number of LED blinks one when EEPROM is full
#define ERASE   3//Number of LED blinks when EEPROM is erased
#define REMOVE  2//Number of LED blinks when last stored remote is removed

const uint8_t Shift[8] = {1,2,4,8,16,32,64,128};
volatile uint8_t Direction = 2;
volatile uint8_t Mode = 0;
volatile uint8_t Temp = 0;

void LED_Blink(uint8_t blinks) { // LED blink at 2.5HZ
    for (int i=0;i<blinks;i++) {
        LEDPIN = 1;
        __delay_ms(200);
        LEDPIN = 0;
        __delay_ms(200);
    }
}

uint16_t Read_Timer(void) {
        uint8_t stat; //16bit register workaround
        uint16_t val;
        
        T1CON = 0b00000000; //Stop timer
        stat = STATUS; //Precaution for 16 bit register read
        val = TMR1; //Read timer
        TMR1 = 0x0000; //Reset timer
        STATUS = stat;
        T1CON=0b00110001; //Start timer at 500KHZ
        return val;
}

void __interrupt() Remote(void) {
    static uint16_t	Fall=0;
	static uint16_t	Rise=0;
	static uint8_t	Preamble = 0;
	static uint8_t	Index = 0;
	static uint8_t  Code[5] = {0};
    static uint8_t  Check = 0;
    static uint8_t  Data = 0;

    if (RAIF) { //Interrupt on change for RF receiver
        if(RF1PIN||RF2PIN) { //Rising edge
            Fall = Read_Timer(); //Cycle time capture
            if (Fall>(Rise*30) && Fall<(Rise*32)) { //Preamble detection
                Preamble = 1;
            	Index = 0;
                Code[3] = Rise&0xFF; //Data cycle time accuracy for different ROSCs
                Code[4] = (Rise>>8)|(RF1PIN?0b10000000:0); //Data cycle time for different ROSCs
            } else if (Preamble) { //Code is being received
                if (Fall>(Rise*2) && Fall<(Rise*4)) { //Bit is zero
                    if (Index<23) {
                        Index++;
                    }
                } else if ((Fall*2)<Rise && (Fall*4)>Rise) { //Bit is one
                    if (Index<8){
                        Code[2] |= Shift[Index];
                        Index++;
                    } else if (Index<16) {
                        Code[1] |= Shift[Index&7];
                        Index++;
                    } else if (Index<23) {
                        Code[0] |= Shift[Index&7]; //Last 4 bits of code and 3 first bits of data
                        Index++;
                    }
                } else { //Invalid signal
                    Preamble = 0;
                    Index = 0;
                }
            }
        } else { //Falling Edge
            Rise = Read_Timer();
            if (Index==23) { //Last bit check for non-continues press
                if (Code[2]<64) { //Checking if the last received bit is zero
                    if (Rise<(Fall*2) && Fall<(Rise*2)) { //Last data bit is one
                        Code[2] |= 0b10000000;
                    }
                } else {
                    if ((Fall*2)<Rise && (Fall*4)>Rise) { //Duplicate one bits at the end
                        Code[2] |= 0b10000000;
                    }
                }
                Check = 0;
                uint8_t Half = Code[2]&0x0F;
                for(int i=0;i<0x7C;i=i+5) { //Check if code exists
                    if (eeprom_read(i+4)==Code[4]) {
                        if (eeprom_read(i+3)<Code[3]+TLRNC&&eeprom_read(i+3)>Code[3]-TLRNC) { //Considering tolerance for using internal oscillator 
                            if (eeprom_read(i+2)==Code[2]&&eeprom_read(i+1)==Code[1]) {
                                if (eeprom_read(i)==Code[0]) { //24 bit code with data for 2 channel
                                    Check = 1;
                                    break;
                                } else if ((eeprom_read(i)&0x0F)==Half) { //20 bit code for 4 channel
                                    Check = 2;
                                }
                            }
                        }
                    }
                } if (LRNKEY) { //Key not pressed
                    if (Check>0) { //Code exists in some form
                        Index++; //Flip-flop repeat prevention
                        Data = (Code[2]>>4);
                        if (Mode==0) {
                            /*if ((Data==1)||(Data==2)||(Data==4)||(Data==8)) { //Mixed channels prohibited*/
                                if (Data!=CHPORT&&CHPORT!=0) { //Preventing conflict
                                    CHPORT = 0;
                                    __delay_ms(2000);
                                } CHPORT ^= Data;
                            /*}*/
                        } else if (Check&&Mode) { //2channel flip-flop
                            CHPORT ^= Direction;
                        }
                    } else { //Code does not exist and learn is not pressed
                        Preamble = 0;
                        Index = 0;
                    }
                } else if (!Check) { //Learn button pressed and 24bit code not stored
                    if (Temp<0x78) {
                        for (int i=0;i<5;i++) {
                            eeprom_write(Temp++,Code[i]);
                        }
                        eeprom_write(0x7E,Temp);
                        //INTCONbits.INTF=0; //Enable EEPROM format and mode select
                        LED_Blink(STORE);
                    } else { //If the EEPROM is full
                        LED_Blink(FULL);
                        eeprom_write(0x7E,0);//Overwrite from the beginning remove for constant memory
                    }
                } else { //Code exists and learn pressed
                    LED_Blink(LEARNED);
                    Preamble = 0;
                    Index = 0;
                }
            } if (Index==24) { //Instant mode exclusive
                if (Mode==2) {
                    CHPORT = Data;
                } else if (Check&&Mode==3) { //2channel instant
                    CHPORT = Direction;
                }
            }
        }
    } else if (INTF) { //External interrupt for mode button
        CHPORT = 0;
        __delay_ms(2500);
        if (MODKEY) {
            eeprom_write(0x7F,++Mode&3);
            LED_Blink(Mode);
        } else { //If Key pressed longer than 3 seconds
            RAIF = 1; //Disable RF receiver
            LEDPIN = 1; //If enabled LED blinks at given number without decreasing one
            __delay_ms(2500);
            uint8_t End = 0;
            uint8_t Notice = 0;
            if (MODKEY&&Temp>0) {
                End = Temp-5;
                Notice = REMOVE;
            } else {
                Notice = ERASE;
            }
            while (Temp>End) {
                eeprom_write(--Temp,0x00);
            }
            eeprom_write(0x7E,End);
            LED_Blink(Notice);
        } INTF=0; //Enable EEPROM erase
    } else if (TMR1IF) {
        T1CON = 0;
        TMR1 = 0;
        Preamble = 0;
        Index = 0;
        TMR1IF = 0;
    }
    RAIF=0; //Clear interrupt flag
    GIE=1; //Enable global interrupt
}

void main(void) { //Options are pretty much obvious
    PORTA=0;
    PORTC=0;
    TRISA=0b110110;
    TRISC=0b000000;
    WPUA=0b000110;
    IOCA=0b110000;
    //ANSEL=0b00000000;
	INTCON=0b10011000;
    PIE1=0b00000001;
    OPTION_REG=0b00000000;
    Mode = eeprom_read(0x7F);
    Temp = eeprom_read(0x7E);
    while (1) {
        if(CHPORT!=0){
            if (Mode>1) {
                __delay_ms(100);
                Direction = ~Direction&3; //2bit NOT
            } else {
                __delay_ms(20000);
            } CHPORT = 0;
        }
    }
}
