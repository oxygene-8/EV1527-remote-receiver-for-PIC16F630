# 1 "main.c"
# 1 "<built-in>" 1
# 1 "<built-in>" 3
# 288 "<built-in>" 3
# 1 "<command line>" 1
# 1 "<built-in>" 2
# 1 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\language_support.h" 1 3
# 2 "<built-in>" 2
# 1 "main.c" 2







#pragma config FOSC = INTRCIO
#pragma config WDTE = OFF
#pragma config PWRTE = OFF
#pragma config MCLRE = OFF
#pragma config BOREN = OFF
#pragma config CP = OFF
#pragma config CPD = OFF


# 1 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\xc.h" 1 3
# 18 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\xc.h" 3
extern const char __xc8_OPTIM_SPEED;

extern double __fpnormalize(double);



# 1 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\c90\\xc8debug.h" 1 3
# 13 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\c90\\xc8debug.h" 3
#pragma intrinsic(__builtin_software_breakpoint)
extern void __builtin_software_breakpoint(void);
# 23 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\xc.h" 2 3




# 1 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\pic.h" 1 3




# 1 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\htc.h" 1 3



# 1 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\xc.h" 1 3
# 4 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\htc.h" 2 3
# 5 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\pic.h" 2 3




extern unsigned char __osccal_val(void);




# 1 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\pic_chip_select.h" 1 3
# 2153 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\pic_chip_select.h" 3
# 1 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\pic16f630.h" 1 3
# 44 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\pic16f630.h" 3
# 1 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\__at.h" 1 3
# 44 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\pic16f630.h" 2 3








extern volatile unsigned char INDF __attribute__((address(0x000)));

__asm("INDF equ 00h");




extern volatile unsigned char TMR0 __attribute__((address(0x001)));

__asm("TMR0 equ 01h");




extern volatile unsigned char PCL __attribute__((address(0x002)));

__asm("PCL equ 02h");




extern volatile unsigned char STATUS __attribute__((address(0x003)));

__asm("STATUS equ 03h");


typedef union {
    struct {
        unsigned C :1;
        unsigned DC :1;
        unsigned Z :1;
        unsigned nPD :1;
        unsigned nTO :1;
        unsigned RP :2;
        unsigned IRP :1;
    };
    struct {
        unsigned :5;
        unsigned RP0 :1;
        unsigned RP1 :1;
    };
    struct {
        unsigned CARRY :1;
        unsigned :1;
        unsigned ZERO :1;
    };
} STATUSbits_t;
extern volatile STATUSbits_t STATUSbits __attribute__((address(0x003)));
# 159 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\pic16f630.h" 3
extern volatile unsigned char FSR __attribute__((address(0x004)));

__asm("FSR equ 04h");




extern volatile unsigned char PORTA __attribute__((address(0x005)));

__asm("PORTA equ 05h");


typedef union {
    struct {
        unsigned RA0 :1;
        unsigned RA1 :1;
        unsigned RA2 :1;
        unsigned RA3 :1;
        unsigned RA4 :1;
        unsigned RA5 :1;
    };
} PORTAbits_t;
extern volatile PORTAbits_t PORTAbits __attribute__((address(0x005)));
# 216 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\pic16f630.h" 3
extern volatile unsigned char PORTC __attribute__((address(0x007)));

__asm("PORTC equ 07h");


typedef union {
    struct {
        unsigned RC0 :1;
        unsigned RC1 :1;
        unsigned RC2 :1;
        unsigned RC3 :1;
        unsigned RC4 :1;
        unsigned RC5 :1;
    };
} PORTCbits_t;
extern volatile PORTCbits_t PORTCbits __attribute__((address(0x007)));
# 266 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\pic16f630.h" 3
extern volatile unsigned char PCLATH __attribute__((address(0x00A)));

__asm("PCLATH equ 0Ah");


typedef union {
    struct {
        unsigned PCLATH :5;
    };
} PCLATHbits_t;
extern volatile PCLATHbits_t PCLATHbits __attribute__((address(0x00A)));
# 286 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\pic16f630.h" 3
extern volatile unsigned char INTCON __attribute__((address(0x00B)));

__asm("INTCON equ 0Bh");


typedef union {
    struct {
        unsigned RAIF :1;
        unsigned INTF :1;
        unsigned T0IF :1;
        unsigned RAIE :1;
        unsigned INTE :1;
        unsigned T0IE :1;
        unsigned PEIE :1;
        unsigned GIE :1;
    };
    struct {
        unsigned :2;
        unsigned TMR0IF :1;
        unsigned :2;
        unsigned TMR0IE :1;
    };
} INTCONbits_t;
extern volatile INTCONbits_t INTCONbits __attribute__((address(0x00B)));
# 364 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\pic16f630.h" 3
extern volatile unsigned char PIR1 __attribute__((address(0x00C)));

__asm("PIR1 equ 0Ch");


typedef union {
    struct {
        unsigned TMR1IF :1;
        unsigned :2;
        unsigned CMIF :1;
        unsigned :3;
        unsigned EEIF :1;
    };
    struct {
        unsigned T1IF :1;
    };
} PIR1bits_t;
extern volatile PIR1bits_t PIR1bits __attribute__((address(0x00C)));
# 406 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\pic16f630.h" 3
extern volatile unsigned short TMR1 __attribute__((address(0x00E)));

__asm("TMR1 equ 0Eh");




extern volatile unsigned char TMR1L __attribute__((address(0x00E)));

__asm("TMR1L equ 0Eh");




extern volatile unsigned char TMR1H __attribute__((address(0x00F)));

__asm("TMR1H equ 0Fh");




extern volatile unsigned char T1CON __attribute__((address(0x010)));

__asm("T1CON equ 010h");


typedef union {
    struct {
        unsigned TMR1ON :1;
        unsigned TMR1CS :1;
        unsigned nT1SYNC :1;
        unsigned T1OSCEN :1;
        unsigned T1CKPS :2;
        unsigned TMR1GE :1;
    };
    struct {
        unsigned :4;
        unsigned T1CKPS0 :1;
        unsigned T1CKPS1 :1;
    };
} T1CONbits_t;
extern volatile T1CONbits_t T1CONbits __attribute__((address(0x010)));
# 492 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\pic16f630.h" 3
extern volatile unsigned char CMCON __attribute__((address(0x019)));

__asm("CMCON equ 019h");


typedef union {
    struct {
        unsigned CM :3;
        unsigned CIS :1;
        unsigned CINV :1;
        unsigned :1;
        unsigned COUT :1;
    };
    struct {
        unsigned CM0 :1;
        unsigned CM1 :1;
        unsigned CM2 :1;
    };
} CMCONbits_t;
extern volatile CMCONbits_t CMCONbits __attribute__((address(0x019)));
# 551 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\pic16f630.h" 3
extern volatile unsigned char OPTION_REG __attribute__((address(0x081)));

__asm("OPTION_REG equ 081h");


typedef union {
    struct {
        unsigned PS :3;
        unsigned PSA :1;
        unsigned T0SE :1;
        unsigned T0CS :1;
        unsigned INTEDG :1;
        unsigned nRAPU :1;
    };
    struct {
        unsigned PS0 :1;
        unsigned PS1 :1;
        unsigned PS2 :1;
        unsigned :4;
        unsigned nGPPU :1;
    };
} OPTION_REGbits_t;
extern volatile OPTION_REGbits_t OPTION_REGbits __attribute__((address(0x081)));
# 628 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\pic16f630.h" 3
extern volatile unsigned char TRISA __attribute__((address(0x085)));

__asm("TRISA equ 085h");


typedef union {
    struct {
        unsigned TRISA0 :1;
        unsigned TRISA1 :1;
        unsigned TRISA2 :1;
        unsigned TRISA3 :1;
        unsigned TRISA4 :1;
        unsigned TRISA5 :1;
    };
} TRISAbits_t;
extern volatile TRISAbits_t TRISAbits __attribute__((address(0x085)));
# 678 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\pic16f630.h" 3
extern volatile unsigned char TRISC __attribute__((address(0x087)));

__asm("TRISC equ 087h");


typedef union {
    struct {
        unsigned TRISC0 :1;
        unsigned TRISC1 :1;
        unsigned TRISC2 :1;
        unsigned TRISC3 :1;
        unsigned TRISC4 :1;
        unsigned TRISC5 :1;
    };
} TRISCbits_t;
extern volatile TRISCbits_t TRISCbits __attribute__((address(0x087)));
# 728 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\pic16f630.h" 3
extern volatile unsigned char PIE1 __attribute__((address(0x08C)));

__asm("PIE1 equ 08Ch");


typedef union {
    struct {
        unsigned TMR1IE :1;
        unsigned :2;
        unsigned CMIE :1;
        unsigned :3;
        unsigned EEIE :1;
    };
    struct {
        unsigned T1IE :1;
    };
} PIE1bits_t;
extern volatile PIE1bits_t PIE1bits __attribute__((address(0x08C)));
# 770 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\pic16f630.h" 3
extern volatile unsigned char PCON __attribute__((address(0x08E)));

__asm("PCON equ 08Eh");


typedef union {
    struct {
        unsigned nBOR :1;
        unsigned nPOR :1;
    };
    struct {
        unsigned nBOD :1;
    };
} PCONbits_t;
extern volatile PCONbits_t PCONbits __attribute__((address(0x08E)));
# 804 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\pic16f630.h" 3
extern volatile unsigned char OSCCAL __attribute__((address(0x090)));

__asm("OSCCAL equ 090h");


typedef union {
    struct {
        unsigned :2;
        unsigned CAL :6;
    };
    struct {
        unsigned :2;
        unsigned CAL0 :1;
        unsigned CAL1 :1;
        unsigned CAL2 :1;
        unsigned CAL3 :1;
        unsigned CAL4 :1;
        unsigned CAL5 :1;
    };
} OSCCALbits_t;
extern volatile OSCCALbits_t OSCCALbits __attribute__((address(0x090)));
# 864 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\pic16f630.h" 3
extern volatile unsigned char WPUA __attribute__((address(0x095)));

__asm("WPUA equ 095h");


extern volatile unsigned char WPU __attribute__((address(0x095)));

__asm("WPU equ 095h");


typedef union {
    struct {
        unsigned WPUA0 :1;
        unsigned WPUA1 :1;
        unsigned WPUA2 :1;
        unsigned :1;
        unsigned WPUA4 :1;
        unsigned WPUA5 :1;
    };
} WPUAbits_t;
extern volatile WPUAbits_t WPUAbits __attribute__((address(0x095)));
# 912 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\pic16f630.h" 3
typedef union {
    struct {
        unsigned WPUA0 :1;
        unsigned WPUA1 :1;
        unsigned WPUA2 :1;
        unsigned :1;
        unsigned WPUA4 :1;
        unsigned WPUA5 :1;
    };
} WPUbits_t;
extern volatile WPUbits_t WPUbits __attribute__((address(0x095)));
# 952 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\pic16f630.h" 3
extern volatile unsigned char IOCA __attribute__((address(0x096)));

__asm("IOCA equ 096h");


extern volatile unsigned char IOC __attribute__((address(0x096)));

__asm("IOC equ 096h");


typedef union {
    struct {
        unsigned IOCA0 :1;
        unsigned IOCA1 :1;
        unsigned IOCA2 :1;
        unsigned IOCA3 :1;
        unsigned IOCA4 :1;
        unsigned IOCA5 :1;
    };
} IOCAbits_t;
extern volatile IOCAbits_t IOCAbits __attribute__((address(0x096)));
# 1005 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\pic16f630.h" 3
typedef union {
    struct {
        unsigned IOCA0 :1;
        unsigned IOCA1 :1;
        unsigned IOCA2 :1;
        unsigned IOCA3 :1;
        unsigned IOCA4 :1;
        unsigned IOCA5 :1;
    };
} IOCbits_t;
extern volatile IOCbits_t IOCbits __attribute__((address(0x096)));
# 1050 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\pic16f630.h" 3
extern volatile unsigned char VRCON __attribute__((address(0x099)));

__asm("VRCON equ 099h");


typedef union {
    struct {
        unsigned VR :4;
        unsigned :1;
        unsigned VRR :1;
        unsigned :1;
        unsigned VREN :1;
    };
    struct {
        unsigned VR0 :1;
        unsigned VR1 :1;
        unsigned VR2 :1;
        unsigned VR3 :1;
    };
} VRCONbits_t;
extern volatile VRCONbits_t VRCONbits __attribute__((address(0x099)));
# 1110 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\pic16f630.h" 3
extern volatile unsigned char EEDAT __attribute__((address(0x09A)));

__asm("EEDAT equ 09Ah");


extern volatile unsigned char EEDATA __attribute__((address(0x09A)));

__asm("EEDATA equ 09Ah");


typedef union {
    struct {
        unsigned EEDAT :8;
    };
} EEDATbits_t;
extern volatile EEDATbits_t EEDATbits __attribute__((address(0x09A)));







typedef union {
    struct {
        unsigned EEDAT :8;
    };
} EEDATAbits_t;
extern volatile EEDATAbits_t EEDATAbits __attribute__((address(0x09A)));
# 1148 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\pic16f630.h" 3
extern volatile unsigned char EEADR __attribute__((address(0x09B)));

__asm("EEADR equ 09Bh");




extern volatile unsigned char EECON1 __attribute__((address(0x09C)));

__asm("EECON1 equ 09Ch");


typedef union {
    struct {
        unsigned RD :1;
        unsigned WR :1;
        unsigned WREN :1;
        unsigned WRERR :1;
    };
} EECON1bits_t;
extern volatile EECON1bits_t EECON1bits __attribute__((address(0x09C)));
# 1193 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\pic16f630.h" 3
extern volatile unsigned char EECON2 __attribute__((address(0x09D)));

__asm("EECON2 equ 09Dh");
# 1206 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\pic16f630.h" 3
extern volatile __bit CAL0 __attribute__((address(0x482)));


extern volatile __bit CAL1 __attribute__((address(0x483)));


extern volatile __bit CAL2 __attribute__((address(0x484)));


extern volatile __bit CAL3 __attribute__((address(0x485)));


extern volatile __bit CAL4 __attribute__((address(0x486)));


extern volatile __bit CAL5 __attribute__((address(0x487)));


extern volatile __bit CARRY __attribute__((address(0x18)));


extern volatile __bit CINV __attribute__((address(0xCC)));


extern volatile __bit CIS __attribute__((address(0xCB)));


extern volatile __bit CM0 __attribute__((address(0xC8)));


extern volatile __bit CM1 __attribute__((address(0xC9)));


extern volatile __bit CM2 __attribute__((address(0xCA)));


extern volatile __bit CMIE __attribute__((address(0x463)));


extern volatile __bit CMIF __attribute__((address(0x63)));


extern volatile __bit COUT __attribute__((address(0xCE)));


extern volatile __bit DC __attribute__((address(0x19)));


extern volatile __bit EEIE __attribute__((address(0x467)));


extern volatile __bit EEIF __attribute__((address(0x67)));


extern volatile __bit GIE __attribute__((address(0x5F)));


extern volatile __bit INTE __attribute__((address(0x5C)));


extern volatile __bit INTEDG __attribute__((address(0x40E)));


extern volatile __bit INTF __attribute__((address(0x59)));


extern volatile __bit IOCA0 __attribute__((address(0x4B0)));


extern volatile __bit IOCA1 __attribute__((address(0x4B1)));


extern volatile __bit IOCA2 __attribute__((address(0x4B2)));


extern volatile __bit IOCA3 __attribute__((address(0x4B3)));


extern volatile __bit IOCA4 __attribute__((address(0x4B4)));


extern volatile __bit IOCA5 __attribute__((address(0x4B5)));


extern volatile __bit IRP __attribute__((address(0x1F)));


extern volatile __bit PEIE __attribute__((address(0x5E)));


extern volatile __bit PS0 __attribute__((address(0x408)));


extern volatile __bit PS1 __attribute__((address(0x409)));


extern volatile __bit PS2 __attribute__((address(0x40A)));


extern volatile __bit PSA __attribute__((address(0x40B)));


extern volatile __bit RA0 __attribute__((address(0x28)));


extern volatile __bit RA1 __attribute__((address(0x29)));


extern volatile __bit RA2 __attribute__((address(0x2A)));


extern volatile __bit RA3 __attribute__((address(0x2B)));


extern volatile __bit RA4 __attribute__((address(0x2C)));


extern volatile __bit RA5 __attribute__((address(0x2D)));


extern volatile __bit RAIE __attribute__((address(0x5B)));


extern volatile __bit RAIF __attribute__((address(0x58)));


extern volatile __bit RC0 __attribute__((address(0x38)));


extern volatile __bit RC1 __attribute__((address(0x39)));


extern volatile __bit RC2 __attribute__((address(0x3A)));


extern volatile __bit RC3 __attribute__((address(0x3B)));


extern volatile __bit RC4 __attribute__((address(0x3C)));


extern volatile __bit RC5 __attribute__((address(0x3D)));


extern volatile __bit RD __attribute__((address(0x4E0)));


extern volatile __bit RP0 __attribute__((address(0x1D)));


extern volatile __bit RP1 __attribute__((address(0x1E)));


extern volatile __bit T0CS __attribute__((address(0x40D)));


extern volatile __bit T0IE __attribute__((address(0x5D)));


extern volatile __bit T0IF __attribute__((address(0x5A)));


extern volatile __bit T0SE __attribute__((address(0x40C)));


extern volatile __bit T1CKPS0 __attribute__((address(0x84)));


extern volatile __bit T1CKPS1 __attribute__((address(0x85)));


extern volatile __bit T1IE __attribute__((address(0x460)));


extern volatile __bit T1IF __attribute__((address(0x60)));


extern volatile __bit T1OSCEN __attribute__((address(0x83)));


extern volatile __bit TMR0IE __attribute__((address(0x5D)));


extern volatile __bit TMR0IF __attribute__((address(0x5A)));


extern volatile __bit TMR1CS __attribute__((address(0x81)));


extern volatile __bit TMR1GE __attribute__((address(0x86)));


extern volatile __bit TMR1IE __attribute__((address(0x460)));


extern volatile __bit TMR1IF __attribute__((address(0x60)));


extern volatile __bit TMR1ON __attribute__((address(0x80)));


extern volatile __bit TRISA0 __attribute__((address(0x428)));


extern volatile __bit TRISA1 __attribute__((address(0x429)));


extern volatile __bit TRISA2 __attribute__((address(0x42A)));


extern volatile __bit TRISA3 __attribute__((address(0x42B)));


extern volatile __bit TRISA4 __attribute__((address(0x42C)));


extern volatile __bit TRISA5 __attribute__((address(0x42D)));


extern volatile __bit TRISC0 __attribute__((address(0x438)));


extern volatile __bit TRISC1 __attribute__((address(0x439)));


extern volatile __bit TRISC2 __attribute__((address(0x43A)));


extern volatile __bit TRISC3 __attribute__((address(0x43B)));


extern volatile __bit TRISC4 __attribute__((address(0x43C)));


extern volatile __bit TRISC5 __attribute__((address(0x43D)));


extern volatile __bit VR0 __attribute__((address(0x4C8)));


extern volatile __bit VR1 __attribute__((address(0x4C9)));


extern volatile __bit VR2 __attribute__((address(0x4CA)));


extern volatile __bit VR3 __attribute__((address(0x4CB)));


extern volatile __bit VREN __attribute__((address(0x4CF)));


extern volatile __bit VRR __attribute__((address(0x4CD)));


extern volatile __bit WPUA0 __attribute__((address(0x4A8)));


extern volatile __bit WPUA1 __attribute__((address(0x4A9)));


extern volatile __bit WPUA2 __attribute__((address(0x4AA)));


extern volatile __bit WPUA4 __attribute__((address(0x4AC)));


extern volatile __bit WPUA5 __attribute__((address(0x4AD)));


extern volatile __bit WR __attribute__((address(0x4E1)));


extern volatile __bit WREN __attribute__((address(0x4E2)));


extern volatile __bit WRERR __attribute__((address(0x4E3)));


extern volatile __bit ZERO __attribute__((address(0x1A)));


extern volatile __bit nBOD __attribute__((address(0x470)));


extern volatile __bit nBOR __attribute__((address(0x470)));


extern volatile __bit nGPPU __attribute__((address(0x40F)));


extern volatile __bit nPD __attribute__((address(0x1B)));


extern volatile __bit nPOR __attribute__((address(0x471)));


extern volatile __bit nRAPU __attribute__((address(0x40F)));


extern volatile __bit nT1SYNC __attribute__((address(0x82)));


extern volatile __bit nTO __attribute__((address(0x1C)));
# 2153 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\pic_chip_select.h" 2 3
# 13 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\pic.h" 2 3
# 30 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\pic.h" 3
#pragma intrinsic(__nop)
extern void __nop(void);
# 78 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\pic.h" 3
__attribute__((__unsupported__("The " "FLASH_READ" " macro function is no longer supported. Please use the MPLAB X MCC."))) unsigned char __flash_read(unsigned short addr);

__attribute__((__unsupported__("The " "FLASH_WRITE" " macro function is no longer supported. Please use the MPLAB X MCC."))) void __flash_write(unsigned short addr, unsigned short data);

__attribute__((__unsupported__("The " "FLASH_ERASE" " macro function is no longer supported. Please use the MPLAB X MCC."))) void __flash_erase(unsigned short addr);



# 1 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\eeprom_routines.h" 1 3
# 114 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\eeprom_routines.h" 3
extern void eeprom_write(unsigned char addr, unsigned char value);
extern unsigned char eeprom_read(unsigned char addr);
# 85 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\pic.h" 2 3






#pragma intrinsic(_delay)
extern __attribute__((nonreentrant)) void _delay(unsigned long);
#pragma intrinsic(_delaywdt)
extern __attribute__((nonreentrant)) void _delaywdt(unsigned long);
# 137 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\pic.h" 3
extern __bank0 unsigned char __resetbits;
extern __bank0 __bit __powerdown;
extern __bank0 __bit __timeout;
# 27 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\xc.h" 2 3
# 16 "main.c" 2

# 1 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\c90\\stdint.h" 1 3
# 13 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\c90\\stdint.h" 3
typedef signed char int8_t;






typedef signed int int16_t;







typedef __int24 int24_t;







typedef signed long int int32_t;
# 52 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\c90\\stdint.h" 3
typedef unsigned char uint8_t;





typedef unsigned int uint16_t;






typedef __uint24 uint24_t;






typedef unsigned long int uint32_t;
# 88 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\c90\\stdint.h" 3
typedef signed char int_least8_t;







typedef signed int int_least16_t;
# 109 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\c90\\stdint.h" 3
typedef __int24 int_least24_t;
# 118 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\c90\\stdint.h" 3
typedef signed long int int_least32_t;
# 136 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\c90\\stdint.h" 3
typedef unsigned char uint_least8_t;






typedef unsigned int uint_least16_t;
# 154 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\c90\\stdint.h" 3
typedef __uint24 uint_least24_t;







typedef unsigned long int uint_least32_t;
# 181 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\c90\\stdint.h" 3
typedef signed char int_fast8_t;






typedef signed int int_fast16_t;
# 200 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\c90\\stdint.h" 3
typedef __int24 int_fast24_t;







typedef signed long int int_fast32_t;
# 224 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\c90\\stdint.h" 3
typedef unsigned char uint_fast8_t;





typedef unsigned int uint_fast16_t;
# 240 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\c90\\stdint.h" 3
typedef __uint24 uint_fast24_t;






typedef unsigned long int uint_fast32_t;
# 268 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\c90\\stdint.h" 3
typedef int32_t intmax_t;
# 282 "B:\\Program Files\\Microchip\\xc8\\v2.10\\pic\\include\\c90\\stdint.h" 3
typedef uint32_t uintmax_t;






typedef int16_t intptr_t;




typedef uint16_t uintptr_t;
# 17 "main.c" 2
# 33 "main.c"
const uint8_t Shift[8] = {1,2,4,8,16,32,64,128};
volatile uint8_t Direction = 2;
volatile uint8_t Mode = 0;
volatile uint8_t Temp = 0;

void LED_Blink(uint8_t blinks) {
    for (int i=0;i<blinks;i++) {
        PORTAbits.RA0 = 1;
        _delay((unsigned long)((200)*(4000000/4000.0)));
        PORTAbits.RA0 = 0;
        _delay((unsigned long)((200)*(4000000/4000.0)));
    }
}

uint16_t Read_Timer(void) {
        uint8_t stat;
        uint16_t val;

        T1CON = 0b00000000;
        stat = STATUS;
        val = TMR1;
        TMR1 = 0x0000;
        STATUS = stat;
        T1CON=0b00110001;
        return val;
}

void __attribute__((picinterrupt(("")))) Remote(void) {
    static uint16_t Fall=0;
 static uint16_t Rise=0;
 static uint8_t Preamble = 0;
 static uint8_t Index = 0;
 static uint8_t Code[5] = {0};
    static uint8_t Check = 0;
    static uint8_t Data = 0;

    if (RAIF) {
        if(PORTAbits.RA5||PORTAbits.RA4) {
            Fall = Read_Timer();
            if (Fall>(Rise*30) && Fall<(Rise*32)) {
                Preamble = 1;
             Index = 0;
                Code[3] = Rise&0xFF;
                Code[4] = (Rise>>8)|(PORTAbits.RA5?0b10000000:0);
            } else if (Preamble) {
                if (Fall>(Rise*2) && Fall<(Rise*4)) {
                    if (Index<23) {
                        Index++;
                    }
                } else if ((Fall*2)<Rise && (Fall*4)>Rise) {
                    if (Index<8){
                        Code[2] |= Shift[Index];
                        Index++;
                    } else if (Index<16) {
                        Code[1] |= Shift[Index&7];
                        Index++;
                    } else if (Index<23) {
                        Code[0] |= Shift[Index&7];
                        Index++;
                    }
                } else {
                    Preamble = 0;
                    Index = 0;
                }
            }
        } else {
            Rise = Read_Timer();
            if (Index==23) {
                if (Code[2]<64) {
                    if (Rise<(Fall*2) && Fall<(Rise*2)) {
                        Code[2] |= 0b10000000;
                    }
                } else {
                    if ((Fall*2)<Rise && (Fall*4)>Rise) {
                        Code[2] |= 0b10000000;
                    }
                }
                Check = 0;
                uint8_t Half = Code[2]&0x0F;
                for(int i=0;i<0x7C;i=i+5) {
                    if (eeprom_read(i+4)==Code[4]) {
                        if (eeprom_read(i+3)<Code[3]+1&&eeprom_read(i+3)>Code[3]-1) {
                            if (eeprom_read(i+2)==Code[2]&&eeprom_read(i+1)==Code[1]) {
                                if (eeprom_read(i)==Code[0]) {
                                    Check = 1;
                                    break;
                                } else if ((eeprom_read(i)&0x0F)==Half) {
                                    Check = 2;
                                }
                            }
                        }
                    }
                } if (PORTAbits.RA1) {
                    if (Check>0) {
                        Index++;
                        Data = (Code[2]>>4);
                        if (Mode==0) {

                                if (Data!=PORTC&&PORTC!=0) {
                                    PORTC = 0;
                                    _delay((unsigned long)((2000)*(4000000/4000.0)));
                                } PORTC ^= Data;

                        } else if (Check&&Mode) {
                            PORTC ^= Direction;
                        }
                    } else {
                        Preamble = 0;
                        Index = 0;
                    }
                } else if (!Check) {
                    if (Temp<0x78) {
                        for (int i=0;i<5;i++) {
                            eeprom_write(Temp++,Code[i]);
                        }
                        eeprom_write(0x7E,Temp);

                        LED_Blink(1);
                    } else {
                        LED_Blink(3);
                        eeprom_write(0x7E,0);
                    }
                } else {
                    LED_Blink(2);
                    Preamble = 0;
                    Index = 0;
                }
            } if (Index==24) {
                if (Mode==2) {
                    PORTC = Data;
                } else if (Check&&Mode==3) {
                    PORTC = Direction;
                }
            }
        }
    } else if (INTF) {
        PORTC = 0;
        _delay((unsigned long)((2500)*(4000000/4000.0)));
        if (PORTAbits.RA2) {
            eeprom_write(0x7F,++Mode&3);
            LED_Blink(Mode);
        } else {
            RAIF=1;
            PORTAbits.RA0 = 1;
            _delay((unsigned long)((2500)*(4000000/4000.0)));
            uint8_t End = 0;
            uint8_t Notice = 0;
            if (PORTAbits.RA2&&Temp>0) {
                End = Temp-5;
                Notice = 2;
            } else {
                Notice = 3;
            }
            while (Temp>End) {
                eeprom_write(--Temp,0x00);
            }
            eeprom_write(0x7E,End);
            LED_Blink(Notice);
        } INTF=0;
    } else if (TMR1IF) {
        T1CON = 0;
        TMR1 = 0;
        Preamble = 0;
        Index = 0;
        TMR1IF = 0;
    }
    RAIF=0;
    GIE=1;
}

void main(void) {
    PORTA=0;
    PORTC=0;
    TRISA=0b110110;
    TRISC=0b000000;
    WPUA=0b000110;
    IOCA=0b110000;

 INTCON=0b10011000;
    PIE1=0b00000001;
    OPTION_REG=0b00000000;
    Mode = eeprom_read(0x7F);
    Temp = eeprom_read(0x7E);
    while (1) {
        if(PORTC!=0){
            if (Mode>1) {
                _delay((unsigned long)((100)*(4000000/4000.0)));
                Direction = ~Direction&3;
            } else {
                _delay((unsigned long)((20000)*(4000000/4000.0)));
            } PORTC = 0;
        }
    }
}
