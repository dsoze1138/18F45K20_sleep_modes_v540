/*
 * File:   main.c
 * Author: dan1138
 * Target: PIC18F45K20
 *
 * Created on August 8, 2020, 6:05 PM
 *
 * Application runs on DM164130-4 - PICkit 44-Pin Demo Board (PIC18F45K20)
 * 
 *                                                    PIC18F47K20
 *               +---------+            +---------+                 +----------+                 +----------+
 *         <>  1 : RC7/RX  :      -- 12 : NC      :           <> 23 : RA4      :           -- 34 : NC       :
 *    LED4 <>  2 : RD4     :      -- 13 : NC      :           <> 24 : RA5      : 32.768KHz <> 35 : RC1/SOSI :
 *    LED5 <>  3 : RD5     :      <> 14 : RB4     :           <> 25 : RE0      :           <> 36 : RC2      :
 *    LED6 <>  4 : RD6     :      <> 15 : RB5/PGM :           <> 26 : RE1      :           <> 37 : RC3/SCL  :
 *    LED7 ->  5 : RD7     :  PGC <> 16 : RB6/PGC :           <> 27 : RE2      :      LED0 <> 38 : RD0      :
 *     GND ->  6 : VSS1    :  PGD <> 17 : RB7/PGD :       PWR -> 28 : VDD      :      LED1 <> 39 : RD1      :
 *     PWR <>  7 : VDD1    :  VPP -> 18 : RE3/VPP :       GND -> 29 : VSS      :      LED2 <> 40 : RD2      :
 *     SW1 <>  8 : RB0/INT0:  POT <> 19 : RA0     :    4.0MHz -> 30 : RA7/OSC1 :      LED3 <> 41 : RD3      :
 *         <>  9 : RB1     :      <> 20 : RA1     :    4.0MHz <- 31 : RA6/OSC2 :           <> 42 : RC4/SDI  :
 *         <> 10 : RB2     :      <> 21 : RA2     : 32.768KHz <> 32 : RC0/SOSO :           <> 43 : RC5/SDO  :
 *         <> 11 : RB3     :      <> 22 : RA3     :           -- 33 : NC       :           <> 44 : RC6/TX   :
 *               +---------+            +---------+                 +----------+                 +----------+
 *                                                     TQFP-44
 *  Description:
 *      Test sleep mode current.
 *
 */
//#pragma config FOSC = INTIO67   // Oscillator Selection bits (Internal oscillator block, port function on RA6 and RA7)
#pragma config FOSC = XT        // Oscillator Selection bits (XT oscillator)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = ON        // Internal/External Oscillator Switchover bit (Oscillator Switchover mode enabled)
#pragma config PWRT = ON        // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOREN = OFF      // Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
#pragma config BORV = 18        // Brown Out Reset Voltage bits (VBOR set to 1.8 V nominal)
#pragma config WDTEN = OFF      // Watchdog Timer Enable bit (WDT is controlled by SWDTEN bit of the WDTCON register)
#pragma config WDTPS = 2048     // Watchdog Timer Postscale Select bits (1:2048) about 8.4 seconds
#pragma config CCP2MX = PORTC   // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<4:0> pins are configured as analog input channels on Reset)
#pragma config LPT1OSC = ON     // Low-Power Timer1 Oscillator Enable bit (Timer1 configured for low-power operation)
#pragma config HFOFST = ON      // HFINTOSC Fast Start-up (HFINTOSC starts clocking the CPU without waiting for the oscillator to stablize.)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))
#pragma config CP0 = OFF, CP1 = OFF, CP2 = OFF, CP3 = OFF
#pragma config CPB = OFF, CPD = OFF
#pragma config WRT0 = OFF, WRT1 = OFF, WRT2 = OFF, WRT3 = OFF
#pragma config WRTC = OFF, WRTB = OFF, WRTD = OFF
#pragma config EBTR0 = OFF, EBTR1 = OFF, EBTR2 = OFF, EBTR3 = OFF
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)


#include <xc.h>
#include <stdint.h>
#include <stdio.h>

#define _XTAL_FREQ (4000000ul)
#define FCYC (_XTAL_FREQ/4L)
/*
 * Initialize this PIC
 */
void PIC_Init(void)
{
    INTCON = 0;     /* Disable all interrupt sources */
    PIE1 = 0;
    PIE2 = 0;
    
    OSCCON = 0x50;  /* INTOSC at 4MHz, Primary oscillator selected by configuration bits */
    OSCTUNE = 0x00; /* LFINTOSC 31.25 kHz, PLL disabled */
    ANSEL = 0xFF;   /* make AN0 to AN7 analog inputs */
    ANSELH= 0x0F;   /* make AN8 to AN11 analog inputs, make AN12 digital input */
    TRISA = 0xEF;   /* make RA4 digital output */
    TRISB = 0x1F;   /* RB0/INT0 as input, RB1 to RB4 analog inputs */
    TRISC = 0x03;   /* T1OSC as inputs */
    TRISD = 0x00;
    LATA  = 0;
    LATB  = 0;
    LATC  = 0x40;   /*Set TXD out high on POR */
    LATD  = 0;
}
/*
 * Set the UART baud rate
 */
#define U1_BAUD 9600
#define U1_BRGH_VALUE 1
#define U1_BRG16_VALUE 1
/*
 * Compute the value for the UART1 baud rate registers
 */
#if U1_BRG16_VALUE
    #if U1_BRGH_VALUE
        #define U1_BRGH_SCALE 1L
    #else
        #define U1_BRGH_SCALE 4L
    #endif
#else
    #if U1_BRGH_VALUE
        #define U1_BRGH_SCALE 4L
    #else
        #define U1_BRGH_SCALE 16L
    #endif
#endif

#define U1_SPBRG_VALUE ( ((FCYC + (U1_BRGH_SCALE * U1_BAUD/2)) / (U1_BRGH_SCALE * U1_BAUD))-1L )

#if U1_BRG16_VALUE
    #if U1_SPBRG_VALUE > 65535
    #error Cannot set up UART1 for the FCY and BAUDRATE.
    #endif
#else
    #if U1_BRGREG > 255
    #error Cannot set up UART1 for the FCY and BAUDRATE.
    #endif
#endif
/*
 * Check if the real baud rate is within 2.5 percent of the require baud rate
 */
#define REAL_BAUDRATE ( FCYC / ( U1_BRGH_SCALE * ( U1_SPBRG_VALUE + 1L) ) )
#if (REAL_BAUDRATE > (U1_BAUD + (U1_BAUD * 25L) / 1000L)) || (REAL_BAUDRATE < (U1_BAUD - (U1_BAUD * 25L) / 1000L))
#error UART baudrate error greater than 2.5 percent for the FCY and U1_BAUD.
#endif
#undef REAL_BAUDRATE
/*
 * Define bits used to initialize the UART
 */
#define BIT_RCIE  PIE1bits.RCIE
#define BIT_TXIE  PIE1bits.TXIE
#define BIT_RCIF  PIR1bits.RCIF
#define BIT_TXIF  PIR1bits.TXIF
#define BIT_TXEN  TXSTAbits.TXEN
#define BIT_TRMT  TXSTAbits.TRMT
#define BIT_CREN  RCSTAbits.CREN
#define BIT_SPEN  RCSTAbits.SPEN
#define BIT_OERR  RCSTAbits.OERR
#define BIT_BRG16 BAUDCONbits.BRG16
#define BIT_BRGH  TXSTAbits.BRGH
/*
 * Initialize UART1 for transmit only
 */
void UART_Init( void )
{
    BIT_RCIE = 0;           /* Disable receiver interrupt */
    BIT_TXIE = 0;           /* Disable transmitter interrupt */
    LATCbits.LATC6 = 1;     /* Make TXD a stop bit when */
    TRISCbits.TRISC6 = 0;   /* the UART is disabled */
    RCSTA = 0;              /* Reset receiver and disable UART */
    TXSTA = 0;              /* Reset transmitter */
    BIT_BRG16 = U1_BRG16_VALUE;
    BIT_BRGH = U1_BRGH_VALUE;
    SPBRG = (unsigned char)(U1_SPBRG_VALUE);
    SPBRGH = (unsigned char)(U1_SPBRG_VALUE >> 8);
    BIT_TXEN = 1;           /* Enable transmitter */
    BIT_SPEN = 1;           /* Enable UART */
}
/*
 * This function allows printf to send output to UART1
 */
void putch(char data)
{
    while(!BIT_TRMT);
    TXREG = data;
}
/*
 * INT0 Initialization
 */
void INT0_Init(void)
{
    INTCONbits.INT0IE = 0;
    INTCON2bits.INTEDG0 = 0;
    INTCONbits.INT0IF = 0;
    INTCONbits.INT0IE = 1;
}
/*
 * Main application
 */
uint16_t SpinWait;
uint16_t WakeCount;
uint16_t INT0Count;

void main(void)
{
    /*
     * Application initialization
     */
    PIC_Init();
    UART_Init();
    INT0_Init();
    for(SpinWait=16000;--SpinWait;)Nop();   /* Give clocks time to stabilize */
    printf("\r\n\r\nTest wake from sleep\r\n");
    while(!BIT_TRMT);
    
    INTCONbits.GIEL = 1;
    INTCONbits.GIEH = 1;
    
    WakeCount = 0;
    INT0Count = 0;
    /*
     * Application loop
     */
    WDTCONbits.SWDTEN = 1;      /* Turn on Watchdog */
    for(;;)
    {
        printf("Enter sleep ");
        while(!BIT_TRMT);
        Sleep();
        Nop();
        for(SpinWait=16000;--SpinWait;)Nop();   /* Give clocks time to stabilize */
        if(INT0Count)
        {
            INTCONbits.INT0IE = 0;
            printf("INT0 %u ",INT0Count);
            INT0Count = 0;
            INTCONbits.INT0IE = 1;
        }

        printf("Wake from sleep %u\r\n", ++WakeCount);
        while(!BIT_TRMT);
    }
}
/*
 * Interrupt Service Routine handlers
 */
void __interrupt(high_priority) ISR_high(void)
{
    if(INTCONbits.INT0IE)
    {
        if(INTCONbits.INT0IF)
        {
            INTCONbits.INT0IF = 0;
            INT0Count++;
        }
    }
}
void __interrupt(low_priority) ISR_low(void)
{
    INTCONbits.GIEL = 0;    /* there are no low priority interrupts in this application */
}
