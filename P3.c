/* 
 * File:   main.c
 * Author: dafne
 *
 * Created on 20 de septiembre de 2021, 12:02 PM
 */

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.


// PIC18F4550 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1L
#pragma config PLLDIV = 5       // PLL Prescaler Selection bits (Divide by 5 (20 MHz oscillator input))
#pragma config CPUDIV = OSC3_PLL4// System Clock Postscaler Selection bits ([Primary Oscillator Src: /3][96 MHz PLL Src: /4])
#pragma config USBDIV = 2       // USB Clock Selection bit (used in Full-Speed USB mode only; UCFG:FSEN = 1) (USB clock source comes from the 96 MHz PLL divided by 2)

// CONFIG1H
#pragma config FOSC = HSPLL_HS  // Oscillator Selection bits (HS oscillator, PLL enabled (HSPLL))
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor disabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRT = ON        // Power-up Timer Enable bit (PWRT enabled)
#pragma config BOR = OFF        // Brown-out Reset Enable bits (Brown-out Reset disabled in hardware and software)
#pragma config BORV = 3         // Brown-out Reset Voltage bits (Minimum setting 2.05V)
#pragma config VREGEN = OFF     // USB Voltage Regulator Enable bit (USB voltage regulator disabled)

// CONFIG2H
#pragma config WDT = OFF        // Watchdog Timer Enable bit (WDT disabled (control is placed on the SWDTEN bit))
#pragma config WDTPS = 32768    // Watchdog Timer Postscale Select bits (1:32768)

// CONFIG3H
#pragma config CCP2MX = OFF     // CCP2 MUX bit (CCP2 input/output is multiplexed with RB3)
#pragma config PBADEN = OFF     // PORTB A/D Enable bit (PORTB<4:0> pins are configured as digital I/O on Reset)
#pragma config LPT1OSC = OFF    // Low-Power Timer 1 Oscillator Enable bit (Timer1 configured for higher power operation)
#pragma config MCLRE = ON       // MCLR Pin Enable bit (MCLR pin enabled; RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = OFF     // Stack Full/Underflow Reset Enable bit (Stack full/underflow will not cause Reset)
#pragma config LVP = OFF        // Single-Supply ICSP Enable bit (Single-Supply ICSP disabled)
#pragma config ICPRT = OFF      // Dedicated In-Circuit Debug/Programming Port (ICPORT) Enable bit (ICPORT disabled)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection bit (Block 0 (000800-001FFFh) is not code-protected)
#pragma config CP1 = OFF        // Code Protection bit (Block 1 (002000-003FFFh) is not code-protected)
#pragma config CP2 = OFF        // Code Protection bit (Block 2 (004000-005FFFh) is not code-protected)
#pragma config CP3 = OFF        // Code Protection bit (Block 3 (006000-007FFFh) is not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) is not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM is not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection bit (Block 0 (000800-001FFFh) is not write-protected)
#pragma config WRT1 = OFF       // Write Protection bit (Block 1 (002000-003FFFh) is not write-protected)
#pragma config WRT2 = OFF       // Write Protection bit (Block 2 (004000-005FFFh) is not write-protected)
#pragma config WRT3 = OFF       // Write Protection bit (Block 3 (006000-007FFFh) is not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) are not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot block (000000-0007FFh) is not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM is not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection bit (Block 0 (000800-001FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection bit (Block 1 (002000-003FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection bit (Block 2 (004000-005FFFh) is not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection bit (Block 3 (006000-007FFFh) is not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot block (000000-0007FFh) is not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include <stdio.h>
#include <stdlib.h>
#include <p18f4550.h>
#include <stdint.h>
#define _XTAL_FREQ  24000000ul
#define __delay_us(x) _delay((unsigned long)((x)*(_XTAL_FREQ/4000000.0)))

/*
 * 
 */

// common catode instructions are commented
void decode_seven_PORTD(int code){
    //assume display is wired to PORTD
    switch(code){
        //***************hgfedcba
        case 0: //PORTD = 0b00111111;
                PORTD = 0b01000000;
                break;
        case 1: //PORTD = 0b00000110;
                PORTD = 0b01111001;
                break;
        case 2: //PORTD = 0b01011011;
                PORTD = 0b00100100;
                break;
        case 3: //PORTD = 0b01001111;
                PORTD = 0b00110000;
                break;
        case 4: //PORTD = 0b01100110;
                PORTD = 0b00011001;
                break;
        case 5: //PORTD = 0b01101101;
                PORTD = 0b00010010;
                break;
        case 6: //PORTD = 0b01111101;
                PORTD = 0b00000010;
                break;
        case 7: //PORTD = 0b00000111;
                PORTD = 0b01111000;
                break;
        case 8: //PORTD = 0b01111111;
                PORTD = 0b00000000;
                break;
        case 9: //PORTD = 0b01101111;
                PORTD = 0b00010000;
                break;
        case 10: //PORTD = 0b01110111;
                 PORTD = 0b00001000;
                break;
        case 11: //PORTD = 0b01111100;
                 PORTD = 0b00000011;
                break;
        case 12: //PORTD = 0b00111001;
                 PORTD = 0b01000110;
                break;
        case 13: //PORTD = 0b01011110;
                 PORTD = 0b00100001;
                break;
        case 14: //PORTD = 0b01111001;
                 PORTD = 0b00000110;
                break;
        case 15: //PORTD = 0b01110001;
                 PORTD = 0b00001110;
                break;
    }
}

void decode_seven_PORTA(int code){
    //assume display is wired to PORTA
    switch(code){
        //***************hgfedcba
        case 0: //PORTA = 0b00111111;
                PORTA = 0b01000000;
                break;
        case 1: //PORTA = 0b00000110;
                PORTA = 0b01111001;
                break;
        case 2: //PORTA = 0b01011011;
                PORTA = 0b00100100;
                break;
        case 3: //PORTA = 0b01001111;
                PORTA = 0b00110000;
                break;
        case 4: //PORTA = 0b01100110;
                PORTA = 0b00011001;
                break;
        case 5: //PORTA = 0b01101101;
                PORTA = 0b00010010;
                break;
        case 6: //PORTA = 0b01111101;
                PORTA = 0b00000010;
                break;
        case 7: //PORTA = 0b00000111;
                PORTA = 0b01111000;
                break;
        case 8: //PORTA = 0b01111111;
                PORTA = 0b00000000;
                break;
        case 9: //PORTA = 0b01101111;
                PORTA = 0b00010000;
                break;
        case 10: //PORTA = 0b01110111;
                 PORTA = 0b00001000;
                break;
        case 11: //PORTA = 0b01111100;
                 PORTA = 0b00000011;
                break;
        case 12: //PORTA = 0b00111001;
                 PORTA = 0b01000110;
                break;
        case 13: //PORTA = 0b01011110;
                 PORTA = 0b00100001;
                break;
        case 14: //PORTA = 0b01111001;
                 PORTA = 0b00000110;
                break;
        case 15: //PORTA = 0b01110001;
                 PORTA = 0b00001110;
                break;
    }
}

void decode_seven_PORTB(int code){
    //assume display is wired to PORTB
    switch(code){
        //***************hgfedcba
        case 0: //PORTB = 0b00111111;
                PORTB = 0b01000000;
                break;
        case 1: //PORTB = 0b00000110;
                PORTB = 0b01111001;
                break;
        case 2: //PORTB = 0b01011011;
                PORTB = 0b00100100;
                break;
        case 3: //PORTB = 0b01001111;
                PORTB = 0b00110000;
                break;
        case 4: //PORTB = 0b01100110;
                PORTB = 0b00011001;
                break;
        case 5: //PORTB = 0b01101101;
                PORTB = 0b00010010;
                break;
        case 6: //PORTB = 0b01111101;
                PORTB = 0b00000010;
                break;
        case 7: //PORTB = 0b00000111;
                PORTB = 0b01111000;
                break;
        case 8: //PORTB = 0b01111111;
                PORTB = 0b00000000;
                break;
        case 9: //PORTB = 0b01101111;
                PORTB = 0b00010000;
                break;
        case 10: //PORTB = 0b01110111;
                 PORTB = 0b00001000;
                break;
        case 11: //PORTB = 0b01111100;
                 PORTB = 0b00000011;
                break;
        case 12: //PORTB = 0b00111001;
                 PORTB = 0b01000110;
                break;
        case 13: //PORTB = 0b01011110;
                 PORTB = 0b00100001;
                break;
        case 14: //PORTB = 0b01111001;
                 PORTB = 0b00000110;
                break;
        case 15: //PORTB = 0b01110001;
                 PORTB = 0b00001110;
                break;
    }
}

uint16_t TMR1_ReadTimer(void)
{
    uint16_t readVal;
    uint8_t readValHigh;
    uint8_t readValLow;
    
    readValLow = TMR1L;
    readValHigh = TMR1H;
    
    readVal = ((uint16_t)readValHigh << 8) | readValLow;

    return readVal;
}

void TMR1_WriteTimer(uint16_t timerVal)
{
    if (T1CONbits.T1SYNC == 1)
    {
        // Stop the Timer by writing to TMRxON bit
        T1CONbits.TMR1ON = 0;

        // Write to the Timer1 register
        TMR1H = (timerVal >> 8);
        TMR1L = (uint8_t) timerVal;

        // Start the Timer after writing to the register
        T1CONbits.TMR1ON =1;
    }
    else
    {
        // Write to the Timer1 register
        TMR1H = (timerVal >> 8);
        TMR1L = (uint8_t) timerVal;
    }
}

int main(int argc, char** argv) {
    //PORTA, PORTB and PORTE are digital
    ADCON1bits.PCFG = 0xF; 
    
    TRISA = 0b00000000;
    TRISB = 0b00000000;
    TRISD = 0b00000000;
    TRISCbits.TRISC0 = 0;
    TRISCbits.TRISC1 = 1;
    
    // SCS PRIOSC; IDLEN disabled; IRCF 4MHz; 
    OSCCON = 0x60;
    // INTSRC 31kHz derived from INTRC internal oscillator; TUN = 0
    OSCTUNE = 0x00;
    
    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 1;
    PIR1bits.TMR1IF = 0;
    PIE1bits.TMR1IE = 1;
    
    T1CONbits.TMR1CS = 0;
    T1CONbits.T1CKPS = 0b00;
    T1CONbits.T1SYNC = 1;
    T1CONbits.RD16 = 1;    
    
    while(1){
        // generate the trigger pulse in RC0 (10 us)
        PORTCbits.RC0 = 1;
        __delay_us(10);
        PORTCbits.RC0 = 0;
        // measure the pulse duration with Timer1
        while(PORTCbits.RC1 == 0);
        TMR1_WriteTimer(0);
        T1CONbits.TMR1ON = 1;
        while(PORTCbits.RC1 == 1);
        T1CONbits.TMR1ON = 0;
                
        // TCys to us
        float time = (float)TMR1_ReadTimer()/(float)6; // 1 TCy = 1/6 us
        // us to cm
        float cm = time/(float)58;
        
        // display
        cm = cm * 10;
        int uni = (int)cm % 10;
        cm = cm / 10;
        int dec = (int)cm % 10;
        cm = cm / 10;
        int cen = (int)cm % 10;
        decode_seven_PORTD(uni);
        decode_seven_PORTB(dec);
        decode_seven_PORTA(cen);
        
        // delay 60 us
        __delay_us(60);
    }
    return 1;
}
