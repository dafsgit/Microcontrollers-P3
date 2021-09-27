/* 
 * File:   main.c
 * Author: dafne
 *
 * Created on 20 de septiembre de 2021, 12:02 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <p18f4550.h>
#include <stdint.h>

/*
 * 
 */
void decode_seven_PORTD(unsigned char code){
    //assume display is wired to PORTD
    switch(code){
        //***************hgfedcba
        case 0: PORTD = 0b00111111;
                break;
        case 1: PORTD = 0b00000110;
                break;
        case 2: PORTD = 0b01011011;
                break;
        case 3: PORTD = 0b01001111;
                break;
        case 4: PORTD = 0b01100110;
                break;
        case 5: PORTD = 0b01101101;
                break;
        case 6: PORTD = 0b01111101;
                break;
        case 7: PORTD = 0b00000111;
                break;
        case 8: PORTD = 0b01111111;
                break;
        case 9: PORTD = 0b01101111;
                break;
        case 10: PORTD = 0b01110111;
                break;
        case 11: PORTD = 0b01111100;
                break;
        case 12: PORTD = 0b00111001;
                break;
        case 13: PORTD = 0b01011110;
                break;
        case 14: PORTD = 0b01111001;
                break;
        case 15: PORTD = 0b01110001;
                break;
    }
}

void decode_seven_PORTA(unsigned char code){
    //assume display is wired to PORTA
    switch(code){
        //***************hgfedcba
        case 0: PORTA = 0b00111111;
                break;
        case 1: PORTA = 0b00000110;
                break;
        case 2: PORTA = 0b01011011;
                break;
        case 3: PORTA = 0b01001111;
                break;
        case 4: PORTA = 0b01100110;
                break;
        case 5: PORTA = 0b01101101;
                break;
        case 6: PORTA = 0b01111101;
                break;
        case 7: PORTA = 0b00000111;
                break;
        case 8: PORTA = 0b01111111;
                break;
        case 9: PORTA = 0b01101111;
                break;
        case 10: PORTA = 0b01110111;
                break;
        case 11: PORTA = 0b01111100;
                break;
        case 12: PORTA = 0b00111001;
                break;
        case 13: PORTA = 0b01011110;
                break;
        case 14: PORTA = 0b01111001;
                break;
        case 15: PORTA = 0b01110001;
                break;
    }
}

void decode_seven_PORTB(unsigned char code){
    //assume display is wired to PORTB
    switch(code){
        //***************hgfedcba
        case 0: PORTB = 0b00111111;
                break;
        case 1: PORTB = 0b00000110;
                break;
        case 2: PORTB = 0b01011011;
                break;
        case 3: PORTB = 0b01001111;
                break;
        case 4: PORTB = 0b01100110;
                break;
        case 5: PORTB = 0b01101101;
                break;
        case 6: PORTB = 0b01111101;
                break;
        case 7: PORTB = 0b00000111;
                break;
        case 8: PORTB = 0b01111111;
                break;
        case 9: PORTB = 0b01101111;
                break;
        case 10: PORTB = 0b01110111;
                break;
        case 11: PORTB = 0b01111100;
                break;
        case 12: PORTB = 0b00111001;
                break;
        case 13: PORTB = 0b01011110;
                break;
        case 14: PORTB = 0b01111001;
                break;
        case 15: PORTB = 0b01110001;
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
    TRISCbits.RC0 = 0;
    TRISCbits.RC1 = 1;
    
    // SCS PRIOSC; IDLEN disabled; IRCF 4MHz; 
    OSCCON = 0x60;
    // INTSRC 31kHz derived from INTRC internal oscillator; TUN = 0
    OSCTUNE = 0x00;
    
    INTCONbits.GIE = 1;
    INTCONbits.PEIE = 1;
    PIR1bits.TMR1IF = 0;
    PIE1bits.TMR1IE = 1;
    
    T1CONbits.TMR1CS = 1; //idk
    T1CONbits.T1CKPS = 0b00;
    T1CONbits.T1SYNC = 1;
    T1CONbits.RD16 = 1;    
    
    while(1){
        // generate the trigger pulse in RC0 (10 us)
        PORTCbits.RC0 = 1;
        _delay(10);
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
        
        
        // delay 60 us
        _delay(60);
    }
    return 1;
}
