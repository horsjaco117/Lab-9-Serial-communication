; LAB 8
; Jacob Horsley
; RCET
; Fifth Semester
; ADC and lights (modified to servo control)
; Device: PIC16F883
; GITHUB: https://github.com/horsjaco117/Lab8_Part2
;-------------------------------------------------------------------------------
; Configuration
    ; CONFIG1
    CONFIG FOSC = HS ; Oscillator Selection bits (XT oscillator)
    CONFIG WDTE = OFF ; Watchdog Timer Enable bit (WDT disabled)
    CONFIG PWRTE = OFF ; Power-up Timer Enable bit (PWRT disabled)
    CONFIG MCLRE = ON ; RE3/MCLR pin function select bit (MCLR)
    CONFIG CP = OFF ; Code Protection bit (Program memory code protection disabled)
    CONFIG CPD = OFF ; Data Code Protection bit (Data memory code protection disabled)
    CONFIG BOREN = OFF ; Brown Out Reset Selection bits (BOR disabled)
    CONFIG IESO = OFF ; Internal External Switchover bit (disabled)
    CONFIG FCMEN = OFF ; Fail-Safe Clock Monitor Enabled bit (disabled)
    CONFIG LVP = OFF ; Low Voltage Programming Enable bit (RB3 is digital I/O)
    ; CONFIG2
    CONFIG BOR4V = BOR40V ; Brown-out Reset Selection bit (set to 4.0V)
    CONFIG WRT = OFF ; Flash Program Memory Self Write Enable bits (Write protection off)
   
; Include Statements
#include <xc.inc>
   
; Code Section
;------------------------------------------------------------------------------
; Register/Variable Setup
    W_TEMP EQU 0X23
    STATUS_TEMP EQU 0X24
    ADC_SAVE EQU 0X22
    TIME EQU 0X33
     
 
; Start of Program
; Reset vector address
    PSECT resetVect, class=CODE, delta=2
    GOTO Start
; Interrupt vector
    PSECT isrVect, class=CODE, delta=2
    GOTO INTERRUPT
; Setup Code
    PSECT code, class=CODE, delta=2
Start:
Setup:
    ; Bank 3 (ANSELH, ANSEL)
    BSF STATUS, 5 ; Go to Bank 3
    BSF STATUS, 6
    CLRF ANSELH ; Set pins to digital I/O
    MOVLW 0X01 ; Pin AN0 selected
    MOVWF ANSEL ; Sets Pins to analog
    CLRF BAUDCTL
   
    ; Bank 2 (CM2CON0, CM2CON1)
    BSF STATUS, 5
    BCF STATUS, 6 ; Go to Bank 2
    CLRF CM2CON1 ; Disable Comparator 2
    CLRF CM2CON0 ; Disable Comparator 1
   
    ; Bank 1 (TRISB, WPUB, IOCB, OPTION_REG, TRISC, TRISA, PIE1, PR2)
    BSF STATUS, 5
    BCF STATUS, 6 ; Go to Bank 1
    MOVLW 0x00 ; PortB as Outputs
    MOVWF TRISB
    MOVLW 0x00 ; Disable weak pull-ups on Port B
    MOVWF WPUB
    MOVLW 0x00 ; Disable interrupt-on-change for RB0/RB1
    MOVWF IOCB
    CLRF OPTION_REG ; Enable global pull-ups, clear Timer0 settings
    MOVLW 0X00
    MOVWF TRISC ; Port C as outputs
    MOVLW 0x01 ; RA0 as an input
    MOVWF TRISA ; Set RA0 as input for ADC
    MOVLW 0x02 ; TMR2IE enabled
    MOVWF PIE1
    MOVLW 0xFF ; Timer2 period set for ~20ms (with pre/postscaler)
    MOVWF PR2
    CLRF ADCON1 ; Left justify
    MOVLW 0X81
    MOVWF SPBRG
    MOVLW 0X00
    MOVWF SPBRGH
    MOVLW 0X10
    MOVWF TXSTA
   
    ; Bank 0 (INTCON, ports, peripherals)
    BCF STATUS, 5 ; Go to Bank 0
    BCF STATUS, 6

    MOVLW 0X41 ; AN0 pin selected, ADON=1, ADCS=01 (Fosc/8)
    MOVWF ADCON0
    MOVLW 0x00 ; Interrupts disabled initially
    MOVWF INTCON
    MOVLW 0XC0  ; Clear peripheral interrupt flags
    MOVWF PIR1 
    MOVLW 0XC0 ; Enable GIE and PEIE
    MOVWF INTCON
    CLRF CCP1CON ; Disable PWM
    MOVLW 0X00
    MOVWF PORTB ; Clear Port B
    CLRF CCP2CON ; Disable second PWM
    CLRF PORTA ; Clear Port A
    CLRF PORTC ; Clear Port C
    MOVLW 0X80
    MOVWF RCSTA ; Disable serial control
    CLRF SSPCON ; Disable serial port
    CLRF T1CON ; Disable Timer1
    CLRF PSTRCON ; Disable PWM pulse steering
    CLRF CM2CON1 ; Disable Comparator 2
    MOVLW 0x00
    MOVWF TMR2 ; Reset Timer2
    MOVLW 0x26 ; Prescaler 1:16, postscaler 1:5, TMR2ON=1
    MOVWF T2CON
    BSF ADCON0, 1 ; Start initial ADC conversion
   
; Main Program Loop
MAINLOOP:
   GOTO MAINLOOP
   
INTERRUPT:
    RETFIE
END


