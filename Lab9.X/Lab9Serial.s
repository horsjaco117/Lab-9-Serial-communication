; LAB 9
; Jacob Horsley
; RCET
; Fifth Semester
; Serial Communication
; Device: PIC16F883
; GITHUB:https://github.com/horsjaco117/Lab-9-Serial-communication
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
    W_TEMP EQU 0X22
    STATUS_TEMP EQU 0X23
    ADC_SAVE EQU 0X24
    TIME EQU 0X25

 DELAY_COUNT_H EQU 0X26
 DELAY_COUNT_L EQU 0X27
    
     
 
; Start of Program
; Reset vector address
    PSECT resetVect, class=CODE, delta=2
    GOTO Start
; Interrupt vector
    PSECT isrVect, class=CODE, delta=2
    GOTO Start
;    GOTO INTERRUPT
; Setup Code
    PSECT code, class=CODE, delta=2
Start:
Setup:
    ; Bank 3 (ANSELH, ANSEL)
    BSF STATUS, 5    ; Go to Bank 3
    BSF STATUS, 6
    CLRF ANSELH      ; Set pins to digital I/O
    MOVLW 0X00       ; Pin AN0 selected
    MOVWF ANSEL      ; Sets AN0 to analog
    CLRF BAUDCTL
    
    ; Bank 2 (CM2CON0, CM2CON1)
    BSF STATUS, 5
    BCF STATUS, 6    ; Go to Bank 2
    CLRF CM2CON1     ; Disable Comparator 2
    CLRF CM2CON0     ; Disable Comparator 1
   
    ; Bank 1 (TRISB, WPUB, IOCB, OPTION_REG, TRISC, TRISA, PIE1, PR2)
    BSF STATUS, 5
    BCF STATUS, 6    ; Go to Bank 1
    MOVLW 0x00       ; PortB as Outputs
    MOVWF TRISB
    MOVLW 0x00       ; Disable weak pull-ups on Port B
    MOVWF WPUB
    MOVLW 0x00       ; Disable interrupt-on-change for RB0/RB1
    MOVWF IOCB
    CLRF OPTION_REG  ; Enable global pull-ups, clear Timer0 settings
    MOVLW 0X80
    MOVWF TRISC      ; RC7 input (RX), RC6 output (TX)
    MOVLW 0x00       ; RA0 as input
    MOVWF TRISA      ; Set RA0 as input for ADC
    CLRF PIE1        ; Disable all peripheral interrupts
    CLRF PR2         ; **CORRECTED: Clear PR2 (no PWM)**
    
    ; UART Setup (Bank 1)
    CLRF ADCON1      ; Left justify
    MOVLW 0X81
    MOVWF SPBRG      ; 9600 baud @ 20MHz
    MOVLW 0X00
    MOVWF SPBRGH
    MOVLW 0X26
    MOVWF TXSTA      ; BRGH=1, TXEN=1
    
 ; Bank 0 (INTCON, ports, peripherals)
BCF STATUS, 5     ; Go to Bank 0
BCF STATUS, 6

; **CRITICAL: Disable ADC completely**
CLRF ADCON0       ; ADON=0, no conversions

; **Enable UART FIRST (clean initialization)**
MOVLW 0x90        ; SPEN=1, CREN=0
MOVWF RCSTA

MOVLW 0x00        ; Interrupts disabled
MOVWF INTCON
MOVLW 0X00
MOVWF PIR1 
MOVLW 0X00
MOVWF PIR2

; **Disable ALL noise sources**
CLRF CCP1CON      ; No PWM1
CLRF CCP2CON      ; No PWM2
CLRF TMR2         ; Reset Timer2
CLRF T2CON        ; Timer2 OFF
CLRF ADCON0       ; ADC OFF (redundant but explicit)

; Clear ports AFTER UART
CLRF PORTB
CLRF PORTA
CLRF PORTC

; Disable unused peripherals
CLRF SSPCON       ; No I2C/SPI
CLRF T1CON        ; No Timer1
CLRF PSTRCON      ; No steering
   
; Main Program Loop
MAINLOOP:
    
FLAGCHECK:
    BTFSS PIR1, 4 ;TXIF Check
    GOTO FLAGCHECK
    MOVLW 0XFF
    MOVWF TXREG
    
   INCF PORTB
   

   GOTO MAINLOOP
   
   
   
;INTERRUPT:
  ;  RETFIE
END


