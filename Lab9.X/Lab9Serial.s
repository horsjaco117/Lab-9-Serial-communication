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
    TIME EQU 0X25
    TIMER_COUNT EQU 0X26
    PR2_PulseWidth EQU 0X27
    PR2_PulseSpace EQU 0X28
    PulseSelect EQU 0X2B
   
     CLRF TIME
     CLRF TIMER_COUNT
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
    MOVWF ANSEL ; Sets AN0 to analog
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
    MOVLW 0X80
    MOVWF TRISC ; RC7 input (RX), RC6 output (TX)
    MOVLW 0X01 ; RA0 as input
    MOVWF TRISA ; Set RA0 as input for ADC
    MOVLW 0X02 ; TMR2IE enabled
    MOVWF PIE1 ; Disable all peripheral interrupts
    BSF PIE1, 5 ; Enable Receive interrupts
    MOVLW 0XF0
    MOVWF PR2 ; Initial PR2
   
    ; UART Setup (Bank 1)
    CLRF ADCON1 ; Left justify
    MOVLW 0X81
    MOVWF SPBRG ; 9600 baud @ 20MHz
    MOVLW 0X00
    MOVWF SPBRGH
    MOVLW 0X26
    MOVWF TXSTA ; BRGH=1, TXEN=1 0x26
   
 ; Bank 0 (INTCON, ports, peripherals)
BCF STATUS, 5 ; Go to Bank 0
BCF STATUS, 6
; Disable ADC completely
CLRF ADCON0 ; ADON=0, no conversions
; Enable UART FIRST (clean initialization)
MOVLW 0x90 ; SPEN=1, CREN=1 0x90
MOVWF RCSTA
MOVLW 0xC0 ; GIE=1 PEIE=1
MOVWF INTCON
MOVLW 0X00
MOVWF PIR1
MOVLW 0X00
MOVWF PIR2
; Disable ALL noise sources
CLRF CCP1CON ; No PWM1
CLRF CCP2CON ; No PWM2
MOVLW 0X00
MOVWF TMR2 ; Reset Timer2
MOVLW 0X7E
MOVWF T2CON ; Timer2 ON with settings for space
CLRF ADCON0 ; ADC OFF (redundant but explicit)
; Clear ports AFTER UART
CLRF PORTB
CLRF PORTA
CLRF PORTC
; Disable unused peripherals
CLRF SSPCON ; No I2C/SPI
CLRF T1CON ; No Timer1
CLRF PSTRCON ; No steering
  
; Main Program Loop
MAINLOOP:
    GOTO MAINLOOP
  
INTERRUPT:
    MOVWF W_TEMP
    SWAPF STATUS, W
    MOVWF STATUS_TEMP
    BTFSC PIR1, 5
    GOTO HANDLE_SERIAL
    BTFSC PIR1, 1
    GOTO HANDLE_SERVO
    GOTO INTERRUPT_END

HANDLE_SERIAL:
    MOVF RCREG, W
    XORLW 0X24
    BTFSC STATUS, 2
    GOTO HANDSHAKE
    XORLW 0X24
SENDDATA:
    MOVWF TXREG
    MOVWF PORTB
    GOTO INTERRUPT_END
   
HANDSHAKE:
    MOVLW 0X24
    MOVWF TXREG
    GOTO INTERRUPT_END
   
HANDLE_SERVO:
    DECFSZ TIMER_COUNT, F
    GOTO INTERRUPT_END
   
    ;SEARCHING FOR THE RIGHT BITS
    BTFSC PORTB,7
    GOTO Bx1UUU
    BTFSC PORTB,6
    GOTO Bx01UU
    BTFSC PORTB,5
    GOTO Bx001U
    BTFSC PORTB,4
    GOTO Bx0001
    GOTO Bx0000
   
Bx1UUU:
    BTFSC PORTB,6
    GOTO Bx11UU
    BTFSC PORTB,5
    GOTO Bx101U
    BTFSC PORTB,4
    GOTO Bx1001
    GOTO Bx1000
   
Bx11UU:
    BTFSC PORTB,5
    GOTO Bx111U
    BTFSC PORTB,4
    GOTO Bx1101
    GOTO Bx1100
   
Bx101U:
    BTFSC PORTB,4
    GOTO Bx1011
    GOTO Bx1010
   
Bx111U:
    BTFSC PORTB,4
    GOTO Bx1111
    GOTO Bx1110
   
Bx01UU:
    BTFSC PORTB,5
    GOTO Bx011U
    BTFSC PORTB,4
    GOTO Bx0101
    GOTO Bx0100
   
Bx011U:
    BTFSC PORTB,4
    GOTO Bx0111
    GOTO Bx0110
   
Bx001U:
    BTFSC PORTB,4
    GOTO Bx0011
    GOTO Bx0010
   
;SETTING THE RIGHT TIMES FOR THE SELECTED BITS
Bx0000:
    MOVLW 0x4D
    MOVWF PR2_PulseWidth
    MOVLW 0xB9
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
   
Bx0001:
    MOVLW 0x52
    MOVWF PR2_PulseWidth
    MOVLW 0xB8
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
   
Bx0010:
    MOVLW 0x57
    MOVWF PR2_PulseWidth
    MOVLW 0xB7
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
   
Bx0011:
    MOVLW 0x5D
    MOVWF PR2_PulseWidth
    MOVLW 0xB7
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
   
Bx0100:
    MOVLW 0x62
    MOVWF PR2_PulseWidth
    MOVLW 0xB6
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
   
Bx0101:
    MOVLW 0x67
    MOVWF PR2_PulseWidth
    MOVLW 0xB5
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
   
Bx0110:
    MOVLW 0x6C
    MOVWF PR2_PulseWidth
    MOVLW 0xB5
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
   
Bx0111:
    MOVLW 0x71
    MOVWF PR2_PulseWidth
    MOVLW 0xB4
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
   
Bx1000:
    MOVLW 0x77
    MOVWF PR2_PulseWidth
    MOVLW 0xB3
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
   
Bx1001:
    MOVLW 0x7C
    MOVWF PR2_PulseWidth
    MOVLW 0xB3
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
   
Bx1010:
    MOVLW 0x81
    MOVWF PR2_PulseWidth
    MOVLW 0xB2
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
   
Bx1011:
    MOVLW 0x86
    MOVWF PR2_PulseWidth
    MOVLW 0xB1
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
   
Bx1100:
    MOVLW 0x8B
    MOVWF PR2_PulseWidth
    MOVLW 0xB1
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
   
Bx1101:
    MOVLW 0x91
    MOVWF PR2_PulseWidth
    MOVLW 0xB0
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
   
Bx1110:
    MOVLW 0x96
    MOVWF PR2_PulseWidth
    MOVLW 0xAF
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
   
Bx1111:
    MOVLW 0x9B
    MOVWF PR2_PulseWidth
    MOVLW 0xAF
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
   
PulseWidthTime:
    MOVF PR2_PulseWidth,0
    BSF STATUS,5
    MOVWF PR2
    BCF STATUS,5
    MOVLW 0x01
    MOVWF TIMER_COUNT
    BSF PORTA,1
    BCF PulseSelect, 0
    MOVLW 0x7D
    MOVWF T2CON
    GOTO INTERRUPT_END
   
PulseSpaceTime:
    MOVF PR2_PulseSpace,0
    BSF STATUS,5
    MOVWF PR2
    BCF STATUS,5
    MOVLW 0x02
    MOVWF TIMER_COUNT
    BCF PORTA,1
    BSF PulseSelect, 0
    MOVLW 0x7E
    MOVWF T2CON
    GOTO INTERRUPT_END
   
INTERRUPT_END:
    BCF PIR1,1 ;Clears TMR2 to PR2 Interrupt Flag
    CLRF TMR2 ;Clears TMR2
    SWAPF STATUS_TEMP, W
    MOVWF STATUS
    SWAPF W_TEMP, F
    SWAPF W_TEMP, W
    RETFIE
    END