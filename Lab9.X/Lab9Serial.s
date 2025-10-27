; Jacob Horsley
; RCET
; Fifth Semester
; Serial Communication with ADC
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
    HANDSHAKE_FLAG EQU 0X2C
    RESULT_HI EQU 0X2D
    RESULT_LO EQU 0X2E
   
     CLRF TIME
     CLRF TIMER_COUNT
     CLRF HANDSHAKE_FLAG
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
    MOVLW 0X08 ; Pin AN3 selected
    MOVWF ANSEL ; Sets AN3 to analog
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
    MOVLW 0X08 ; RA3 as input
    MOVWF TRISA ; Set RA3 as input for ADC
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
; Enable ADC on AN3
MOVLW 0x8D ; ADCS=10 (Fosc/32), CHS=0011 (AN3), ADON=1
MOVWF ADCON0
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
    BSF ADCON0, 1 ; Start ADC conversion
WAIT_ADC:
    BTFSC ADCON0, 1 ; Check if GO/DONE is still set
    GOTO WAIT_ADC
    MOVF ADRESH, W ; Get high 8 bits (left justify)
    MOVWF RESULT_HI
    BTFSS PIR1, 4 ; Wait for TXIF (TX ready)
    GOTO $-1
    MOVWF TXREG ; Send ADC high byte via serial
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
    BTFSS HANDSHAKE_FLAG, 0
    GOTO INTERRUPT_END
    XORLW 0X24
SENDDATA:
    MOVWF TXREG
    MOVWF PORTB
    BCF HANDSHAKE_FLAG, 0
    GOTO INTERRUPT_END
    
HANDSHAKE:
    MOVLW 0X24
    MOVWF TXREG
    BSF HANDSHAKE_FLAG, 0
    GOTO INTERRUPT_END
    
HANDLE_SERVO:
    DECFSZ TIMER_COUNT, F
    GOTO INTERRUPT_END
    
    ;SEARCHING FOR THE RIGHT BITS
    BTFSC PORTB,7
    GOTO Bx1UUUU
    BTFSC PORTB,6
    GOTO Bx01UUU
    BTFSC PORTB,5
    GOTO Bx001UU
    BTFSC PORTB,4
    GOTO Bx0001U
    GOTO Bx0000U
    
Bx1UUUU:
    BTFSC PORTB,6
    GOTO Bx11UUU
    BTFSC PORTB,5
    GOTO Bx101UU
    BTFSC PORTB,4
    GOTO Bx1001U
    GOTO Bx1000U
    
Bx11UUU:
    BTFSC PORTB,5
    GOTO Bx111UU
    BTFSC PORTB,4
    GOTO Bx1101U
    GOTO Bx1100U
    
Bx101UU:
    BTFSC PORTB,4
    GOTO Bx1011U
    GOTO Bx1010U
    
Bx111UU:
    BTFSC PORTB,4
    GOTO Bx1111U
    GOTO Bx1110U
    
Bx01UUU:
    BTFSC PORTB,5
    GOTO Bx011UU
    BTFSC PORTB,4
    GOTO Bx0101U
    GOTO Bx0100U
    
Bx011UU:
    BTFSC PORTB,4
    GOTO Bx0111U
    GOTO Bx0110U
    
Bx001UU:
    BTFSC PORTB,4
    GOTO Bx0011U
    GOTO Bx0010U
    
Bx0000U:
    BTFSC PORTB,3
    GOTO Bx00001
    GOTO Bx00000
    
Bx0001U:
    BTFSC PORTB,3
    GOTO Bx00011
    GOTO Bx00010
    
Bx0010U:
    BTFSC PORTB,3
    GOTO Bx00101
    GOTO Bx00100
    
Bx0011U:
    BTFSC PORTB,3
    GOTO Bx00111
    GOTO Bx00110
    
Bx0100U:
    BTFSC PORTB,3
    GOTO Bx01001
    GOTO Bx01000
    
Bx0101U:
    BTFSC PORTB,3
    GOTO Bx01011
    GOTO Bx01010
    
Bx0110U:
    BTFSC PORTB,3
    GOTO Bx01101
    GOTO Bx01100
    
Bx0111U:
    BTFSC PORTB,3
    GOTO Bx01111
    GOTO Bx01110
    
Bx1000U:
    BTFSC PORTB,3
    GOTO Bx10001
    GOTO Bx10000
    
Bx1001U:
    BTFSC PORTB,3
    GOTO Bx10011
    GOTO Bx10010
    
Bx1010U:
    BTFSC PORTB,3
    GOTO Bx10101
    GOTO Bx10100
    
Bx1011U:
    BTFSC PORTB,3
    GOTO Bx10111
    GOTO Bx10110
    
Bx1100U:
    BTFSC PORTB,3
    GOTO Bx11001
    GOTO Bx11000
    
Bx1101U:
    BTFSC PORTB,3
    GOTO Bx11011
    GOTO Bx11010
    
Bx1110U:
    BTFSC PORTB,3
    GOTO Bx11101
    GOTO Bx11100
    
Bx1111U:
    BTFSC PORTB,3
    GOTO Bx11111
    GOTO Bx11110
    
;SETTING THE RIGHT TIMES FOR THE SELECTED BITS
Bx00000:
    MOVLW 0x26 ;38
    MOVWF PR2_PulseWidth
    MOVLW 0xBD ;189
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
    
Bx00001:
    MOVLW 0x2B ;43
    MOVWF PR2_PulseWidth
    MOVLW 0xBD ;189
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
    
Bx00010:
    MOVLW 0x30 ;48
    MOVWF PR2_PulseWidth
    MOVLW 0xBC ;188
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
    
Bx00011:
    MOVLW 0x35 ;53
    MOVWF PR2_PulseWidth
    MOVLW 0xBC ;188
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
    
Bx00100:
    MOVLW 0x3A ;58
    MOVWF PR2_PulseWidth
    MOVLW 0xBB ;187
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
    
Bx00101:
    MOVLW 0x3F ;63
    MOVWF PR2_PulseWidth
    MOVLW 0xBA ;186
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
    
Bx00110:
    MOVLW 0x44 ;68
    MOVWF PR2_PulseWidth
    MOVLW 0xBA ;186
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
    
Bx00111:
    MOVLW 0x49 ;73
    MOVWF PR2_PulseWidth
    MOVLW 0xB9 ;185
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
    
Bx01000:
    MOVLW 0x4E ;78
    MOVWF PR2_PulseWidth
    MOVLW 0xB8 ;184
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
    
Bx01001:
    MOVLW 0x53 ;83
    MOVWF PR2_PulseWidth
    MOVLW 0xB8 ;184
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
    
Bx01010:
    MOVLW 0x58 ;88
    MOVWF PR2_PulseWidth
    MOVLW 0xB7 ;183
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
    
Bx01011:
    MOVLW 0x5E ;94
    MOVWF PR2_PulseWidth
    MOVLW 0xB6 ;182
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
    
Bx01100:
    MOVLW 0x63 ;99
    MOVWF PR2_PulseWidth
    MOVLW 0xB6 ;182
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
    
Bx01101:
    MOVLW 0x68 ;104
    MOVWF PR2_PulseWidth
    MOVLW 0xB5 ;181
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
    
Bx01110:
    MOVLW 0x6D ;109
    MOVWF PR2_PulseWidth
    MOVLW 0xB5 ;181
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
    
Bx01111:
    MOVLW 0x72 ;114
    MOVWF PR2_PulseWidth
    MOVLW 0xB4 ;180
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
    
Bx10000:
    MOVLW 0x77 ;119
    MOVWF PR2_PulseWidth
    MOVLW 0xB3 ;179
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
    
Bx10001:
    MOVLW 0x7C ;124
    MOVWF PR2_PulseWidth
    MOVLW 0xB3 ;179
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
    
Bx10010:
    MOVLW 0x81 ;129
    MOVWF PR2_PulseWidth
    MOVLW 0xB2 ;178
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
    
Bx10011:
    MOVLW 0x86 ;134
    MOVWF PR2_PulseWidth
    MOVLW 0xB1 ;177
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
    
Bx10100:
    MOVLW 0x8B ;139
    MOVWF PR2_PulseWidth
    MOVLW 0xB1 ;177
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
    
Bx10101:
    MOVLW 0x90 ;144
    MOVWF PR2_PulseWidth
    MOVLW 0xB0 ;176
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
    
Bx10110:
    MOVLW 0x95 ;149
    MOVWF PR2_PulseWidth
    MOVLW 0xB0 ;176
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
    
Bx10111:
    MOVLW 0x9A ;154
    MOVWF PR2_PulseWidth
    MOVLW 0xAF ;175
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
    
Bx11000:
    MOVLW 0x9F ;159
    MOVWF PR2_PulseWidth
    MOVLW 0xAE ;174
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
    
Bx11001:
    MOVLW 0xA4 ;164
    MOVWF PR2_PulseWidth
    MOVLW 0xAE ;174
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
    
Bx11010:
    MOVLW 0xA9 ;169
    MOVWF PR2_PulseWidth
    MOVLW 0xAD ;173
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
    
Bx11011:
    MOVLW 0xAE ;174
    MOVWF PR2_PulseWidth
    MOVLW 0xAC ;172
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
    
Bx11100:
    MOVLW 0xB3 ;179
    MOVWF PR2_PulseWidth
    MOVLW 0xAC ;172
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
    
Bx11101:
    MOVLW 0xB8 ;184
    MOVWF PR2_PulseWidth
    MOVLW 0xAB ;171
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
    
Bx11110:
    MOVLW 0xBD ;189
    MOVWF PR2_PulseWidth
    MOVLW 0xAB ;171
    MOVWF PR2_PulseSpace
    BTFSC PulseSelect, 0
    GOTO PulseWidthTime
    GOTO PulseSpaceTime
    
Bx11111:
    MOVLW 0xC2 ;194
    MOVWF PR2_PulseWidth
    MOVLW 0xAA ;170
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