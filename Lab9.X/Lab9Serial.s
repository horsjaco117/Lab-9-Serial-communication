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
    POINTER EQU 0X28
    
     CLRF TIME
 
; Start of Program
; Reset vector address
    PSECT resetVect, class=CODE, delta=2
    GOTO Start
; Interrupt vector
    PSECT isrVect, class=CODE, delta=2
    GOTO INTERRUPT
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
    MOVLW 0X00	    ;1A for max prescalers and postscalers
    MOVWF PIE1        ; Disable all peripheral interrupts
    BSF PIE1,4   ;Enable Transmit interrupts
    BSF PIE1, 5
    ;BCF PIE1, 5  ;Enable Recieve interrupts
    CLRF PR2         ; **CORRECTED: Clear PR2 (no PWM)**
    
    ; UART Setup (Bank 1)
    CLRF ADCON1      ; Left justify
    MOVLW 0X81
    MOVWF SPBRG      ; 9600 baud @ 20MHz
    MOVLW 0X00
    MOVWF SPBRGH
    MOVLW 0X26
    MOVWF TXSTA      ; BRGH=1, TXEN=1 0x26
    
    
 ; Bank 0 (INTCON, ports, peripherals)
BCF STATUS, 5     ; Go to Bank 0
BCF STATUS, 6

; **CRITICAL: Disable ADC completely**
CLRF ADCON0       ; ADON=0, no conversions

; **Enable UART FIRST (clean initialization)**
MOVLW 0x90        ; SPEN=1, CREN=0 0x90
MOVWF RCSTA

MOVLW 0xC0        ; Interrupts disabled COMMAND C0 IS TIMER2 INTERRUPT
MOVWF INTCON
MOVLW 0X00
MOVWF PIR1 
MOVLW 0X00
MOVWF PIR2

; **Disable ALL noise sources**
CLRF CCP1CON      ; No PWM1
CLRF CCP2CON      ; No PWM2
MOVLW 0X00
MOVWF TMR2         ; Reset Timer2
MOVLW 0X00 ;7E FOR FURTHER APPLICATIONS
MOVWF T2CON        ; Timer2 OFF
CLRF ADCON0       ; ADC OFF (redundant but explicit)

; Clear ports AFTER UART
CLRF PORTB
CLRF PORTA
CLRF PORTC

; Disable unused peripherals
CLRF SSPCON       ; No I2C/SPI
CLRF T1CON        ; No Timer1
CLRF PSTRCON      ; No steering
 
 
;For echo check
;MOVLW 0b00100100
;MOVLW 0X24 ;ASCII $ 
;MOVWF TXREG
 
; Main Program Loop
MAINLOOP:
    
FLAGCHECK:
    ;BTFSS PIR1, 4 ;TXIF Check
    ;GOTO FLAGCHECK
    ;MOVLW 0b00100100
    ;MOVLW 0X24 ;ASCII $
    ;MOVWF TXREG
    
   
   ;Recieving Echo POLLING FUNCTIONALITY
   ;BTFSS PIR1, 5
   ;GOTO FLAGCHECK
   ;MOVF RCREG, W
   ;XORLW 0X24
   ;BTFSC STATUS, 2
   ;GOTO FLAGCHECK
   ;XORLW 0X24
   ;MOVWF TXREG
   ;MOVWF PORTB 
   
   ;CLRF PORTA
  ; MOVLW 0XFF
   ;MOVWF PORTA
   
   ;INCF PORTB
   GOTO MAINLOOP
   
INTERRUPT:
    
INTCHECK:
   BTFSS PIR1, 5    ;If the EUART Flag is set then it continues to communication
  ;GOTO SERVO_TX    ;The saved data can actually be sent from this portion
   GOTO INTCHECK    ;Prevents innecessary transmission of zeroes
   MOVF RCREG, W    ;Receiver register
   XORLW 0X24	    ;If the code is 24 status register is set
   BTFSC STATUS, 2  ;Checks for Handshake
   GOTO HANDSHAKE
   XORLW 0X24	    ;if the code isn't a 24 it is reverted back to what it was

   SENDDATA:
   MOVWF TXREG	    ;data prepared to return to the PIC
   MOVWF PORTB	    ;Data display on board for troubleshooting
   ;GOTO _RETFIE
   GOTO SERVO_TX    ;Sends recieved data to the servo controls
   
   HANDSHAKE:
   MOVLW 0X24
   MOVWF TXREG
   
   GOTO _RETFIE
   
   SERVO_TX:
    MOVWF W_TEMP	; Save W
    SWAPF STATUS, W
    MOVWF STATUS_TEMP	; Save STATUS
    
    ;BCF PIR1, 1         ; Clear TMR2IF
    
    CLRF ADC_SAVE       ; Clear position accumulator
    
    BTFSC PORTB, 3     ; Check bit 3
    CALL BIT_3
    BTFSC PORTB, 4     ; Check bit 4
    CALL BIT_4
    BTFSC PORTB, 5     ; Check bit 5
    CALL BIT_5
    BTFSC PORTB, 6     ; Check bit 6
    CALL BIT_6
    BTFSC PORTB, 7     ; Check bit 7
    CALL BIT_7
    
    MOVF ADC_SAVE, W    ; Load position
    ADDWF PCL, F        ; Jump table
    
    GOTO P1
    GOTO P2
    GOTO P2
    GOTO P3
    GOTO P4
    GOTO P4
    GOTO P5
    GOTO P6
    GOTO P6
    GOTO P7
    GOTO P8
    GOTO P8
    GOTO P9
    GOTO P10
    GOTO P10
    GOTO P11
    GOTO P12
    GOTO P12
    GOTO P13
    GOTO P14
    GOTO P14
    GOTO P15
    GOTO P16
    GOTO P16
    GOTO P17
    GOTO P18
    GOTO P18
    GOTO P19
    GOTO P20
    GOTO P20
    GOTO P21
    GOTO P21
    
P1:
    MOVLW 0x32
    MOVWF TIME
    BSF PORTA, 0        ; Modified: Pulse high on PORTB bit 0 (RB0)
    GOTO DELAY
    
P2:
    MOVLW 0x3C
    MOVWF TIME
    BSF PORTA, 0
    GOTO DELAY
    
P3:
    MOVLW 0x46
    MOVWF TIME
    BSF PORTA, 0
    GOTO DELAY
    
P4:
    MOVLW 0x50
    MOVWF TIME
    BSF PORTA, 0
    GOTO DELAY
    
P5:
    MOVLW 0x5A
    MOVWF TIME
    BSF PORTA, 0
    GOTO DELAY
    
P6:
    MOVLW 0x64
    MOVWF TIME
    BSF PORTA, 0
    GOTO DELAY
    
P7:
    MOVLW 0x6E
    MOVWF TIME
    BSF PORTA, 0
    GOTO DELAY
    
P8:
    MOVLW 0x78
    MOVWF TIME
    BSF PORTA, 0
    GOTO DELAY
    
P9:
    MOVLW 0x82
    MOVWF TIME
    BSF PORTA, 0
    GOTO DELAY
    
P10:
    MOVLW 0x8C
    MOVWF TIME
    BSF PORTA, 0
    GOTO DELAY
    
P11:
    MOVLW 0x96
    MOVWF TIME
    BSF PORTA, 0
    GOTO DELAY
    
P12:
    MOVLW 0xA0
    MOVWF TIME
    BSF PORTA, 0
    GOTO DELAY
    
P13:
    MOVLW 0xAA
    MOVWF TIME
    BSF PORTA, 0
    GOTO DELAY
    
P14:
    MOVLW 0xB4
    MOVWF TIME
    BSF PORTA, 0
    GOTO DELAY
    
P15:
    MOVLW 0xBE
    MOVWF TIME
    BSF PORTA, 0
    GOTO DELAY
    
P16:
    MOVLW 0xC8
    MOVWF TIME
    BSF PORTA, 0
    GOTO DELAY
    
P17:
    MOVLW 0xD2
    MOVWF TIME
    BSF PORTA, 0
    GOTO DELAY
    
P18:
    MOVLW 0xDC
    MOVWF TIME
    BSF PORTA, 0
    GOTO DELAY
    
P19:
    MOVLW 0xE6
    MOVWF TIME
    BSF PORTA, 0
    GOTO DELAY
    
P20:
    MOVLW 0xF2
    MOVWF TIME
    BSF PORTA, 0
    GOTO DELAY
    
P21:
    MOVLW 0xFF
    MOVWF TIME
    BSF PORTA, 0
    GOTO DELAY
    
DELAY:
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    NOP
    DECFSZ TIME, F
    GOTO DELAY
    
    BCF PORTA, 0        ; Modified: Pulse low on PORTB bit 0 (RB0)
   ; BSF ADCON0, 1       ; Start next ADC conversion
    
    BCF PIR1, 4		;Flag must be reset due to no tx data transmission.
    GOTO _RETFIE
BIT_3:
    MOVLW 0x01
    ADDWF ADC_SAVE, F
    RETURN
    
BIT_4:
    MOVLW 0x02
    ADDWF ADC_SAVE, F
    RETURN
    
BIT_5:
    MOVLW 0x04
    ADDWF ADC_SAVE, F
    RETURN
    
BIT_6:
    MOVLW 0x08
    ADDWF ADC_SAVE, F
    RETURN
    
BIT_7:
    MOVLW 0x10
    ADDWF ADC_SAVE, F
    RETURN
 

   _RETFIE:
    SWAPF STATUS_TEMP, W
    MOVWF STATUS
    SWAPF W_TEMP, F
    SWAPF W_TEMP, W
    
    
   RETFIE
   END