;*******************************************************************************
;                                                                              *
;    FILENAME:    SERIAL.ASM						       *
;    AUTHOR: 	  RODRIGO FIGUEROA Y STEFAN SCHWENDENER			       *	
;    DESCRIPTION: MINI PROYECTO DE LABORATORIO DE MICROCONTROLADORES           *
;*******************************************************************************
; PROCESSOR INCLUSSION
;*******************************************************************************
#include "p16f887.inc"
;*******************************************************************************
; STARTUP CONFIGURATION
;*******************************************************************************
; CONFIG1
; __config 0xE0F4
 __CONFIG _CONFIG1, _FOSC_INTRC_NOCLKOUT & _WDTE_OFF & _PWRTE_OFF & _MCLRE_OFF & _CP_OFF & _CPD_OFF & _BOREN_OFF & _IESO_OFF & _FCMEN_OFF & _LVP_OFF
; CONFIG2
; __config 0xFFFF
 __CONFIG _CONFIG2, _BOR4V_BOR40V & _WRT_OFF
;*******************************************************************************
; VARIABLE DEFINITIONS
;*******************************************************************************
GPR_VAR        UDATA
W_TEMP         RES          1      ; w register for context saving (ACCESS)
STATUS_TEMP    RES          1      ; status used for context saving
DELAY1	       RES	    1
DELAY2	       RES	    1
TEMP	       RES	    1
SERVO1	       RES	    1
;*******************************************************************************
; RESET VECTOR
;*******************************************************************************
RES_VECT  CODE    0x0000            ; processor reset vector
    GOTO    START                   ; go to beginning of program
;*******************************************************************************
; INTERRUPT SERVICE ROUTINES
;*******************************************************************************
ISR_VECT  CODE	  0x0004
PUSH:
    MOVWF W_TEMP
    SWAPF STATUS, W
    MOVWF STATUS_TEMP
ISR:
			    ; RECIBE EN RX y lo muestra en PORTD
    BTFSS   PIR1, RCIF
    BCF RCSTA, CREN ; SE REINICIA LA COMUNICACION PARA LIMPIAR EL PUERTO
    BSF RCSTA, CREN
    GOTO    POP
    BTFSC RCSTA, FERR ; VERIFICA SI OCURRIO UN ERROR DE FRAMING
    GOTO ERRORCOM
    BTFSC RCSTA, OERR ; VERIFICA SI OCURRIO UN ERROR DE OVERRUN
    GOTO ERRORCOM
    MOVF    RCREG, W
    CALL DELAY_50MS
    MOVWF   TEMP
    ERRORCOM
    BCF RCSTA, CREN ; SE REINICIA LA COMUNICACION PARA LIMPIAR EL PUERTO
    BSF RCSTA, CREN
    GOTO POP	
POP:
    SWAPF STATUS_TEMP, W  
    MOVWF STATUS
    SWAPF W_TEMP, F
    SWAPF W_TEMP, W
    RETFIE
;*******************************************************************************
; MAIN PROGRAM
;*******************************************************************************
MAIN_PROG CODE                      ; let linker place main program
START
    CALL    CONFIG_IO ;CONFIGURACION DE PUERTOS
    CALL    CONFIG_RELOJ ; RELOJ INTERNO DE 500KHz
    CALL    CONFIG_TX_RX ; 10417hz
    CALL    CONFIG_ADC ; CHANNEL 0, FOSC/8, ADC ENABLED, LEFT JUSTIFICATION, INTERNAL VOLTAGE REF (0-5V)
    CALL    CONFIG_PWM ;CONFIGURACION DEL MODULO PWM
    BANKSEL PORTA ;INGRESAMOS AL BANCO DEL PUERTO A PARA TRABAJAR CON EL ADC
    GOTO    LOOP ;VAMOS AL LOOP PRINCIPAL
;*******************************************************************************
; MAIN LOOP
;*******************************************************************************
LOOP:
    CALL    DELAY_50MS ;50 MS DELAY
    BSF	    ADCON0, GO ; CONVERSION START
CHECK_AD:
    BTFSC   ADCON0, GO ;CHECK WETHER THE FLAG ENABLED
    GOTO    $-1 ;IF FLAG ISN'T CLEAR GOTO PREVIOUS POSITION
    BCF	    PIR1, ADIF; CLEAR ADC INTERRUPT FLAG
    MOVF    ADRESH, W ;MOVEMOS EL VALOR DE CONVERSION A W
    MOVWF   SERVO1 ;MOVEMOS W A UNA VARIABLE QUE UTILZAREMOS MAS ADELANTE
    MOVF    SERVO1, W ;MOVEMOS LA VARIABLE A W
    MOVWF   PORTB; MOVEMOS EL VALOR AL PUERTO B DONDE ESTAN LOS LEDS
CHECK_RCIF:
    BTFSS   PIR1, RCIF ;REVISAMOS SI LA BANDERA DEL EUSART ESTA ENABLED
    GOTO    CHECK_TXIF;IF FLAG IS CLEAR GOTO CHECK TX
    MOVF    RCREG, W; 
    MOVWF   TEMP 
CHECK_TXIF: 
    MOVFW   PORTB ;ENVÍA PORTB POR EL TX
    MOVWF   TXREG
    BTFSS   PIR1, TXIF
    GOTO    $-1
    MOVFW TEMP
    MOVWF PORTD   
SERVO_MOTOR:
    MOVF   SERVO1, W; MOVEMOS LA VARIABLE DEL SERVO A W
    MOVWF  CCPR2L; MANDAMOS LA INFORMACION AL PUERTO DONDE SE ENCUENTRA EL SERVO
    GOTO LOOP ; REGRESAMOS AL LOOP PRINCIPAL
;*******************************************************************************
; MAIN CONFIGURATION
;*******************************************************************************
CONFIG_IO
    BANKSEL TRISA
    CLRF    TRISA ;POTENCIOMETRO EN PUERTO A0 COMO INPUT
    BSF	    TRISA, 0 ;A0 ENABLED
    CLRF    TRISB ;LEDS DE PRUEBA
    CLRF    TRISC ;CCP A SERVO
    CLRF    TRISD ;NADA
    CLRF    TRISE ;NADA
    BANKSEL ANSEL
    CLRF    ANSEL ;NO ANALOG PORTA
    CLRF    ANSELH ;NO ANALOG PORTB
    BANKSEL PORTA
    CLRF    PORTA ;LIMPIEZA DE PUERTOS
    CLRF    PORTB
    CLRF    PORTC
    CLRF    PORTD
    CLRF    PORTE
    CLRF    TEMP ;LIMPIEZA DE VARIABLE TEMPORAL
    RETURN
CONFIG_RELOJ
    BANKSEL TRISA
    BSF OSCCON, IRCF2
    BCF OSCCON, IRCF1
    BCF OSCCON, IRCF0 ; FRECUECNIA DE 1MHz
    RETURN
CONFIG_TX_RX
    BANKSEL TXSTA
    BCF	    TXSTA, SYNC	; ASINCRÓNO
    BSF	    TXSTA, BRGH	; LOW SPEED
    BANKSEL BAUDCTL
    BSF	    BAUDCTL, BRG16 ; 8 BITS BAURD RATE GENERATOR
    BANKSEL SPBRG
    MOVLW   .25	    
    MOVWF   SPBRG ; CARGAMOS EL VALOR DE BAUDRATE CALCULADO
    CLRF    SPBRGH
    BANKSEL RCSTA
    BSF	    RCSTA, SPEN ; HABILITAR SERIAL PORT
    BCF	    RCSTA, RX9 ; SOLO MANEJAREMOS 8BITS DE DATOS
    BSF	    RCSTA, CREN ; HABILITAMOS LA RECEPCIÓN 
    BANKSEL TXSTA
    BSF	    TXSTA, TXEN	; HABILITO LA TRANSMISION
    RETURN
CONFIG_OSCILATOR
    BANKSEL TRISA
    BCF OSCCON, IRCF2
    BSF OSCCON, IRCF1
    BSF OSCCON, IRCF0 ;FRECUECNIA DE 500kHz
    RETURN
CONFIG_ADC
    BANKSEL PORTA
    BCF ADCON0, ADCS1
    BSF ADCON0, ADCS0 ; FOSC/8 RELOJ TAD
    BCF ADCON0, CHS3 ; CH0
    BCF ADCON0, CHS2
    BCF ADCON0, CHS1
    BCF ADCON0, CHS0	
    BANKSEL TRISA
    BCF ADCON1, ADFM ; JUSTIFICACIÓN A LA IZQUIERDA
    BCF ADCON1, VCFG1 ; VSS COMO REFERENCIA VREF-
    BCF ADCON1, VCFG0 ; VDD COMO REFERENCIA VREF+
    BANKSEL PORTA
    BSF ADCON0, ADON ; ENCIENDO EL MÓDULO ADC
    BANKSEL TRISA
    BSF	    TRISA, RA0 ; RA0 COMO ENTRADA
    BANKSEL ANSEL
    BSF	    ANSEL, 0 ; ANS0 COMO ENTRADA ANALÓGICA
    RETURN
CONFIG_PWM
    BANKSEL TRISC
    BSF	    TRISC, RC1 ; ESTABLEZCO RC1 / CCP2 COMO ENTRADA
    MOVLW   .155
    MOVWF   PR2 ; COLOCO EL VALOR DEL PERIODO DE MI SEÃ‘AL 20mS
    BANKSEL PORTA
    BSF	    CCP2CON, CCP2M3
    BSF	    CCP2CON, CCP2M2
    BSF	    CCP2CON, CCP2M1
    BSF	    CCP2CON, CCP2M0 ; MODO PWM
    MOVLW   B'00011011'
    MOVWF   CCPR2L ; MSB   DEL DUTY CICLE
    BSF	    CCP2CON, DC2B0
    BSF	    CCP2CON, DC2B1 ; LSB del duty cicle
    BCF	    PIR1, TMR2IF
    BSF	    T2CON, T2CKPS1
    BSF	    T2CON, T2CKPS0 ; PRESCALER 1:16
    BSF	    T2CON, TMR2ON ; HABILITAMOS EL TMR2
    BTFSS   PIR1, TMR2IF
    GOTO    $-1
    BCF	    PIR1, TMR2IF
    BANKSEL TRISC
    BCF	    TRISC, RC1 ; RC1 / CCP2 SALIDA PWM
    RETURN
;*******************************************************************************
; SUBROUTINES
;*******************************************************************************
DELAY_50MS
    MOVLW   .100 ; 1US 
    MOVWF   DELAY2
    CALL    DELAY_500US
    DECFSZ  DELAY2 ;DECREMENTA CONT1
    GOTO    $-2	; IR A LA POSICION DEL PC - 1
    RETURN   
DELAY_500US
    MOVLW   .250 ; 1US 
    MOVWF   DELAY1	    
    DECFSZ  DELAY1 ;DECREMENTA CONT1
    GOTO    $-1 ; IR A LA POSICION DEL PC - 1
    RETURN
;*******************************************************************************
; END
;*******************************************************************************
    END
