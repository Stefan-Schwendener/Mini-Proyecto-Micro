;*******************************************************************************
;                                                                              *
;    Filename:    Serial.asm
;    Autor: Rodrigo F Stefan S					
;    Description: Mini Projecto                                     *
;   El código convierte un valor del adc y lo guarda en el puerto b. A la vez
;   lo envía a través del TX. También recibe un dato y este lo muestra en el 
;   puerto d. Para ver funcionar ambos se puede colocar un jumper entre rx y tx
;*******************************************************************************
#include "p16f887.inc"

; CONFIG1
; __config 0xE0F4
 __CONFIG _CONFIG1, _FOSC_INTRC_NOCLKOUT & _WDTE_OFF & _PWRTE_OFF & _MCLRE_OFF & _CP_OFF & _CPD_OFF & _BOREN_OFF & _IESO_OFF & _FCMEN_OFF & _LVP_OFF
; CONFIG2
; __config 0xFFFF
 __CONFIG _CONFIG2, _BOR4V_BOR40V & _WRT_OFF
;*******************************************************************************
GPR_VAR        UDATA
W_TEMP         RES        1      ; w register for context saving (ACCESS)
STATUS_TEMP    RES        1      ; status used for context saving
DELAY1	  RES	    1
DELAY2	  RES	    1
TEMP	  RES	    1
;*******************************************************************************
; Reset Vector
;*******************************************************************************

RES_VECT  CODE    0x0000            ; processor reset vector
    GOTO    START                   ; go to beginning of program

;*******************************************************************************
ISR_VECT  CODE	  0x0004
PUSH:
    MOVWF W_TEMP
    SWAPF STATUS, W
    MOVWF STATUS_TEMP
  
ISR:
			    ; RECIBE EN RX y lo muestra en PORTD
    BTFSS   PIR1, RCIF
    	BCF RCSTA, CREN		; SE REINICIA LA COMUNICACION PARA LIMPIAR EL PUERTO
	BSF RCSTA, CREN
    GOTO    POP
    BTFSC RCSTA, FERR		; VERIFICA SI OCURRIO UN ERROR DE FRAMING
    GOTO ERRORCOM
    BTFSC RCSTA, OERR		; VERIFICA SI OCURRIO UN ERROR DE OVERRUN
    GOTO ERRORCOM
    
    MOVF    RCREG, W
    CALL DELAY_50MS
    MOVWF   TEMP
    
    ERRORCOM
	BCF RCSTA, CREN		; SE REINICIA LA COMUNICACION PARA LIMPIAR EL PUERTO
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
;*******************************************************************************
    CALL    CONFIG_RELOJ		; RELOJ INTERNO DE 500KHz
    CALL    CONFIG_IO
    CALL    CONFIG_TX_RX		; 10417hz
    CALL    CONFIG_ADC			; canal 0, fosc/8, adc on, justificado a la izquierda, Vref interno (0-5V)
    CALL    CONFIG_PWM
    CALL    CONFIG_OSCILATOR
    BANKSEL PORTA
;*******************************************************************************
   
;*******************************************************************************
; CICLO INFINITO
;*******************************************************************************
LOOP:
    CALL    DELAY_50MS
    BSF	    ADCON0, GO		    ; EMPIEZA LA CONVERSIÓN
CHECK_AD:
    BTFSC   ADCON0, GO			; revisa que terminó la conversión
    GOTO    $-1
    BCF	    PIR1, ADIF			; borramos la bandera del adc
    ;RRF	    ADRESH, F		
    ;RRF	    ADRESH, F
   ; RRF	    ADRESH, W		; LE QUITAMOS LOS 3 BITS MENOS SIGNIFICATIVOS A LA CONVERSION
    ;ANDLW   B'00011111'
    MOVFW   ADRESH
    ;MOVWF   CCPR2L
    ;MOVF    CCPR2L,W
    MOVWF   PORTB			; mueve adresh al puerto b
CHECK_RCIF:			    ; RECIBE EN RX y lo muestra en PORTD
    BTFSS   PIR1, RCIF
    GOTO    CHECK_TXIF
    MOVF    RCREG, W
    MOVWF   TEMP
        
    
CHECK_TXIF: 
    MOVFW   PORTB		    ; ENVÍA PORTB POR EL TX
    MOVWF   TXREG
   
    BTFSS   PIR1, TXIF
    GOTO    $-1
    MOVFW TEMP
    MOVWF PORTD
    
PROBAR_SERVO:
    MOVFW TEMP
    MOVWF  CCPR2L
    GOTO LOOP
    
    
;*******************************************************************************
CONFIG_RELOJ
    BANKSEL TRISA
    
    BSF OSCCON, IRCF2
    BCF OSCCON, IRCF1
    BCF OSCCON, IRCF0		    ; FRECUECNIA DE 1MHz
;    
;    BCF OSCCON, OSTS		    ; UTILIZAREMOS RELOJ INTERNO
;    BSF OSCCON, HTS		    ; ESTABLE
;    BSF OSCCON, SCS		    ; SELECCIONAMOS RELOJ INTERNO COMO EL RELOJ DEL SISTEMA
    RETURN
 
 ;--------------------------------------------------------
CONFIG_TX_RX
    BANKSEL TXSTA
    BCF	    TXSTA, SYNC		    ; ASINCRÓNO
    BSF	    TXSTA, BRGH		    ; LOW SPEED
    BANKSEL BAUDCTL
    BSF	    BAUDCTL, BRG16	    ; 8 BITS BAURD RATE GENERATOR
    BANKSEL SPBRG
    MOVLW   .12	    
    MOVWF   SPBRG		    ; CARGAMOS EL VALOR DE BAUDRATE CALCULADO, 2403 baudios a 500Khz
    CLRF    SPBRGH
    BANKSEL RCSTA
    BSF	    RCSTA, SPEN		    ; HABILITAR SERIAL PORT
    BCF	    RCSTA, RX9		    ; SOLO MANEJAREMOS 8BITS DE DATOS
    BSF	    RCSTA, CREN		    ; HABILITAMOS LA RECEPCIÓN 
    BANKSEL TXSTA
    BSF	    TXSTA, TXEN		    ; HABILITO LA TRANSMISION
    
    ;BANKSEL PORTD
    ;CLRF    PORTD
    RETURN
;--------------------------------------
CONFIG_IO
    BANKSEL TRISA
    CLRF    TRISA
    CLRF    TRISB
    CLRF    TRISC
    CLRF    TRISD
    CLRF    TRISE
    BANKSEL ANSEL
    CLRF    ANSEL
    CLRF    ANSELH
    BANKSEL PORTA
    CLRF    PORTA
    CLRF    PORTB
    CLRF    PORTC
    CLRF    PORTD
    CLRF    PORTE
    CLRF    TEMP
    RETURN
;--------------------------------------
CONFIG_OSCILATOR
    BANKSEL TRISA
    BCF OSCCON, IRCF2
    BSF OSCCON, IRCF1
    BSF OSCCON, IRCF0		; FRECUECNIA DE 500kHz
    RETURN
;-----------------------------------------------
CONFIG_ADC
    BANKSEL PORTA
    BCF ADCON0, ADCS1
    BSF ADCON0, ADCS0		; FOSC/8 RELOJ TAD
    
    BCF ADCON0, CHS3		; CH0
    BCF ADCON0, CHS2
    BCF ADCON0, CHS1
    BCF ADCON0, CHS0	
    BANKSEL TRISA
    BCF ADCON1, ADFM		; JUSTIFICACIÓN A LA IZQUIERDA
    BCF ADCON1, VCFG1		; VSS COMO REFERENCIA VREF-
    BCF ADCON1, VCFG0		; VDD COMO REFERENCIA VREF+
    BANKSEL PORTA
    BSF ADCON0, ADON		; ENCIENDO EL MÓDULO ADC
    
    BANKSEL TRISA
    BSF	    TRISA, RA0		; RA0 COMO ENTRADA
    BANKSEL ANSEL
    BSF	    ANSEL, 0		; ANS0 COMO ENTRADA ANALÓGICA
    
    RETURN
;-----------------------------------------------
CONFIG_PWM
    BANKSEL TRISC
    BSF	    TRISC, RC1		; ESTABLEZCO RC1 / CCP2 COMO ENTRADA
    MOVLW   .155
    MOVWF   PR2			    ; COLOCO EL VALOR DEL PERIODO DE MI SEÃAL 20mS
    
    BANKSEL PORTA
    BSF	    CCP2CON, CCP2M3
    BSF	    CCP2CON, CCP2M2
    BSF	    CCP2CON, CCP2M1
    BSF	    CCP2CON, CCP2M0		    ; MODO PWM
    

    
    MOVLW   B'00011011'
    MOVWF   CCPR2L		    ; MSB   DEL DUTY CICLE
    BSF	    CCP2CON, DC2B0
    BSF	    CCP2CON, DC2B1	    ; LSB del duty cicle
    
    BCF	    PIR1, TMR2IF
    
    BSF	    T2CON, T2CKPS1
    BSF	    T2CON, T2CKPS0	    ; PRESCALER 1:16
    
    BSF	    T2CON, TMR2ON	    ; HABILITAMOS EL TMR2
    BTFSS   PIR1, TMR2IF
    GOTO    $-1
    BCF	    PIR1, TMR2IF
    
    BANKSEL TRISC
    BCF	    TRISC, RC1		    ; RC1 / CCP2 SALIDA PWM
    RETURN
;-----------------------------------------------    
DELAY_50MS
    MOVLW   .100		    ; 1US 
    MOVWF   DELAY2
    CALL    DELAY_500US
    DECFSZ  DELAY2		    ;DECREMENTA CONT1
    GOTO    $-2			    ; IR A LA POSICION DEL PC - 1
    RETURN
    
DELAY_500US
    MOVLW   .250		    ; 1US 
    MOVWF   DELAY1	    
    DECFSZ  DELAY1		    ;DECREMENTA CONT1
    GOTO    $-1			    ; IR A LA POSICION DEL PC - 1
    RETURN
    
    END
