;****************************************************************************************************************************************************;
;   FILENAME: PROGRAM.ASM															     ;
;   NAMES: DIEGO ALEGRIA, RODRIGO FIGUEROA, STEFAN SCHWENDENER											     ;
;   DATE: 23/11/2018																     ;
;****************************************************************************************************************************************************;
;   PROCESSOR INCLUSION
;****************************************************************************************************************************************************;
#include "p16f887.inc"
;****************************************************************************************************************************************************;
;   STARTUP CONFIGURATION
;****************************************************************************************************************************************************;
; CONFIG1
; __config 0xFCD4
 __CONFIG _CONFIG1, _FOSC_INTRC_NOCLKOUT & _WDTE_OFF & _PWRTE_OFF & _MCLRE_OFF & _CP_OFF & _CPD_OFF & _BOREN_OFF & _IESO_OFF & _FCMEN_OFF & _LVP_OFF
; CONFIG2
; __config 0xFFFF
 __CONFIG _CONFIG2, _BOR4V_BOR40V & _WRT_OFF
;****************************************************************************************************************************************************;
;   VARIABLE DEFINITIONS
;****************************************************************************************************************************************************;
GPR_VAR	    UDATA
DELAY1	    RES	1
W_TEMP	    RES	1
STATUS_TEMP RES	1
GVARO	    RES	1
CONTADOR    RES	1
GVARA	    RES	1
GVARE	    RES	1
GVARU	    RES	1
DGVARO	    RES	1
DGVARA	    RES	1
DGVARE	    RES	1
DGVARU	    RES	1
GVAROM	    RES	1
GVARAM	    RES	1
GVAREM	    RES	1
GVARUM	    RES	1
BANDERA	    RES 1
;****************************************************************************************************************************************************;
;   RESET VECTOR
;****************************************************************************************************************************************************;
RES_VECT    CODE    0x0000		; processor reset vector
    GOTO    START			; go to beginning of program
;****************************************************************************************************************************************************;
;   INTERRUPT SERVICE ROUTINES
;****************************************************************************************************************************************************;
ISR_VECT    CODE    0x0004
PUSH:
    MOVWF   W_TEMP
    SWAPF   STATUS, W
    MOVWF   STATUS_TEMP
ISR:
;    BTFSC   INTCON, T0IF
;    CALL    JUMPADC
;    MOVLW   .210
;    MOVWF   TMR0
;    BCF	    INTCON, T0IF
POP:
    SWAPF   STATUS_TEMP, W
    MOVWF   STATUS
    SWAPF   W_TEMP, F
    SWAPF   W_TEMP, W
    RETFIE 
    RETURN
;****************************************************************************************************************************************************;
;   INTERRUPT SERVICE SUBROUTINES
;****************************************************************************************************************************************************;
    
;****************************************************************************************************************************************************;
;   MAIN PROGRAM
;****************************************************************************************************************************************************;
MAIN_PROG CODE                      ; let linker place main program

START
    CALL    CONFIG_OSCILATOR
    CALL    CONFIG_IO
    CALL    CONFIG_ADC
    CALL    CONFIG_PWM
    CALL    CONFIG_TX_RX
    CALL    CONFIG_T0
    CALL    CONFIG_INTERRUPT
    BANKSEL PORTA
    GOTO    LOOP1
;****************************************************************************************************************************************************;
;   MAIN LOOP
;****************************************************************************************************************************************************;   
LOOP1:
    CALL JUMPADC
    GOTO LOOP1
;****************************************************************************************************************************************************;
;   ADC CONVERTER
;****************************************************************************************************************************************************;		
JUMPADC:
    BTFSC BANDERA,0
    GOTO CHECKADC1
    BTFSC BANDERA,1
    GOTO CHECKADC2
    BTFSC BANDERA,2
    GOTO CHECKADC3
    BTFSC BANDERA,3
    GOTO CHECKADC4
CHECKADC1:
    BCF	    ADCON0,CHS0
    BCF	    ADCON0, CHS1
    CALL    DELAY_500US
    BSF    ADCON0, GO
    BTFSC   ADCON0, GO		; LOOP HASTA QUE TERMINE DE CONVERTIR
    GOTO    $-1
    MOVF    ADRESH, W
    BCF	    PIR1, ADIF		; BORRAMOS BANDERA DE
    RRF	    ADRESH, F		
    RRF	    ADRESH, F
    RRF	    ADRESH, W		; LE QUITAMOS LOS 3 BITS MENOS SIGNIFICATIVOS A LA CONVERSION
    ANDLW   B'00011111'
    MOVWF   GVARO			; MOVEMOS EL VALOR HACIA EL PERÍODO DEL PWM
    MOVF    GVARO,W
    CALL    CHECK_TXIF
    CLRF BANDERA
    BSF BANDERA, 1
    RETURN
CHECKADC2:
    BSF	    ADCON0,CHS0
    BCF	    ADCON0, CHS1
    CALL    DELAY_500US
    BSF    ADCON0, GO
    BTFSC   ADCON0, GO		; LOOP HASTA QUE TERMINE DE CONVERTIR
    GOTO    $-1
    MOVF    ADRESH, W
    BCF	    PIR1, ADIF		; BORRAMOS BANDERA DE
    RRF	    ADRESH, F		
    RRF	    ADRESH, F
    RRF	    ADRESH, W		; LE QUITAMOS LOS 3 BITS MENOS SIGNIFICATIVOS A LA CONVERSION
    ANDLW   B'00011111'
    MOVWF   GVARA			; MOVEMOS EL VALOR HACIA EL PERÍODO DEL PWM
    MOVF    GVARA,W
    CALL    CHECK_TXIF
    CLRF BANDERA
    BSF BANDERA, 2
    RETURN
CHECKADC3:
    BCF	    ADCON0,CHS0
    BSF	    ADCON0, CHS1
    CALL    DELAY_500US
    BSF    ADCON0, GO
    BTFSC   ADCON0, GO		; LOOP HASTA QUE TERMINE DE CONVERTIR
    GOTO    $-1
    MOVF    ADRESH, W			
    BCF	    PIR1, ADIF		; BORRAMOS BANDERA DE
    RRF	    ADRESH, F		
    RRF	    ADRESH, F
    RRF	    ADRESH, W		; LE QUITAMOS LOS 3 BITS MENOS SIGNIFICATIVOS A LA CONVERSION
    ANDLW   B'00011111'		
    MOVWF   GVARE
    MOVF   GVARE,W
    MOVF    GVARE,W
    CALL    CHECK_TXIF
    CLRF BANDERA
    BSF BANDERA, 3
    RETURN
CHECKADC4:
    BSF	    ADCON0,CHS0
   BSF	    ADCON0, CHS1
    CALL    DELAY_500US
    BSF    ADCON0, GO
    BTFSC   ADCON0, GO		; LOOP HASTA QUE TERMINE DE CONVERTIR
    GOTO    $-1
    MOVF    ADRESH, W
    BCF	    PIR1, ADIF		; BORRAMOS BANDERA DE
    RRF	    ADRESH, F		
    RRF	    ADRESH, F
    RRF	    ADRESH, W		; LE QUITAMOS LOS 3 BITS MENOS SIGNIFICATIVOS A LA CONVERSION
    ANDLW   B'00011111'		
    MOVWF   GVARU
    MOVF   GVARU,W
    CALL    CHECK_TXIF   
    CLRF BANDERA
    BSF BANDERA,0
    RETURN   
;****************************************************************************************************************************************************;
;   SUBROUTINES
;****************************************************************************************************************************************************;
PRESS1:
    BTFSC PORTD,0
    GOTO $-1
    CALL DELAY_500US
PRESS2:
    BTFSC PORTD,1
    GOTO $-1
    CALL DELAY_500US
    GOTO LOOP1
    
CHECK_RCIF:			    ; RECIBE EN RX y lo muestra en PORTD
    BTFSS   PIR1, RCIF
    GOTO    CHECK_TXIF
    MOVF    RCREG, W
CHECK_TXIF: 		    ; ENVÍA PORTB POR EL TX
    CALL DELAY_500US
    MOVWF   TXREG
    BTFSS   PIR1, TXIF
    GOTO    $-1
    RETURN
DELAY_500US
    MOVLW   .250		    ; 1US 
    MOVWF   DELAY1	    
    DECFSZ  DELAY1		    ;DECREMENTA CONT1
    GOTO    $-1			    ; IR A LA POSICION DEL PC - 1
    RETURN
DELAY_50US
    MOVLW   .25		    ; 1US 
    MOVWF   DELAY1	    
    DECFSZ  DELAY1		    ;DECREMENTA CONT1
    GOTO    $-1			    ; IR A LA POSICION DEL PC - 1
    RETURN
DELAY_SERVO1
    BSF   PORTB,7
    MOVF    GVARO,W
    MOVWF   DGVARO
    DECFSZ  DGVARO
    GOTO    $-1
    INCF   GVAROM
    MOVLW   .1
    SUBWF   GVAROM
    BTFSS   STATUS,Z
    GOTO    DELAY_SERVO1
    BCF	    STATUS,Z
    CLRF    GVAROM
    BCF    PORTB,7
    RETURN
DELAY_SERVO2
    BSF   PORTB,6
    MOVF    GVARA,W
    MOVWF   DGVARA
    DECFSZ  DGVARA
    GOTO    $-1
    INCF   GVARAM
    MOVLW   .1
    SUBWF   GVARAM
    BTFSS   STATUS,Z
    GOTO    DELAY_SERVO2
    BCF	    STATUS,Z
    CLRF    GVARAM
    BCF    PORTB,6
    RETURN
;****************************************************************************************************************************************************;
;   MAIN CONFIGURATION
;****************************************************************************************************************************************************;  
CONFIG_IO
    BANKSEL TRISA
    CLRF    TRISA
    BSF	    TRISA, RA0	; RA0 COMO ENTRADA
    CLRF    TRISB
    CLRF    TRISC
    CLRF    TRISD
    BSF	    TRISD,0
    BSF	    TRISD,1
    CLRF    TRISE
    BANKSEL ANSEL
    CLRF    ANSEL
    CLRF    ANSELH
    BSF	    ANSEL, 0	; ANS0 COMO ENTRADA ANALÓGICA
    BSF	    ANSEL, 1
    BSF	    ANSEL, 2
    BSF	    ANSEL, 3
    BANKSEL PORTA
    CLRF    PORTA
    CLRF    PORTB
    CLRF    PORTC
    CLRF    PORTD
    CLRF    GVARO
    CLRF    GVARA
    CLRF    GVARE
    CLRF    GVARU
    CLRF    GVAROM
    CLRF    GVARAM
    CLRF    GVAREM
    CLRF    GVARUM
    CLRF    DGVARO
    CLRF    DGVARA
    CLRF    DGVARE
    CLRF    DGVARU
    CLRF    BANDERA
    RETURN

CONFIG_INTERRUPT
    BSF	    INTCON, GIE		
    BSF	    INTCON, PEIE
    BSF	    INTCON, T0IE
    BSF	    INTCON, T0IF
    RETURN

CONFIG_ADC
    BANKSEL PORTA
    BCF ADCON0, ADCS1
    BSF ADCON0, ADCS0		; FOSC/8 RELOJ TAD
    
    BCF ADCON0, CHS3		; CANAL 0 PARA LA CONVERSION
    BCF ADCON0, CHS2
    BCF ADCON0, CHS1
    BCF ADCON0, CHS0	
    BANKSEL TRISA
    BCF ADCON1, ADFM		; JUSTIFICACIÓN A LA IZQUIERDA
    BCF ADCON1, VCFG1		; VSS COMO REFERENCIA VREF-
    BCF ADCON1, VCFG0		; VDD COMO REFERENCIA VREF+
    BANKSEL PORTA
    BSF ADCON0, ADON		; ENCIENDO EL MÓDULO ADC
    RETURN

CONFIG_OSCILATOR
    BANKSEL TRISA
    BSF OSCCON, IRCF2
    BCF OSCCON, IRCF1
    BCF OSCCON, IRCF0		; FRECUECNIA DE 1mHZ
    RETURN
CONFIG_T0
    BANKSEL TRISA
    BCF	    OPTION_REG,	T0CS
    BCF	    OPTION_REG,	PSA
    BSF	    OPTION_REG,	PS2
    BSF	    OPTION_REG,	PS1
    BSF	    OPTION_REG,	PS0
    BANKSEL PORTA
    MOVLW   .210
    MOVWF   TMR0
    BCF	    INTCON, T0IF
    RETURN
CONFIG_TX_RX
    BANKSEL TXSTA
    BCF	    TXSTA, SYNC		    ; A/SINCRÓNO
    BSF	    TXSTA, BRGH		    ; sin LOW SPEED
    BANKSEL BAUDCTL
    BSF	    BAUDCTL, BRG16	    ; 8 BITS BAURD RATE GENERATOR
    BANKSEL SPBRG
    MOVLW   .25	    
    MOVWF   SPBRG		    ; CARGAMOS EL VALOR DE BAUDRATE CALCULADO, 9603 baudios a 500Khz
    CLRF    SPBRGH
    BANKSEL RCSTA
    BSF	    RCSTA, SPEN		    ; HABILITAR SERIAL PORT
    BCF	    RCSTA, RX9		    ; SOLO MANEJAREMOS 8BITS DE DATOS
    BSF	    RCSTA, CREN		    ; HABILITAMOS LA RECEPCIÓN 
    BANKSEL TXSTA
    BSF	    TXSTA, TXEN		    ; HABILITO LA TRANSMISION
CONFIG_PWM
    BANKSEL TRISC
    BSF	    TRISC, RC1	
    BSF	    TRISC, RC2		; ESTABLEZCO RC1 / CCP2 COMO ENTRADA
    MOVLW   .47
    MOVWF   PR2			    ; COLOCO EL VALOR DEL PERIODO DE MI SEÑAL 20mS
    
    BANKSEL PORTA
    BSF	    CCP2CON, CCP2M3
    BSF	    CCP2CON, CCP2M2
    BSF	    CCP2CON, CCP2M1
    BSF	    CCP2CON, CCP2M0		    ; MODO PWM
    
    BSF	    CCP1CON, CCP1M3
    BSF	    CCP1CON, CCP1M2
    BSF	    CCP1CON, CCP1M1
    BSF	    CCP1CON, CCP1M0
    
    MOVLW   B'00011011'
    MOVWF   CCPR2L		    ; MSB   DEL DUTY CICLE
    BSF	    CCP2CON, DC2B0
    BSF	    CCP2CON, DC2B1	    ; LSB del duty cicle
    
    MOVLW   B'00011011'
    MOVWF   CCPR1L		    ; MSB   DEL DUTY CICLE
    BSF	    CCP1CON, DC1B0
    BSF	    CCP1CON, DC1B1	    ; LSB del duty cicl
    
    BCF	    PIR1, TMR2IF
    
    BSF	    T2CON, T2CKPS1
    BSF	    T2CON, T2CKPS0	    ; PRESCALER 1:16
    
    BSF	    T2CON, TMR2ON	    ; HABILITAMOS EL TMR2
    BTFSS   PIR1, TMR2IF
    GOTO    $-1
    BCF	    PIR1, TMR2IF
    
    BANKSEL TRISC
    BCF	    TRISC, RC1		    ; RC1 / CCP2 SALIDA PWM
    BCF	    TRISC, RC2		    ; RC1 / CCP2 SALIDA PWM
    RETURN
;****************************************************************************************************************************************************;
;   END
;****************************************************************************************************************************************************;	
END