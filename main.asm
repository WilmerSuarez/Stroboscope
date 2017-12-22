;***************************************************************************
;*
;* Title: Stroboscope
;* Author: Wilmer Suarez	
;* Version: 1.0
;* Last updated: 12/21/2017
;* Target: ATmega324A
;* 
;***************************************************************************

//========================================= GLOBAL_VARIABLES =========================================//
.dseg
freq_L: .byte 1 ;low byte of frequency setting in binary
freq_H: .byte 1 ;high byte of frequency setting in binary
rpm_L:	.byte 1 ;rpm low byte in binary
rpm_H:	.byte 1 ;rpm high byte in binary
.cseg

//========================================== VECTOR_TABLE ==========================================//
reset:
.org RESET				// Reset interrupt vector
	rjmp start					
.org INT0addr			// INT0 interrupt vector
	rjmp pushbutton_isr
.org OC1Aaddr			// Timer/Counter 1 Overflow interrupt vector
	rjmp pulse_isr	

//========================================= INCLUDE_FILEs =========================================//
.nolist
.include "m324adef.inc"
.include "lcd_dog_asm_driver_m324a.inc" // LCD DOG init/update procedures
.list

//========================================= CONFIGURATION =========================================//
start:
	//----- INITIALIZE_LCD_DISPLAY_USING_SPI -----//
	call init_lcd_dog	// Initialize LCD Display
	call clr_dsp_buffs	// Clear all dsp_buff_1-3

	//--------------------- PORT_SET_UP ---------------------//
	ldi r16, $FF		// Make PORTB and output	
	out DDRB, r16
	ldi r16, $01		// Make PD2 (Interrupt INT0) an input 
	out DDRD, r16		// (to read debounced signal from EO)
						// and PD0 an output (pulse output)

	//------------------- INITIALIZE_STACK -------------------//
    ldi r16, LOW(RAMEND)    // Load low byte of stack pointer
	out SPL, r16		     
	ldi r16, HIGH(RAMEND)   // Load high byte of stack pointer
    out SPH, r16

	// ------------------- INITIAL_VALUE_FOR_OCR1A_&_FREQ_&_RPM ------------------- //
	ldi r17, HIGH(1000)		// Load inital value of 1000Hz into FREQ Variable 
	ldi r16, LOW(1000)
	sts freq_H, r17
	sts freq_L, r16
	ldi r17, HIGH(60000)	// Load inital value of 60000 RPM into RPM Variable 
	ldi r16, LOW(60000)
	sts rpm_H, r17
	sts rpm_L, r16
	ldi r17, HIGH(125)		// Load initial value of 125 into OCR1A
	ldi r16, LOW(125)			
	sts OCR1AH, r17
	sts OCR1AL, r16

	//-------- TIMER1_INTERRUPT_OUTPUT_COMPARE_MATCH_A --------//
	ldi r16, (1 << OCIE1A)	// Compare_Match_A interrupt enable
	sts TIMSK1, r16

	//------------------- CONFIGURE_INTERRUPT0_SENCE_CONTROL_BITS -------------------//
	ldi r16, (1 << ISC00) | (1 << ISC01)	// Set interrupt to trigger at rising edge
	sts EICRA, r16						
	ldi r16, (1 << INT0)	// Enable INT0 Interrupt
	out EIMSK, r16

	//--------------------- TIMER_MODE=FAST_PWM_WITH_TOP_@OCR1A ---------------------//
	ldi r16, (1 << WGM11) | (1 << WGM10)	// TIMER COUNTER 1 @ FAST PWM (MODE 15)
	sts TCCR1A, r16	
	ldi r16, (1 << WGM13) | (1 << WGM12) | (1 << CS11)   // Prescaler = (CLK_I/O) / 8    
	sts TCCR1B, r16
    
	//-------------------- LOAD_frequency_display_INTO_dsp_buff_1 --------------------//
	ldi  ZH, high(frequency_display<<1)	// Point to beginning of frequency_display line
	ldi  ZL, low(frequency_display<<1)   
	rcall load_msg	// Load message into dsp_buff_1 (LCD line 1)

	//--------------- LOAD_frequency_display_INTO_dsp_buff_2 ---------------//
	ldi  ZH, high(rpm_display<<1)  // Point to beginning of rpm_display line
	ldi  ZL, low(rpm_display<<1)   
	rcall load_msg	// Load message into dsp_buff_2 (LCD line 2)

	call update_lcd_dog	// Display current contents of dsp_buff_1-3 on LCD  	
	
	//----- FREQUENCY_&_RPM_STRING_TABLE -----//
	frequency_display: .db 1, "FREQ: 01000", 0  ; message for line #1.
	rpm_display: .db 2, "RPM: 60000", 0  ; message for line #2.

	//-------- ENABLE_GLOGBAL_INTERRUPTS --------//
	sei			// Set the Status Register global 
				// interrupt bit (I)

//=========================================== MAIN_LOOP ===========================================//
main_loop:
	nop
	rjmp main_loop

//============================================= ISRs =============================================//
;***************************************************************************
;* 
;* "pushbutton_isr" - title
;*
;* Description: Interrupt Service rouine triggered when any button is 
;* pressed.
;* Subroutine checks which button was pressed and then executes the 
;* corresponding buttons function. 
;*
;* Author: Wilmer Suarez
;* Version: 1.0
;* Last updated: 12/22/2017
;* Target: ATmega324a
;*
;***************************************************************************
pushbutton_isr:
	//----- STORE_SREG_AND_USED_REGISTERS -----//
	push r16		
	in r16, SREG
	push r16
	push r17
	push r18
	push r19

delay_1:
	ldi r16, 255
	call var_delay	// Delay for some time (to ensure button press)
key_press:
	sbis PIND, 2	// Skip if PD2 is set (button pressed)
	rjmp done		// If button is not pressed, return

//--------------------- INPUT DATA ---------------------//
	in r16, PINA	
	swap r16		// Take bits 4-7 (/A0, /A1, /A2)
	com r16			// Complement bits 
	andi r16, $07	// Mask 4th bit in lower nibble (Only want bits 0-2)

/------------------- CHECK_BUTTONS -------------------//
	cpi r16, $07	// If button 7 was pressed, half OCR1A
	breq half
	cpi r16, $06	// If button 6 was pressed, double OCR1A
	breq double
	cpi r16, $05	// If button 5 was pressed, add 50 to OCR1A
	breq add_50
	cpi r16, $04	// If button 4 was pressed, subtract 50 from OCR1A
	breq sub50
	cpi r16, $03	// If button 3 was pressed, add 10 to OCR1A
	breq add10
	cpi r16, $02	// If button 2 was pressed, subtract 10 from OCR1A
	breq sub10
	cpi r16, $00	// If button 0 was pressed, decrement frequency by 1
	breq freq_plus
	cpi r16, $01	// If button 1 was pressed, increment frequency by 1
	breq freq_minus

	rjmp continue

//--------------------- HALF_OCR1A ---------------------//
is_zero_half:
	cpi r16, $01
	breq half_done
	rjmp continue_half
half:
	lds r16, OCR1AL	// Read value of OCR1A
	lds r17, OCR1AH
	cpi r17, $00		// If high register is less than one
	breq is_zero_half	// check low register
continue_half:
	ldi r19, $00	// Load $0002 
	ldi r18, $02
	call div16u		// Divide OCR1A by 2 
	sts OCR1AH, r17	// Store value of OCR1A halved
	sts OCR1AL, r16
half_done:
	rjmp done		

//-------------------- DOUBLE_OCR1A --------------------//
double:
	lds r16, OCR1AL		// Read value of OCR1A
	lds r17, OCR1AH
	sbrc r17, 7
	rjmp double_done	
	ldi r19, $00		// Load $0002 
	ldi r18, $02
	call mpy16u			// Multiply OCR1A by 2 
	sts OCR1AH, r19		// Store value of OCR1A doubled
	sts OCR1AL, r18
check_press_6:
	ldi r16, 255
	call var_delay		// Delay
	sbic PORTD, 2		// If button still pressed, call sub_50 again
	rjmp double
double_done:
	rjmp done

//-------------------- ADD_50_TO_OCR1A --------------------//
add_50:
	lds r24, OCR1AL		// Read value of OCR1A
	lds r25, OCR1AH
	sbrc r25, 7		
	rjmp add_50_done
	adiw r25:r24, 50
	sts OCR1AH, r25		// Store value of OCR1A + 50
	sts OCR1AL, r24
check_press_5:
	ldi r16, 255
	call var_delay		// Delay
	sbic PORTD, 2		// If button still pressed, call sub_50 again
	rjmp add_50
add_50_done:
	rjmp done

//-------------------- SUB_50_TO_OCR1A --------------------//
is_one_sub_50:
	cpi r16, $32
	breq sub_50_done
	rjmp continue_sub_50
sub_50:
	lds r24, OCR1AL		// Read value of OCR1A
	lds r25, OCR1AH
	cpi r17, $01		// If high register is less than one
	brlo is_one_sub_50	// check low register
continue_sub_50:
	sbiw r25:r24, 50
	sts OCR1AH, r25		// Store value of OCR1A - 50
	sts OCR1AL, r24
check_press_4:
	ldi r16, 255
	call var_delay		// Delay
	sbic PORTD, 2		// If button still pressed, call sub_50 again
	rjmp sub_50
sub_50_done:
	rjmp done

//-------------------- ADD_10_TO_OCR1A --------------------//
add_10:
	lds r24, OCR1AL		// Read value of OCR1A
	lds r25, OCR1AH
	sbrc r25, 7		
	rjmp add_10_done
	adiw r25:r24, 10
	sts OCR1AH, r25		// Store value of OCR1A + 10
	sts OCR1AL, r24
check_press_3:
	ldi r16, 255
	call var_delay		// Delay
	sbic PORTD, 2		// If button still pressed, call sub_50 again
	rjmp add_10
add_10_done:
	rjmp done

//-------------------- SUB_10_TO_OCR1A --------------------//
sub_10:
	lds r24, OCR1AL		// Read value of OCR1A
	lds r25, OCR1AH
	sbiw r25:r24, 10
	sts OCR1AH, r25		// Store value of OCR1A - 10
	sts OCR1AL, r24
check_press_2:
	ldi r16, 255
	call var_delay		// Delay
	sbic PORTD, 2		// If button still pressed, call sub_50 again
	rjmp sub_10

freq_minus:
	call freq_minus_1
	rjmp continue
freq_plus:
	call freq_plus_1
	rjmp continue

continue:
	sei				// Enable global interrupt to allow pulse to continue updating
	
	call update_lcd_dog	// Display current contents of dsp_buff_1-3 on LCD

	ldi r16, 255
	call var_delay	// Delay for some time (to ensure button press)

	rjmp delay_1	// Check if button is still pressed

done:
	//----- RESTORE_SREG_AND_USED_REGISTERS -----//
	pop r19
	pop r18
	pop r17
	pop r16
	out SREG, r16
	pop r16

	reti

;***************************************************************************
;* 
;* "pulse_isr" - title
;*
;* Description: Interrrupt Service Routine triggered when the timer/counter 1
;* compare match A occurs.  
;* The subroutine sends a pulse at Pin 2, Port D for 1ms. 
;*
;* Author: Wilmer Suarez
;* Version: 1.0
;* Last updated: 12/22/2017
;* Target: ATmega324a
;*
;***************************************************************************
pulse_isr: 
	//----- STORE_SREG_AND_USED_REGISTERS -----//
	push r16
	in r16, SREG
	push r16
	push r17

	sbi PORTD, 0		// Set PORT_D, PIN_0
	ldi r16, 10			// Variable to delay for 1ms
	call var_delay
	cbi PORTD, 0		// Clear PORT_D, PIN_0

	//----- RESTORE_SREG_AND_USED_REGISTERS -----//
	pop r17
	pop r16
	out SREG, r16
	pop r16
	reti

//========================================== SUBROUTINES ==========================================//
;***************************************************************************
;* 
;* "freq_minus_1" - title
;*
;* Description: Subroutine called when button 1 is pressed. The subroutine
;* decreases the frequency by 1. 
;*
;* Author: Wilmer Suarez
;* Version: 1.0
;* Last updated: 12/22/2017
;* Target: ATmega324a
;*
;***************************************************************************
freq_minus_1:
	//----- STORE_SREG_AND_USED_REGISTERS -----//
	push r16
	in r16, SREG
	push r16
	push r24
	push r25

	lds r24, freq_L		// Load frequency value
	lds r25, freq_H

	sbiw r25:r24, 1		// Subtract one from the frequency 

	sts freq_L, r24		// Store updated frequency value
	sts freq_H, r25

	call freq_ocr1a_ldval	// Update OCR1A
	call x_60_rpm			// Update RPM

	call dsp_update_freq	// Display updated frequency on LCD
	call dsp_update_rpm		// Display updated RPM on LCD

	//----- RESTORE_SREG_AND_USED_REGISTERS -----//
	pop r25
	pop r24
	pop r16
	out SREG, r16
	pop r16

	ret

;***************************************************************************
;* 
;* "freq_plus_1" - title
;*
;* Description: Subroutine called when button 0 is pressed. The subroutine
;* increases the frequency by 1.
;*
;* Author: Wilmer Suarez
;* Version: 1.0
;* Last updated: 12/22/2017
;* Target: ATmega324a
;*
;***************************************************************************
freq_plus_1:
	//----- STORE_SREG_AND_USED_REGISTERS -----//
	push r16
	in r16, SREG
	push r16
	push r24
	push r25

	lds r24, freq_L		// Load frequency value
	lds r25, freq_H

	adiw r25:r24, 1		// Add one to the frequency 

	sts freq_L, r24		// Store updated frequency value
	sts freq_H, r25

	call freq_ocr1a_ldval	// Update OCR1A
	call x_60_rpm			// Update RPM

	call dsp_update_freq	// Display updated frequency on LCD
	call dsp_update_rpm		// Display updated RPM on LCD

	//----- RESTORE_SREG_AND_USED_REGISTERS -----//
	pop r25
	pop r24
	pop r16
	out SREG, r16
	pop r16

	ret

;***************************************************************************
;* 
;* "freq_ocr1a_ldval" - title
;*
;* Description: This subroutine calculates the OCR1A from the value of
;* the frequency. OCR1A = C(125000)/FREQUENCY.
;*
;* Author: Wilmer Suarez
;* Version: 1.0
;* Last updated: 12/22/2017
;* Target: ATmega324a 
;*
;***************************************************************************
freq_ocr1a_ldval:
	//----- STORE_SREG_AND_USED_REGISTERS -----//
	push r12
	push r13
	push r14
	push r15
	push r18
	in r18, SREG
	push r18
	push r19
	push r20
	push r21
	push r22
	push r23
	push r24
	push r25

	//----- CONSTANT_VALUE(125000)_INTO_DIVIDEND -----//
	ldi r18, LOW(125000)
	ldi r19, BYTE2(125000)
	ldi r20, BYTE3(125000)
	ldi r21, BYTE4(125000)

	//----- FREQUENCY_VALUE_INTO_DIVISOR -----//
	lds r22, freq_L
	lds r23, freq_H
	ldi r24, 0
	ldi r25, 0

	call div32u	// Divide (Cf)/(frequency)

	sts OCR1AH, r19	// Store Quotient into OCR1A
	sts OCR1AL, r18

	//----- RESTORE_SREG_AND_USED_REGISTERS -----//
	pop r25
	pop r24
	pop r23
	pop r22
	pop r21
	pop r20
	pop r19
	pop r18
	out SREG, r18
	pop r18
	pop r15
	pop r14
	pop r13
	pop r12

	ret

;***************************************************************************
;* 
;* "rpm" - title
;*
;* Description: This subrtouine calculates the RPM from the frequency. 
;* RPM = FREQUENCY * 60
;*
;* Author: Wilmer Suarez
;* Version: 1.0
;* Last updated: 12/22/2017
;* Target: ATmega324a 
;*
;***************************************************************************
 x_60_rpm:
	//----- STORE_SREG_AND_USED_REGISTERS -----//
	push r16
	in r16, SREG
	push r16
	push r17
	push r18
	push r19
	push r20
	push r21
	push r22

	ldi r16, LOW(60)	// Multiplicand (60)
	ldi r17, HIGH(60)

	lds r18, freq_L		// Multiplier (Frequency variable)
	lds r19, freq_H

	call mpy16u	// Multiply Frequency * 60 = RPM

	sts rpm_H, r19
	sts rpm_L, r18	// Store product in RPM variable 

	//----- RESTORE_SREG_AND_USED_REGISTERS -----//
	pop r22
	pop r21
	pop r20
	pop r19
	pop r18
	pop r17
	pop r16
	out SREG, r16
	pop r16

	ret

;***************************************************************************
;* 
;* "dsp_update_freq" - title
;*
;* Description: This subroutine displays the value of the current frequency
;* on the LCD.
;*
;* Author: Wilmer Suarez
;* Version: 1.0
;* Last updated: 12/22/2017
;* Target: ATmega324a 
;*
;***************************************************************************
 dsp_update_freq:
	//----- STORE_SREG_AND_USED_REGISTERS -----//
	push r16
	in r16, SREG
	push r16
	push r17
	push r18
	push r19
	push r20
	push r13
	push r14
	push r15

	lds r16, freq_L		// Loading Frequency value
	lds r17, freq_H

	call bin2BCD16		// Converting Frequency value to BCD

	//---------- Digit_1 ----------//
	mov r16, r13		// Copy 1st and 2nd digit to r16
	mov r17, r13		// Copy 1st and 2nd digit to r17
	andi r16, $0F		// Mask high byte of r16 to acquire 1st digit
	ori r16, $30		// Convert 1st digit to ASCII
	
	//---------- Digit_2 ----------//
	swap r17			// Swap nibbles in r17
	andi r17, $0F		// Mask high byte of r17 to acquire 2nd digit
	ori r17, $30		// Convert 2nd digit to ASCII

	//---------- Digit_3 ----------//
	mov r18, r14		// Copy 3rd and 4th digit to r18
	mov r19, r14		// Copy 3rd and 4th digit to r19
	andi r18, $0F		// Mask high byte of r18 to acquire 3rd digit
	ori r18, $30		// Convert 3rd digit to ASCII	

	//---------- Digit_4 ----------//
	swap r19			// Swap nibbles in r19
	andi r19, $0F		// Mask high byte of r19 to acquire 4th digit
	ori r19, $30		// Convert 4th digit to ASCII

	//---------- Digit_5 ----------//
	mov r20, r15		// Copy 5th digit to r20
	andi r20, $0F		// Mask high byte of r20 to acquire 5th digit 
	ori r20, $30		// Convert 5th digit to ASCII

	sts dsp_buff_1 + 10, r16	// Display Digit 1
	sts dsp_buff_1 + 9, r17		// Display Digit 2
	sts dsp_buff_1 + 8, r18		// Display Digit 3
	sts dsp_buff_1 + 7, r19		// Display Digit 4
	sts dsp_buff_1 + 6, r20		// Display Digit 5

	call update_lcd_dog	// Display current contents of dsp_buff_1-3 on LCD 

	//----- RESTORE_SREG_AND_USED_REGISTERS -----//
	pop r15
	pop r14
	pop r13
	pop r20
	pop r19
	pop r18
	pop r17
	pop r16
	out SREG, r16
	pop r16

	ret

;***************************************************************************
;* 
;* "dsp_update_rpm" - title
;*
;* Description: This subroutine displays the current value of the rpm on the
;* LCD.
;*
;* Author: Wilmer Suarez
;* Version: 1.0
;* Last updated: 12/22/2017
;* Target: ATmega324a 
;*
;***************************************************************************
 dsp_update_rpm:
	//----- STORE_SREG_AND_USED_REGISTERS -----//
	push r16
	in r16, SREG
	push r16
	push r17
	push r18
	push r19
	push r20
	push r13
	push r14
	push r15

	lds r16, rpm_L		// Loading RPM value
	lds r17, rpm_H

	call bin2BCD16		// Converting RPM value to BCD

	//---------- Digit_1 ----------//
	mov r16, r13		// Copy 1st and 2nd digit to r16
	mov r17, r13		// Copy 1st and 2nd digit to r17
	andi r16, $0F		// Mask high byte of r16 to acquire 1st digit
	ori r16, $30		// Convert 1st digit to ASCII
	
	//---------- Digit_2 ----------//
	swap r17			// Swap nibbles in r17
	andi r17, $0F		// Mask high byte of r17 to acquire 2nd digit
	ori r17, $30		// Convert 2nd digit to ASCII

	//---------- Digit_3 ----------//
	mov r18, r14		// Copy 3rd and 4th digit to r18
	mov r19, r14		// Copy 3rd and 4th digit to r19
	andi r18, $0F		// Mask high byte of r18 to acquire 3rd digit
	ori r18, $30		// Convert 3rd digit to ASCII	

	//---------- Digit_4 ----------//
	swap r19			// Swap nibbles in r19
	andi r19, $0F		// Mask high byte of r19 to acquire 4th digit
	ori r19, $30		// Convert 4th digit to ASCII

	//---------- Digit_5 ----------//
	mov r20, r15		// Copy 5th digit to r20
	andi r20, $0F		// Mask high byte of r20 to acquire 5th digit 
	ori r20, $30		// Convert 5th digit to ASCII

	sts dsp_buff_2 + 9, r16		// Display Digit 1
	sts dsp_buff_2 + 8, r17		// Display Digit 2
	sts dsp_buff_2 + 7, r18		// Display Digit 3
	sts dsp_buff_2 + 6, r19		// Display Digit 4
	sts dsp_buff_2 + 5, r20		// Display Digit 5
	
	call update_lcd_dog	// Display current contents of dsp_buff_1-3 on LCD 

	//----- RESTORE_SREG_AND_USED_REGISTERS -----//
	pop r15
	pop r14
	pop r13
	pop r20
	pop r19
	pop r18
	pop r17
	pop r16
	out SREG, r16
	pop r16

	ret

;***************************************************************************
;*
;* "bin2BCD16" - 16-bit Binary to BCD conversion
;*
;* This subroutine converts a 16-bit number (fbinH:fbinL) to a 5-digit
;* packed BCD number represented by 3 bytes (tBCD2:tBCD1:tBCD0).
;* MSD of the 5-digit number is placed in the lowermost nibble of tBCD2.
;*
;* Number of words	:25
;* Number of cycles	:751/768 (Min/Max)
;* Low registers used	:3 (tBCD0,tBCD1,tBCD2)
;* High registers used  :4(fbinL,fbinH,cnt16a,tmp16a)	
;* Pointers used	:Z
;*
;***************************************************************************

;***** Subroutine Register Variables

.equ	AtBCD0	=13		;address of tBCD0
.equ	AtBCD2	=15		;address of tBCD1

.def	tBCD0	=r13		;BCD value digits 1 and 0
.def	tBCD1	=r14		;BCD value digits 3 and 2
.def	tBCD2	=r15		;BCD value digit 4
.def	fbinL	=r16		;binary value Low byte
.def	fbinH	=r17		;binary value High byte
.def	cnt16a	=r18		;loop counter
.def	tmp16a	=r19		;temporary value

;***** Code

bin2BCD16:
	ldi	cnt16a,16	;Init loop counter	
	clr	tBCD2		;clear result (3 bytes)
	clr	tBCD1		
	clr	tBCD0		
	clr	ZH		;clear ZH (not needed for AT90Sxx0x)
bBCDx_1:lsl	fbinL		;shift input value
	rol	fbinH		;through all bytes
	rol	tBCD0		;
	rol	tBCD1
	rol	tBCD2
	dec	cnt16a		;decrement loop counter
	brne	bBCDx_2		;if counter not zero
	ret			;   return

bBCDx_2:ldi	r30,AtBCD2+1	;Z points to result MSB + 1
bBCDx_3:
	ld	tmp16a,-Z	;get (Z) with pre-decrement
;----------------------------------------------------------------
;For AT90Sxx0x, substitute the above line with:
;
;	dec	ZL
;	ld	tmp16a,Z
;
;----------------------------------------------------------------
	subi	tmp16a,-$03	;add 0x03
	sbrc	tmp16a,3	;if bit 3 not clear
	st	Z,tmp16a	;	store back
	ld	tmp16a,Z	;get (Z)
	subi	tmp16a,-$30	;add 0x30
	sbrc	tmp16a,7	;if bit 7 not clear
	st	Z,tmp16a	;	store back
	cpi	ZL,AtBCD0	;done all three?
	brne	bBCDx_3		;loop again if not
	rjmp	bBCDx_1		

;********************************************************************
;NAME:      load_msg
;FUNCTION:  Loads a predefined string msg into a specified diplay
;           buffer.
;ASSUMES:   Z = offset of message to be loaded. Msg format is 
;           defined below.
;RETURNS:   nothing.
;MODIFIES:  r16, Y, Z
;CALLS:     nothing
;CALLED BY:  
;********************************************************************
; Message structure:
;   label:  .db <buff num>, <text string/message>, <end of string>
;
; Message examples (also see Messages at the end of this file/module):
;   msg_1: .db 1,"First Message ", 0   ; loads msg into buff 1, eom=0
;   msg_2: .db 1,"Another message ", 0 ; loads msg into buff 1, eom=0
;
; Notes: 
;   a) The 1st number indicates which buffer to load (either 1, 2, or 3).
;   b) The last number (zero) is an 'end of string' indicator.
;   c) Y = ptr to disp_buffer
;      Z = ptr to message (passed to subroutine)
;********************************************************************
load_msg:
     ldi YH, high (dsp_buff_1) ; Load YH and YL as a pointer to 1st
     ldi YL, low (dsp_buff_1)  ; byte of dsp_buff_1 (Note - assuming 
                               ; (dsp_buff_1 for now).
     lpm R16, Z+               ; get dsply buff number (1st byte of msg).
     cpi r16, 1                ; if equal to '1', ptr already setup.
     breq get_msg_byte         ; jump and start message load.
     adiw YH:YL, 16            ; else set ptr to dsp buff 2.
     cpi r16, 2                ; if equal to '2', ptr now setup.
     breq get_msg_byte         ; jump and start message load.
     adiw YH:YL, 16            ; else set ptr to dsp buff 2.
        
get_msg_byte:
     lpm R16, Z+               ; get next byte of msg and see if '0'.        
     cpi R16, 0                ; if equal to '0', end of message reached.
     breq msg_loaded           ; jump and stop message loading operation.
     st Y+, R16                ; else, store next byte of msg in buffer.
     rjmp get_msg_byte         ; jump back and continue...
msg_loaded:
     ret


;***************************************************************************
;*
;* "div32u" - 32/32 Bit Unsigned Division
;*
;* Ken Short
;*
;* This subroutine divides the two 32-bit numbers 
;* "dd32u3:dd32u2:dd32u1:dd32u0" (dividend) and "dv32u3:dv32u2:dv32u3:dv32u2"
;* (divisor). 
;* The result is placed in "dres32u3:dres32u2:dres32u3:dres32u2" and the
;* remainder in "drem32u3:drem32u2:drem32u3:drem32u2".
;*  
;* Number of words	:
;* Number of cycles	:655/751 (Min/Max) ATmega16
;* #Low registers used	:2 (drem16uL,drem16uH)
;* #High registers used  :5 (dres16uL/dd16uL,dres16uH/dd16uH,dv16uL,dv16uH,
;*			    dcnt16u)
;* A $0000 divisor returns $FFFF
;*
;***************************************************************************

;***** Subroutine Register Variables

.def	drem32u0=r12    ;remainder
.def	drem32u1=r13
.def	drem32u2=r14
.def	drem32u3=r15

.def	dres32u0=r18    ;result (quotient)
.def	dres32u1=r19
.def	dres32u2=r20
.def	dres32u3=r21

.def	dd32u0	=r18    ;dividend
.def	dd32u1	=r19
.def	dd32u2	=r20
.def	dd32u3	=r21

.def	dv32u0	=r22    ;divisor
.def	dv32u1	=r23
.def	dv32u2	=r24
.def	dv32u3	=r25

.def	dcnt32u	=r17

;***** Code

div32u:
	clr	drem32u0	;clear remainder Low byte
    clr drem32u1
    clr drem32u2
	sub	drem32u3,drem32u3;clear remainder High byte and carry
	ldi	dcnt32u,33	;init loop counter
d32u_1:
	rol	dd32u0		;shift left dividend
	rol	dd32u1
	rol	dd32u2    
	rol	dd32u3
	dec	dcnt32u		;decrement counter
	brne	d32u_2		;if done
	ret			;    return
d32u_2:
	rol	drem32u0	;shift dividend into remainder
    rol	drem32u1
    rol	drem32u2
	rol	drem32u3

	sub	drem32u0,dv32u0	;remainder = remainder - divisor
    sbc	drem32u1,dv32u1
    sbc	drem32u2,dv32u2
	sbc	drem32u3,dv32u3	;
	brcc	d32u_3		;   branch if reult is pos or zero

	add	drem32u0,dv32u0	;    if result negative restore remainder
	adc	drem32u1,dv32u1
	adc	drem32u2,dv32u2
	adc	drem32u3,dv32u3
	clc			;    clear carry to be shifted into result
	rjmp	d32u_1		;else
d32u_3:	sec			;    set carry to be shifted into result
	rjmp	d32u_1

;***************************************************************************
;*
;* "mpy16u" - 16x16 Bit Unsigned Multiplication
;*
;* This subroutine multiplies the two 16-bit register variables 
;* mp16uH:mp16uL and mc16uH:mc16uL.
;* The result is placed in m16u3:m16u2:m16u1:m16u0.
;*  
;* Number of words	:14 + return
;* Number of cycles	:153 + return
;* Low registers used	:None
;* High registers used  :7 (mp16uL,mp16uH,mc16uL/m16u0,mc16uH/m16u1,m16u2,
;*                          m16u3,mcnt16u)	
;*
;***************************************************************************

;***** Subroutine Register Variables

.def	mc16uL	=r16		;multiplicand low byte
.def	mc16uH	=r17		;multiplicand high byte
.def	mp16uL	=r18		;multiplier low byte
.def	mp16uH	=r19		;multiplier high byte
.def	m16u0	=r18		;result byte 0 (LSB)
.def	m16u1	=r19		;result byte 1
.def	m16u2	=r20		;result byte 2
.def	m16u3	=r21		;result byte 3 (MSB)
.def	mcnt16u	=r22		;loop counter

;***** Code

mpy16u:	
	clr	m16u3		;clear 2 highest bytes of result
	clr	m16u2
	ldi	mcnt16u,16	;init loop counter
	lsr	mp16uH
	ror	mp16uL

m16u_1:	brcc	noad8		;if bit 0 of multiplier set
	add	m16u2,mc16uL	;add multiplicand Low to byte 2 of res
	adc	m16u3,mc16uH	;add multiplicand high to byte 3 of res
noad8:	ror	m16u3		;shift right result byte 3
	ror	m16u2		;rotate right result byte 2
	ror	m16u1		;rotate result byte 1 and multiplier High
	ror	m16u0		;rotate result byte 0 and multiplier Low
	dec	mcnt16u		;decrement loop counter
	brne	m16u_1		;if not done, loop more
	ret

;************************
;NAME:      clr_dsp_buffs
;FUNCTION:  Initializes dsp_buffers 1, 2, and 3 with blanks (0x20)
;ASSUMES:   Three CONTIGUOUS 16-byte dram based buffers named
;           dsp_buff_1, dsp_buff_2, dsp_buff_3.
;RETURNS:   nothing.
;MODIFIES:  r25,r26, Z-ptr
;CALLS:     none
;CALLED BY: main application and diagnostics
;********************************************************************
clr_dsp_buffs:
     ldi R25, 48               ; load total length of both buffer.
     ldi R26, ' '              ; load blank/space into R26.
     ldi ZH, high (dsp_buff_1) ; Load ZH and ZL as a pointer to 1st
     ldi ZL, low (dsp_buff_1)  ; byte of buffer for line 1.
   
    ;set DDRAM address to 1st position of first line.
store_bytes:
     st  Z+, R26       ; store ' ' into 1st/next buffer byte and
                       ; auto inc ptr to next location.
     dec  R25          ; 
     brne store_bytes  ; cont until r25=0, all bytes written.
     ret

;***************************************************************************
;* 
;* "var_delay" - title
;*
;* Description: This subroutine allows for a delay to occur depending on the 
;* current value of register 16. 
;*
;* Author: Wilmer Suarez
;* Version: 1.0
;* Last updated: 12/22/2017
;* Target: ATmega324a 
;*
;* Notes: // Delay for ATmega324a @ 1MHz = r16 * 0.1ms
;*
;***************************************************************************
var_delay:
	//----- STORE_SREG_AND_USED_REGISTERS -----//
	push r18		
	in r18, SREG		
	push r18
	push r16
	push r17

outer_loop:
	ldi r17, 32
inner_loop:
	dec r17
	brne inner_loop
	dec r16
	brne outer_loop

	//----- RESTORE_SREG_AND_USED_REGISTERS -----//
	pop r17
	pop r16
	pop r18				
	out SREG, r18
	pop r18

	ret