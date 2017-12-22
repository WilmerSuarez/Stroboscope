;***************************************************************************
;*
;* Title: prog_freq
;* Author: Wilmer Suarez
;* Version: 1.0	
;* Last updated: 11/10/2017
;* Target: Atmega324A
;*
;* DESCRIPTION
;* Generating Programmable Frequency Pulses with Timer/Counter 1
;*
;* VERSION HISTORY
;* 1.0 Original version
;***************************************************************************

.nolist
.include "m324adef.inc"
.list

.cseg

reset:
.org RESET				// Reset interrupt vector
	rjmp start					
.org INT0addr			// INT0 interrupt vector
	rjmp pushbutton_isr
.org OC1Aaddr			// Timer/Counter 1 Overflow interrupt vector
	rjmp pulse_isr			

start:
//------------------- PORT_SET_UP -------------------//
	ldi r16, $FF		// Make PORTB and output	
	out DDRB, r16
	ldi r16, $01		// Make PD2 (Interrupt INT0) an input 
	out DDRD, r16		// (to read debounced signal from EO)
						// and PD0 an output (pulse output)

//----------------- INITIALIZE_STACK -----------------//
    ldi r16, LOW(RAMEND)    // Load low byte of stack pointer
	out SPL, r16		     
	ldi r16, HIGH(RAMEND)   // Load high byte of stack pointer
    out SPH, r16

//----------------- INITIAL_VALUE_FOR_OCR1A_&_Limit -----------------//
	ldi r17, HIGH($8000)		// Load initial value of $8000 into OCR1A
	ldi r16, LOW($8000)			// and limit variable
	sts OCR1AH, r17
	sts OCR1AL, r16

//---- CONFIGURE_TIMER1_INTERRUPT_ON_OUTPUT_COMPARE_A_MATCH ----//
	ldi r16, (1 << OCIE1A)	// Overflow interrupt enable
	sts TIMSK1, r16

//------ CONFIGURE_INTERRUPT0_SENCE_CONTROL_BITS ------//
	ldi r16, (1 << ISC00) | (1 << ISC01)
	sts EICRA, r16						// Set interrupt to trigger at rising edge
	ldi r16, (1 << INT0)				// Enable Interrupt request at INT0
	out EIMSK, r16

//------------ ENABLE_GLOGBAL_INTERRUPTS ------------//
	sei			// Set the Status Register global 
				// interrupt bit (I)

//---------- TIMER_MODE=FAST_PWM_WITH_TOP_@OCR1A ----------//
	ldi r16, (1 << WGM11) | (1 << WGM10)	// TIMER COUNTER 1 @ FAST PWM
	sts TCCR1A, r16	
	ldi r16, (1 << WGM13) | (1 << WGM12) | (1 << CS11)   // Prescaler = (CLK_I/O) / 8    
	sts TCCR1B, r16

main_loop:
	rjmp main_loop	// Continue updating_LCD_display

;***************************************************************************
pushbutton_isr:
	push r18		// Store status register
	in r18, SREG
	push r18
	push r17
	push r19
	push r24
	push r25

	sei

//--------------------- INPUT DATA ---------------------//
	in r16, PINA	
	swap r16		// Take bits 4-7 (/A0, /A1, /A2)
	com r16			// Complement bits 
	andi r16, $07	// Mask 4th bit in lower nibble (Only want bits 0-2)

//------------------- CHECK_BUTTONS -------------------//
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

	rjmp done		// If button 1 pressed, do nothing
	sub50:
		rjmp sub_50
	sub10:
		rjmp sub_10
	add10:
		rjmp add_10
		
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

done:
	pop r25
	pop r24
	pop r19
	pop r17
	pop r18				// Restore status register
	out SREG, r18
	pop r18

	reti

;***************************************************************************
pulse_isr: 
	push r16
	in r16, SREG
	push r16
	push r17

	sbi PORTD, 0		// Set PORT_D, PIN_0
	ldi r16, 10			// Variable to delay for 1ms
	call var_delay
	cbi PORTD, 0		// Clear PORT_D, PIN_0

	pop r17
	pop r16
	out SREG, r16
	pop r16
	reti

;***************************************************************************
var_delay:
	push r18			// Delay for ATmega324a @ 1MHz = r16 * 0.1ms
	in r18, SREG		// Store status register
	push r18

outer_loop:
	ldi r17, 32
inner_loop:
	dec r17
	brne inner_loop
	dec r16
	brne outer_loop

	pop r18				// Restore status register
	out SREG, r18
	pop r18

	ret

;***************************************************************************
;*
;* "div16u" - 16/16 Bit Unsigned Division
;*
;* This subroutine divides the two 16-bit numbers 
;* "dd8uH:dd8uL" (dividend) and "dv16uH:dv16uL" (divisor). 
;* The result is placed in "dres16uH:dres16uL" and the remainder in
;* "drem16uH:drem16uL".
;*  
;* Number of words	:19
;* Number of cycles	:235/251 (Min/Max)
;* Low registers used	:2 (drem16uL,drem16uH)
;* High registers used  :5 (dres16uL/dd16uL,dres16uH/dd16uH,dv16uL,dv16uH,
;*			    dcnt16u)
;*
;***************************************************************************

;***** Subroutine Register Variables

.def	drem16uL=r14
.def	drem16uH=r15
.def	dres16uL=r16
.def	dres16uH=r17
.def	dd16uL	=r16
.def	dd16uH	=r17
.def	dv16uL	=r18
.def	dv16uH	=r19
.def	dcnt16u	=r20

;***** Code

div16u:	clr	drem16uL	;clear remainder Low byte
	sub	drem16uH,drem16uH;clear remainder High byte and carry
	ldi	dcnt16u,17	;init loop counter
d16u_1:	rol	dd16uL		;shift left dividend
	rol	dd16uH
	dec	dcnt16u		;decrement counter
	brne	d16u_2		;if done
	ret			;    return
d16u_2:	rol	drem16uL	;shift dividend into remainder
	rol	drem16uH
	sub	drem16uL,dv16uL	;remainder = remainder - divisor
	sbc	drem16uH,dv16uH	;
	brcc	d16u_3		;if result negative
	add	drem16uL,dv16uL	;    restore remainder
	adc	drem16uH,dv16uH
	clc			;    clear carry to be shifted into result
	rjmp	d16u_1		;else
d16u_3:	sec			;    set carry to be shifted into result
	rjmp	d16u_1

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

mpy16u:	clr	m16u3		;clear 2 highest bytes of result
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