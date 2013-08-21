; Copyright (c) 2008 Jeremy Chin
; All rights reserved.
; 
; Redistribution and use in source and binary forms, with or without
; modification, are permitted provided that the following conditions
; are met:
; 1. Redistributions of source code must retain the above copyright
;    notice, this list of conditions and the following disclaimer.
; 2. Redistributions in binary form must reproduce the above copyright
;    notice, this list of conditions and the following disclaimer in the
;    documentation and/or other materials provided with the distribution.
; 3. The name of the author may not be used to endorse or promote products
;    derived from this software without specific prior written permission.
; 
; THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
; IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
; OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
; IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
; INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
; NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
; DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
; THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
; (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
; THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

; the FLAG_XMIT code (detecting the Saturn pad) has been disabled because it was
; causing problems. To use it uncomment lines 362 and 335, and connect Saturn
; pin 6 (VCC In) to pin 11 on the chip.


.nolist
.include "tn2313def.inc"

; ugly key mapping stuff
.equ SAT_0_Z = 5
.equ SAT_0_Y = 4
.equ SAT_0_X = 3
.equ SAT_0_R = 2

.equ SAT_2_B = 5
.equ SAT_2_C = 4
.equ SAT_2_A = 3
.equ SAT_2_St = 2

.equ SAT_1_Up = 5
.equ SAT_1_Dn = 4
.equ SAT_1_Lt = 3
.equ SAT_1_Rt = 2

.equ SAT_3_L = 2
.equ SAT_3_L1 = 5  ; These should be held low and high for detection
.equ SAT_3_L2 = 4
.equ SAT_3_H1 = 3

.equ SAT_CONNECTED = 6 ; pin 6 is high if pad is connected

; PSX0 _must_ be in 1st 3 groups, so priority to SAT_1 and SAT_2 (01 and 10...)
; sequence is therefore
; 01
; 11
; 10
; 00

.equ PSX_0_Sel = 7
.equ PSX_0_L3 = 6 ; on a psx pad?
.equ PSX_0_R3 = 5
.equ PSX_0_St = 4
.equ PSX_0_Up = 3
.equ PSX_0_Rt = 2
.equ PSX_0_Dn = 1
.equ PSX_0_Lt = 0

.equ PSX_1_L2 = 7
.equ PSX_1_R2 = 6
.equ PSX_1_L1 = 5
.equ PSX_1_R1 = 4
.equ PSX_1_Tr = 3
.equ PSX_1_Ci = 2
.equ PSX_1_X  = 1
.equ PSX_1_Sq = 0

; it would be nice to have macros to do the mapping, but how?

.list

; Base values are for an 8mhz clock.
; These values are not the absolute delay, because of other delays in the code. 
; Actual time is about DELAY * 3/8e6 (for a 8mhz clock)

.equ CLK_DIV = 1

.equ DELAY1CLK=40 * CLK_DIV ; ~15us
.equ DELAY2CLK=3 * CLK_DIV  ; 1.125us

.macro delay
ldi counter, @0 ; 1 clock
count: dec counter ; 1 clock
brne count ; taken: 2 clocks
.endmacro

.macro delay_wait
delay DELAY1CLK
.endmacro

.macro delay_pulse
delay DELAY2CLK
.endmacro

; LEDs on pins 11 and 12 for debugging purposes. LED2 disabled to allow for other stuff.

.macro LED1_on
sbi PortB, 0
.endmacro
.macro LED1_off
cbi PortB, 0
.endmacro

.macro LED2_on
sbi PortD, 6
.endmacro
.macro LED2_off
cbi PortD, 6
.endmacro

.def temp1=R16
.def tosend=R17
.def counter=R18
.def zero=R19
.def temp2=R21
.def PSX0 = R22
.def PSX1 = R23
.def flags = R24

.equ FLAG_CONNECTED = 1 ; saturn pad is connected or not
.equ FLAG_XMIT = 2 ; FLAG_CONNECTED delayed by 1 frame, to prevent glitches

; psx byte 0 doesn't matter, it's not actually sent.
.equ PSX_BYTE1 = SRAM_START
.equ PSX_BYTE2 = SRAM_START + 1
.equ PSX_BYTE3 = SRAM_START + 2 ; buttons #1
.equ PSX_BYTE4 = SRAM_START + 3 ; buttons #2

; R26 = X, don't touch

.org 0x0
rjmp main
; No interrupts needed

.org 0x12
reti

main:
	ldi r16, low(RAMEND)
	out SPL, r16 ; from datasheet, set stack pointer

	ldi temp1, (1 << PUD)
	out MCUCR, temp1

	; Configure port B, the PSX side
	ldi temp1, 0b00000001
	sbi PortB, 6 ; ACK line high
	; Leave PB4 alone, it will be controlled by DDRB.4
	out DDRB, temp1

	; Port D, saturn side
	ldi temp1, 0b00000011
	out DDRD, temp1

	sbi PortD, 0 ; select the correct initial state (0 1)

	; Prepare the data to be sent to the PSX
	ldi XH, 0
	ldi XL, PSX_BYTE1
	ldi temp1, 0x82 ; Controller ID for a PSX controller (0x41 reversed)
	st X+, temp1 ; 60
	ldi temp1, 0x5A ; End of header or something
	st X+, temp1 ; 61
	ldi temp1, 0xff ; All buttons released (will be changed before sending)

	st X+, temp1 ; 62
	st X+, temp1 ; 63
	ldi XL, PSX_BYTE1

;	ldi flags, (1 << FLAG_CONNECTED)
	ldi flags, 0 ; assume not connected

	sbi PortD, 0
	cbi PortD, 1

mainloop:
	ldi PSX0, 0xff   ; Clear button mask
	ldi PSX1, 0xff
;	sbi PortD, 0     ; Saturn select 01
;	cbi PortD, 1
	sts PSX_BYTE3, PSX0
	sts PSX_BYTE4, PSX1

	; Wait until ATT goes high
	waitATTH:
	sbis PinB, 3
	rjmp waitATTH

	; Prepare to set the USI. Don't want to delay after ATT goes low.
	ldi temp1, (1 << USIWM0) | (1 << USICS1)

	; Wait until ATT goes low
	waitATTL:
	sbic PinB, 3
	rjmp waitATTL

	; ATT is low, start of communication.
	out USISR, zero ; reset the count
	out USICR, temp1 ; Start the USI


	LED1_off Entering main transmission stage, red off
	; Don't enable the output yet, wait until the first byte is clear (it's
	; not important anyway)

; START OF TRANSFERS
	;;;; SATURN THREAD

	; state = 0 1
	in temp2, PinD
	bst temp2, SAT_1_Up
	bld PSX0, PSX_0_Up
	bst temp2, SAT_1_Dn
	bld PSX0, PSX_0_Dn

	sbi PortD, 1 ; 0 1 -> 1 1

	;;;; PSX THREAD

	; wait for the transfer to finish
	byteno1:
	sbic PinB, 3 ; make sure ATT is still low
	rjmp restart
	sbis USISR, USIOIF
	rjmp byteno1

	delay_wait    ; Don't send the pulse too early

	sbic PinB, 3
restart_short:		; Can't do a jump right to the end (too far)
	rjmp restart
	in temp1, USIDR
	cpi temp1, 0x80
	brne abort_short
	sbi DDRB, 6    ; Now enable the output
	rcall nextbyte


	;;;; SATURN THREAD 
	; state = 1 1
	; Finish stuff from the first byte...
	bst temp2, SAT_1_Lt
	bld PSX0, PSX_0_Lt
	bst temp2, SAT_1_Rt
	bld PSX0, PSX_0_Rt

	; Now do the easy 2nd byte
	in temp2, PinD
	; Make sure the pad is connected
	mov temp1, temp2
	; mask of all the detect bits
	andi temp1, (1 << SAT_3_L1 | 1 << SAT_3_L2 | 1 << SAT_3_H1 | 1 << SAT_CONNECTED)
	cpi temp1, (1 << SAT_3_H1 | 1 << SAT_CONNECTED)
	breq saturn_ok
	; saturn not ok
	cbr flags, (1 << FLAG_CONNECTED | 1 << FLAG_XMIT)

	rjmp detect_continue
	saturn_ok:
	sbr flags, (1 << FLAG_CONNECTED)
	rjmp detect_continue ; Leave a space for a jump

; Branches have limited range, so need to go short to here first
abort_short:
	rjmp abort
; Normal code resumes here

detect_continue:
	bst temp2, SAT_3_L
	bld PSX1, PSX_1_L2
	cbi PortD, 0 ; 1 1 -> 1 0

	;;;; PSX THREAD

	byteno2:
	sbic PinB, 3
	rjmp restart
	sbis USISR, USIOIF
	rjmp byteno2

	delay_wait

	sbic PinB, 3
	rjmp restart
	in temp1, USIDR
	cpi temp1, 0x42
	brne abort
	rcall nextbyte

	;;;; SATURN THREAD
	; state = 1 0
	in temp2, PinD

	; Figure out the start button
	sbrc temp2, SAT_2_St
	rjmp start_off
	
	bst temp2, SAT_2_A ; Start pressed -> modifiers
	bld PSX0, PSX_0_St
	bst temp2, SAT_2_B
	bld PSX0, PSX_0_Sel

	; start + C -> start, select, up (PSX button on ps3)
	bst temp2, SAT_2_C
	brts psx0_out

	cbr PSX0, (1 << PSX_0_ST | 1 << PSX_0_SEL | 1 << PSX_0_Up)

	rjmp psx0_out

start_off:
	bst temp2, SAT_2_A ; Start not pressed -> normal
	bld PSX1, PSX_1_X
	bst temp2, SAT_2_B
	bld PSX1, PSX_1_Ci
	bst temp2, SAT_2_C
	bld PSX1, PSX_1_R1

psx0_out:
	cbi PortD, 1 ; 1 0 -> 0 0
	; PSX0 done, write back
;	sbrc flags, FLAG_XMIT
	sts PSX_BYTE3, PSX0

	;;;; PSX thread
	byteno3:
	sbis USISR, USIOIF
	rjmp byteno3

	delay_wait

	sbic PinB, 3
	rjmp restart
	; No need to check the PSX data any more
	rcall nextbyte

	;;;; SATURN THREAD
	; state = 0 0
	in temp2, PinD
	bst temp2, SAT_0_Z
	bld PSX1, PSX_1_L1
	bst temp2, SAT_0_Y
	bld PSX1, PSX_1_Tr
	bst temp2, SAT_0_X
	bld PSX1, PSX_1_Sq
	bst temp2, SAT_0_R
	bld PSX1, PSX_1_R2
	sbi PortD, 0 ; 0 0 -> 0 1
;	sbrc flags, FLAG_XMIT
	sts PSX_BYTE4, PSX1

	byteno4:
	sbis USISR, USIOIF
	rjmp byteno4

	delay_wait

	sbic PinB, 3
	rjmp restart
	rcall nextbyte

; misc stuff: Transfer the CONNECTED flag forward to XMIT
	bst flags, FLAG_CONNECTED
	bld flags, FLAG_XMIT

	byteno5:
	sbis USISR, USIOIF
	rjmp byteno5

	; We've reached here, no error
;	LED2_off
abort:

; if ATT is still low, return 0s
finishxmit:
	delay_wait ; Just in case it needs time to go high
	cbi DDRB, 6 ; Nothing important to output, disable
	sbic PinB, 3
	rjmp clean
	rcall nextbyte_0
;
byteend:
	sbis PinB, 3
	rjmp clean
	sbic USISR, USIOIF
	rjmp finishxmit
	rjmp byteend

restart:
;	LED2_on  ; Some sort of transmission glitch. Get out the scope.

clean:

	cbi DDRB, 6 ; Disable the output
	cbi DDRB, 4 ; Disable the ACK line
	ldi XL, PSX_BYTE1
	sbi USISR, USIOIF
	out USICR, zero  ; Stop the USI

	LED1_on ; finished, red back on
	rjmp mainloop

;;;;;;;; End main loop, start subroutines ;;;;;;;;;;;;;

; Ugly coupled routines
nextbyte:
	ld tosend, X+
nextbyte_inner:
	out USIDR, tosend
	sbi USISR, USIOIF
	sbi DDRB, 4
	delay_pulse
	cbi DDRB, 4
	ret

nextbyte_0:
	ldi tosend, 0
	rjmp nextbyte_inner

