#include "p12f675.inc" ; generic constants for this uC
; __config 0xF194
 __CONFIG _FOSC_INTRCIO & _WDTE_OFF & _PWRTE_OFF & _MCLRE_OFF & _BOREN_OFF & _CP_OFF & _CPD_OFF
; I don't use watchdog timer, prefer mclr as a i/o-pin and don't protect memory

; Servo tester
; ------------
; PIC12f675 base servo tester written by Ketil Duna April 2015
; The goal of the project is to create a handy small device to do servo testing
; both "gear check" and to test in-plane behaviour of servos while building
; There are three inputs to the tester - a pot, rail to rail (10k), and two
; three position switches each connecting center to each rail with a 4.7k resistor.
; I prefer 4MHz clock, thus 1MHz instruction cycle and timer1 frequency
; There are three modes of operation:

; mode 1 - pot drives servo directly. Range is 1500us +/- 600us
; mode 2 - sweep from end to end, tri decides high/both/low and pot limits endpoint
; mode 3 - three-position switch drives servo directly, pot limits endpoint

; TODO: there are still two pins free. Perhaps the uC should drive the LED
; Enable some sort of sleep function in case one forgets to turn off the
; tester - and wake the uC on change of a switch ?
; The mode switch is on the INT pin already ..
; Porting to a chip with more pins, a display could show the pulse length
; in us or percent deflection ..

; Note on precision: There is little need for perfect precision here, BUT
; it is nice to get the center position right. To achieve this, the uC clock
; must be as close to specification as possible. On 12f675 this requires a
; calibration of the on-board clock. From factory, the chip has this calibration
; value programmed in position 0x3ff in program memory. It can be recovered if
; overwritten using pickit2 app with this function built-in
; I have tested with some hitec and graupner servos, and all look fine on the
; scope and the servos are not "humming" in extreme positions. If, however, the
; clock is off, and perhaps the servo prefers a smaller bandwith signal
; the net result could be attempting to drive the servo beyond physical limits.
; This could damage the servo, and certainly draw a lot of power

; PIN 1 - Vdd
; PIN 2 - GP5 - [Servo signal output]
; PIN 3 - GP4 - [TRI-switch input] none, submode, servo position
; PIN 4 - GP3 * nMCLR/Vpp, and input-only w/o wpu, no ADC (not used)
; PIN 5 - GP2 * [MOD-switch input] mode 1, 2, 3 * programming data input
; PIN 6 - GP1 * programming clock input (not used)
; PIN 7 - GP0 - [POT input] servo position, endpoint adjust, endpoint adjust
; PIN 8 - Gnd/Vss
; * used during icsp programming

; diagram of finished circuit
; -----------------------------------------------------------
;   |         |         |           |                   |
;   |         |         | 10k       |                   |
;   | +      | |150     | Ohm       o                   o
;  --- 4.5v  | |Ohm    | |            to           servo
;   -         |        | K---to      switches      -----o
;   | -       |        | |   "pot"                 output
;   |         V  LED    |           o                   o
;   |        --- "on"   |           |                   |
;   |         |         |           |                   |
; -----------------------------------------------------------

; each switch looks like this
; -----------------------------------------------------------
;           |          |
;          | |10k      |
;          | |         |
;           |     o----
;    uC -------o---    center tap biased by resistors when floating, and
;           |     o----   can be shorted to either Vdd or Vss
;          | |10k      |     to set one of three input states
;          | |         |  I soldered resistors directly on the switch
;           |          |  , wired center to uC and ends to Vss/Vdd
; -----------------------------------------------------------

; no special care for uC, but a small capacitor can be useful
; to filter out spikes and avoid glitches. Connect close to uC
; Processor voltage is 2.5 - 5.5v, a servo might not work well
; below 4 - i chose 3 x AAA batteries for a total of 4.5v
; -----------------------------------------------------------
;             |--------||------
;             |      4.7uF     |
;             | ------------   |
;           Vdd |1         8| Vss -----
;     servo out |2 12f675  7| pot     |
;           tri |3         6|         |
;               |4         5| mode    |
;               -------------         |
;                                     |
; -----------------------------------------------------------

; some declarations not in include-file
AN0 EQU H'0000' ; on GP0 - pin 7
AN1 EQU H'0001' ; on GP1 - pin 6
AN2 EQU H'0002' ; on GP2 - pin 5
AN3 EQU H'0003' ; on GP4 - pin 3

; inputs/outputs
POT  EQU GP0 ; 0-Vdd input from position-pot
POTA EQU AN0 ; analog mapping
TRI  EQU GP4 ; a switch for min-center-max output
TRIA EQU AN3 ; analog mapping - AN3 belongs to GP4, on pin 3
MOD  EQU GP2 ; a switch for mode; pot - sweep - three levels
MODA EQU AN2 ; analog mapping
LED  EQU GP4 ; active low here
OUT  EQU GP5 ; servo output

; uC settings
; ansel 6:4 = conversion clock. For our 4MHz chip, the value is
; 001 for a 2us conversion, or 101 for a 4us conversion
; ansel uses AN-mapping (not GPIO-mapping) for selecting analog input channel
ANSELBITS EQU (b'101' << ADCS0) | (1 << POTA) | (1 << TRIA) | (1 << MODA)
T1CONBITS EQU b'00000100' ; no prescaler for timer1, and don't start
CMCONBITS EQU b'00000111' ; turn off comparator, not used here
TRISBITS EQU 0x3f ^(1 << OUT) ; all inputs, except OUT-pin
OPTIONBITS = b'00000110' ; 1:128 prescaler for TMR0 for pulse period timekeeping

; Timer 0 is responsible for counting out the period between output pulses
; This should be about 50Hz, or 20ms - 20000 clock ticks
; with a 128 prescaler, we need to count ~157 between pulses, or -157 be loaded
; into timer 0 for each pulse period set
; an interrupt will reload the value on each interrupt
TMR0_PERIOD EQU .256-.157
SPEED EQU 0x08 ; default sweep speed - step 8 us width change each 20ms period

DIRECTION_CW EQU 0x01 ; clockwise direction for sweep
DIRECTION_CCW EQU 0xff ; counterclockwise direction for sweep

THRESHOLD_HIGH EQU 0xc0 ; above 0xc0 is a high input
; anything between 0x40-0xc0 is a medium input
THRESHOLD_LOW EQU 0x40 ; below 0x40 is a low input

; Pulse has two contexts here - the calculated value (typ 0-1200) and
; the output value, typ. 1500 +/- 600 - former is VALUE, latter is PULSE
; a third, OUTPUT, describes the value put in TIMER1 to overflow at the correct
; time - a negative number, with a theoretical value of 0-VALUE_HIGH, to which
; we add output/VALUE to obtain TIMER1 preload
PULSE_CENTER EQU .1500
VALUE_MAX EQU .1200  ; max bandwith of our output pulse
VALUE_CENTER EQU VALUE_MAX >> 1 ; with a center at the half of max (ie 600, 0x258)
VALUE_CENTER_LSB EQU VALUE_CENTER & 0xff ;  low byte
VALUE_CENTER_MSB EQU (VALUE_CENTER >> 0x08) & 0xff ; high byte
VALUE_HIGH_LSB EQU VALUE_MAX & 0xff
VALUE_HIGH_MSB EQU (VALUE_MAX >> 0x08) & 0xff

PULSE_OFFSET EQU PULSE_CENTER - (VALUE_MAX >> 1) ; conversion between value and pulse
PULSE_OFFSET_LSB EQU PULSE_OFFSET & 0xff ; least significant byte
PULSE_OFFSET_MSB EQU (PULSE_OFFSET >> 0x08) & 0xff ; most significant byte

  cblock 0x20  ; up to 64 bytes free for variables
    direction  ; 0x01 or 0xff for cw and ccw (sweep variable)
    sweepL     ; low byte of sweep value
    sweepH     ; high byte of sweep value
    outputL    ; pulse queued for output - in µs - low byte
    outputH    ; high byte of output
    potL       ; ADC low byte from pot-reading
    potH       ; ADC high byte from pot-reading
    potB       ; 8 most significant bits of pot
    pot2L      ; half value of pot..
    pot2H      ; for endpoint adjust in mode 2 & 3
    tristate   ; raw and eventually refined value of tri-state-switch
    mode       ; raw and eventually refined value of mode switch
    prevMode   ; remember last mode

    tempW      ; context saving during isr
    tempStatus ; context saving during isr
    temp       ; generic temp-field
    tempL      ; 16-bit math low byte
    tempH      ; 16-bit math high byte

    maxSweepL  ; sweep control - limit of servo travel
    maxSweepH
    minSweepL
    minSweepH

    lowSweepL  ; pre-calculated limits for upper and lower values, based on
    lowSweepH  ; tri-state switch and pot value
    highSweepL
    highSweepH
  endc

  org 0x00     ; reset vector
  goto init
  org 0x04     ; isr vector
  goto isr

init ; one time setup of the uC and peripheralse. Note fuses at beginning of file
  bcf STATUS, RP0 ; bank0
  clrf INTCON ; no interrupts

  bsf STATUS, RP0 ; initialization in bank1
  movlw TRISBITS
  movwf TRISIO    ; GPIO input/output select
  movlw ANSELBITS
  movwf ANSEL     ; analog / digital pin select
  movlw OPTIONBITS
  movwf OPTION_REG ; uC options
  call 0x3ff ; retrieve calibration value for this specific chip
  movwf OSCCAL ; set osccal value for the internal oscillator

  bcf STATUS, RP0 ; initialization in bank0
  movlw CMCONBITS
  movwf CMCON     ; comparator ( switch off, we are running on batteries )
  movlw T1CONBITS
  movwf T1CON     ; timer1 initial setup (no prescaler, don't start)

; init app-variables
  movlw 0x01 ; defaults for the variables
  movwf direction
  movlw PULSE_CENTER & 0xff
  movwf sweepL
  movlw PULSE_CENTER >> 0x08
  movwf sweepH

; prepare timer0 interrupt sequence to occur at 50Hz / 20ms interval
  movlw TMR0_PERIOD ; period of output pulse
  movwf TMR0 ; timer0 is always running, but will pause 2us when set like this
  bcf INTCON, T0IF ; clear timer 0 interrupt flag
  bsf INTCON, T0IE ; enable timer 0 interrupt
  bsf INTCON, GIE ; enable general interrupts

main
  
  call readInputs ; get inputs from POT, TRI and MOD. Calculate running values
  
; mode changed ?
  movfw prevMode
  xorwf mode, W ; will result in Z set if equal
  btfsc STATUS, Z
  goto modeChangeDone

; mode changed. set Sweep to center and direcion clockwise
  movlw VALUE_CENTER_LSB
  movwf sweepL
  movlw VALUE_CENTER_MSB
  movwf sweepH
  movlw DIRECTION_CW
  movwf direction
modeChangeDone

; remember previous mode
  movfw mode
  movwf prevMode ; used for identifying mode changes
  movwf temp     ; used for routing to mode-handler
; route to the selected mode
  decfsz temp, f ; temp used to decode mode-value into action
  goto mainM ; >1
  goto main1 ; = 1  goto handling pot -> output
mainM
  decfsz temp, f
  goto main3 ; >2 - goto switch tri -> output
  goto main2 ; =2 - goto automagic sweep function

; main 1 - ADC, POT drives servo directly
main1 
; inject the pot-value into output-variables
  movfw potL
  movwf outputL
  movfw potH
  movwf outputH

  goto mainDone ; end of direct control

; main 2 - automagic sweep
main2

; what submode does tristate dictate ?
  movfw tristate
  movwf temp
  decfsz temp
  goto main2M

; tristate in low position - sweep lower half of output
  movfw lowSweepL
  movwf minSweepL
  movfw lowSweepH
  movwf minSweepH
  movlw VALUE_CENTER_LSB
  movwf maxSweepL
  movlw VALUE_CENTER_MSB
  movwf maxSweepH
  goto main2EndpointsDone
main2M
  decfsz temp
  goto main2High
; tristate in middle position - sweep whole range
  movfw lowSweepL  
  movwf minSweepL
  movfw lowSweepH
  movwf minSweepH
  movfw highSweepL
  movwf maxSweepL
  movfw highSweepH
  movwf maxSweepH
  goto main2EndpointsDone
main2High
; high range - onsly sweep the upper half of the full range
  movlw VALUE_CENTER_LSB
  movwf minSweepL
  movlw VALUE_CENTER_MSB
  movwf minSweepH
  movfw highSweepL
  movwf maxSweepL
  movfw highSweepH
  movwf maxSweepH

main2EndpointsDone

  ; get speed value into W
  movlw SPEED
  ; check direction
  btfsc direction, 7 ; bit 7 set = ccw (negative flag set)
  goto main2ccw
main2cw
  addwf sweepL, F ; increase sweep-variable
  btfsc STATUS, C
  incf sweepH, F
  ; if out of bounds, reverse direction
  movfw sweepH
  subwf maxSweepH, W
  btfss STATUS, Z
  goto main2cwTest
  movfw sweepL
  subwf maxSweepL, W
main2cwTest ; sweep CAN overshoot max at extreme, but will be truncated @ output
  btfsc STATUS, C ; carry clear if sweep >= PULSE_HIGH
  goto main2Done
  ; direction now 1, make it -1
  movlw DIRECTION_CCW
  movwf direction ;
  goto main2Done

main2ccw
  subwf sweepL, F ; step down
  btfss STATUS, C ; carry clear, must adjust high-byte
  decf sweepH, F  ; step down high byte

  ; if sweep <= min, invert direction
  btfss sweepH, 7 ; bit 7 set = negative value
  goto main2NotNegative
  movlw 0x01 ; next step will be clockwise
  movwf direction
  movfw lowSweepL
  movwf sweepL
  movfw lowSweepH
  movwf sweepH
  goto main2Done

main2NotNegative
  movfw minSweepH
  subwf sweepH, W
  btfss STATUS, Z
  goto main2ccwtest
  movfw minSweepL
  subwf sweepL, W
main2ccwtest
  btfsc STATUS, C ; Carry clear if sweep < minsweep
  goto main2Done
  movlw DIRECTION_CW
  movwf direction

main2Done
  movfw sweepL
  movwf outputL
  movfw sweepH
  movwf outputH
  goto mainDone

; main 3 ********************************************************************
main3 ; mode 3; direct control with three position switch
; limiting output with pot value
  movfw tristate ; what tri-state are we in ?
  movwf temp
  decf temp, f
  btfss STATUS, Z
  goto main3above1

main3equalsone ; low ouput, pot-adjusted
  movfw lowSweepL
  movwf outputL
  movfw lowSweepH
  movwf outputH
  goto mainDone
main3above1
  decf temp, f
  btfss STATUS, Z
  goto main3equals3

main3equalstwo ; centre
  movlw VALUE_CENTER_LSB
  movwf outputL
  movlw VALUE_CENTER_MSB
  movwf outputH
  goto mainDone

main3equals3 ; high output - pot-adjusted
  movfw highSweepL
  movwf outputL
  movfw highSweepH
  movwf outputH

mainDone
  ; wait for our signal to proceed .. provided by the isr of TMR0
  ; calculated output is now in output variable(s)
  btfss INTCON, T0IF ; TMR0 overflow ?
  goto mainDone      ; no, wait for it - just waste time
  ; send the pulse
  call outPulse
  bcf INTCON, T0IF ; clear timer0 interrupt flag
  bsf INTCON, T0IE ; re-enable timer0 interrupt for next pulse
  goto main ; main loop restart

readInputs ; read mode, the three state modifier switch and pot from A/D

  ; read mode switch
  movlw (1 << ADON) | (TRIA << CHS0) ; adon, CHS = TRI-switch
  movwf ADCON0
  call charge ; wait for acquire
  movlw tristate ; did I walk into the naming trap ? tristate and threestate ?
  call threestate ; read ad, post 1-3 result in w-pointed-at variable

  ; read switch 1
  movlw (1 << ADON) | (MODA << CHS0) ; adon, CHS = MOD-switch
  movwf ADCON0
  call charge ; acquire voltage on internal sample and hold
  movlw mode ; the mode-switch also has three possible values
  call threestate

  ; read POT
  movlw (1 << ADON) | (1 << ADFM) | (POTA << CHS0) ; right justify, adon and POT input
  ; the switches are read using only upper 8 bits of A/D - more than enough
  ; precision. The pot is read using the full 10 bits for a bit more resolution
  ; a 10-bit value nearly matches our +/- 600us bandwidth with its 1024 range
  movwf ADCON0
  call charge ; setup for analog read of POTA by charging internal circuitry
  bsf ADCON0, GO ; start conversion ..
potwait
  btfsc ADCON0, GO ; result not ready until GO/NOT_DONE is cleared again
  goto potwait

; copy 10-bit pot-value to different pot variables
  bsf STATUS, RP0 ; bank 1
  movfw ADRESL    ; read low-byte ad-result
  bcf STATUS, RP0 ; bank 0
  movwf potL      ; store in pot low-byte
  movwf potB      ; prepare byte-version of pot
  movwf tempL     ; calc a quarter-value to add 25% for final pot-value
  movfw ADRESH    ; read high-byte ad-result (only two lsb bits are used)
  bcf ADCON0,ADON ; turn off adc to save on batteries ..
  movwf potH      ; store in pot high-byte
  movwf temp      ; for byte-version of pot
  movwf tempH     ; for quarter operation
  rrf temp,f      ; get bit 6 from MSB
  rrf potB,f      ; shift into byte value, discarding lsb
  rrf temp,f      ; bit 7
  rrf potB,f      ; potB is now ready, less bit 0&1, added bit 8&9
  
; and box-limit the finished value as well ..
  movfw potB      ; add the quarter conveniently residing in potB
  addwf potL,f
  btfsc STATUS, C
  incf potH,f     ; carry, add one to high byte

; subtract .40 to create a small deadband and zero in on middle position
  movlw .40
  subwf potL, f
  btfss STATUS, C
  decf potH, f

; if negative, zero is the limit .
  btfss potH, 7   ; test bit 7 to establish a negative
  goto checkHighRange
  clrf potL
  clrf potH
  goto halfPot
checkHighRange    ; check if pot > VALUE_HIGH
  movfw potH
  sublw VALUE_HIGH_MSB  ; subtract the actual value from the constant
  btfss STATUS, Z       ; if high bytes are equal, check low bytes
  goto tooHigh?
  movfw potL
  sublw VALUE_HIGH_LSB
tooHigh?
  btfsc STATUS, C       ; a clear carry indicates pot is too high
  goto halfPot          ; whereas a set carry indicates pot <= VALUE_HIGH
  movlw VALUE_HIGH_LSB  ; bollocks, set to max value
  movwf potL
  movlw VALUE_HIGH_MSB
  movwf potH
  
halfPot
  movfw potL
  movwf pot2L
  movfw potH
  andlw 0x03            ; maybe not needed, but clears out bits other than 0, 1
  movwf pot2H
  bcf STATUS, C
  rrf pot2H             ; divide by 2
  rrf pot2L

;highSweep, max sweep value = value_center + pot2
  movlw VALUE_CENTER_LSB ; 600
  addwf pot2L, W         ; [0-512]
  movwf highSweepL
  movlw VALUE_CENTER_MSB
  btfsc STATUS, C
  addlw 0x01
  addwf pot2H, W
  movwf highSweepH

; lowsweep, min sweep value = value_center - pot2
  clrf temp
  movfw pot2L
  sublw VALUE_CENTER_LSB
  movwf lowSweepL
  movfw pot2H
  btfss STATUS, C
  decf temp, f
  sublw VALUE_CENTER_MSB
  addwf temp, w
  movwf lowSweepH

  return ; from readInputs

charge ; turning on the AD must be followed by a period of acquisition
    movlw 0x04 ; documentation calls for about 20us; call, return and this loop
chargeloop
    addlw 0xff
    btfss STATUS, Z
    goto chargeloop
    return

threestate ;adcon0 already set up for read channel, W = pointer to result variable
  clrf PIR1 ; clear all peripheral interrupt flags
  movwf FSR ; where to place the result - W is variable pointer on entry
  bsf ADCON0, GO ; complete the ad read set up by caller
adwait
  btfsc ADCON0, GO ; is ad read done yet ?
  goto adwait   ; no, iterate while set

  movlw 0x01 ; assume 1 as result
  movwf INDF ; and store in result variable
  movfw ADRESH ; get upper 8 bits from ADC result
  movwf temp ; copy to temp variable
  movlw THRESHOLD_LOW ; check for three level-switch, for threshold between levels
  subwf temp, W ; subtract THRESHOLD_LOW from temp (without altering temp)
  btfss STATUS, C ; if set, measurement is equal to or above THRESHOLD_LOW
  return ; if clear, less than THRESHOLD_LOW, and we are done, result is 1
  movlw 0x02 ; result is now assumed to be 2
  movwf INDF
  movlw THRESHOLD_HIGH ; next level
  subwf temp, W ; subtract THRESHOLD_HIGH from temp
  btfss STATUS, C ; if clear, below THRESHOLD_HIGH = medium
  return
  movlw 0x03 ; carry is set, above or equal to THRESHOLD_HIGH - result = 3
  movwf INDF
  return

outPulse ; send the pulse. We will waste at least 900 instructions here ..

  call boxOutput ; make sure we are within range, so servo is not damaged

  ; output H/L should now be [0-1200], convert numer to [-2100 - -900]
  ; by subtracting outputL/outputH from -900
  movfw outputL
  sublw (0x10000-.900) & 0xff
  movwf outputL
  movfw outputH
  btfss STATUS, C
  addlw 0x01
  sublw ((0x10000-.900) >> 0x08) & 0xff
  movwf outputH

  ; make sure timer1 is stopped, and flags cleared
  bcf T1CON, TMR1ON ; timer stop
  bcf PIR1, TMR1IF ; clear overflow flag

; load the new value to Timer1
  movfw outputL
  movwf TMR1L
  movfw outputH
  movwf TMR1H

  ; set pulse high, and start timer1
  bsf GPIO, OUT
  bsf T1CON, TMR1ON

  ; wait for timer1 overflow
outpulse_wait
  btfss PIR1, TMR1IF
  goto outpulse_wait

  ; stop output pulse
  bcf GPIO, OUT
  bcf T1CON, TMR1ON ; stop timer
  return

isr ; interrupt service routine - flag new output pulse, and reload timer value
    ; note: tmr0 is the only enabled interrupt, so no source checking is done
  ; save registers (recipe from Microchip documentation)
  movwf tempW
  swapf STATUS, W
  bcf STATUS, RP0
  movwf tempStatus

  ; do the job
  movlw TMR0_PERIOD ; prepare next event
  movwf TMR0
  bcf INTCON, T0IE ; disable future interrupts until handled
  ; the interrupt flag is kept on, for main loop to discover

  ; get registers back
  swapf tempStatus, W
  movwf STATUS
  swapf tempW, F
  swapf tempW, W

  retfie

boxOutput ; reduce output to absolute value limits to protect servo [0-1200]
; negative value ? set to zero
  btfss outputH, 7 ; test bit 7 for negative input
  goto boxOutputHigh ; positive/clear, check high limit
  clrf outputH ; negative - set to minimum = 0
  clrf outputL
  return

boxOutputHigh
  ; if output > VALUE_HIGH then output = VALUE_HIGH
  movfw outputH
  sublw VALUE_HIGH_MSB 
  btfss STATUS, C
  goto setHighOutput
  btfss STATUS, Z ; equal ?
  return ; no, less, which is ok
  movfw outputL
  sublw VALUE_HIGH_LSB
  btfss STATUS, C
  goto setHighOutput ; carry clear - too high output
  return ; output ok

setHighOutput ; above high limit, set to high limit
  movlw VALUE_HIGH_MSB ; copy high constant
  movwf outputH        ; into output
  movlw VALUE_HIGH_LSB
  movwf outputL
  return

  end
