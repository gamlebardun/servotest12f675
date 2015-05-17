# servotest12f675
RC servo test appliance using Microchip PIC 12F675

The idea was to create a hand-held appliance to work with servos when installing in rc airplanes, and ensure correct movement of wing surfaces, throttle valve and perhaps most important - zero point of servo.
I added a bit more as i tested and used the appliance - it is now a neat tool for doing the intended job.

There are four external components in addition to battery (with power on/off) and the PIC 12F675 itself ;
* two three-position switches on-off-on
* a potensiometer with a good finger-operated knob
* 10uF capacitor decoupling input power close to the IC. 
 
Two unused pins should be coupled to Vdd with a 10k resistor.

Mode 1: Manual setting of servo position with pot. This is useful to carefully move in a problematic range

Mode 2: Automatic sweep end to end. I first gave the pot control over speed, but found that limiting deflection was a better use. The unused three-position switch was assigned the task of choosing upper, entire or lower range of movement with respect to zero

Mode 3: The three position switch places the servo in either extreme position or zero. The pot again can limit the output at extreme position.s

An altoid size box would easily fit all components to make it portable.
