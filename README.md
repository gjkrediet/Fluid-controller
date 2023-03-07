# Fluid-controller
A wireless battery powered pendant for FluidNC based on esp32

![20230125_140826](https://user-images.githubusercontent.com/20277013/214572050-37decb40-87fa-4ef0-94c6-3ae00d252642.jpg)

I will update the description in the next weeks
# Main functions
Display shows: 
Machine coördinates, Work coördinates, Spindle speed and state, Set speed and step size when using joystick and rotation encoder, Overrides (Speed, Feed, Rapid),	Machine state, Battery state, Current buttons functions

Main functions buttons, joystick and dial: 
Wake up, Jog gantry and spindle via joystick and/or dial, Set Jog step size and speed via dial, Set spindle speed and switch on/off,	Set overrides (Speed, Feed, Rapid),	Cancel/abort job.

Main functions menu: 
Home machine, Zero workcoördinates, Go to zero, Probe.

# Hardware
1. LilyGO TTGO T-Display V1.1 which is a developmentboard with an ESP32, with a built-in tft (IPS) display and battery-charger. I purchased it for about €13 from internet.
2. Rotary encoder / dial (about €1)
3. Three simple colored buttons NO (total of €2)
4. PSP joystick (about €3)
5. Lion 18650 battery (€7)
6. Single-sided pcb
7. PLA (25 meter)

# Schematic and PCB

The pcb only holds the TTGO developmentboard and the rotary-encoder. The other components are mounted on the lid and wired to the pcb. The schematic of the pcb (in Kicad) is therefore very simple. The isolation-routing of the PCB was done with the very same CNC-router I made this controller for. To fit in the joystick, a corner of the PCB had to be sawed of (of course I shoud have done this in Kicad but forgot to).

The three buttons are at one side connected to the ground (all to J1.1) and are connected tot J1.2, J1.3 and J1.4 respectively.
The joystick is connected tot J2 with J2.2 unconnected (since I later decided to wire the VCC of the joystick tot an output-pin).

# Programming

...

# Case

<img width="440" alt="pendantV2case_lid2" src="https://user-images.githubusercontent.com/20277013/214568520-32bf0ae3-2ae2-4814-8294-004ee3288210.png">
<img width="440" alt="pendantV2lid_case" src="https://user-images.githubusercontent.com/20277013/214570138-59b09fc4-4332-4c2e-8d71-3366ad1cf684.png">

# Assembly

Schematic drawing of the lid with positioning of the components (excl. battery).

![afbeelding](https://user-images.githubusercontent.com/20277013/214533466-feb4534e-7402-42ab-9b6a-d86a29457133.png)

Female headers and the rotary encoder are soldered on the pcb. The other components are connected to the PCB with wires. Plus and minus wires are directly soldered to the battery and connected to the TTGO with a JST-connector.

![inside1](https://user-images.githubusercontent.com/20277013/214524701-8da7550d-7ca5-4af7-9b36-db64cc68cfa0.jpg)

To further reduce the powerconsumption I decided to feed the joystick via an outputpin of the esp32, which is why you see the red wire soldered on later.

![inside2](https://user-images.githubusercontent.com/20277013/214524742-b8a347a4-dc82-47c2-a12c-f6e3894d6a2d.jpg)

The battery is wired to the JST-connector on the TTGO

# How to operate

The functions of the pendant were tailored to my needs. Below an oversight of the functions of the various buttons and button-combinations and the menu-options. It might be somewhat cryptic but I hope it may help.

JS=Joystick, BD=dial, BR=red button, BG=green button, BY=yellow button

Switch the pendant on: press JS

Reset the pendant: press JS

The pendant powers off after some time when idle or via the appropriate menu-option.


The pendant can either be in home-state (PH) or menu-state (PM).

Function of the dial-knob can be changed by rotating the dial while pressing BG.

Functions can be (default is DJ; function is indicated on the display with a blue circle):
- DX: Jog 1 step X-axis
- DY: Jog 1 step Y-axis
-	DZ: Jog 1 step Z-axis
-	DS: Set spindle-speed
-	DJ: set jogspeed and step-size
-	DOF: set override Feed
-	DOS: set override Speed
-	DOR: set override Rapid

The function of the buttons depend on the machine-state (MS, shown in the middle of the display of the pendant; function of BR, BG, BY shown on the bottom line of the display). They are as follows.

PH:
  -	JS + MS=Idle or MS=Jog:
    -	Up/down: jog Y-axis
    -	Left/Right: jog X-axis
  -	JS + MS=Idle or MS=Jog + BR (long-press)
    -	Up/down: jog Z-axis
    -	Left/Right: no function
  -	BG: enter PM
  -	BY: abort machine-operation
  -	BG (keep pressed) + BD: Change function dial-knob
  -	BR + MS=Alarm: Unlock Machine
  -	BR + DS: Spindle on/off

PM:
  -	BD: navigate menu (poweroff, spindlespeed, spindle on/off, small/large steps for changing jogspeed/stepsize, exit menu, home the machine, Set current Work XY-position to zero, Set current Work Z-position to zero, Probe Z-axis, Jog machine to XY-position zero, unlock Machine (grbl), reset Machine (grbl), Set and save brightness).
  -	BG: confirm menu-choice
  -	BR: Cancel
  -	BR (long press): poweroff

