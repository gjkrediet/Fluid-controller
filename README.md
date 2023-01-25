# Fluid-controller
A wireless battery powered pendant for FluidNC based on esp32

![20230125_140826](https://user-images.githubusercontent.com/20277013/214572050-37decb40-87fa-4ef0-94c6-3ae00d252642.jpg)

I will update the description in the next weeks
# main functions
### Display shows: 
Machine coördinates, Work coördinates, Spindle speed and state, Step size and speed for joystick and rotation encoder, Overrides (Speed, Feed, Rapid),	Machine state, Battery state, Current buttons functions
### Main functions buttons, joystick and rotary knob: 
Wake up, Jog gantry and spindle via joystick and/or dial, Set Jog step size and speed via dial, Set spindle speed and switch on/off,	Set overrides (Speed, Feed, Rapid),	Cancel/abort job.
### Main functions menu: 
Home machine, Zero workcoördinates, Go to zero, Probe.

# hardware
1. LilyGO TTGO T-Display V1.1 which is a developmentboard with an ESP32, with a built-in tft (IPS) display and battery-charger. I purchased it for about €13 from internet.
2. Rotary encoder (about €1)
3. Three simple colored buttons NO (total of €2)
4. PSP joystick (about €3)
5. Lion 18650 battery (€7)
6. Single-sided pcb

# Schematic and PCB

The pcb only holds the TTGO developmentboard and the rotary-encoder. The other components are mounted on the lid and wired to the pcb. The schematic of the pcb (in Kicad) is therefore very simple. The isolation-routing of the PCB was done with the very same CNC-router I made this controller for. To fit in the joystick, a corner of the PCB had to be sawed of (of course I shoud have done this in Kicad but did not).

The three buttons have common ground (J1.1) and are connected tot J1.2, J1.3 and J1.4 respectively.
The joystick is connected tot J2 with J2.2 unconnected (since I later decided to wire the VCC of the joystick tot an output-pin).

# Programming

...

# Case

<img width="440" alt="pendantV2case_lid2" src="https://user-images.githubusercontent.com/20277013/214568520-32bf0ae3-2ae2-4814-8294-004ee3288210.png">
<img width="440" alt="pendantV2lid_case" src="https://user-images.githubusercontent.com/20277013/214570138-59b09fc4-4332-4c2e-8d71-3366ad1cf684.png">

# Assembly

Schematic drawing of the lid with positioning of the components (excl. battery).

![afbeelding](https://user-images.githubusercontent.com/20277013/214533466-feb4534e-7402-42ab-9b6a-d86a29457133.png)

![inside1](https://user-images.githubusercontent.com/20277013/214524701-8da7550d-7ca5-4af7-9b36-db64cc68cfa0.jpg)

To further reduce the powerconsumption I decided to feed the joystick via an outputpin of the esp32, which is why you see the red wire soldered on later. 
![inside2](https://user-images.githubusercontent.com/20277013/214524742-b8a347a4-dc82-47c2-a12c-f6e3894d6a2d.jpg)

The battery is wired to the JST-connector on the TTGO
