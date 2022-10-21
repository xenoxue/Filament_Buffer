# Filament_Buffer
A Filament buffer system for Klipper, Inspired by BambuLab X1

# Requirement
1. Indenpeden mcu for buffer stepper. I'm using BTT EBB canbus board at this stage.
2. An extruder, BIQU H2, orbiter, or even MK8 will work, I'm using BIQU H2 because I have spares.
3. An Endstop, MicroSwitch/Optical/Others type depends on what your board is support. I have using Microwitch on H2 style, and Optical on Bambu-like buffer.
4. 2 Filament Tube + 2 PC4-01 quick connector
5. (optional) For Bambu-like Buffer you will need a Bambu Spring at 11mm diameter.

# Usage
1. Get your STLs printed and assembled.
2. copy and paste Klipper/extras/buffer_stepper.py in to your Klipper/Klipper/extras folder.
3. restart Klipper Sevice, and restart Klipper

you are now good to Go!
