# polyphonic_dac
Polyphonic DAC for controlling polyphonic analog synths with a teensy 3.6 (currently 3.6) and MCP4822 chips
work in progress, unstable
https://imgur.com/vuFifvL

Features: 
* USBhost to pitch & gate
* Unison mode - bool variable for tuning oscillators (sends same pitch on every DAC with gate) (unisonMode=true/false)
* Gate mode - sends gate on channel B of MCP4822
* Normal mode - sends different pitches on every channel and DAC (no gates)

Chip select pins are by default on pins 2, 3, 4, 5, 7

to do:
lowest note priority / note allocation according to time pressed
