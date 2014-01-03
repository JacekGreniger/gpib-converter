gpib-converter
==============

GPIB to USB converter

Source code for the device published in Elektronika Praktyczna 2/2013

Short PB5 to GND during powering on to enter printer (dummy) mode. In this mode all data sent over GPIB
are transferred to PC (use your favourite terminal program to save data into file). Use this mode to get plots from your GPIB device (spectrum analyzer, oscilloscope).
Received plots can be rendered using KE5FX HP7470 emulator package. Printer mode can be also entered using
command "p".

ATmega32 is clocked using 12MHz clock taken from pin C0 of FT232RL USB to serial converter. Use programming
order as below:

1. Program FT232RL using proper utility (see hw/ft232_settings.jpg)
2. Program ATmega32 using ISP programmer (sw/gpib_conv_v4.hex)
3. Set proper fusebits (see hw/fusebits.jpg)
