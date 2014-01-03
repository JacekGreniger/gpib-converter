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


Parts list:
IC1 - ATmega32, TQFP44
IC2 - FT232RL
LED1 - LED 0805
R1 - 560ohm, 0805
C1 - 100nF, 0805
C2 - 4.7uF, 10V, B size
X1 - connector mini-usb (TME: ESB34101000Z, Farnell: 2112374)
Connector Wtyk 24pin Centronics (TME: CENTR-224, Farnell: 2112386)
