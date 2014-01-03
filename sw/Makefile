
CC= avr-gcc
MCU=atmega32

OPTIMIZE       = -O

# Definicje plików z wygenerowanymi listingami
LST = $(SRC:.c=.lst) $(ASRC:.asm=.lst) 
PRG = mapa

override LDFLAGS       = -Wl,-Map,$(PRG).map
CFLAGS=  $(OPTIMIZE) -g -Wall -ffreestanding -mmcu=$(MCU)
#CFLAGS += -ahlms=$(<:.c=.lst)

.SUFFIXES: .s .bin .out .hex .srec

.c.s:
	$(CC) $(CFLAGS) -S $<

.S.o:
	$(CC) $(ASFLAGS) -c $<

.o.out:
	$(CC) $(CFLAGS) -o $@ $<

.out.bin:
	avr-objcopy -O binary $< $@

.out.hex:
	avr-objcopy -O ihex $< $@

.out.srec:
	avr-objcopy -O srec $< $@

all:	gpib_conv_v4.hex

OBJS = main.o usart.o

gpib_conv_v4.out: $(OBJS)
	$(CC) -o gpib_conv_v4.out $(CFLAGS) $(LDFLAGS) $(OBJS) $(LDLIBS)

clean:
	rm -f *~ *.out *.bin *.hex *.srec *.s *.o *.pdf *core


