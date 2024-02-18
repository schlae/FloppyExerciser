PRG            = floppystep
OBJ            = floppystep.o
MCU_TARGET     = atmega328pb
OPTIMIZE       = -O2

DEFS           =
LIBS           =

# You should not have to change anything below here.

CC             = avr-gcc

# Override is only needed by avr-lib build system.

override CFLAGS        = -g -Wall $(OPTIMIZE) -mmcu=$(MCU_TARGET) $(DEFS)
override LDFLAGS       = -Wl,-Map,$(PRG).map

OBJCOPY        = avr-objcopy
OBJDUMP        = avr-objdump

all: $(PRG).elf hex

$(PRG).elf: $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

clean:
	rm -rf *.o $(PRG).elf *.bak
	rm -rf *.map *.hex

hex:  $(PRG).hex

%.hex: %.elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

prog:
	avrdude -c usbtiny -p m328pb -Uflash:w:$(PRG).hex:i

fuses:
	avrdude -c usbtiny -p m328pb -Ulfuse:w:0xE2:m -Uhfuse:w:0xD9:m -Uefuse:w:0xF7:m
