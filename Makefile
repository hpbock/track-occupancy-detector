MCU=attiny2313
AVRDUDEMCU=t2313
CC=/usr/bin/avr-gcc
CFLAGS=-g -Os -Wall -mcall-prologues -mmcu=$(MCU)
OBJ2HEX=/usr/bin/avr-objcopy
AVRDUDE=/usr/bin/avrdude
TARGET=gbmx4-cpu

program : $(TARGET).bin $(TARGET).hex
	$(AVRDUDE) -p $(AVRDUDEMCU) -c usbtiny -U flash:w:$(TARGET).bin -U eeprom:w:$(TARGET).hex -U lfuse:w:0xe4:m -U hfuse:w:0xdb:m

%.obj : %.o
	$(CC) $(CFLAGS) $< -o $@

%.bin : %.obj
	$(OBJ2HEX) -R .eeprom -O binary $< $@


%.hex : %.obj
	$(OBJ2HEX) -j .eeprom -O ihex $< $@

clean :
	rm -f *.bin *.hex *.obj *.o
