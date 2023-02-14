CC = xtensa-esp32s3-elf-gcc
CFLAGS = -Ofast -v

all: libsvg2gcode.a

libsvg2gcode.a: svg2gcode.o
	xtensa-esp32s3-elf-ar rcs libsvg2gcode.a svg2gcode.o

svg2gcode.o: svg2gcode.c nanosvg.h
	$(CC) $(CFLAGS) -c svg2gcode.c

clean:
	rm -fr svg2gcode.o libsvg2gcode.a
