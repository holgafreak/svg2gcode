CC = clang
CFLAGS = -Ofast

all: svg2gcode

svg2gcode:	svg2gcode.c nanosvg.h
	$(CC) -o svg2gcode -g svg2gcode.c -lm

clean:
	rm -fr svg2code *.o

