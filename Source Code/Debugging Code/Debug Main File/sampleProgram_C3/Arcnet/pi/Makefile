##############
#  Makefile  #
##############

CC=qcc
CFLAGS= -O2 -w8
OBJ=main.o pcidev.o PI_128LH.o
LIBS= -lm

all:robot

clean:
	/bin/rm -f *.o *~
.c.o.h:
	$(CC) $(CFLAGS) -c $(LIBS) $<

pcidev.o: pcidev.c pcidev.h 

PI_128LH.o: PI_128LH.h PI_128LH.c  pcidev.h

main.o: main.c PI_128LH.h

robot: $(OBJ) 
	$(CC) -o robot $(OBJ) $(LIBS)

