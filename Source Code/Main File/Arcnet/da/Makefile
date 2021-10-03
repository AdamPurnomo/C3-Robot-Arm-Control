##############
#  Makefile  #
##############

CC=qcc
CFLAGS= -O2 -w8
OBJ=main.o pcidev.o DA16_16L.o
LIBS= -lm

all:robot

clean:
	/bin/rm -f *.o *~
.c.o.h:
	$(CC) $(CFLAGS) -c $(LIBS) $<

pcidev.o: pcidev.c pcidev.h 

DA16_16L.o: DA16_16L.c DA16_16L.h  pcidev.h

main.o: main.c DA16_16L.h

robot: $(OBJ) 
	$(CC) -o robot $(OBJ) $(LIBS)

