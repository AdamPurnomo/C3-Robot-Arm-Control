##############
#  Makefile  #
##############

CC=qcc
CFLAGS= -O2 -w8
OBJ=main.o pcidev.o PO_128LH.o
LIBS= -lm

all:robot

clean:
	/bin/rm -f *.o *~
.c.o.h:
	$(CC) $(CFLAGS) -c $(LIBS) $<

pcidev.o: pcidev.c pcidev.h 

PO_128LH.o: PO_128LH.h PO_128LH.c  pcidev.h

main.o: main.c PO_128LH.h

robot: $(OBJ) 
	$(CC) -o robot $(OBJ) $(LIBS)

