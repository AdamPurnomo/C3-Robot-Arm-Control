##############
#  Makefile  #
##############

CC=qcc
CFLAGS= -O2 -w8
OBJ=main.o pcidev.o DIO_6464T2.o
LIBS= -lm

all:robot

clean:
	/bin/rm -f *.o *~
.c.o.h:
	$(CC) $(CFLAGS) -c $(LIBS) $<

pcidev.o: pcidev.c pcidev.h 

DIO_6464T2.o: DIO_6464T2.c DIO_6464T2.h  pcidev.h

main.o: main.c DIO_6464T2.h

robot: $(OBJ) 
	$(CC) -o robot $(OBJ) $(LIBS)

