##############
#  Makefile  #
##############

CC=qcc
CFLAGS= -O2 -w8
OBJ=main.o REX_PCI60D.o pcidev.o ../dio/DIO_6464T2.o
LIBS= -lm

all:robot

clean:
	/bin/rm -f *.o *~
.c.o.h:
	$(CC) $(CFLAGS) -c $(LIBS) $<

pcidev.o: pcidev.c pcidev.h 

REX_PCI60D.o: REX_PCI60D.c REX_PCI60D.h pcidev.h

main.o: main.c REX_PCI60D.h pcidev.h ../dio/DIO_6464T2.h

robot: $(OBJ) 
	$(CC) -o robot $(OBJ) $(LIBS)

