##############
#  Makefile  #
##############

CC=qcc
CFLAGS= -O2 -w8
OBJ=arc_pci.o device.o
LIBS= -lm

all: arc_pci.o

clean:
	/bin/rm -f *.o *~
.c.o.h:
	$(CC) $(CFLAGS) -c $(LIBS) $<

arc_pci.o: arc_pci.c arc_pci.h ./DeviceIF/DeviceIF.h

