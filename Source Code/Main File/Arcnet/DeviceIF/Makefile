##############
#  Makefile  #
##############

CC=qcc
CFLAGS= -O2 -w8
OBJ=pcidev.o ../pi/PI_128LH.o ../po/PO_128LH.o ../dio/DIO_6464T2.o ../da/DA16_16L.o ../cnt/CNT32_8M.o DeviceIF.o main.o
BOARD_H=../pi/PI_128LH.h ../po/PO_128LH.h ../dio/DIO_6464T2.h ../da/DA16_16L.h ../cnt/CNT32_8M.h
LIBS= -lm

all:robot

clean:
	/bin/rm -f *.o *~
.c.o.h:
	$(CC) $(CFLAGS) -c $(LIBS) $<

pcidev.o: pcidev.c pcidev.h 

DeviceIF.o: DeviceIF.c DeviceIF.h pcidev.h ../../cfg_arm.h ../../main_cfg.h $(BOARD_H)

main.o: main.c DeviceIF.h

robot: $(OBJ) 
	$(CC) -o robot $(OBJ) $(LIBS)

