########################
#  !!!GOD Makefile!!!  #
########################

CC=qcc
CFLAGS= -O2 -w8
OBJ=main.o Arcnet/arc_pci.o cp.o timer.o command.o
OBJ2=Arcnet/DeviceIF/pcidev.o Arcnet/pi/PI_128LH.o Arcnet/po/PO_128LH.o Arcnet/dio/DIO_6464T2.o Arcnet/da/DA16_16L.o Arcnet/cnt/CNT32_8M.o Arcnet/DeviceIF/DeviceIF.o
BOARD_H=Arcnet/pi/PI_128LH.h Arcnet/po/PO_128LH.h Arcnet/dio/DIO_6464T2.h Arcnet/da/DA16_16L.h Arcnet/cnt/CNT32_8M.h
LIBS= -lm

all:robot

clean:
	/bin/rm -f *.o *~
.c.o.h:
	$(CC) $(CFLAGS) -c $(LIBS) $<

pcidev.o: Arcnet/DeviceIF/pcidev.c Arcnet/DeviceIF/pcidev.h 

Arcnet/DeviceIF/DeviceIF.o: Arcnet/DeviceIF/DeviceIF.c Arcnet/DeviceIF/DeviceIF.h Arcnet/DeviceIF/pcidev.h cfg_arm.h main_cfg.h $(BOARD_H)

Arcnet/arc_pci.o: Arcnet/arc_pci.h Arcnet/arc_pci_cfg.h main_cfg.h Arcnet/DeviceIF/DeviceIF.h

timer.o: timer.c timer.h main_cfg.h

main.o: main_cfg.h cfg_arm.h params_arm.h Arcnet/arc_pci.h timer.h 

cp.o: cp.c 

command.o: params_arm.h main_cfg.h

robot: $(OBJ) 
	$(CC) -o robot $(OBJ) $(OBJ2) $(LIBS)

