##############
#  Makefile  #
##############

CC=qcc
CFLAGS= -O2 -w8
OBJ=main.o pcidev.o CNT32_8M.o
LIBS= -lm

all:robot

clean:
	/bin/rm -f *.o *~
.c.o.h:
	$(CC) $(CFLAGS) -c $(LIBS) $<

pcidev.o: pcidev.c pcidev.h 

CNT32_8M.o: CNT32_8M.c CNT32_8M.h  pcidev.h

main.o: main.c CNT32_8M.h

robot: $(OBJ) 
	$(CC) -o robot $(OBJ) $(LIBS)

