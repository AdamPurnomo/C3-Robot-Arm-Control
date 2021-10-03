#!/bin/sh
pwd
cd /home/yoshinaga/graviy_compensation_C3/Arcnet/
ld -r device.o arc_pci2.o -o arc_pci.o
