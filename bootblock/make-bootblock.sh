#!/bin/bash

z88dk-z80asm -b bootblock.asm

dd if=/dev/zero of=fill bs=892 count=1

cat bootblock.bin fill >temp.bin

rm bootblock.bin

mv temp.bin bootblock.bin

