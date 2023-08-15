all: bt_bio.elf

bt_bio.elf: bt_bio.c mpu9250.h
	msp430-gcc -mmcu=msp430g2553 -O0 -g -o bt_bio.elf bt_bio.c

bt_bio.s: bt_bio.c mpu9250.h
	msp430-gcc -S -mmcu=msp430g2553 -O0 -g bt_bio.c

obj-dump:
	objdump -s bt_bio.elf > bt_bio.dump

clean:
	rm -f *.elf *.hex
