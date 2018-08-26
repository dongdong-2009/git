all:start.o main.o mmu.o gpio.o interrupt.o nand.o
	arm-linux-ld $^ -Tmyboot.lds -o myboot.elf 
	arm-linux-objcopy myboot.elf -O binary myboot.bin

#对不同后缀的文件进行编译，需要分开写
%.o : %.S
	arm-linux-gcc -g -c $^ -o $@
%.o : %.c
	arm-linux-gcc -g -c $^ -o $@	

#清除
clean:
	@rm *.o *.elf *.bin 
