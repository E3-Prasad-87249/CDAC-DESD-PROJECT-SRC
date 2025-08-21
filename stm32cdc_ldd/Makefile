obj-m = stm32cdc_driver.o

stm32cdc_driver.ko: stm32cdc_driver.c
	make -C /lib/modules/$$(uname -r)/build M=$$(pwd) modules

clean:
	make -C /lib/modules/$$(uname -r)/build M=$$(pwd) clean

.PHONY : clean
