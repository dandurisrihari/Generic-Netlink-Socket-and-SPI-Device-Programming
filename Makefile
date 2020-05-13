


KDIR:=/opt/iot-devkit/1.7.2/sysroots/i586-poky-linux/usr/src/kernel
ARCH = x86
CC = i586-poky-linux-gcc
CROSS_COMPILE = i586-poky-linux-
SROOT=/opt/iot-devkit/1.7.2/sysroots/i586-poky-linux
#PWD:= $(shell pwd)
export PATH:=/opt/iot-devkit/1.7.2/sysroots/x86_64-pokysdk-linux/usr/bin:/opt/iot-devkit/1.7.2/sysroots/x86_64-pokysdk-linux/usr/bin/i586-poky-linux:$(PATH)


EXTRA_CFLAGS += -g -Wall -O0

LDLIBS = -L$(SROOT)/usr/lib
CCFLAGS = -I$(SROOT)/usr/include/libnl3

APP = 
EXAMPLE = client

obj-m:= server.o

.PHONY:all
all: #client.ko client 
	make ARCH=x86 CROSS_COMPILE=i586-poky-linux- -C $(SROOT)/usr/src/kernel M=$(PWD) modules
	$(CC) -Wall -o $(EXAMPLE) client.c $(CCFLAGS) -lnl-genl-3 -lnl-3 -pthread 
# client:
# 	$(CC) -Wall -o $(EXAMPLE) client.c $(CCFLAGS) -lnl-genl-3 -lnl-3

.PHONY:clean
clean:
	make -C $(KDIR) M=$(PWD) clean
	rm -f *.o $(EXAMPLE) $(APP)

# deploy:
# 	tar czf programs.tar.gz $(APP) $(EXAMPLE) server.ko
# 	scp programs.tar.gz root@10.0.1.100:/home/root
# 	ssh root@10.0.1.100 'tar xzf programs.tar.gz'
