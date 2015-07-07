
# For building the kernel module
obj-m := certikos_ivc_kmod.o lynxsecure_ivc_kmod.o

all: certikos_ivc_kmod.ko lynxsecure_ivc_kmod.ko

certikos_ivc_kmod.ko: certikos_ivc_kmod.c
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) certikos_ivc_kmod.ko

lynxsecure_ivc_kmod.ko: lynxsecure_ivc_kmod.c
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) lynxsecure_ivc_kmod.ko

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean


