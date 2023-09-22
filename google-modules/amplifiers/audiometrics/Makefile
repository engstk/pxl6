obj-$(CONFIG_AUDIOMETRICS)     += audiometrics.o

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)

KBUILD_OPTIONS += CONFIG_AUDIOMETRICS=m

headers-y += uapi/audiometrics_api.h

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $(KBUILD_OPTIONS) $(@)
