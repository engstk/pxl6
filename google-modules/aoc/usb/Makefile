#
# ASoC-based USB driver
#
obj-$(CONFIG_AOC_DRIVER) += aoc_usb_driver.o
aoc_usb_driver-objs := aoc_usb_dev.o xhci_hooks_impl_whi.o usb_hooks_impl_whi.o snd_usb_audio_hook_impl_whi.o

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)
KBUILD_OPTIONS += CONFIG_AOC_DRIVER=m
EXTRA_CFLAGS := -I$(srctree)/$(src)/../../aoc_ipc -I$(srctree)/$(src)/.. -I$(srctree)/$(src)/../../../drivers/usb/host -I$(srctree)/$(src)/../../../sound/usb

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $(KBUILD_OPTIONS) EXTRA_CFLAGS="$(EXTRA_CFLAGS)" $(@)
