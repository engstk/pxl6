# SPDX-License-Identifier: GPL-2.0
#
# Makefile for EdgeTPU framework and dependent drivers.
#

obj-$(CONFIG_ABROLHOS)		+= abrolhos.o

edgetpu-objs	:= edgetpu-async.o edgetpu-dmabuf.o edgetpu-iremap-pool.o \
		   edgetpu-kci.o edgetpu-mailbox.o edgetpu-mapping.o \
		   edgetpu-sw-watchdog.o edgetpu-telemetry.o \
		   edgetpu-firmware-util.o edgetpu-firmware.o

abrolhos-objs	:= abrolhos-core.o abrolhos-debug-dump.o \
		   abrolhos-device-group.o abrolhos-device.o \
		   abrolhos-firmware.o abrolhos-fs.o abrolhos-iommu.o \
		   abrolhos-platform.o abrolhos-pm.o abrolhos-thermal.o \
		   abrolhos-usage-stats.o abrolhos-wakelock.o \
		   $(edgetpu-objs)



KBUILD_OPTIONS += CONFIG_ABROLHOS=m

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $(KBUILD_OPTIONS) $(@)
