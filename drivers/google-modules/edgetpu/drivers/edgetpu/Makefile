# SPDX-License-Identifier: GPL-2.0
#
# Makefile for EdgeTPU framework and dependent drivers.
#

obj-$(CONFIG_ABROLHOS)		+= abrolhos.o

GIT_PATH=$(srctree)
ifeq ($(shell git --git-dir=$(GIT_PATH)/.git rev-parse --is-inside-work-tree),true)
	GIT_REPO_STATE=$(shell (git --git-dir=$(GIT_PATH)/.git --work-tree=$(GIT_PATH) status --porcelain | grep -q .) && echo -dirty)
	ccflags-y       += -DGIT_REPO_TAG=\"$(shell git --git-dir=$(GIT_PATH)/.git rev-parse --short HEAD)$(GIT_REPO_STATE)\"
else
	ccflags-y       += -DGIT_REPO_TAG=\"Not\ a\ git\ repository\"
endif

edgetpu-objs	:= edgetpu-async.o edgetpu-dmabuf.o edgetpu-iremap-pool.o \
		   edgetpu-kci.o edgetpu-mailbox.o edgetpu-mapping.o \
		   edgetpu-sw-watchdog.o edgetpu-telemetry.o \
		   edgetpu-firmware-util.o edgetpu-firmware.o \
		   edgetpu-domain-pool.o

abrolhos-objs	:= abrolhos-core.o abrolhos-debug-dump.o \
		   abrolhos-device-group.o abrolhos-device.o \
		   abrolhos-firmware.o abrolhos-fs.o abrolhos-iommu.o \
		   abrolhos-platform.o abrolhos-pm.o abrolhos-thermal.o \
		   abrolhos-usage-stats.o abrolhos-wakelock.o \
		   $(edgetpu-objs)



KBUILD_OPTIONS += CONFIG_ABROLHOS=m

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) $(KBUILD_OPTIONS) $(@)
