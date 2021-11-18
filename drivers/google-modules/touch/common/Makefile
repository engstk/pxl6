obj-$(CONFIG_TOUCHSCREEN_TBN)		+= touch_bus_negotiator.o
obj-$(CONFIG_TOUCHSCREEN_HEATMAP)	+= heatmap.o
obj-$(CONFIG_TOUCHSCREEN_OFFLOAD)	+= touch_offload.o

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)
KERNEL_UAPI_HEADERS_DIR ?= $(shell readlink -m ${COMMON_OUT_DIR}/kernel_uapi_headers)

KBUILD_OPTIONS	+= CONFIG_TOUCHSCREEN_TBN=m
KBUILD_OPTIONS	+= CONFIG_TOUCHSCREEN_HEATMAP=m
KBUILD_OPTIONS	+= CONFIG_TOUCHSCREEN_OFFLOAD=m
EXTRA_CFLAGS	+= -DDYNAMIC_DEBUG_MODULE
EXTRA_CFLAGS	+= -I$(KERNEL_SRC)/../google-modules/touch/common/include

modules clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) \
	$(KBUILD_OPTIONS) \
	EXTRA_CFLAGS="$(EXTRA_CFLAGS)" \
	$(@)

modules_install: headers_install
	$(MAKE) -C $(KERNEL_SRC) M=$(M) \
	$(KBUILD_OPTIONS) \
	EXTRA_CFLAGS="$(EXTRA_CFLAGS)" \
	$(@)

headers_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) \
	INSTALL_HDR_PATH="${KERNEL_UAPI_HEADERS_DIR}/usr" \
	$(@)
