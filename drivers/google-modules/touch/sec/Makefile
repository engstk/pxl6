ifeq ($(CONFIG_SAMSUNG_PRODUCT_SHIP), y)
  obj-$(CONFIG_TOUCHSCREEN_SEC_TS)   += sec_ts.o sec_ts_fw.o sec_ts_fn.o sec_cmd.o
else
  obj-$(CONFIG_TOUCHSCREEN_SEC_TS)   += sec_ts.o sec_ts_fw.o sec_ts_fn.o sec_cmd.o sec_ts_only_vendor.o
endif

KBUILD_OPTIONS	+= CONFIG_TOUCHSCREEN_SEC_TS=m
EXTRA_CFLAGS	+= -DDYNAMIC_DEBUG_MODULE
EXTRA_CFLAGS	+= -DCONFIG_TOUCHSCREEN_TBN
EXTRA_CFLAGS	+= -DCONFIG_TOUCHSCREEN_HEATMAP
EXTRA_CFLAGS	+= -DCONFIG_TOUCHSCREEN_OFFLOAD
EXTRA_CFLAGS	+= -I$(KERNEL_SRC)/../google-modules/display
EXTRA_CFLAGS	+= -I$(KERNEL_SRC)/../google-modules/touch/common
EXTRA_CFLAGS	+= -I$(KERNEL_SRC)/../google-modules/touch/common/include
EXTRA_SYMBOLS	+= $(OUT_DIR)/../google-modules/touch/common/Module.symvers

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) \
	$(KBUILD_OPTIONS) \
	EXTRA_CFLAGS="$(EXTRA_CFLAGS)" \
	KBUILD_EXTRA_SYMBOLS="$(EXTRA_SYMBOLS)" \
	$(@)
