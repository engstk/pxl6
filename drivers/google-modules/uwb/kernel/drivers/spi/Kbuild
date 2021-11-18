kernel_version_ge = $(shell \
test $$((($(VERSION) << 16) + ($(PATCHLEVEL) << 8) + $(if $(SUBLEVEL);$(SUBLEVEL),0))) \
-ge $$((($(1) << 16) + ($(2) << 8) + $(3))) && echo 1 || echo 0 \
)

# Build spi-geni-qcom-lowlatency.ko only if SPI and Qualcomm arch are supported
# on target kernel and the kernel version is at least 4.20.0. The Qualcomm GENI
# SPI driver appeared in version 4.20.0.
ifdef CONFIG_SPI_MASTER
ifdef CONFIG_ARCH_QCOM
ifeq ($(call kernel_version_ge,4,20,0),1)
obj-$(CONFIG_QCOM_SPI_LOWLATENCY) += spi-geni-qcom-lowlatency.o
endif
ifdef CONFIG_ARCH_MSM8998
# Build spi-qsd-qcom-lowlatency.ko only on AOSP msm kernel like:
# https://android.googlesource.com/kernel/msm/+/refs/heads/android-msm-wahoo-4.4-android11
obj-$(CONFIG_QCOM_SPI_LOWLATENCY) += spi-qsd-qcom-lowlatency.o
endif
endif
endif
