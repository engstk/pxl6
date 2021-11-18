export CONFIG_MCPS_FAKE := n
export CONFIG_MCPS802154_TESTMODE := y
export CONFIG_QCOM_SPI_LOWLATENCY := n

KDIR ?= /lib/modules/$(shell uname -r)/build
PROJECTS_BUILD := $(abspath $(CURDIR)/../projects/linux-build)

ifeq ($(PROJECTS_BUILD),$(abspath $(KDIR)/../../../../..))

# Build in projects/linux-build, use a separated build tree.

O := $(abspath $(KDIR))
M := ../../../kernel
B := $(abspath $O/$M)

$(info *** Building in $B)

default: modules

modules modules_install:
	mkdir -p $B
	ln -srTf include $B/include
	ln -srTf Kbuild $B/Kbuild
	$(MAKE) -C $(KDIR) O=$O M=$M $@

clean help kernelrelease:
	$(MAKE) -C $(KDIR) O=$O M=$M $@

else # ifeq ($(PROJECTS_BUILD),$(abspath $(KDIR)/../../../../..))

# Regular build, build here.
ifneq ($(KERNEL_SRC),)
# Compile Android external module in OUT directory
# KERNEL_SRC = Kernel path
$(info *** Building Android in $O/$M )
KDIR := $(KERNEL_SRC)

all:
	$(MAKE) -C $(KDIR) M=$(M) modules $(KBUILD_OPTIONS)

else # ifneq ($(KERNEL_SRC),)
M := $(CURDIR)
endif # ifneq ($(KERNEL_SRC),)

B := $(abspath $O/$M)

default:
	$(MAKE) -C $(KDIR) M=$(M)
modules modules_install clean help kernelrelease:
	$(MAKE) -C $(KDIR) M=$(M) $@

endif # ifeq ($(PROJECTS_BUILD),$(abspath $(KDIR)/../../../../..))

# Install generated dw3000.dtbo
dtbo_install: modules_install
	d=`echo $(INSTALL_MOD_PATH)/lib/modules/* | sed 's#lib/modules#boot/dtbs#'` && \
	mkdir -p $$d/overlays && \
	find $B -name '*.dtbo' -print -exec cp -v {} $$d/overlays \;
