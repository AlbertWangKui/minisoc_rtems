# Makefile for running menuconfig.
# The main build is now handled by WAF.

# Default to silent operation. Use 'make V=1' for verbose output.
ifeq ("$(origin V)", "command line")
    KBUILD_VERBOSE = $(V)
else
    KBUILD_VERBOSE = 0
endif

ifeq ($(KBUILD_VERBOSE),1)
    Q =
else
    Q = @
endif

srctree := .
KCONFIG ?= $(srctree)/Kconfig
PYTHON ?= python

.PHONY: all menuconfig

# Default target to inform the user about the build system change.
all:
	@echo "This Makefile is for configuration only."
	@echo "To build the project, please use the WAF build system (e.g., './waf build')."
	@echo "To configure the project, run: 'make menuconfig'"

# Target to run Kconfig menuconfig.
menuconfig:
	$(Q)$(PYTHON) $(srctree)/scripts/Kconfiglib/menuconfig.py $(Kconfig)

# The 'clean' target is handled by WAF ('./waf clean').
clean:
	@echo "The 'clean' command is now handled by WAF. Please run './waf clean'."

