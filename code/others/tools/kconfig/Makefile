PWD = $(shell pwd)

KCONFIG_DIR ?= $(PWD)/src

KCONFIG_CONFIG ?= $(PWD)/config/.config
export KCONFIG_CONFIG

KCONFIG_AUTOHEADER ?= $(PWD)/config/my_config.h
export KCONFIG_AUTOHEADER

KCONFIG_TRISTATE ?= $(PWD)/config/tristate.conf
export KCONFIG_TRISTATE

KCONFIG_AUTOCONFIG ?= $(PWD)/config/auto.conf
export KCONFIG_AUTOCONFIG

menuconfig:
	@make -C $(KCONFIG_DIR) mconf
	@$(KCONFIG_DIR)/mconf Kconfig
	
clean:
	@make -C $(KCONFIG_DIR) clean
	@rm -rf $(PWD)/config/.config
	
