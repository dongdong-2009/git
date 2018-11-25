################################################################################
#
# gmp
#
################################################################################

#自定义从svn下载 固定版本
GMP_VERSION = head
GMP_SOURCE = gmp-6.1.2.tar.xz
GMP_SITE = http://192.168.154.15/svn/tools/third_party_repository/$(GMP_SOURCE)
GMP_SITE_METHOD = svn

#GMP_VERSION = 6.1.2
#GMP_SITE = $(BR2_GNU_MIRROR)/gmp
#GMP_SOURCE = gmp-$(GMP_VERSION).tar.xz
GMP_INSTALL_STAGING = YES
GMP_LICENSE = LGPL-3.0+ or GPL-2.0+
GMP_LICENSE_FILES = COPYING.LESSERv3 COPYINGv2
GMP_DEPENDENCIES = host-m4
HOST_GMP_DEPENDENCIES = host-m4

# GMP doesn't support assembly for coldfire or mips r6 ISA yet
# Disable for ARM v7m since it has different asm constraints
ifeq ($(BR2_m68k_cf)$(BR2_MIPS_CPU_MIPS32R6)$(BR2_MIPS_CPU_MIPS64R6)$(BR2_ARM_CPU_ARMV7M),y)
GMP_CONF_OPTS += --disable-assembly
endif

$(eval $(autotools-package))
$(eval $(host-autotools-package))
