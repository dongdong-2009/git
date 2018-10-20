################################################################################
#
# mpfr
#
################################################################################

#自定义从svn下载
MPFR_VERSION = head
MPFR_SOURCE = mpfr-3.1.6.tar.xz
MPFR_SITE = http://192.168.154.15/svn/tools/third_party_repository/$(MPFR_SOURCE)
MPFR_SITE_METHOD = svn

#MPFR_VERSION = 3.1.6
#MPFR_SITE = http://www.mpfr.org/mpfr-$(MPFR_VERSION)
#MPFR_SOURCE = mpfr-$(MPFR_VERSION).tar.xz
MPFR_LICENSE = LGPL-3.0+
MPFR_LICENSE_FILES = COPYING.LESSER
MPFR_INSTALL_STAGING = YES
MPFR_DEPENDENCIES = gmp
HOST_MPFR_DEPENDENCIES = host-gmp
MPFR_MAKE_OPTS = RANLIB=$(TARGET_RANLIB)

$(eval $(autotools-package))
$(eval $(host-autotools-package))
