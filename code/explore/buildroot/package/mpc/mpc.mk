################################################################################
#
# mpc
#
################################################################################

#自定义从svn下载
MPC_VERSION = head
MPC_SOURCE = mpc-1.0.3.tar.gz
MPC_SITE = http://192.168.154.15/svn/tools/third_party_repository/$(MPC_SOURCE)
MPC_SITE_METHOD = svn

#MPC_VERSION = 1.0.3
#MPC_SITE = $(BR2_GNU_MIRROR)/mpc
MPC_LICENSE = LGPL-3.0+
MPC_LICENSE_FILES = COPYING.LESSER
MPC_INSTALL_STAGING = YES
MPC_DEPENDENCIES = gmp mpfr
HOST_MPC_DEPENDENCIES = host-gmp host-mpfr

$(eval $(autotools-package))
$(eval $(host-autotools-package))
