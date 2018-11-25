################################################################################
#
# m4
#
################################################################################

#自定义从svn下载
M4_VERSION = head
M4_SOURCE = m4-1.4.18.tar.xz
M4_SITE = http://192.168.154.15/svn/tools/third_party_repository/$(M4_SOURCE)
M4_SITE_METHOD = svn

#M4_VERSION = 1.4.18
#M4_SOURCE = m4-$(M4_VERSION).tar.xz
#M4_SITE = $(BR2_GNU_MIRROR)/m4
M4_LICENSE = GPL-3.0+
M4_LICENSE_FILES = COPYING
HOST_M4_CONF_OPTS = --disable-static

$(eval $(host-autotools-package))
