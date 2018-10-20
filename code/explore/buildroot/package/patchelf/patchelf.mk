################################################################################
#
# patchelf
#
################################################################################

#自定义从svn下载
PATCHELF_VERSION = head
PATCHELF_SOURCE = patchelf-0.9.tar.bz2
PATCHELF_SITE = http://192.168.154.15/svn/tools/third_party_repository/$(PATCHELF_SOURCE)
PATCHELF_SITE_METHOD = svn

#PATCHELF_VERSION = 0.9
#PATCHELF_SITE = http://releases.nixos.org/patchelf/patchelf-$(PATCHELF_VERSION)
#PATCHELF_SOURCE = patchelf-$(PATCHELF_VERSION).tar.bz2
PATCHELF_LICENSE = GPL-3.0+
PATCHELF_LICENSE_FILES = COPYING

$(eval $(host-autotools-package))
