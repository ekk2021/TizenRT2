############################################################################
#
#Copyright 2018 Samsung Electronics All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
############################################################################
############################################################################
# arch/xtensa/src/Makefile
#
#   Copyright (C) 2016 Gregory Nutt. All rights reserved.
#   Author: Gregory Nutt <gnutt@nuttx.org>
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name NuttX nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

-include $(TOPDIR)/Make.defs
-include chip/Make.defs

ifeq ($(CONFIG_ARCH_XTENSA),y) 
ARCH_SUBDIR = xtensa
endif

ifeq ($(CONFIG_ARCH_FAMILY_LX6),y)
ARCH_SUBDIR_LX6 = lx6
endif

CPPFLAGS += $(EXTRADEFINES)
CFLAGS += $(EXTRADEFINES)
CXXFLAGS += $(EXTRADEFINES)

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
  ARCH_SRCDIR = $(TOPDIR)\arch\$(CONFIG_ARCH)\src
  TINYARA = "$(TOPDIR)\$(OUTBIN_DIR)\tinyara$(EXEEXT)"
  CFLAGS += -I$(ARCH_SRCDIR)\chip
  CFLAGS += -I$(ARCH_SRCDIR)\$(ARCH_SUBDIR)
  CFLAGS += -I$(TOPDIR)\sched
else
  ARCH_SRCDIR = $(TOPDIR)/arch/$(CONFIG_ARCH)/src
ifeq ($(WINTOOL),y)
  TINYARA = "${shell cygpath -w $(TOPDIR)/$(OUTBIN_DIR)/tinyara$(EXEEXT)}"
  CFLAGS += -I "${shell cygpath -w $(ARCH_SRCDIR)/chip}"
  CFLAGS += -I "${shell cygpath -w $(ARCH_SRCDIR)/$(ARCH_SUBDIR)}"
  CFLAGS += -I "${shell cygpath -w $(TOPDIR)/sched}"
else
  TINYARA = "$(TOPDIR)/$(OUTBIN_DIR)/tinyara$(EXEEXT)"
  CFLAGS += -I$(ARCH_SRCDIR)/chip
  CFLAGS += -I$(ARCH_SRCDIR)/$(ARCH_SUBDIR)
  CFLAGS += -I$(ARCH_SRCDIR)/$(ARCH_SUBDIR_LX6)
  CFLAGS += -I$(TOPDIR)/kernel
  #CFLAGS += -I$(TOPDIR)/kernel/sched
endif
endif

HEAD_AOBJ = $(HEAD_ASRC:.S=$(OBJEXT))
HEAD_COBJ = $(HEAD_CSRC:.c=$(OBJEXT))
STARTUP_OBJS ?= $(HEAD_AOBJ) $(HEAD_COBJ)

ASRCS = $(CHIP_ASRCS) $(CMN_ASRCS)
AOBJS = $(ASRCS:.S=$(OBJEXT))

CSRCS = $(CHIP_CSRCS) $(CMN_CSRCS)
COBJS = $(CSRCS:.c=$(OBJEXT))

SRCS = $(ASRCS) $(CSRCS)
OBJS = $(AOBJS) $(COBJS)

# Override in Make.defs if linker is not 'ld'

LDSTARTGROUP ?= --start-group
LDENDGROUP ?= --end-group

LDFLAGS += $(ARCHSCRIPT)
EXTRA_LIBS ?=
EXTRA_LIBPATHS ?=
LINKLIBS ?=

ifeq ($(CONFIG_WINDOWS_NATIVE),y)
  BOARDMAKE = $(if $(wildcard .\board\Makefile),y,)
  LIBPATHS += -L"$(TOPDIR)\$(LIBRARIES_DIR)"
ifeq ($(BOARDMAKE),y)
  LIBPATHS += -L"$(TOPDIR)\arch\$(CONFIG_ARCH)\src\board"
endif

else
  BOARDMAKE = $(if $(wildcard ./board/Makefile),y,)

ifeq ($(WINTOOL),y)
  LIBPATHS += -L"${shell cygpath -w "$(TOPDIR)/$(LIBRARIES_DIR)"}"
ifeq ($(BOARDMAKE),y)
  LIBPATHS += -L"${shell cygpath -w "$(TOPDIR)/arch/$(CONFIG_ARCH)/src/board"}"
endif

else
  LIBPATHS += -L"$(TOPDIR)/$(LIBRARIES_DIR)"
ifeq ($(BOARDMAKE),y)
  LIBPATHS += -L"$(TOPDIR)/arch/$(CONFIG_ARCH)/src/board"
endif
endif
endif

LDLIBS = $(patsubst %.a,%,$(patsubst lib%,-l%,$(LINKLIBS)))
ifeq ($(BOARDMAKE),y)
  LDLIBS += -lboard
endif

LIBGCC = "${shell "$(CC)" $(ARCHCPUFLAGS) -print-libgcc-file-name}"
GCC_LIBDIR := ${shell dirname $(LIBGCC)}

VPATH += chip
VPATH += $(ARCH_SUBDIR)

all: $(STARTUP_OBJS) libarch$(LIBEXT) install

.PHONY: board/libboard$(LIBEXT)

$(AOBJS) $(HEAD_AOBJ): %$(OBJEXT): %.S
	$(call ASSEMBLE, $<, $@)

$(COBJS) $(HEAD_COBJ): %$(OBJEXT): %.c
	$(call COMPILE, $<, $@)

libarch$(LIBEXT): $(OBJS)
	$(call ARCHIVE, $@, $(OBJS))

board/libboard$(LIBEXT):
	$(Q) $(MAKE) -C board TOPDIR="$(TOPDIR)" all EXTRADEFINES=$(EXTRADEFINES)

$(OUTBIN_DIR)/tinyara$(EXEEXT): $(STARTUP_OBJS) board/libboard$(LIBEXT)
	$(Q) echo "LD: tinyara"
	$(Q) $(LD) --entry=__start $(LDFLAGS) $(LIBPATHS) $(EXTRA_LIBPATHS) \
		-o $(TINYARA) $(STARTUP_OBJS) $(EXTRA_OBJS) \
		--start-group $(LDLIBS) $(EXTRA_LIBS) $(LIBGCC) --end-group -Map $(TOPDIR)/../build/output/bin/tinyara.map
ifneq ($(CONFIG_WINDOWS_NATIVE),y)
	$(Q) $(NM) $(TINYARA) | \
	grep -v '\(compiled\)\|\(\$(OBJEXT)$$\)\|\( [aUw] \)\|\(\.\.ng$$\)\|\(LASH[RL]DI\)' | \
	sort > $(TOPDIR)/$(OUTBIN_DIR)/System.map
endif

install:
# External esp32 wifi static Lib builds
ifeq ($(CONFIG_ESP32_WIFI_SUPPORT),y)
	$(Q) cp $(EXTDIR)/esp_idf_port/esp32/lib/libcoexist$(LIBEXT) $(LIBRARIES_DIR)/libcoexist$(LIBEXT)
	$(Q) cp $(EXTDIR)/esp_idf_port/esp32/lib/libcore$(LIBEXT) $(LIBRARIES_DIR)/libcore$(LIBEXT)
	$(Q) cp $(EXTDIR)/esp_idf_port/esp32/lib/libespnow$(LIBEXT) $(LIBRARIES_DIR)/libespnow$(LIBEXT)
	$(Q) cp $(EXTDIR)/esp_idf_port/esp32/lib/libmesh$(LIBEXT) $(LIBRARIES_DIR)/libmesh$(LIBEXT)
	$(Q) cp $(EXTDIR)/esp_idf_port/esp32/lib/libnet80211$(LIBEXT) $(LIBRARIES_DIR)/libnet80211$(LIBEXT)
	$(Q) cp $(EXTDIR)/esp_idf_port/esp32/lib/libphy$(LIBEXT) $(LIBRARIES_DIR)/libphy$(LIBEXT)
	$(Q) cp $(EXTDIR)/esp_idf_port/esp32/lib/libpp$(LIBEXT) $(LIBRARIES_DIR)/libpp$(LIBEXT)
	$(Q) cp $(EXTDIR)/esp_idf_port/esp32/lib/librtc$(LIBEXT) $(LIBRARIES_DIR)/librtc$(LIBEXT)
	$(Q) cp $(EXTDIR)/esp_idf_port/esp32/lib/libsmartconfig$(LIBEXT) $(LIBRARIES_DIR)/libsmartconfig$(LIBEXT)
	$(Q) cp $(EXTDIR)/esp_idf_port/esp32/lib/libwpa2$(LIBEXT) $(LIBRARIES_DIR)/libwpa2$(LIBEXT)
	$(Q) cp $(EXTDIR)/esp_idf_port/esp32/lib/libwpa$(LIBEXT) $(LIBRARIES_DIR)/libwpa$(LIBEXT)
	$(Q) cp $(EXTDIR)/esp_idf_port/esp32/lib/libwps$(LIBEXT) $(LIBRARIES_DIR)/libwps$(LIBEXT)
endif

# This is part of the top-level export target

export_startup: board/libboard$(LIBEXT) $(STARTUP_OBJS)
	$(Q) if [ -d "$(EXPORT_DIR)/startup" ]; then \
		cp -f $(STARTUP_OBJS) "$(EXPORT_DIR)/startup"; \
	 else \
		echo "$(EXPORT_DIR)/startup does not exist"; \
		exit 1; \
	 fi

# Dependencies

.depend: Makefile chip/Make.defs $(SRCS)
ifeq ($(BOARDMAKE),y)
	$(Q) $(MAKE) -C board TOPDIR="$(TOPDIR)" depend
endif
	$(Q) $(MKDEP) --dep-path chip --dep-path chip/spi_flash --dep-path $(ARCH_SUBDIR) \
	 "$(CC)" -- $(CFLAGS) -- $(SRCS) >Make.dep
	$(Q) touch $@

depend: .depend

clean:
ifeq ($(BOARDMAKE),y)
	$(Q) $(MAKE) -C board TOPDIR="$(TOPDIR)" clean
endif
	$(call DELFILE, libarch$(LIBEXT))
	$(call CLEAN)

distclean: clean
ifeq ($(BOARDMAKE),y)
	$(Q) $(MAKE) -C board TOPDIR="$(TOPDIR)" distclean
endif
	$(call DELFILE, Make.dep)
	$(call DELFILE, .depend)

-include Make.dep
