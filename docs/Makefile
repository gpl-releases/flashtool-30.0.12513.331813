########################################################################
#
#  GPL LICENSE SUMMARY
#
#  Copyright(c) 2011-2012 Intel Corporation. All rights reserved.
#
#  This program is free software; you can redistribute it and/or modify
#  it under the terms of version 2 of the GNU General Public License as
#  published by the Free Software Foundation.
#
#  This program is distributed in the hope that it will be useful, but
#  WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
#  General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
#  The full GNU General Public License is included in this distribution
#  in the file called LICENSE.GPL.
#
#  Contact Information:
#  intel.com
#  Intel Corporation
#  2200 Mission College Blvd.
#  Santa Clara, CA  95052
#  USA
#  (408) 765-8080
#
########################################################################

export COMPONENTTITLE = flashtool
export OUTPUT_DEST = html
export PROJECT_TITLE	= \
        "Intel(R) Flashtool API"

export DOXINCLUDE = doxygen

export DOC_SOURCE_FILES = \
    $(DOXINCLUDE)/mainpage.h \
    ../flashtool/include/common.h \
    ../flashtool/include/config.h \
    ../flashtool/include/flash_private_data.h \
    ../flashtool/include/flashapi.h \
    ../flashtool/include/partition.h 

#------------------------------------------------------------------------
# Convenience targets
#------------------------------------------------------------------------

.PHONY: all
all: doc

#------------------------------------------------------------------------
# Build targets
#------------------------------------------------------------------------

.PHONY: doc
doc: $(DOC_SOURCE_FILES)
	@mkdir -p $(OUTPUT_DEST)
	doxygen -g - | cat - $(DOXINCLUDE)/doxygen.cfg | doxygen -
	@cp $(DOXINCLUDE)/intel_1s_100.gif $(OUTPUT_DEST)

#------------------------------------------------------------------------
# Clean targets
#------------------------------------------------------------------------

.PHONY: clean
clean:
	@rm -rf $(OUTPUT_DEST) 
