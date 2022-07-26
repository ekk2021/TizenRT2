#!/bin/bash
###########################################################################
#
# Copyright 2017 Samsung Electronics All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing,
# software distributed under the License is distributed on an
# "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
# either express or implied. See the License for the specific
# language governing permissions and limitations under the License.
#
###########################################################################
# apps/tools/mkkconfig.sh
#
#   Copyright (C) 2015 Gregory Nutt. All rights reserved.
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

# Get the input parameter list

USAGE () {
  echo "USAGE: mkkconfig.sh [-d] [-h] [-m <menu>] [-o <kconfig-file>] [-mindepth <mindepth>] [-maxdepth <maxdepth>]"
  echo "Where:"
  echo " <-d>:"
  echo "   Enables debug output"
  echo " -m <menu>:"
  echo "   Menu description"
  echo " -o <kconfig-file>:"
  echo "   Identifies the specific configuratin for the selected <board-name>."
  echo "   This must correspond to a sub-directory under the board directory at"
  echo "   under build/configs/<board-name>/."
  echo " -mindepth <mindepth>:"
  echo "   Set the minimum depth to find configurations in sub-directory"
  echo " -maxdepth <maxdepth>:"
  echo "   Set the maximum depth to find configurations in sub-directory"
  echo " <-h>:"
  echo "   Prints this message and exits."
}

KCONFIG=Kconfig
unset MENU
DEPTH_MIN=2
DEPTH_MAX=2

while [ ! -z "$1" ]; do
  case $1 in
    -d )
      set -x
      ;;
    -m )
      shift
      MENU=$1
      ;;
    -o )
      shift
      KCONFIG=$1
      ;;
    -h )
      USAGE
      exit 0
      ;;
    -mindepth )
      shift
      DEPTH_MIN=$1
      ;;
    -maxdepth )
      shift
      DEPTH_MAX=$1
      ;;
    * )
      echo "ERROR: Unrecognized argument: $1"
      USAGE
      exit 1
      ;;
    esac
  shift
done

if [ ${DEPTH_MIN} -gt ${DEPTH_MAX} ]; then
  { echo "ERROR: Invalid depths, min ${DEPTH_MIN}, max ${DEPTH_MAX}"; exit 1; }
fi

if [ -f ${KCONFIG} ]; then
  rm ${KCONFIG} || { echo "ERROR: Failed to remove $PWD/${KCONFIG}"; exit 1; }
fi

echo mkkconfig in $PWD

KCONFIG_LIST=`find $PWD -mindepth ${DEPTH_MIN} -maxdepth ${DEPTH_MAX} -name ${KCONFIG} | sort -n`

echo "#" > ${KCONFIG}
echo "# For a description of the syntax of this configuration file," >> ${KCONFIG}
echo "# see kconfig-language at https://www.kernel.org/doc/Documentation/kbuild/kconfig-language.txt" >> ${KCONFIG}
echo "#" >> ${KCONFIG}
echo "# This file is autogenerated, do not edit." >> ${KCONFIG}
echo "#" >> ${KCONFIG}
echo "" >> ${KCONFIG}

if [ ! -z "${MENU}" ]; then
  echo "menu \"${MENU}\"" >> ${KCONFIG}
fi

for FILE in ${KCONFIG_LIST}; do
  echo "source \"${FILE}\"" >> ${KCONFIG}
done

if [ ! -z "${MENU}" ]; then
  echo "endmenu # ${MENU}" >> ${KCONFIG}
fi

