# ***** BEGIN GPL LICENSE BLOCK *****
#
# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 2
# of the License, or (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software Foundation,
# Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
#
# ***** END GPL LICENSE BLOCK *****

ExternalProject_Add(external_breakpad
	GIT_REPOSITORY ${BREAKPAD_URI}
	DOWNLOAD_DIR ${DOWNLOAD_DIR}
	URL_HASH MD5=${BREAKPAD_MD5}
	PREFIX ${BUILD_DIR}/breakpad
	PATCH_COMMAND ${PATCH_CMD} -p 1 -d ${BUILD_DIR}/breakpad/src/external_breakpad < ${PATCH_DIR}/breakpad.diff
	UPDATE_COMMAND git -c user.name=test -c user.email=test@test.test stash create
	CONFIGURE_COMMAND ${CONFIGURE_ENV} && ${BREAKPAD_ENV} cd ${BUILD_DIR}/breakpad/src/external_breakpad/ && aclocal && automake && ${CONFIGURE_COMMAND} --prefix=${LIBDIR}/breakpad
	BUILD_COMMAND ${CONFIGURE_ENV} && cd ${BUILD_DIR}/breakpad/src/external_breakpad/ && make -j${MAKE_THREADS}
	INSTALL_COMMAND ${CONFIGURE_ENV} && cd ${BUILD_DIR}/breakpad/src/external_breakpad/ && make install
	#INSTALL_DIR ${LIBDIR}/breakpad
)

#the new file is included in the breakpad.diff now

#set(LSS_URI https://chromium.googlesource.com/linux-syscall-support)
#set(LSS_HARVEST_COMMAND
#	cd ${BUILD_DIR} && 
#	git clone ${LSS_URI} && 
#	mkdir ${BUILD_DIR}/breakpad/src/external_breakpad/src/third_party/lss &&
#	cp ${BUILD_DIR}/linux-syscall-support/linux_syscall_support.h ${BUILD_DIR}/breakpad/src/external_breakpad/src/third_party/lss/linux_syscall_support.h)

#set(GIT_ENVIRONMENT COMMAND
#	export GIT_COMMITTER_NAME="Test" && 
#	export GIT_COMMITTER_EMAIL="test@test.test")

#ExternalProject_Add_Step(external_breakpad after_patch    
#	#COMMAND ${LSS_HARVEST_COMMAND}
#	COMMAND ${GIT_ENVIRONMENT}
#	DEPENDEES patch #mkdir update patch download configure build install
#)
