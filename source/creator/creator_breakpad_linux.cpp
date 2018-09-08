/*
 * ***** BEGIN GPL LICENSE BLOCK *****
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * ***** END GPL LICENSE BLOCK *****
 */

/** \file creator/creator_crashpad.cpp
 *  \ingroup creator
 */

#include <map>
#include <string>
#include <vector>

#include "client/linux/handler/exception_handler.h"
#include "common/linux/http_upload.h"
extern "C"
{

//old dummy shit for game engine... taken from creator.c

/* for passing information between creator and gameengine */
#ifdef WITH_GAMEENGINE
#  include "BL_System.h"
#else /* dummy */
#  define SYS_SystemHandle int
#endif

#include "BLI_path_util.h"
#include "BLI_utildefines.h"
#include "BKE_appdir.h"
#include "BKE_blender_version.h"
#include "creator_intern.h"
}

using namespace google_breakpad;

namespace {

	static bool dumpCallback(const MinidumpDescriptor& descriptor, void* context, bool succeeded)
	{
		  std::map<std::string, std::string> files;
		  std::map<std::string, std::string> annotations;
		  std::string url(CRASHPAD_URL);
		  char blender_version[PATH_MAX] = { 0 };

		  printf("Minidump file: %s\n", descriptor.path());

		  files["upload_file_minidump"] = descriptor.path();
		  
		  sprintf(blender_version, BLEND_VERSION_STRING_FMT);
		  annotations["product"] = "blender"; //prod
		  annotations["build_version"] = blender_version; //ver

	#ifdef BUILD_DATE
		  annotations["build_version_char"] = STRINGIFY(BLENDER_VERSION_CHAR);
		  annotations["build_cycle"] = STRINGIFY(BLENDER_VERSION_CYCLE);
		  annotations["build_date"] = build_date;
		  annotations["build_time"] = build_time;
		  annotations["build_hash"] = build_hash;
		  annotations["build_commit_date"] = build_commit_date;
		  annotations["build_commit_time"] = build_commit_time;
		  annotations["build_branch"] = build_branch;
		  annotations["build_platform"] = build_platform;
		  annotations["build_type"] = build_type;
		  annotations["build_cflags"] = build_cflags;
		  annotations["build_cxxflags"] = build_cxxflags;
		  annotations["build_linkflags"] = build_linkflags;
		  annotations["build_system"] = build_system;
	#endif

		  // Send it
		  string response, error;
		  bool success = HTTPUpload::SendRequest(url,
				                                 annotations,
				                                 files,
				                                 "", //options->proxy,
				                                 "", // options->proxy_user_pwd,
				                                 "",
				                                 &response,
				                                 NULL,
				                                 &error);
		  if (success) {
			printf("Successfully sent the minidump file.\n");
		  } else {
			printf("Failed to send minidump: %s\n", error.c_str());
		  }
		  printf("Response:\n");
		  printf("%s\n", response.c_str());

		  return succeeded;
	}

	static void startCrashHandler()
	{
#if 0
		 MinidumpDescriptor descriptor("/tmp");
	 	 ExceptionHandler eh(descriptor, NULL, dumpCallback, NULL, true, -1);
		 printf("Exception Handler registered!\n");

		 //int* whoops = (int*)0;
		 //*whoops = 41;	
#endif
		ExceptionHandler::WriteMinidump("/tmp", dumpCallback, NULL);
	}
}

extern "C" 
{
	void breakpad_write()
	{
		startCrashHandler();
	}
}

