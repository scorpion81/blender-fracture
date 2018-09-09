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

#include "BLI_path_util.h"
#include "BLI_utildefines.h"
#include "BKE_appdir.h"
#include "BKE_blender_version.h"
#include "creator_intern.h"
}

#include <signal.h>

using namespace google_breakpad;

namespace {

	static ExceptionHandler *eh;
	static void (*crash_handler)(int);
	static void (*abort_handler)(int);

	static void sendMinidump(const MinidumpDescriptor &descriptor);
	static bool dumpCallback(const MinidumpDescriptor& descriptor, void* context, bool succeeded)
	{
		int buttonid;
		printf("Minidump file ready to send: %s\n", descriptor.path());

		//the simplest solutions are often the best, lol... SDL and co just dont work here anymore
		buttonid = system("xmessage -buttons 'Yes':0,'No':1 -title 'Uh-Ow something unexpected has happened' "
	"'Would you like to submit information about this crash to blender.org \nto help improving future versions of blender?' ");

		if (buttonid == 0) {
			sendMinidump(descriptor);
		}

		return succeeded;
	}

	static void sendMinidump(const MinidumpDescriptor& descriptor)
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
	}

	static void handleSignals(int signum, siginfo_t* info, void *uc)
	{
		eh->HandleSignal(signum, info, uc);
		delete eh;

		//additionally try to call blenders old passed in function pointers here (old handler routines)
		if (signum == SIGSEGV) {
			crash_handler(signum);
		}

		if (signum == SIGABRT) {
			abort_handler(signum);
		}
	}

	static void startCrashHandler(void (*crash)(int), void (*abort)(int))
	{
		const int kExceptionSignals[] = {
		  SIGSEGV, SIGABRT, SIGFPE, SIGILL, SIGBUS, SIGTRAP
		};

		int size = sizeof(kExceptionSignals) / sizeof(int);

		//ExceptionHandler::WriteMinidump("/tmp", dumpCallback, NULL);
		MinidumpDescriptor descriptor("/tmp");
		eh = new ExceptionHandler(descriptor, NULL, dumpCallback, NULL, false, -1);
		crash_handler = crash;
		abort_handler = abort;

		//try to manually install handlers here (ExceptionHandlers own installation routine seems a bit brittle)
		struct sigaction handler;
		memset(&handler, 0, sizeof(handler));
		sigemptyset(&handler.sa_mask);

		handler.sa_flags = SA_ONSTACK | SA_SIGINFO;
		handler.sa_sigaction = handleSignals;

		for (int i = 0; i < size; i++)
		{
			sigaction(kExceptionSignals[i], &handler, NULL);
		}
	}
}

extern "C" 
{
	void breakpad_init(void (*crash_handler)(int), void (*abort_handler)(int))
	{
		startCrashHandler(crash_handler, abort_handler);
	}
}
