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

#include "client/crashpad_client.h"
#include "client/settings.h"
#include "base/files/file_path.h"
#include "client/crash_report_database.h"
extern "C"
{
#include "BLI_path_util.h"
#include "BLI_utildefines.h"
#include "BKE_appdir.h"
#include "BKE_blender_version.h"
#include "creator_intern.h"
}

#include <WinBase.h>

using namespace std;

using namespace crashpad;
#if _MSC_VER
static std::wstring StringToWString(const std::string& s)
{
	std::wstring temp(s.length(), L' ');
	std::copy(s.begin(), s.end(), temp.begin());
	return temp;
}
#endif

static CrashpadClient client;
static bool startCrashHandler()
{
	  bool rc;
	  std::map<std::string, std::string> annotations;
	  std::vector<std::string> arguments;
	  char handler_path_cstr[MAX_PATH] = { 0 };
	  char blender_version[MAX_PATH] = { 0 };


	  std::string db_path_ascii = BKE_appdir_folder_id_create(BLENDER_USER_DATAFILES, "crashpad");
	  std::wstring db_path(StringToWString(db_path_ascii));

	  BLI_path_append(handler_path_cstr, sizeof(handler_path_cstr), BKE_appdir_program_dir());
	  BLI_path_append(handler_path_cstr, sizeof(handler_path_cstr), "crashpad_handler.exe");
	  std::string handler_str(handler_path_cstr);
	  std::wstring handler_path(StringToWString(handler_str));
	  printf("x");

	  std::string url(CRASHPAD_URL);
	  arguments.push_back("--no-upload-gzip");
	  arguments.push_back("--no-rate-limit");

	  sprintf(blender_version, BLEND_VERSION_STRING_FMT);
	  annotations["build_version"] = blender_version;
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

	  base::FilePath db(db_path);
	  base::FilePath handler(handler_path);

	  std::unique_ptr<CrashReportDatabase> database =
		  crashpad::CrashReportDatabase::Initialize(db);
	  if (database == nullptr || database->GetSettings() == NULL)
		  return false;

	  /* Enable automated uploads. however the handler will only be called when */
	  /* the user consents to it, this way no crashdumps will be made if they */ 
	  /* decline, and we can still have some custom UI */
	  database->GetSettings()->SetUploadsEnabled(true);

	  rc = client.StartHandler(handler,
		  db,
		  db,
		  url,
		  annotations,
		  arguments,
		  true,
		  true);
	  if (rc == false)
		  return false;

	  /* Optional, wait for Crashpad to initialize. */
	  rc = client.WaitForHandlerStart(INFINITE);
	  if (rc == false)
		  return false;
	  
	  return true;
  }


 extern "C" 
 {
	 void crashpad_init()
	 {
		 startCrashHandler();
	 }

	 void crashpad_activate(void *ExceptionInfo)
	 {
		 if (MessageBoxA(NULL,
			 "Would you like to submit information about this crash to blender.org\n"
			 "to help improving future versions of blender?",
			 "Uh-Ow something unexpected has happened",
			 MB_YESNO | MB_ICONERROR) == IDYES) {
			 client.DumpAndCrash((EXCEPTION_POINTERS*)ExceptionInfo);
		 }
	 }
 }
 
