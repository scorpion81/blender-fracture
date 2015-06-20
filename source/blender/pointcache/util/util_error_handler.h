/*
 * Copyright 2013, Blender Foundation.
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
 */

#ifndef PTC_UTIL_ERROR_HANDLER_H
#define PTC_UTIL_ERROR_HANDLER_H

#include <stdio.h>

#ifdef WITH_ALEMBIC
#include <Alembic/Abc/ErrorHandler.h>
#endif

extern "C" {
#include "BLI_utildefines.h"
#include "BLI_string.h"
}

#include "util_types.h"

struct ModifierData;
struct ReportList;

namespace PTC {

class ErrorHandler
{
public:
	ErrorHandler();
	virtual ~ErrorHandler();
	
	virtual void handle(PTCErrorLevel level, const char *message) = 0;
	void set_error_level(PTCErrorLevel level);
	PTCErrorLevel max_error_level() const { return m_max_level; }
	
	static ErrorHandler *get_default_handler() { return m_default_handler; }
	static void set_default_handler(ErrorHandler *handler);
	static void clear_default_handler();
	
private:
	PTCErrorLevel m_max_level;
	
	static ErrorHandler *m_default_handler;
};


class StdErrorHandler : public ErrorHandler
{
public:
	StdErrorHandler(PTCErrorLevel level);
	
	void handle(PTCErrorLevel level, const char *message);
	
	PTCErrorLevel get_verbosity() const { return m_verbosity; }
	void set_verbosity(PTCErrorLevel level) { m_verbosity = level; }
	
private:
	PTCErrorLevel m_verbosity;
};


/* Use Blender reports system to log Alembic errors */
class CallbackErrorHandler : public ErrorHandler
{
public:
	CallbackErrorHandler(PTCErrorCallback cb, void *userdata);
	
	void handle(PTCErrorLevel level, const char *message);
	
private:
	PTCErrorCallback m_callback;
	void *m_userdata;
};


class ModifierErrorHandler : public ErrorHandler
{
public:
	ModifierErrorHandler(ModifierData *md);
	
	void handle(PTCErrorLevel level, const char *message);
	
private:
	ModifierData *m_modifier;
};

/* -------------------------------- */

#ifdef WITH_ALEMBIC

/* XXX With current Alembic version 1.5 we only get a combined error message.
 * This function try to extract some more information and return a nicer message format.
 */
BLI_INLINE void split_alembic_error_message(const char *msg, const char **origin, const char **base_msg)
{
	const char delim[] = {'\n', '\0'};
	char *sep, *suffix;
	
	BLI_str_partition(msg, delim, &sep, &suffix);
	if (suffix) {
		*origin = msg;
		BLI_str_partition(suffix, delim, &sep, &suffix);
		if (suffix) {
			*base_msg = suffix;
		}
		else {
			*base_msg = msg;
		}
	}
	else {
		*origin = *base_msg = msg;
	}
}

/* wrapper templates so the exception macro can be used with references as well as pointers */

template <typename T>
void handle_alembic_exception(T &handler, PTCErrorLevel level, const Alembic::Util::Exception &e)
{
	const char *origin, *msg;
	split_alembic_error_message(e.what(), &origin, &msg);
	
	handler.set_error_level(level);
	handler.handle(level, msg);
}

template <typename T>
void handle_alembic_exception(T *handler, PTCErrorLevel level, const Alembic::Util::Exception &e)
{
	static StdErrorHandler default_handler(PTC_ERROR_WARNING);
	if (!handler)
		handler = &default_handler;
	
	const char *origin, *msg;
	split_alembic_error_message(e.what(), &origin, &msg);
	
	handler->set_error_level(level);
	handler->handle(level, msg);
}

#endif

/* -------------------------------- */

/* macros for convenient exception handling */

#define PTC_SAFE_CALL_BEGIN \
	try {

#ifdef WITH_ALEMBIC
#define PTC_SAFE_CALL_END_HANDLER(handler) \
	} \
	catch (Alembic::Util::Exception e) { \
		handle_alembic_exception((handler), PTC_ERROR_CRITICAL, e); \
	}
#else
#define PTC_SAFE_CALL_END_HANDLER(handler) \
	}
#endif

#ifdef WITH_ALEMBIC
#define PTC_SAFE_CALL_END_HANDLER_LEVEL(handler, level) \
	} \
	catch (Alembic::Util::Exception e) { \
		handle_alembic_exception((handler), (level), e); \
	}
#else
#define PTC_SAFE_CALL_END_HANDLER_LEVEL(handler, level) \
	}
#endif

#ifdef WITH_ALEMBIC
#define PTC_SAFE_CALL_END \
	} \
	catch (Alembic::Util::Exception e) { \
		handle_alembic_exception(ErrorHandler::get_default_handler(), PTC_ERROR_CRITICAL, e); \
	}
#else
#define PTC_SAFE_CALL_END \
	}
#endif

/* -------------------------------- */

} /* namespace PTC */

#endif  /* PTC_UTIL_ERROR_HANDLER_H */
