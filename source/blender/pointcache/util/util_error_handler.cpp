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

#include <iostream>

#include "util_error_handler.h"

extern "C" {
#include "BKE_modifier.h"
}

namespace PTC {

ErrorHandler *ErrorHandler::m_default_handler = new StdErrorHandler(PTC_ERROR_INFO);

ErrorHandler::ErrorHandler() :
    m_max_level(PTC_ERROR_NONE)
{
}

ErrorHandler::~ErrorHandler()
{
}

void ErrorHandler::set_error_level(PTCErrorLevel level)
{
	if (level > m_max_level)
		m_max_level = level;
}

void ErrorHandler::set_default_handler(ErrorHandler *handler)
{
	if (m_default_handler)
		delete m_default_handler;
	
	if (handler)
		m_default_handler = handler;
	else
		m_default_handler = new StdErrorHandler(PTC_ERROR_INFO);
}

void ErrorHandler::clear_default_handler()
{
	if (m_default_handler)
		delete m_default_handler;
	
	m_default_handler = new StdErrorHandler(PTC_ERROR_INFO);
}


StdErrorHandler::StdErrorHandler(PTCErrorLevel level) :
    m_verbosity(level)
{
}

void StdErrorHandler::handle(PTCErrorLevel level, const char *message)
{
	/* ignore levels below the verbosity setting */
	if (level >= m_verbosity) {
		std::cerr << message << std::endl;
	}
}


CallbackErrorHandler::CallbackErrorHandler(PTCErrorCallback cb, void *userdata) :
    m_callback(cb),
    m_userdata(userdata)
{
}

void CallbackErrorHandler::handle(PTCErrorLevel level, const char *message)
{
	m_callback(m_userdata, level, message);
}


ModifierErrorHandler::ModifierErrorHandler(ModifierData *md) :
    m_modifier(md)
{
}

void ModifierErrorHandler::handle(PTCErrorLevel UNUSED(level), const char *message)
{
	modifier_setError(m_modifier, "%s", message);
}

} /* namespace PTC */
