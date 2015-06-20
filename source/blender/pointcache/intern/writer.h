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

#ifndef PTC_WRITER_H
#define PTC_WRITER_H

#include <string>

#include "util_error_handler.h"

struct ID;

namespace PTC {

class WriterArchive {
public:
	virtual ~WriterArchive() {}
	
	virtual void use_render(bool enable) = 0;
};

class Writer {
public:
	Writer();
	Writer(ErrorHandler *handler);
	virtual ~Writer();
	
	void set_error_handler(ErrorHandler *handler);
	bool valid() const;
	
	virtual void init(WriterArchive *archive) = 0;
	
	/* create references to other objects */
	virtual void create_refs() {}
	
	virtual void write_sample() = 0;
	
protected:
	ErrorHandler *m_error_handler;
};

} /* namespace PTC */

#endif  /* PTC_WRITER_H */
