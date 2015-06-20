/*
 * Copyright 2015, Blender Foundation.
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

#ifndef PTC_UTIL_THREAD_H
#define PTC_UTIL_THREAD_H

#include "BLI_threads.h"

namespace PTC {

class thread_mutex {
public:
	thread_mutex()
	{
		BLI_mutex_init(&mutex_);
	}

	~thread_mutex()
	{
		BLI_mutex_end(&mutex_);
	}

	void lock()
	{
		BLI_mutex_lock(&mutex_);
	}

	bool trylock()
	{
		return BLI_mutex_trylock(&mutex_);
	}

	void unlock()
	{
		BLI_mutex_unlock(&mutex_);
	}

protected:
	ThreadMutex mutex_;
};

class thread_scoped_lock {
public:
	explicit thread_scoped_lock(thread_mutex& mutex)
	  : mutex_(mutex)
	{
		mutex_.lock();
	}

	~thread_scoped_lock() {
		mutex_.unlock();
	}
protected:
	thread_mutex& mutex_;
};

}  /* namespace PTC */

#endif  /* PTC_UTIL_THREAD_H */
