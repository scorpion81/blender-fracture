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

#include "util_task.h"

#include "BLI_task.h"
#include "BLI_threads.h"

namespace PTC {

class UtilTask {
public:
	explicit UtilTask(const UtilTaskFunction& run) : run_(run) {}
	void run()
	{
		run_();
	}
protected:
	UtilTaskFunction run_;
};

static void task_function(TaskPool * /*pool*/,
                          void *taskdata,
                          int /*threadid*/)
{
	UtilTask *task = reinterpret_cast<UtilTask*>(taskdata);
	task->run();
	delete task;
}

UtilTaskPool::UtilTaskPool()
{
	scheduler_ = BLI_task_scheduler_get();
	pool_ = BLI_task_pool_create(scheduler_, NULL);
}

UtilTaskPool::~UtilTaskPool()
{
	BLI_task_pool_free(pool_);
}

void UtilTaskPool::push(const UtilTaskFunction& run, bool front)
{
	UtilTask *task = new UtilTask(run);
	BLI_task_pool_push(pool_,
	                   task_function,
	                   task,
	                   false,
	                   front? TASK_PRIORITY_HIGH: TASK_PRIORITY_LOW);
}

void UtilTaskPool::wait_work()
{
	BLI_task_pool_work_and_wait(pool_);
}

void UtilTaskPool::cancel()
{
	BLI_task_pool_cancel(pool_);
}

void UtilTaskPool::stop()
{
	BLI_task_pool_stop(pool_);
}

bool UtilTaskPool::cancelled()
{
	return BLI_task_pool_canceled(pool_);
}

}  /* namespace PTC */
