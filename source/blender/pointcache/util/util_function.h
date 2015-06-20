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

#ifndef PTC_UTIL_FUNCTION
#define PTC_UTIL_FUNCTION

#if __cplusplus > 199711L

#include <functional>

namespace PTC {

using std::function;
using namespace std::placeholders;
#define function_bind std::bind

}  /* namespace PTC */

#else

#include <boost/bind.hpp>
#include <boost/function.hpp>

namespace PTC {

using boost::function;
#define function_bind boost::bind

}  /* namespace PTC */

#endif

#endif  /* PTC_UTIL_FUNCTION */
