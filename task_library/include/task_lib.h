/*
 * Copyright 2021 IBM
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef H_TASK_LIB_INCLUDE_H
#define H_TASK_LIB_INCLUDE_H

#include <stdint.h>

#include "base_task_types.h"
#include "scheduler.h"

extern "C" void initialize_task_lib();

extern std::map<size_t, uint64_t[SCHED_MAX_ACCEL_TYPES]> cpu_profile;

#endif
