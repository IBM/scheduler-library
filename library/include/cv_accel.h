/*
 * Copyright 2019 IBM
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

#ifndef H_CV_ACCEL_INCLUDE_H
#define H_CV_ACCEL_INCLUDE_H

#include <stdint.h>
#include <pthread.h>
#include <sys/time.h>

void do_cv_accel_type_initialization(scheduler_datastate_block_t* sptr);
void do_cv_accel_type_closeout(scheduler_datastate_block_t* sptr);
void output_cv_accel_type_run_stats(scheduler_datastate_block_t* sptr, unsigned my_accel_id, unsigned total_task_types);

#endif
