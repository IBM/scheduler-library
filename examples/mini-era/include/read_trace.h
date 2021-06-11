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

#ifndef H_READ_TRACE_H
#define H_READ_TRACE_H
#ifndef HPVM
#include "plan_ctrl_task.h"
#else
#include "base_types.h"
#endif


/* File pointer to the input trace */
extern FILE *input_trace;


status_t init_trace_reader(char* trace_filename);
bool read_next_trace_record(vehicle_state_t vs);
bool eof_trace_reader();
void closeout_trace_reader();

#endif
