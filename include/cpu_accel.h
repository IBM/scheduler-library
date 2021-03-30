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

#ifndef H_CPU_ACCEL_INCLUDE_H
#define H_CPU_ACCEL_INCLUDE_H


void init_cpu_parameters(unsigned n, uint32_t log_nsamples);

void do_cpu_accel_type_initialization();
void do_cpu_accel_type_closeout();
void output_cpu_accel_type_run_stats();

#endif
