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


void print_cv_metadata_block_contents(task_metadata_block_t* mb);

void do_cv_task_type_initialization();
void do_cv_task_type_closeout();
void output_cv_task_type_run_stats();

void execute_hwr_cv_accelerator(task_metadata_block_t* task_metadata_block);
void execute_cpu_cv_accelerator(task_metadata_block_t* task_metadata_block);

#endif
