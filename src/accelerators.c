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

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>

#include <fcntl.h>
#include <pthread.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>

#include "utils.h"
//#define VERBOSE
#include "verbose.h"

#ifdef COMPILE_TO_ESP
#include "contig.h"
#include "mini-era.h"
#endif

#include "scheduler.h"

#include "fft_accel.h"
#include "vit_accel.h"
#include "cv_accel.h"
#include "accelerators.h" // include AFTER scheduler.h -- needs types form scheduler.h

/*static unsigned DMA_WORD_PER_BEAT(unsigned _st)
  {
  return (sizeof(void *) / _st);
  }*/


void
do_task_type_initialization()
{
  do_fft_task_type_initialization();

  do_vit_task_type_initialization();

  do_cv_task_type_initialization();
}


void
do_task_type_closeout()
{
  // Clean up any hardware accelerator stuff
  do_fft_task_type_closeout();
  do_vit_task_type_closeout();
  do_cv_task_type_closeout();
}


void
output_task_type_run_stats()
{
  printf("\nPer-MetaData-Block Job Timing Data:\n");
  output_fft_task_type_run_stats();
  output_vit_task_type_run_stats();
  output_cv_task_type_run_stats();
}

