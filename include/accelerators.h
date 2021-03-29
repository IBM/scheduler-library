/*
 * Copyright 2020 IBM
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

#ifndef H_ACCELERATORS_H
#define H_ACCELERATORS_H

#define DMA_WORD_PER_BEAT(_st)  (sizeof(void *) / _st)

/* static unsigned DMA_WORD_PER_BEAT(unsigned _st) */
/* { */
/*   return (sizeof(void *) / _st); */
/* } */

extern void do_task_type_initialization();
extern void do_task_type_closeout();
extern void output_task_type_run_stats();

#endif
