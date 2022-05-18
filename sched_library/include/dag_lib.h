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

#ifndef H_DAG_LIB_INCLUDE_H
#define H_DAG_LIB_INCLUDE_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <limits.h>
#include <iostream>
#include <string>

#include "scheduler.h"

#include "verbose.h"

 //Functions to create static DAGs
extern "C" graph_wrapper_t * create_root_graph(char * graphml_filename);
extern "C" graph_wrapper_t * _create_root_graph(std::string graphml_filename, graph_wrapper_t * parent_root_graph = NULL, bool parent_graph_call = true);

extern "C" graph_wrapper_t * create_leaf_graph(char * graphml_filename);
extern "C" graph_wrapper_t * _create_leaf_graph(std::string graphml_filename, bool parent_graph_call = true);

#endif
