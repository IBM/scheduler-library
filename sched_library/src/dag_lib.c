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

#include "dag_lib.h"

extern "C" {
    Graph * create_graph(char * graphml_filename) {
        //Create empty DFG
        Graph * dfg_ptr = new(Graph);

        //Open graphml file
        std::ifstream inFile;
        inFile.open(graphml_filename, std::ifstream::in);

        //Read Graphml into static DFG graph
        boost::dynamic_properties dp;
        dp.property("task_type", boost::get(&dag_vertex_t::task_type, *dfg_ptr));
        dp.property("vertex_id", boost::get(&dag_vertex_t::vertex_id, *dfg_ptr));
        boost::read_graphml(inFile, *dfg_ptr, dp);

        DEBUG(print_dag_graph(*dfg_ptr););

        return dfg_ptr;
    }
}