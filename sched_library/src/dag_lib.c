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

void populate_task_map(Graph * graph_ptr, int32_t dag_vertex_id, std::map<int32_t, std::list<int32_t>> & task_id_map) {
    Graph::vertex_iterator v, vend;
    Graph & graph = *graph_ptr;

    for (boost::tie(v, vend) = vertices(graph); v != vend; ++v) {
        task_id_map[dag_vertex_id].push_back(graph[*v].task_vertex_id);
    }
}

extern "C" {
    graph_wrapper_t * create_leaf_graph(char * graphml_filename) {
        DEBUG(printf("Entering wrapper for create leaf graph\n"));
        std::string filename;
        filename.append(graphml_filename);
        return(_create_leaf_graph(filename, true));
    }
    graph_wrapper_t * create_root_graph(char * graphml_filename) {
        std::string filename;
        filename.append(graphml_filename);
        return(_create_root_graph(filename, NULL, true));
    }

    graph_wrapper_t * _create_leaf_graph(std::string graphml_filename, bool parent_graph_call) {
        DEBUG(printf("Entering create leaf graph\n"));
        // Create graph_wrapper_t struct
        graph_wrapper_t * graph_wptr = new(graph_wrapper_t);

        //Create empty DFG and assign to graph_wrapper_t
        graph_wptr->graph_ptr = new(Graph);

        //Pass the parent_graph_call bool
        graph_wptr->parent_graph_call = parent_graph_call;
        if (parent_graph_call) {
            //Leaf graph in application
            graph_wptr->dag_vertex_id = -1;
            graph_wptr->parent_graph_wptr = NULL;
        }
        graph_wptr->leafGraph = true;

        //Create useful temporary variables
        Graph * dfg_ptr = graph_wptr->graph_ptr;
        Graph & graph = *dfg_ptr;

        //Open graphml file
        std::ifstream inFile;
        inFile.open(graphml_filename, std::ifstream::in);

        //Read Graphml into static DFG graph
        boost::dynamic_properties dp;

        dp.property("vertex_id", boost::get(&dag_vertex_t::task_vertex_id, *dfg_ptr));
        dp.property("task_type", boost::get(&dag_vertex_t::task_type, *dfg_ptr));

        boost::read_graphml(inFile, *dfg_ptr, dp);

        DEBUG(print_dag_graph(*dfg_ptr););


        return graph_wptr;
    }


    graph_wrapper_t * _create_root_graph(std::string graphml_filename, graph_wrapper_t * parent_graph_wptr, bool parent_graph_call) {

        // Create graph_wrapper_t struct
        graph_wrapper_t * graph_wptr = new(graph_wrapper_t);

        if (parent_graph_call) {
            parent_graph_wptr = graph_wptr;
            graph_wptr->dag_vertex_id = -1;
            graph_wptr->dag_status = FREE_DAG;
        }

        //Create empty DFG and assign to graph_wrapper_t
        graph_wptr->graph_ptr = new(Graph);

        //Pass the parent_graph_call bool
        graph_wptr->parent_graph_call = parent_graph_call;
        graph_wptr->leafGraph = false;

        //Create useful temporary variables
        Graph * dfg_ptr = graph_wptr->graph_ptr;
        Graph & graph = *dfg_ptr;

        //Open graphml file
        std::ifstream inFile;
        inFile.open(graphml_filename, std::ifstream::in);

        //Read Graphml into static DFG graph
        boost::dynamic_properties dp;

        //TODO: Create node of a root dag to be graph_wrapper_t instead of dag_vertex_t

        dp.property("filename", boost::get(&dag_vertex_t::graphml_filename, *dfg_ptr));
        dp.property("dag_id", boost::get(&dag_vertex_t::dag_vertex_id, *dfg_ptr));
        dp.property("leaf_dag", boost::get(&dag_vertex_t::leaf_dag_type, *dfg_ptr));

        boost::read_graphml(inFile, *dfg_ptr, dp);

        DEBUG(print_dag_graph(*dfg_ptr););

        graph_wrapper_t * vertex_graph_wptr;

        //Pre-process which task_vertex_id is in which dag_vertex_id
        Graph::vertex_iterator v, vend;
        for (boost::tie(v, vend) = vertices(graph); v != vend; ++v) {
            if (graph[*v].leaf_dag_type) {
                vertex_graph_wptr = _create_leaf_graph(graph[*v].graphml_filename, false);
                vertex_graph_wptr->dag_vertex_id = graph[*v].dag_vertex_id;
                Graph * v_leaf_graph_ptr = vertex_graph_wptr->graph_ptr;
                parent_graph_wptr->num_task_vertices += boost::num_vertices(*v_leaf_graph_ptr);

                populate_task_map(v_leaf_graph_ptr, graph[*v].dag_vertex_id, parent_graph_wptr->task_id_map);
                break;
            }
            else {
                vertex_graph_wptr = _create_root_graph(graph[*v].graphml_filename, parent_graph_wptr, false);
                vertex_graph_wptr->dag_vertex_id = graph[*v].dag_vertex_id;

            }
            graph[*v].dag_status = FREE_DAG;
            vertex_graph_wptr->dag_status = FREE_DAG;
            vertex_graph_wptr->parent_dag_vertex_id = graph_wptr->dag_vertex_id;
            parent_graph_wptr->dag_id_map[graph[*v].dag_vertex_id] = std::make_pair(graph[*v].leaf_dag_type, vertex_graph_wptr);
        }

        return graph_wptr;
    }
}

