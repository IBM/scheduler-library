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

void populate_task_map(Graph * graph_ptr, int32_t dag_vertex_id, std::map<int32_t, std::map<int32_t, vertex_t>> & task_id_map) {
    Graph::vertex_iterator v, vend;
    Graph & graph = *graph_ptr;

    for (boost::tie(v, vend) = vertices(graph); v != vend; ++v) {
        task_id_map[dag_vertex_id][graph[*v].task_vertex_id] = *v;
    }
}

extern "C" {
    Graph * create_leaf_graph(char * graphml_filename) {
        std::string filename;
        filename.append(graphml_filename);
        return(_create_leaf_graph(filename, true));
    }
    RootGraph * create_root_graph(std::string graphml_filename) {
        std::string filename;
        filename.append(graphml_filename);
        return(_create_root_graph(filename, NULL, true));
    }

    Graph * _create_leaf_graph(std::string graphml_filename, bool root_graph_call) {
        //Create empty DFG
        Graph * dfg_ptr = new(Graph);
        Graph & graph = *dfg_ptr;

        //Open graphml file
        std::ifstream inFile;
        inFile.open(graphml_filename, std::ifstream::in);

        //Read Graphml into static DFG graph
        boost::dynamic_properties dp;

        dp.property("task_id", boost::get(&dag_vertex_t::task_vertex_id, *dfg_ptr));
        dp.property("task_type", boost::get(&dag_vertex_t::task_type, *dfg_ptr));

        boost::read_graphml(inFile, *dfg_ptr, dp);

        DEBUG(print_dag_graph(*dfg_ptr););

        return dfg_ptr;
    }


    RootGraph * _create_root_graph(std::string graphml_filename, RootGraph * parent_root_graph, bool root_graph_call) {

        // Create RootGraph struct
        RootGraph * root_graph_ptr = new(RootGraph);

        if (root_graph_call) {
            parent_root_graph = root_graph_ptr;
        }

        //Create empty DFG and assign to rootgraph
        root_graph_ptr->graph_ptr = new(Graph);

        //Pass the root_graph_call bool
        root_graph_ptr->root_graph_call = root_graph_call;

        //Create useful temporary variables
        Graph * dfg_ptr = root_graph_ptr->graph_ptr;
        Graph & graph = *dfg_ptr;

        //Open graphml file
        std::ifstream inFile;
        inFile.open(graphml_filename, std::ifstream::in);

        //Read Graphml into static DFG graph
        boost::dynamic_properties dp;

        dp.property("filename", boost::get(&dag_vertex_t::graphml_filename, *dfg_ptr));
        dp.property("dag_id", boost::get(&dag_vertex_t::dag_vertex_id, *dfg_ptr));
        dp.property("leaf_dag", boost::get(&dag_vertex_t::leaf_dag_type, *dfg_ptr));

        boost::read_graphml(inFile, *dfg_ptr, dp);

        DEBUG(print_dag_graph(*dfg_ptr););

        void * vertex_graph_ptr;

        //Pre-process which task_vertex_id is in which dag_vertex_id
        Graph::vertex_iterator v, vend;
        for (boost::tie(v, vend) = vertices(graph); v != vend; ++v) {
            if (graph[*v].leaf_dag_type) {
                Graph * v_leaf_graph_ptr = _create_leaf_graph(graph[*v].graphml_filename, false);
                vertex_graph_ptr = (void *) v_leaf_graph_ptr;
                populate_task_map(v_leaf_graph_ptr, graph[*v].dag_vertex_id, parent_root_graph->task_id_map);
                break;
            }
            else {
                RootGraph * v_root_graph_ptr = _create_root_graph(graph[*v].graphml_filename, parent_root_graph, false);
                vertex_graph_ptr = (void *) v_root_graph_ptr;
            }
            parent_root_graph->dag_id_map[graph[*v].task_vertex_id] = std::make_pair(graph[*v].leaf_dag_type, vertex_graph_ptr);
        }

        return root_graph_ptr;
    }
}

