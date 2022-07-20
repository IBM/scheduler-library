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
        return(_create_leaf_graph(filename, -1, true));
    }

    graph_wrapper_t * _create_leaf_graph(std::string graphml_filename, int32_t dag_vertex_id, bool root_parent_graph_call) {
        DEBUG(printf("Entering create leaf graph %s\n", graphml_filename.c_str()));
        // Create graph_wrapper_t struct
        graph_wrapper_t * graph_wptr = new(graph_wrapper_t);

        //Create empty DFG and assign to graph_wrapper_t
        graph_wptr->graph_ptr = new(Graph);

        //Set DAG vertex ID
        graph_wptr->dag_vertex_id = dag_vertex_id;

        //Set graphml name
        graph_wptr->graphml_filename = graphml_filename;

        //Pass the root_parent_graph_call bool
        graph_wptr->root_parent_graph_call = root_parent_graph_call;
        graph_wptr->dag_status = FREE_DAG;
        if (root_parent_graph_call) {
            //Leaf graph in application
            graph_wptr->parent_dag_vertex_id = -1;
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

    graph_wrapper_t * create_root_graph(char * graphml_filename) {
        DEBUG(printf("Entering wrapper for create root graph  %s\n", graphml_filename););
        std::string filename;
        filename.append(graphml_filename);
        graph_wrapper_t * return_root_graph_wptr = _create_root_graph(filename, -1, NULL, true);
        DEBUG(printf("Create root graph with %d leaf vertices\n", return_root_graph_wptr->num_task_vertices););
        return(return_root_graph_wptr);
    }

    graph_wrapper_t * _create_root_graph(std::string graphml_filename, int32_t dag_vertex_id, graph_wrapper_t * root_parent_graph_wptr, bool root_parent_graph_call) {
        DEBUG(printf("Entering create root graph for %s\n", graphml_filename.c_str()));
        // Create graph_wrapper_t struct
        graph_wrapper_t * graph_wptr = new(graph_wrapper_t);

        graph_wptr->dag_vertex_id = dag_vertex_id;
        graph_wptr->graphml_filename = graphml_filename;

        if (root_parent_graph_call) {
            root_parent_graph_wptr = graph_wptr;
            graph_wptr->parent_dag_vertex_id = -1;
            graph_wptr->dag_id_map[graph_wptr->dag_vertex_id] = std::make_pair(false, graph_wptr);
            root_parent_graph_wptr->num_task_vertices = 0;
        }
        graph_wptr->root_parent_graph_wptr = root_parent_graph_wptr;
        graph_wptr->dag_status = FREE_DAG;

        //Create empty DFG and assign to graph_wrapper_t
        graph_wptr->graph_ptr = new(Graph);

        //Pass the root_parent_graph_call bool
        graph_wptr->root_parent_graph_call = root_parent_graph_call;
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

        // DEBUG(boost::write_graphviz(std::cout, *dfg_ptr););

        graph_wrapper_t * vertex_graph_wptr;

        //Pre-process which task_vertex_id is in which dag_vertex_id
        Graph::vertex_iterator v, vend;
        for (boost::tie(v, vend) = vertices(graph); v != vend; ++v) {
            if (graph[*v].leaf_dag_type) {
                vertex_graph_wptr = _create_leaf_graph(graph[*v].graphml_filename, graph[*v].dag_vertex_id, false);

                //Populate task id map
                Graph * v_leaf_graph_ptr = vertex_graph_wptr->graph_ptr;
                DEBUG(printf("%s has %d leaf task vertices\n", graph[*v].graphml_filename.c_str(), boost::num_vertices(*v_leaf_graph_ptr)););
                root_parent_graph_wptr->num_task_vertices += boost::num_vertices(*v_leaf_graph_ptr);
                populate_task_map(v_leaf_graph_ptr, graph[*v].dag_vertex_id, root_parent_graph_wptr->task_id_map);
            }
            else {
                vertex_graph_wptr = _create_root_graph(graph[*v].graphml_filename, graph[*v].dag_vertex_id, root_parent_graph_wptr, false);
            }
            graph[*v].dag_status = FREE_DAG;
            vertex_graph_wptr->dag_status = FREE_DAG;
            vertex_graph_wptr->parent_dag_vertex_id = graph_wptr->dag_vertex_id;
            vertex_graph_wptr->root_parent_graph_wptr = root_parent_graph_wptr;
            root_parent_graph_wptr->dag_id_map[graph[*v].dag_vertex_id] = std::make_pair(graph[*v].leaf_dag_type, vertex_graph_wptr);

            DEBUG(printf("Create DAG ID: %d : Leaf: %d Graph_wptr: %p Parent DAG ID: %d %d\n", graph[*v].dag_vertex_id, graph[*v].leaf_dag_type, vertex_graph_wptr, vertex_graph_wptr->parent_dag_vertex_id, graph_wptr->dag_vertex_id););
            // std::cout << std::boolalpha << "DAG ID: " << graph[*v].dag_vertex_id << " : Leaf: " << graph[*v].leaf_dag_type << " Graph_wptr: " << vertex_graph_wptr << " Parent DAG ID: " << vertex_graph_wptr->parent_dag_vertex_id << std::endl;
        }

        return graph_wptr;
    }
}

