#pragma once
#include "searchnode.h"
#include "successor.h"
#include "mesh.h"
#include "point.h"
#include "cpool.h"
#include "timer.h"
#include <queue>
#include <vector>
#include <ctime>
#include <chrono>
#include <algorithm>
#include <cpd.h>


namespace polyanya
{

    template<typename T, typename Compare = std::greater<T> >
    struct PointerComp5
    {
        bool operator()(const T* x,
                        const T* y) const
        {
            return Compare()(*x, *y);
        }
    };

    typedef Mesh* MeshPtr;
    typedef cpd *CPDPtr;
// Polyanya instance for point to point search
    class epsSearchInstance
    {
        typedef std::priority_queue<SearchNodePtr, std::vector<SearchNodePtr>,
                PointerComp5<SearchNode> > pq;
    private:
        warthog::mem::cpool* node_pool;
        MeshPtr mesh;
        CPDPtr cpd;

        double shortest_path;

        // set by init_search
        pq open_list;
        int size;
        // Best g value for a specific vertex.
        std::vector<double> root_g_values;
        // Contains the current search id if the root has been reached by
        // the search.
        std::vector<int> root_search_ids;  // also used for root-level pruning

        std::vector<double> distance_cache;
        // Contains the current search id if the root has been reached by
        // the search.
        std::vector<int> distance_cache_id;

        std::vector<int> visible_vertices_list;
        int vertice_search_id;

        warthog::timer timer;
        warthog::timer timer2;

        // Pre-initialised variables to use in search().
        Successor* search_successors;
        SearchNode* search_nodes_to_push;

        void init()
        {
            verbose = false;
            search_successors = new Successor [mesh->max_poly_sides + 2];
            search_nodes_to_push = new SearchNode [mesh->max_poly_sides + 2];
            node_pool = new warthog::mem::cpool(sizeof(SearchNode));
            init_root_pruning();
        }
        void init_root_pruning()
        {
            assert(mesh != nullptr);
            search_id = 0;
            vertice_search_id =0;

            size_t num_vertices = mesh->mesh_vertices.size();
            root_g_values.resize(num_vertices);
            root_search_ids.resize(num_vertices);
            fill(root_search_ids.begin(), root_search_ids.end(), 0);
            distance_cache.resize(num_vertices);

            distance_cache_id.resize(num_vertices);
            fill(distance_cache_id.begin(), distance_cache_id.end(), 0);

            visible_vertices_list.resize(num_vertices);
            fill(visible_vertices_list.begin(), visible_vertices_list.end(), 0);

            size = num_vertices;
            path_pool = vector<int>(200);
            distance_pool = vector<double>(200);
            start_vertices = vector<int> (200);
        }
        void init_search()
        {
            assert(node_pool);
            node_pool->reclaim();
            search_id++;
            open_list = pq();
            final_node = nullptr;
            nodes_generated = 0;
            nodes_pushed = 0;
            nodes_popped = 0;
            nodes_pruned_post_pop = 0;
            successor_calls = 0;
            set_end_polygon();
            gen_initial_nodes();
        }


        void init_search(PointLocation start_pl ,PointLocation end_pl)
        {
            assert(node_pool);
            node_pool->reclaim();
            search_id++;
            vertice_search_id ++;
            open_list = pq();
            final_node = nullptr;
            nodes_generated = 0;
            nodes_pushed = 0;
            nodes_popped = 0;
            nodes_pruned_post_pop = 0;
            successor_calls = 0;
            start_index = 0;

            set_end_polygon(end_pl);
            gen_initial_nodes(start_pl);
        }



        void set_end_polygon();
        void gen_initial_nodes();
        int succ_to_node(
                SearchNodePtr parent, Successor* successors,
                int num_succ, SearchNode* nodes
        );
        void print_node(SearchNodePtr node, std::ostream& outfile);

    public:

        int nodes_generated;        // Nodes stored in memory
        int nodes_pushed;           // Nodes pushed onto open
        int nodes_popped;           // Nodes popped off open
        int nodes_pruned_post_pop;  // Nodes we prune right after popping off

        int successor_calls;        // Times we call get_successors
        bool verbose;
        int search_id;

        vector<int> path_pool;
        vector<double> distance_pool;
        int end_polygon;
        SearchNodePtr final_node;


        vector<int> start_vertices;
        int start_index;
        Point start, goal;
        vector<int> path;
        epsSearchInstance() = default;
        epsSearchInstance(MeshPtr m , CPDPtr c) : mesh(m) , cpd(c) { init(); }

        epsSearchInstance(epsSearchInstance const &) = delete;
        void operator=(epsSearchInstance const &x) = delete;
        ~epsSearchInstance()
        {
            if (node_pool)
            {
                delete node_pool;
            }
            delete[] search_successors;
            delete[] search_nodes_to_push;
        }

        void set_start_goal(Point s, Point g)
        {
            start = s;
            goal = g;
            final_node = nullptr;
        }

        bool search();

        bool search_visible_vertices();


        double get_SP_cost(){
            if(shortest_path == INF){
                return -1;
            }
            return shortest_path;
        }

        double get_cost()
        {
            if (final_node == nullptr)
            {
                return -1;
            }

            return final_node->f;
        }

        double get_search_micro()
        {
            return timer2.elapsed_time_micro();
        }

        double get_search_nano()
        {
            return timer2.elapsed_time_nano();
        }
        void set_shortest_path(double cost){
            shortest_path =cost;
        }
        void print_search_nodes(std::ostream& outfile);

        void gen_initial_nodes(const PointLocation& start_pl);

        void set_end_polygon(const PointLocation& end_pl);

        double get_cpd_distance(int goal_id,int start_id);

        void initial_search(const PointLocation &start_pl, const PointLocation &end_pl);

        int get_visible_vertices(vector<int> &current_vertices);

        int get_cpd_path(int start_id, int goal_id, vector <Point> &path);

        bool isTautVertex(int vertex_id);

        bool is_taut_path(Point s_or_t, const Vertex &vertex, const Vertex &vertex2);
    };

}
