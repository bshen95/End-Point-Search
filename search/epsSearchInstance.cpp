#include "epsSearchInstance.h"
#include "expansion.h"
#include "geometry.h"
#include "searchnode.h"
#include "successor.h"
#include "vertex.h"
#include "mesh.h"
#include "point.h"
#include "consts.h"
#include <queue>
#include <vector>
#include <cassert>
#include <iostream>
#include <algorithm>
#include <ctime>
#include <unordered_set>

namespace polyanya {

    int epsSearchInstance::succ_to_node(
            SearchNodePtr parent, Successor* successors, int num_succ,
            SearchNode* nodes
    )
    {
        assert(mesh != nullptr);
#ifndef NDEBUG
        if (verbose)
        {    for (int i = 0; i < num_succ; i++) {
                const Successor &succ = successors[i];
                std::cerr << "received successors: ";
                std::cerr << succ;
                std::cerr << std::endl;
            }
        }
#endif

        const Polygon& polygon = mesh->mesh_polygons[parent->next_polygon];
        const std::vector<int>& V = polygon.vertices;
        const std::vector<int>& P = polygon.polygons;

        double right_g = -1, left_g = -1;

        int out = 0;




        for (int i = 0; i < num_succ; i++)
        {

            const Successor& succ = successors[i];
            const int next_polygon = P[succ.poly_left_ind];
            if (next_polygon == -1)
            {
                continue;
            }


            // If the successor we're about to push pushes into a one-way polygon,
            // and the polygon isn't the end polygon, just continue.

            //turn this off if we want to find all vetices.
            if (mesh->mesh_polygons[next_polygon].is_one_way &&
                next_polygon != end_polygon)
            {
                continue;
            }
            const int left_vertex  = V[succ.poly_left_ind];
            const int right_vertex = succ.poly_left_ind ?
                                     V[succ.poly_left_ind - 1] :
                                     V.back();

            // Note that g is evaluated twice here. (But this is a lambda!)
            // Always try to precompute before using this macro.
            // We implicitly set h to be zero and let search() update it.
            const auto p = [&](const int root, const double g)
            {
                if (root != -1)
                {
                    assert(root >= 0 && root < (int) root_g_values.size());
                    // Can POSSIBLY prune?
                    if (root_search_ids[root] != search_id)
                    {
                        // First time reaching root
                        root_search_ids[root] = search_id;
                        root_g_values[root] = g;
                    }
                    else
                    {
                        // We've been here before!
                        // Check whether we've done better.
                        if (root_g_values[root] + EPSILON <  g  )
                        {
                            // We've done better!
                            return;
                        }else{
                            // This is better.
                            root_g_values[root] = g;
                        }

                    }

                }
                nodes[out++] = {nullptr, root, succ.left, succ.right, left_vertex,
                                right_vertex, next_polygon, g, g};
            };

            const Point& parent_root = (parent->root == -1 ?
                                        start :
                                        mesh->mesh_vertices[parent->root].p);
#define get_g(new_root) parent->g + parent_root.distance(new_root)

            switch (succ.type)
            {
                case Successor::RIGHT_NON_OBSERVABLE:
                    if (right_g == -1)
                    {
                        right_g = get_g(parent->right);
                    }
                    p(parent->right_vertex, right_g);
                    break;

                case Successor::OBSERVABLE:
                    p(parent->root, parent->g);
                    break;

                case Successor::LEFT_NON_OBSERVABLE:
                    if (left_g == -1)
                    {
                        left_g = get_g(parent->left);
                    }
                    p(parent->left_vertex, left_g);
                    break;

                default:
                    assert(false);
                    break;
            }
#undef get_h
#undef get_g
        }

        return out;
    }

    void epsSearchInstance::set_end_polygon()
    {
        // Any polygon is fine.
        end_polygon = get_point_location_in_search(goal, mesh, verbose).poly1;
    }
    void epsSearchInstance::set_end_polygon(const PointLocation& end_pl)
    {
        // Any polygon is fine.
        end_polygon = end_pl.poly1;
    }

    void epsSearchInstance::gen_initial_nodes()
    {
        // {parent, root, left, right, next_polygon, right_vertex, f, g}
        // be VERY lazy and abuse how our function expands collinear search nodes
        // if right_vertex is not valid, it will generate EVERYTHING
        // and we can set right_vertex if we want to omit generating an interval.
        const PointLocation pl = get_point_location_in_search(start, mesh, verbose);

        const double h = start.distance(goal);
#define get_lazy(next, left, right) new (node_pool->allocate()) SearchNode \
        {nullptr, -1, start, start, left, right, next, h, 0}

#define v(vertex) mesh->mesh_vertices[vertex]

        const auto push_lazy = [&](SearchNodePtr lazy)
        {
            const int poly = lazy->next_polygon;
            if (poly == -1)
            {
                return;
            }
            if (poly == end_polygon)
            {
                // Trivial case - we can see the goal from start!
                final_node = lazy;
#ifndef NDEBUG
                if (verbose)
                {
                    std::cerr << "got a trivial case!" << std::endl;
                }
#endif
                // we should check final_node after each push_lazy
                return;
            }
            // iterate over poly, throwing away vertices if needed
            const std::vector<int>& vertices =
                    mesh->mesh_polygons[poly].vertices;
            Successor* successors = new Successor [vertices.size()];
            int last_vertex = vertices.back();
            int num_succ = 0;
            for (int i = 0; i < (int) vertices.size(); i++)
            {
                const int vertex = vertices[i];
                if (vertex == lazy->right_vertex ||
                    last_vertex == lazy->left_vertex)
                {
                    last_vertex = vertex;
                    continue;
                }
                successors[num_succ++] =
                        {Successor::OBSERVABLE, v(vertex).p,
                         v(last_vertex).p, i};
                last_vertex = vertex;
            }
            SearchNode* nodes = new SearchNode [num_succ];
            const int num_nodes = succ_to_node(lazy, successors,
                                               num_succ, nodes);
            delete[] successors;
            for (int i = 0; i < num_nodes; i++)
            {
                SearchNodePtr n = new (node_pool->allocate())
                        SearchNode(nodes[i]);
                const Point& n_root = (n->root == -1 ? start :
                                       mesh->mesh_vertices[n->root].p);
                n->f += get_h_value(n_root, goal, n->left, n->right);
                n->parent = lazy;
#ifndef NDEBUG
                if (verbose)
                {
                    std::cerr << "generating init node: ";
                    print_node(n, std::cerr);
                    std::cerr << std::endl;
                }
#endif
                open_list.push(n);
            }
            delete[] nodes;
            nodes_generated += num_nodes;
            nodes_pushed += num_nodes;
        };

        switch (pl.type)
        {
            // Don't bother.
            case PointLocation::NOT_ON_MESH:
                break;

                // Generate all in an arbirary polygon.
            case PointLocation::ON_CORNER_VERTEX_AMBIG:
                // It's possible that it's -1!
                if (pl.poly1 == -1)
                {
                    break;
                }
            case PointLocation::ON_CORNER_VERTEX_UNAMBIG:
                // Generate all in the polygon.
            case PointLocation::IN_POLYGON:
            case PointLocation::ON_MESH_BORDER:
            {
                SearchNodePtr lazy = get_lazy(pl.poly1, -1, -1);
                push_lazy(lazy);
                nodes_generated++;
            }
                break;

            case PointLocation::ON_EDGE:
                // Generate all in both polygons except for the shared side.
            {
                SearchNodePtr lazy1 = get_lazy(pl.poly2, pl.vertex1, pl.vertex2);
                SearchNodePtr lazy2 = get_lazy(pl.poly1, pl.vertex2, pl.vertex1);
                push_lazy(lazy1);
                nodes_generated++;
                if (final_node)
                {
                    return;
                }
                push_lazy(lazy2);
                nodes_generated++;
            }
                break;


            case PointLocation::ON_NON_CORNER_VERTEX:
            {
                for (int& poly : v(pl.vertex1).polygons)
                {
                    SearchNodePtr lazy = get_lazy(poly, pl.vertex1, pl.vertex1);
                    push_lazy(lazy);
                    nodes_generated++;
                    if (final_node)
                    {
                        return;
                    }
                }
            }
                break;


            default:
                assert(false);
                break;
        }

#undef v
#undef get_lazy
    }


    void epsSearchInstance::gen_initial_nodes(const PointLocation& start_pl)
    {
        // {parent, root, left, right, next_polygon, right_vertex, f, g}
        // be VERY lazy and abuse how our function expands collinear search nodes
        // if right_vertex is not valid, it will generate EVERYTHING
        // and we can set right_vertex if we want to omit generating an interval.
        const PointLocation pl = start_pl;

        const double h = start.distance(goal);
#define get_lazy(next, left, right) new (node_pool->allocate()) SearchNode \
        {nullptr, -1, start, start, left, right, next, h, 0}

#define v(vertex) mesh->mesh_vertices[vertex]

        const auto push_lazy = [&](SearchNodePtr lazy)
        {
            const int poly = lazy->next_polygon;
            if (poly == -1)
            {
                return;
            }
            if (poly == end_polygon)
            {
                // Trivial case - we can see the goal from start!
                final_node = lazy;
#ifndef NDEBUG
                if (verbose)
                {
                    std::cerr << "got a trivial case!" << std::endl;
                }
#endif
                // we should check final_node after each push_lazy
                return;
            }
            // iterate over poly, throwing away vertices if needed
            const std::vector<int>& vertices =
                    mesh->mesh_polygons[poly].vertices;
            Successor* successors = new Successor [vertices.size()];
            int last_vertex = vertices.back();
            int num_succ = 0;
            for (int i = 0; i < (int) vertices.size(); i++)
            {
                const int vertex = vertices[i];
                const Vertex& v = mesh->mesh_vertices[vertex];
                if(visible_vertices_list[vertex] != vertice_search_id){
                    //not retrieved before;
                    if(!v.is_ambig&& v.is_turning_vertex&&isTautVertex(vertex)) {
                        start_vertices[start_index] = vertex;
                        start_index++;
                    }
                    visible_vertices_list[vertex] = vertice_search_id;
                }

                if (vertex == lazy->right_vertex ||
                    last_vertex == lazy->left_vertex)
                {
                    last_vertex = vertex;
                    continue;
                }
                successors[num_succ++] =
                        {Successor::OBSERVABLE, v(vertex).p,
                         v(last_vertex).p, i};
                last_vertex = vertex;
            }
            SearchNode* nodes = new SearchNode [num_succ];
            const int num_nodes = succ_to_node(lazy, successors,
                                               num_succ, nodes);
            delete[] successors;
            for (int i = 0; i < num_nodes; i++)
            {
                SearchNodePtr n = new (node_pool->allocate())
                        SearchNode(nodes[i]);
                const Point& n_root = (n->root == -1 ? start :
                                       mesh->mesh_vertices[n->root].p);
                n->f += get_h_value(n_root, goal, n->left, n->right);
                n->parent = lazy;
#ifndef NDEBUG
                if (verbose)
                {
                    std::cerr << "generating init node: ";
                    print_node(n, std::cerr);
                    std::cerr << std::endl;
                }
#endif
                open_list.push(n);
            }
            delete[] nodes;
            nodes_generated += num_nodes;
            nodes_pushed += num_nodes;
        };

        switch (pl.type)
        {
            // Don't bother.
            case PointLocation::NOT_ON_MESH:
                break;

                // Generate all in an arbirary polygon.
            case PointLocation::ON_CORNER_VERTEX_AMBIG:
                // It's possible that it's -1!
                if (pl.poly1 == -1)
                {
                    break;
                }
            case PointLocation::ON_CORNER_VERTEX_UNAMBIG:
                // Generate all in the polygon.
            case PointLocation::IN_POLYGON:
            case PointLocation::ON_MESH_BORDER:
            {
                SearchNodePtr lazy = get_lazy(pl.poly1, -1, -1);
                push_lazy(lazy);
                nodes_generated++;
            }
                break;

            case PointLocation::ON_EDGE:
                // Generate all in both polygons except for the shared side.
            {
                SearchNodePtr lazy1 = get_lazy(pl.poly2, pl.vertex1, pl.vertex2);
                SearchNodePtr lazy2 = get_lazy(pl.poly1, pl.vertex2, pl.vertex1);
                push_lazy(lazy1);
                nodes_generated++;
                if (final_node)
                {
                    return;
                }
                push_lazy(lazy2);
                nodes_generated++;
            }
                break;


            case PointLocation::ON_NON_CORNER_VERTEX:
            {
                for (int& poly : v(pl.vertex1).polygons)
                {
                    SearchNodePtr lazy = get_lazy(poly, pl.vertex1, pl.vertex1);
                    push_lazy(lazy);
                    nodes_generated++;
                    if (final_node)
                    {
                        return;
                    }
                }
            }
                break;


            default:
                assert(false);
                break;
        }

#undef v
#undef get_lazy
    }







#define root_to_point(root) ((root) == -1 ? start : mesh->mesh_vertices[root].p)

    bool epsSearchInstance::search()
    {
        timer.start();
        init_search();

        if (mesh == nullptr || end_polygon == -1)
        {
            timer.stop();
            return false;
        }

        if (final_node != nullptr)
        {
            timer.stop();
            return true;
        }

        while (!open_list.empty())
        {
            SearchNodePtr node = open_list.top(); open_list.pop();

#ifndef NDEBUG
            if (verbose)
            {
                std::cerr << "popped off: ";
                print_node(node, std::cerr);
                std::cerr << std::endl;
            }
#endif

            nodes_popped++;
            const int next_poly = node->next_polygon;
            if (next_poly == end_polygon)
            {
                // Make the TRUE final node.
                // (We usually push it onto the open list, but we know it's going
                // to be immediately popped off anyway.)

                // We need to find whether we need to turn left/right to ge
                // to the goal, so we do an orientation check like how we
                // special case triangle successors.

                const int final_root = [&]()
                {
                    const Point& root = root_to_point(node->root);
                    const Point root_goal = goal - root;
                    // If root-left-goal is not CW, use left.
                    if (root_goal * (node->left - root) < -EPSILON)
                    {
                        return node->left_vertex;
                    }
                    // If root-right-goal is not CCW, use right.
                    if ((node->right - root) * root_goal < -EPSILON)
                    {
                        return node->right_vertex;
                    }
                    // Use the normal root.
                    return node->root;
                }();

                const SearchNodePtr true_final =
                        new (node_pool->allocate()) SearchNode
                                {node, final_root, goal, goal, -1, -1, end_polygon,
                                 node->f, node->g};

                nodes_generated++;

                timer.stop();

#ifndef NDEBUG
                if (verbose)
                {
                    std::cerr << "found end - terminating!" << std::endl;
                }
#endif

                final_node = true_final;
                return true;
            }
            // We will never update our root list here.
            const int root = node->root;
            if (root != -1)
            {
                assert(root >= 0 && root < (int) root_g_values.size());
                if (root_search_ids[root] == search_id)
                {
                    // We've been here before!
                    // Check whether we've done better.
                    if (root_g_values[root] + EPSILON < node->g)
                    {
                        //root < node -> g;
                        nodes_pruned_post_pop++;

#ifndef NDEBUG
                        if (verbose)
                        {
                            std::cerr << "node is dominated!" << std::endl;
                        }
#endif

                        // We've done better!
                        continue;
                    }
                }
            }
            int num_nodes = 1;
            search_nodes_to_push[0] = *node;

            // We use a do while here because the first iteration is guaranteed
            // to work.
            do
            {
                SearchNode cur_node = search_nodes_to_push[0];
                // don't forget this!!!
                if (cur_node.next_polygon == end_polygon)
                {
                    break;
                }
                int num_succ = get_observable_successors(cur_node, start, *mesh,
                                                         search_successors);
                successor_calls++;
                num_nodes = succ_to_node(&cur_node, search_successors,
                                         num_succ, search_nodes_to_push);
                if (num_nodes == 1)
                {
                    // Did we turn?
                    if (cur_node.g != search_nodes_to_push[0].g)
                    {
                        // Turned. Set the parent of this, and set the current
                        // node pointer to this after allocating space for it.
                        search_nodes_to_push[0].parent = node;
                        node = new (node_pool->allocate())
                                SearchNode(search_nodes_to_push[0]);
                        nodes_generated++;
                    }

#ifndef NDEBUG
                    if (verbose)
                    {
                        std::cerr << "\tintermediate: ";
                        print_node(&search_nodes_to_push[0], std::cerr);
                        std::cerr << std::endl;
                    }
#endif
                }
            }
            while (num_nodes == 1); // if num_nodes == 0, we still want to break

            for (int i = 0; i < num_nodes; i++)
            {
                // We need to update the h value before we push!
                const SearchNodePtr n = new (node_pool->allocate())
                        SearchNode(search_nodes_to_push[i]);
                const Point& n_root = (n->root == -1 ? start :
                                       mesh->mesh_vertices[n->root].p);
                n->f += get_h_value(n_root, goal, n->left, n->right);

                // This node's parent should be nullptr, so we should set it.
                n->parent = node;

#ifndef NDEBUG
                if (verbose)
                {
                    std::cerr << "\tpushing: ";
                    print_node(n, std::cerr);
                    std::cerr << std::endl;
                }
#endif

                open_list.push(n);
            }
            nodes_generated += num_nodes;
            nodes_pushed += num_nodes;
        }

        timer.stop();
        return false;
    }

    void epsSearchInstance::initial_search(const PointLocation& start_pl , const PointLocation& end_pl){
        init_search(start_pl,end_pl);
    }

    int epsSearchInstance::get_visible_vertices(vector<int>& current_vertices){
        bool found_visible = false;
        int number_of_visible = 0;
        while (!open_list.empty())
        {
            SearchNodePtr node = open_list.top(); open_list.pop();

#ifndef NDEBUG
            if (verbose)
            {
                std::cerr << "popped off: ";
                print_node(node, std::cerr);
                std::cerr << std::endl;
            }
#endif


            nodes_popped++;
            const int next_poly = node->next_polygon;
            if (next_poly == end_polygon)
            {
                // Make the TRUE final node.
                // (We usually push it onto the open list, but we know it's going
                // to be immediately popped off anyway.)

                // We need to find whether we need to turn left/right to ge
                // to the goal, so we do an orientation check like how we
                // special case triangle successors.
                const Point& root = root_to_point(node->root);

                if(fabs(root.distance(goal)+node->g- node->f) < EPSILON) {
                    const int final_root = [&]() {
                        const Point &root = root_to_point(node->root);
                        const Point root_goal = goal - root;
                        // If root-left-goal is not CW, use left.
                        if (root_goal * (node->left - root) < -EPSILON) {
                            return node->left_vertex;
                        }
                        // If root-right-goal is not CCW, use right.
                        if ((node->right - root) * root_goal < -EPSILON) {
                            return node->right_vertex;
                        }
                        // Use the normal root.
                        return node->root;
                    }();

                    const SearchNodePtr true_final =
                            new(node_pool->allocate()) SearchNode
                                    {node, final_root, goal, goal, -1, -1, end_polygon,
                                     node->f, node->g};

                    nodes_generated++;

                    timer.stop();

#ifndef NDEBUG
                    if (verbose) {
                        std::cerr << "found end - terminating!" << std::endl;
                    }
#endif

                    final_node = true_final;
                    return 0;
                }
            }
            if(node->f > shortest_path){
                continue;
            }

            // We will never update our root list here.
            const int root = node->root;
            if (root != -1)
            {
                assert(root >= 0 && root < (int) root_g_values.size());
                if (root_search_ids[root] == search_id)
                {
                    // We've been here before!
                    // Check whether we've done better.
                    if (root_g_values[root] + EPSILON < node->g)
                    {
                        //root < node -> g;
                        nodes_pruned_post_pop++;

#ifndef NDEBUG
                        if (verbose)
                        {
                            std::cerr << "node is dominated!" << std::endl;
                        }
#endif

                        // We've done better!
                        continue;
                    }
                }
            }
            int num_nodes = 1;
            search_nodes_to_push[0] = *node;

            SearchNode cur_node = search_nodes_to_push[0];
            int num_succ = get_observable_successors(cur_node, start, *mesh,
                                                     search_successors);
            successor_calls++;
            num_nodes = succ_to_node(&cur_node, search_successors,
                                     num_succ, search_nodes_to_push);

            for (int i = 0; i < num_nodes; i++)
            {
                // We need to update the h value before we push!
                const SearchNodePtr n = new (node_pool->allocate())
                        SearchNode(search_nodes_to_push[i]);
                const Point& n_root = (n->root == -1 ? start :
                                       mesh->mesh_vertices[n->root].p);
                n->f += get_h_value(n_root, goal, n->left, n->right);
                if(n->f > shortest_path){
                    continue;
                }
                // This node's parent should be nullptr, so we should set it.
                n->parent = node;

#ifndef NDEBUG
                if (verbose)
                {
                    std::cerr << "\tpushing: ";
                    print_node(n, std::cerr);
                    std::cerr << std::endl;
                }
#endif
//                print_node(n,std::cout);
//                std::cout<<std::endl;
                nodes_pushed ++;
                nodes_generated ++;
                open_list.push(n);
            }



            const Vertex&  left_v = mesh->mesh_vertices[ node->left_vertex];
            const Vertex&  right_v = mesh->mesh_vertices[ node->right_vertex];

            if(node->left == left_v.p ){
                if(visible_vertices_list[node->left_vertex] != vertice_search_id){
                    //not retrieved before;
                    if(left_v.is_turning_vertex&& !left_v.is_ambig && isTautVertex(node->left_vertex)) {
                        found_visible = true;
                        current_vertices[number_of_visible] = node->left_vertex;
                        number_of_visible++;
                    }
                    visible_vertices_list[node->left_vertex] = vertice_search_id;
                }
            }

            if(node->right == right_v.p ){
                if(visible_vertices_list[node->right_vertex] != vertice_search_id){
                    if (right_v.is_turning_vertex && !right_v.is_ambig&&isTautVertex(node->right_vertex)) {
                        found_visible = true;
                        current_vertices[number_of_visible] = node->right_vertex;
                        number_of_visible++;
                    }
                    visible_vertices_list[node->right_vertex] = vertice_search_id;
                }
            }


            if(found_visible){
                return number_of_visible;
            }
        }

        timer.stop();
        return -1;
    }




    void epsSearchInstance::print_node(SearchNodePtr node, std::ostream& outfile)
    {
        outfile << "root=" << root_to_point(node->root) << "; left=" << node->left
                << "; right=" << node->right << "; f=" << node->f << ", g="
                << node->g;
        /*
        outfile << "; col=" << [&]() -> std::string
                {
                    switch (node->col_type)
                    {
                        case SearchNode::NOT:
                            return "NOT";
                        case SearchNode::RIGHT:
                            return "RIGHT";
                        case SearchNode::LEFT:
                            return "LEFT";
                        case SearchNode::LAZY:
                            return "LAZY";
                        default:
                            return "";
                    }
                }();
        */
    }


    void epsSearchInstance::print_search_nodes(std::ostream& outfile)
    {
        if (final_node == nullptr)
        {
            return;
        }
        SearchNodePtr cur_node = final_node;
        while (cur_node != nullptr)
        {
            print_node(cur_node, outfile);
            outfile << std::endl;
            mesh->print_polygon(outfile, cur_node->next_polygon);
            outfile << std::endl;
            cur_node = cur_node->parent;
        }
    }

    bool epsSearchInstance::isTautVertex(int vertex_id){
        const Vertex& v  = mesh->mesh_vertices[vertex_id];
        const Vertex& v1  = mesh->mesh_vertices[v.obstacle_edge[0]];
        const Vertex& v2  = mesh->mesh_vertices[v.obstacle_edge[1]];
        const Orientation  o1 = get_orientation(start,v.p,v1.p);
        const Orientation  o2 = get_orientation(start,v.p,v2.p);
        if(o1==Orientation::COLLINEAR || o2==Orientation::COLLINEAR ){
            return true;
        }else if(o1 == o2 ){
            return true;
        }
        return false;
    }





// This is some old implementation where I created a list of point(cpd_list) to keep all the convex vertices.
// So we dont need to maintain the distance cost graph, but need to compute the distance on the fly!
// Another improvment I haven't done yet:
// For the distance cost graph (mesh->cpd_distance_cost) and out going vertices graph (mesh->cpd_out_vertices)
// probably, it's better to implement it as adjacent lists. Anyway that's really minor improvement. Up to u, programmer!


//    double epsSearchInstance::get_cpd_distance( int goal_id,int start_id){
//        auto retrieve_next_move = [&](const int& source, const int& target) {
//            if(source == target){
//                return -1;
//            }
//            const int& first_move = cpd->get_first_move(source, target);
//            return  first_move-2;
//        };
//        bool reached = false;
//        int cur_id = goal_id;
//        const vector<Vertex>& mesh_vertices  = mesh->mesh_vertices;
//
//
//        Point* current =  &mesh->cpd_list[goal_id];
//        const int& goal_v_id = current->vertexId;
//        if(distance_cache_id[goal_v_id ] == search_id){
//            return distance_cache[goal_v_id];
//        }
//        int next_move = retrieve_next_move ( start_id,cur_id);
//        if(next_move == -2){
//            return -1;
//        }else if(next_move != -1){
//            int first_id = mesh->cpd_out_vertices[start_id][next_move];
//            const int& first_v_id = mesh->cpd_list[first_id].vertexId;
//            if(!is_taut_path(start,mesh_vertices[mesh->cpd_list[start_id].vertexId],mesh_vertices[first_v_id])){
//                return -1;
//            }
//        }else {
//            return mesh->cpd_list[start_id].distance(*current);
//        }
//
//
//        int index = 0;
//        path_pool[index] = goal_v_id;
//        distance_pool[index] = 0;
//        index ++;
//        double final_distance = 0;
//        while (!reached){
//            next_move = retrieve_next_move ( cur_id,start_id);
//            if(next_move == -2){
//                return -1;
//            }else if(next_move == -1){
//                distance_pool[index-1] =  mesh->cpd_list[start_id].distance(*current);
//                reached = true;
//                index --;
//            }else{
//                cur_id = mesh->cpd_out_vertices[cur_id][next_move];
//                assert(index >= 0);
//                distance_pool[index-1] =  mesh->cpd_list[cur_id].distance(*current);
//                const int& v_id = mesh->cpd_list[cur_id].vertexId;
//                if(index == 1){
//                    if(!is_taut_path(goal,mesh_vertices[current->vertexId],mesh_vertices[v_id])){
//                        return -1;
//                    }
//                }
//
//                if(distance_cache_id[v_id] == search_id){
//
//                    final_distance = distance_cache[v_id];
//                    reached = true;
//                }
//                current = &mesh->cpd_list[cur_id];
//                path_pool[index] = v_id;
//                distance_pool[index] = 0;
//            }
//            index++;
//        }
//        assert(index >= 0);
//        index = index -1;
//        while(index >= 0){
//            assert(index<distance_pool.size() && index>=0);
//            final_distance = final_distance + distance_pool[index];
//            const int& vertex_id = path_pool[index];
//            assert(vertex_id<distance_cache_id.size() && vertex_id >= 0);
//            distance_cache_id[vertex_id] = search_id;
//            distance_cache[vertex_id] = final_distance;
//            index--;
//        }
//        return final_distance;
//    }

    bool epsSearchInstance::is_taut_path(Point s_or_t,const Vertex& vertex,const Vertex& vertex2){
        const vector<Vertex>& mesh_vertices  = mesh->mesh_vertices;
        const Orientation& v_right_ori = get_orientation(s_or_t,vertex.p, vertex2.p);
        if(v_right_ori == Orientation::COLLINEAR){
            const double& d1 = s_or_t.distance(vertex.p);
            return s_or_t.distance(vertex2.p) >= vertex.p.distance(vertex2.p)
                   && s_or_t.distance(vertex2.p) >= d1;
        }

        Point mid_point = {(mesh_vertices[vertex.obstacle_edge[0]].p.x+ mesh_vertices[vertex.obstacle_edge[1]].p.x)/2,
                           (mesh_vertices[vertex.obstacle_edge[0]].p.y+ mesh_vertices[vertex.obstacle_edge[1]].p.y)/2
        };
        const Orientation& start_v_ori = get_orientation(s_or_t, vertex.p, mid_point);

        if(v_right_ori == start_v_ori){
            //same side;
            const Orientation& o1 = get_orientation(mid_point, vertex.p, s_or_t);
            const Orientation& o2 = get_orientation(mid_point, vertex.p, vertex2.p);
            return o1 != o2;
        }else{
            return false;
        }
    }


    double epsSearchInstance::get_cpd_distance(  int goal_id,int start_id){
        // retrieve path from goal to start using CPD
        auto retrieve_next_move = [&](const int& source, const int& target) {
            if(source == target){
                return -1;
            }
            const int& first_move = cpd->get_first_move(source, target);
            return  first_move-2;
        };

        const vector<Vertex>& mesh_vertices  = mesh->mesh_vertices;
        const vector<int>& cpd_mapper  = mesh->cpd_to_vertices_mapper;
        const int& goal_v_id =  cpd_mapper[goal_id];
        const int& start_v_id =  cpd_mapper[start_id];


        int cur_id = goal_id;
        if(distance_cache_id[goal_v_id] == search_id){
            return distance_cache[goal_v_id];
        }

        int next_move = retrieve_next_move (start_id,cur_id);
        if(next_move == -2){
            return -1;
        }else if(next_move != -1){
            int first_id = mesh->cpd_out_vertices[start_id][next_move];
            const int& first_v_id =  mesh->cpd_to_vertices_mapper[first_id];
            if(!is_taut_path(start,mesh_vertices[start_v_id],mesh_vertices[first_v_id])){
                return -1;
            }
        }else {
            // co-visible
            return mesh_vertices[start_v_id].p.distance(mesh_vertices[goal_v_id].p);
        }

        int index = 0;
        path_pool[index] = goal_v_id;
        distance_pool[index] = 0;
        index ++;
        double final_distance = 0;
        bool reached = false;
        while (!reached){
            next_move = retrieve_next_move ( cur_id,start_id);
            if(next_move == -2){
                return -1;
            }else if(next_move == -1){
                // co-visible or reach start
                distance_pool[index-1] =  mesh_vertices[start_v_id].p.distance(mesh_vertices[cpd_mapper[cur_id]].p);
                reached = true;
                index --;
            }else{
                distance_pool[index-1]  = mesh->cpd_distance_cost[cur_id][next_move];
                cur_id = mesh->cpd_out_vertices[cur_id][next_move];
                assert(index >= 0);
                const int& cur_v_id = cpd_mapper[cur_id];
                if(index == 1){
                    if(!is_taut_path(goal,mesh_vertices[goal_v_id],mesh_vertices[cur_v_id])){
                        return -1;
                    }
                }
                if(distance_cache_id[cur_v_id] == search_id){
                    final_distance = distance_cache[cur_v_id];
                    reached = true;
                }
                path_pool[index] = cur_v_id;
                distance_pool[index] = 0;
            }
            index++;
        }
        assert(index >= 0);
        index = index -1;
        while(index >= 0){
            assert(index<distance_pool.size() && index>=0);
            final_distance = final_distance + distance_pool[index];
            const int& vertex_id = path_pool[index];
            assert(vertex_id<distance_cache_id.size() && vertex_id >= 0);
            distance_cache_id[vertex_id] = search_id;
            distance_cache[vertex_id] = final_distance;
            index--;
        }
        return final_distance;

    }



    int epsSearchInstance::get_cpd_path( int start_id, int goal_id, vector<Point>& path){
        auto retrieve_next_move = [&](const int& source, const int& target) {
            const int& first_move = cpd->get_first_move(source, target);
            if(source == target){
                return -1;
            }
            return  first_move-2;
        };
        int number_of_v = 0;
        bool reached = false;
        int cur_id = start_id;
        path[number_of_v] = start;
        number_of_v ++;
        const vector<Vertex>& mesh_vertices  = mesh->mesh_vertices;
        const vector<int>& mapper  = mesh->cpd_to_vertices_mapper;
//        Point current = mesh->cpd_list[start_id];
        path[number_of_v] = mesh_vertices[mapper[start_id]].p;
        number_of_v ++;
        while (!reached){
            int next_move = retrieve_next_move (cur_id,goal_id);
            if(next_move == -2){
                return -1;
            }else if(next_move == -1){
                path[number_of_v] =mesh_vertices[mapper[goal_id]].p;
                number_of_v ++;
                reached = true;
            }else{
                cur_id = mesh->cpd_out_vertices[cur_id][next_move];
                path[number_of_v] = mesh_vertices[mapper[cur_id]].p;
                number_of_v ++;

            }
        }
        path[number_of_v] = goal;
        number_of_v ++;
        return  number_of_v;
    }






#undef root_to_point

}