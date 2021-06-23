//
// Created by Bojie Shen on 13/12/19.
//

#include "epsSearchInstance.h"
#include "expansion.h"

namespace polyanya {
    typedef Mesh* MeshPtr;
    typedef cpd *CPDPtr;
// Polyanya instance for point to point search
    class eps
    {
    private:
        epsSearchInstance* start_search;
        epsSearchInstance* goal_search;
        MeshPtr mesh;
        warthog::timer timer;
    public:

        Point start, goal;
        vector<int >start_vertices,goal_vertices;
        vector<double >start_vertices_distance,goal_vertices_distance;
        int start_number;
        int goal_number;
        double shortest_path;
        vector<Point >optimial_path;
        int best_start;
        int best_goal;
        int number_of_vertice;

        CPDPtr cpd;
        eps() = default;
        eps(MeshPtr m , CPDPtr c) : start_search(new epsSearchInstance(m, c)), goal_search(new epsSearchInstance(m, c)), mesh(m) { init();cpd = c; }
        eps(eps const &) = delete;
        void operator=(eps const &x) = delete;

        void init(){
            timer = warthog::timer();
            start_vertices = vector<int>(500);
            goal_vertices = vector<int>(500);
            start_vertices_distance = vector<double>(500);
            goal_vertices_distance = vector<double>(500);
            optimial_path = vector<Point>(500);
        }

        void set_start_goal(Point s, Point g)
        {
            start = s;
            goal = g;
        }




        bool search(){
            // To do: Maybe we should add visibility testing between s and t, if it's visible, then we dont need to run search.
            // Anyway, up to u programmer!.
            number_of_vertice = 0;
            shortest_path = INF;
            start_number =0 ;
            goal_number = 0;
            best_goal = -1;
            best_start = -1;

            timer.start();
            start_search->set_shortest_path(shortest_path);
            goal_search->set_shortest_path(shortest_path);
            start_search->set_start_goal(start,goal);
            goal_search->set_start_goal(goal,start);
            const PointLocation& start_pl = get_point_location_in_search(start, mesh, false);
            const PointLocation& goal_pl = get_point_location_in_search(goal, mesh, false);



            vector<int>current_vertices = vector<int>(2);
            start_search->initial_search(start_pl,goal_pl);
            if (start_search->end_polygon == -1)
            {
                timer.stop();
                return false;
            }

            if (start_search->final_node != nullptr)
            {
                shortest_path = start.distance(goal);
                timer.stop();
                return true;
            }




            goal_search->initial_search(goal_pl,start_pl);
            if (goal_search->end_polygon == -1)
            {
                timer.stop();
                return false;
            }
            bool finish_search = false;
            int n_start = 2;
            int n_goal = 2 ;

            for (int  p =0 ; p < start_search->start_index;p++){
                int current_v =  start_search->start_vertices[p];
                const Vertex &start_v = mesh->mesh_vertices[current_v];
                double D_start = start_v.p.distance(start);
                for (int i =0 ; i < goal_search->start_index;i++){
                    int goal_vid =  goal_search->start_vertices[i];
                    const Vertex &goal_v = mesh->mesh_vertices[goal_vid];

                    double D_goal = goal_v.p.distance(goal);
                    if (D_start + D_goal + start_v.p.distance(goal_v.p) > shortest_path) {
                        continue;
                    }
                    const double &cpd_distance = start_search->get_cpd_distance( goal_v.cpd_id,start_v.cpd_id);
                    if (cpd_distance != -1) {
                        double path_distance = D_start + D_goal + cpd_distance;
                        if (path_distance < shortest_path) {
                            best_goal = goal_v.cpd_id;
                            best_start =  start_v.cpd_id;
                            shortest_path = path_distance;
                        }
                    }
                }
                start_search->search_id++;
                start_vertices[start_number] = current_v;
                start_vertices_distance[start_number] = D_start;
                start_number ++;
            }


            for (int i =0 ; i < goal_search->start_index;i++){
                int goal_vid =  goal_search->start_vertices[i];
                goal_vertices[goal_number] = goal_vid;
                goal_vertices_distance[goal_number] = mesh->mesh_vertices[goal_vid].p.distance(goal);
                goal_number ++;
            }

            start_search->set_shortest_path(shortest_path);
            goal_search->set_shortest_path(shortest_path);
            while(!finish_search){

                if(n_start != -1) {
                    n_start = start_search->get_visible_vertices(current_vertices);
                    if (n_start == 0) {

                        timer.stop();
                        shortest_path = start.distance(goal);
                        break;
                    } else if (n_start > 0) {
                        while (n_start > 0) {
                            n_start--;
                            int current_v = current_vertices[n_start];
                            const Vertex &start_v = mesh->mesh_vertices[current_v];

                            double D_start = start_v.p.distance(start);
                            for(int i = 0; i<goal_number;i++){
                                int goal_vid = goal_vertices[i];
                                const Vertex &goal_v = mesh->mesh_vertices[goal_vid];

                                double D_goal = goal_vertices_distance[i];
                                if (D_start + D_goal + start_v.p.distance(goal_v.p) > shortest_path) {
                                    continue;
                                }
                                const double &cpd_distance = start_search->get_cpd_distance( goal_v.cpd_id,start_v.cpd_id);
                                if (cpd_distance != -1) {
                                    double path_distance = D_start + D_goal + cpd_distance;
                                    if (path_distance < shortest_path) {
                                        best_goal = goal_v.cpd_id;
                                        best_start =  start_v.cpd_id;
                                        shortest_path = path_distance;
                                    }
                                }
                            }
                            start_search->search_id++;
                            start_vertices[start_number] = current_v;
                            start_vertices_distance[start_number] = D_start;
                            start_number ++;
                        }
                    }
                    start_search->set_shortest_path(shortest_path);
                }



                if(n_goal != -1) {
                    n_goal = goal_search->get_visible_vertices( current_vertices);
                    if (n_goal == 0) {
                        timer.stop();
                        shortest_path = start.distance(goal);
                        break;
                    } else if (n_goal > 0) {
                        while (n_goal > 0) {
                            n_goal--;
                            int current_v = current_vertices[n_goal];
                            const Vertex &start_v = mesh->mesh_vertices[current_v];
                            double D_start = start_v.p.distance(goal);
                            for(int i = 0; i<start_number;i++){
                                int goal_vid = start_vertices[i];
                                const Vertex &goal_v = mesh->mesh_vertices[goal_vid];
                                double D_goal = start_vertices_distance[i];
                                if (D_start + D_goal + start_v.p.distance(goal_v.p) > shortest_path) {
                                    continue;
                                }
                                const double &cpd_distance = goal_search->get_cpd_distance( goal_v.cpd_id,start_v.cpd_id
                                );
                                if (cpd_distance != -1) {
                                    double path_distance = D_start + D_goal + cpd_distance;
                                    if (path_distance < shortest_path) {
                                        best_goal = start_v.cpd_id;
                                        best_start =  goal_v.cpd_id;
                                        shortest_path = path_distance;
                                    }
                                }
                            }
                            goal_search->search_id++;
                            goal_vertices[goal_number] = current_v;
                            goal_vertices_distance[goal_number] = D_start;
                            goal_number ++;
                        }
                    }
                    goal_search->set_shortest_path(shortest_path);
                }

                if(n_goal == -1 && n_start == -1){
                    if(best_goal != -1 ||  best_start != -1) {
                        number_of_vertice = start_search->get_cpd_path(best_start, best_goal, optimial_path);
                    }
                    timer.stop();
                    finish_search = true;
                }
            }
            return shortest_path != INF;
        }


        double get_cost(){
            if(shortest_path == INF){
                return -1;
            }else{
                if(number_of_vertice !=0) {
                    Point current = optimial_path[0];
                    double distance = 0;
                    for (int i = 1; i < number_of_vertice; i++) {
                        distance = distance + optimial_path[i].distance(current);
                        current = optimial_path[i];
                    }
                    if (fabs(distance - shortest_path)>EPSILON) {
                        std::cout << distance << std::endl;
                        std::cout << shortest_path << std::endl;
                    }
                }
            }
            return shortest_path;
        }

        double get_search_micro()
        {
            return timer.elapsed_time_micro();
        }

        double get_search_nano()
        {
            return timer.elapsed_time_nano();
        }


        void print_path(){
            for (int i = 0; i < number_of_vertice; i++) {
                std::cout<<optimial_path[i]<<std::endl;
            }
        }

        vector<Point > get_path (){
            return optimial_path;
        }

    };


}