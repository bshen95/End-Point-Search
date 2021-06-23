#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include <id_queue.h>
#include <constants.h>
#include <timestamp_flag.h>
#include <vector>
#include <iostream>
#include <set>
#include <graph.h>
#include <consts.h>

using namespace std;
namespace polyanya{
class Dijkstra{
public:

    Dijkstra(const Graph* g){
        graphPtr = g;
        number_of_vertices = g->number_of_vertices;
        queue = MinIDQueue(number_of_vertices);
        tentative_distance = std::vector<double>(number_of_vertices);
        predecessor_arc = std::vector<int>(number_of_vertices);
        was_pushed = TimestampFlags(number_of_vertices);

        //for building cpd
        first_move = vector<set<int>>(number_of_vertices);
    }
	Dijkstra&reset(){
		queue.clear();
		return *this;
	}

    void set_ordering( vector<int> order){
        ordering = order;

    }

    const vector<set<int>>& run_single_source_dijkstra(const int source){

            was_pushed.reset_all();
            queue.clear();
            for(int i = 0; i < number_of_vertices; i++){
                set<int> move = set<int>();
                first_move[i] = move;
            }
            fill(first_move.begin(), first_move.end(), set<int>());



            tentative_distance[source] = 0;
            was_pushed.set(source);


            // add first move start from 2;
            int fm = 2 ;

            for(unsigned a= graphPtr->vertices[source]; a<graphPtr->vertices[source+1]; ++a){
                double next_cost = graphPtr->distance_cost[a];
                int next = graphPtr->out_vertices[a];

                tentative_distance[next] = next_cost;
                first_move[next].insert(fm);
                was_pushed.set(next);
                queue.push({ next, next_cost});
                fm++;
            }



            while(!queue.empty()){
                auto x = queue.pop();
                for(unsigned a= graphPtr->vertices[x.id]; a<graphPtr->vertices[x.id+1]; ++a){
                    double next_cost = x.key + graphPtr->distance_cost[a];
                    int next = graphPtr->out_vertices[a];
                    if(was_pushed.is_set(next)){
                        if( next_cost < tentative_distance[next]){
                            tentative_distance[next] = next_cost;
                            queue.decrease_key({next,next_cost});
                            first_move[next] = first_move[x.id];
                        }else if(fabs(next_cost- tentative_distance[next]<EPSILON)){
                            for(int i :first_move[x.id]){
                                first_move[next].insert(i);
                            }
                        }
                    }else{
                        queue.push({ next, next_cost});
                        tentative_distance[next] = next_cost;
                        was_pushed.set(next);
                        first_move[next] = first_move[x.id];
                    }

                }

            }

            //non-reachable
            for (int i = 0; i < number_of_vertices; i++){
                if(!was_pushed.is_set(i)){
                    set<int> move = set<int>();
                    move.insert(0);
                    first_move[i] = move;
                }
            }
            //euclidean
            for(unsigned a= graphPtr->vertices[source]; a<graphPtr->vertices[source+1]; ++a){
                int next = graphPtr->out_vertices[a];
                first_move[next].insert(1);
            }

            first_move[source] =  set<int>();




            return first_move;




        }


    vector<double> get_all_distance(const int source){

        was_pushed.reset_all();
        queue.clear();
        fill( tentative_distance.begin(), tentative_distance.end(), -1);

        tentative_distance[source] = 0;
        was_pushed.set(source);


        for(unsigned a= graphPtr->vertices[source]; a<graphPtr->vertices[source+1]; ++a){
            double next_cost = graphPtr->distance_cost[a];
            int next = graphPtr->out_vertices[a];
            tentative_distance[next] = next_cost;
            was_pushed.set(next);
            queue.push({ next, next_cost});
        }



        while(!queue.empty()){
            auto x = queue.pop();
            for(unsigned a= graphPtr->vertices[x.id]; a<graphPtr->vertices[x.id+1]; ++a){
                double next_cost = x.key + graphPtr->distance_cost[a];
                int next = graphPtr->out_vertices[a];
                if(was_pushed.is_set(next)){
                    if( next_cost < tentative_distance[next]){
                        tentative_distance[next] = next_cost;
                        queue.decrease_key({next,next_cost});
                    }
                }else{
                    queue.push({ next, next_cost});
                    tentative_distance[next] = next_cost;
                    was_pushed.set(next);
                }

            }

        }
        return tentative_distance;
    }



    vector<int> get_shortest_path_tree(const int source){
        was_pushed.reset_all();
        queue.clear();
        fill( predecessor_arc.begin(), predecessor_arc.end(), -1);

        tentative_distance[source] = 0;
        was_pushed.set(source);
        predecessor_arc[source] = -1;

        for(unsigned a= graphPtr->vertices[source]; a<graphPtr->vertices[source+1]; ++a){
            double next_cost = graphPtr->distance_cost[a];
            int next = graphPtr->out_vertices[a];
            tentative_distance[next] = next_cost;
            was_pushed.set(next);
            queue.push({ next, next_cost});
            predecessor_arc[next] = source;
        }



        while(!queue.empty()){
            auto x = queue.pop();
            for(unsigned a= graphPtr->vertices[x.id]; a<graphPtr->vertices[x.id+1]; ++a){
                double next_cost = x.key + graphPtr->distance_cost[a];
                int next = graphPtr->out_vertices[a];
                if(was_pushed.is_set(next)){
                    if( next_cost < tentative_distance[next]){
                        tentative_distance[next] = next_cost;
                        queue.decrease_key({next,next_cost});
                        predecessor_arc[next] = x.id;
                    }
                }else{
                    queue.push({ next, next_cost});
                    tentative_distance[next] = next_cost;
                    was_pushed.set(next);
                    predecessor_arc[next] = x.id;
                }

            }

        }
        return predecessor_arc;
    }

private:
    std::vector<double>tentative_distance;
    std::vector<int>predecessor_arc;
    const Graph* graphPtr;
    MinIDQueue queue;
    TimestampFlags was_pushed;

    int number_of_vertices;

    vector<set<int>> first_move;

    vector<int> ordering;
};


}

#endif
