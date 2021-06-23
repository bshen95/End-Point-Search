//
// Created by Bojie Shen on 6/4/21.
//


#include <iostream>
#include <graph.h>
#include <timer.h>
#include <dijkstra.h>
#include <cpd.h>
#include <omp.h>

namespace pl = polyanya;


double cpd_search(int start_id, int goal_id, const polyanya::Graph& g,const polyanya::cpd* cpd){

        auto retrieve_next_move = [&](const int& source, const int& target) {
            if(source == target){
                return -1;
            }
            const int& first_move = cpd->get_first_move(source, target);
            return  first_move-2;
        };
        bool reached = false;
        int cur_id = start_id;

        double distance = 0;

        while (!reached){
            int next_move = retrieve_next_move (cur_id,goal_id);

            if(next_move == -2){
                return -1;
            }else if(next_move == -1){
                int arc_id = -1;
                if( cur_id != goal_id){
                    for(int a  = g.vertices[cur_id]; a < g.vertices[cur_id+1]; ++a){
                        if(g.out_vertices[a] == goal_id){
                            arc_id = a;
                            break;
                        }
                    }
                    if( arc_id == -1){
                        std::cout<<"error"<<std::endl;
                    }
                    distance = distance + g.distance_cost[arc_id];
                }

                reached = true;
            }else{
                distance = distance + g.distance_cost[g.vertices[cur_id] +next_move];
                cur_id = g.out_vertices[g.vertices[cur_id]+next_move];

            }
        }
        return distance;



}



void test_cpd(const polyanya::Graph& g,const polyanya::cpd* cpd){

    pl::Dijkstra dij = pl::Dijkstra(&g);

    for(int i = 0; i < g.number_of_vertices; i ++){

        for(int j = 0; j < g.number_of_vertices; j ++){
           const vector<double>& dis = dij.get_all_distance(i);
            double dij_distance =dis[j];
            double cpd_distance = cpd_search(i,j,g,cpd);
            if(fabs(cpd_distance - dij_distance) > EPSILON*10){
                double distance = cpd_search(i,j,g,cpd);
                std::cout<<"error"<<std::endl;
            }

        }

    }
    std::cout<<"passed"<<std::endl;

}



void construct_cpd(const string& input_file, const string& output_file) {


    pl::Graph g = pl::Graph();
    g.load_graph(input_file);
    // only need free flow cost;
    warthog::timer timer1 = warthog::timer();
    timer1.start();
    std::cout<<"Loading visibility graph ...."<<std::endl;
    vector<int> dfs_ordering =g.generate_DFS_ordering();
    g.resort_graph(dfs_ordering);

    cout << "Building CPD ... " << flush;
    std::vector<pl::Dijkstra*> dijkstra;
    for (int i = 0; i < omp_get_max_threads(); i ++){
        dijkstra.push_back(new pl::Dijkstra(&g));
    }


    unsigned number_of_nodes = g.number_of_vertices;
    pl::cpd* cpd = new pl::cpd();
    {
        {
            pl::Dijkstra dij(&g);
            dij.set_ordering(dfs_ordering);
            warthog::timer t;
            t.start();
            dij.run_single_source_dijkstra(0);
            t.stop();
            double tots = t.elapsed_time_micro()*number_of_nodes / 1000000;
            printf("Estimated sequential running time : %fmin\n", tots / 60.0);

        }

        printf("Using %d threads\n", omp_get_max_threads());
        std::vector<pl::cpd>thread_cpd(omp_get_max_threads());

        int progress = 0;

#pragma omp parallel
        {
            const int thread_count = omp_get_num_threads();
            const int thread_id = omp_get_thread_num();
//            const int begin_int  = 0;
            const int node_count = number_of_nodes;

            int node_begin = (node_count*thread_id) / thread_count ;
            int node_end = (node_count*(thread_id+1)) / thread_count ;
            pl::Dijkstra& thread_dij = *dijkstra[thread_id];

            for(int source_node=node_begin; source_node < node_end; ++source_node){
//                thread_dij.full_search(source_node,weight);
//                const std::vector<unsigned>& result =thread_dij.get_first_move_table(source_node);
//                thread_cpd[thread_id].append_row(source_node,result);

                thread_cpd[thread_id].append_row(source_node,thread_dij.run_single_source_dijkstra(source_node));
#pragma omp critical
                {
                    ++progress;
                    if(progress % 100 == 0) {
                        double ratio = (double)progress / number_of_nodes * 100.0;
                        std::cout << "Progress: [" << progress << "/" << number_of_nodes << "] "
                                  << std::setprecision(3) << ratio << "% \r";
                        std::cout.flush();
                    }
                }
            }
        }

        for(auto&x:thread_cpd)
            cpd->append_rows(x);
    }
    timer1.stop();
    for (auto i: dijkstra) delete i;


    printf("Saving data to cpd.txt \n");
    printf("begin size: %d, entry size: %d\n", cpd->entry_count(), cpd->get_entry_size());
    std::string fname = output_file+ ".cpd";
    FILE*f = fopen(fname.c_str(), "wb");
    cpd->save(f);
    fclose(f);
    std::cout.flush();
    std::cout<<"done"<<std::endl;

    string mapper_file = output_file +".mapper";
    cout<<"Saving mapper:"<< mapper_file<< endl;

    vector<int> mapper = pl::invert_permutation(dfs_ordering);
    save_vector(mapper_file,mapper);
    std::cout<<std::endl;
//    test_cpd(g,cpd);



}






int main(int argc, char*argv[]){

    try{
        const char *input_file;
        const char *output_file;

        if(argc != 3){
            cerr << argv[0] << "input_file output_file" << endl;
            return 1;
        }else{
            input_file = argv[1];
            output_file = argv[2];
        }


        construct_cpd(input_file,output_file);

    }catch(exception&err){
        cerr << "Stopped on exception : " << err.what() << endl;
    }
}