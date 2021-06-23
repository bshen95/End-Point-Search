//
// Created by Bojie Shen on 22/10/20.
//
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <expansion.h>
#include <searchinstance.h>
#include "visibleSearchInstance.h"
#include "point.h"
#include <omp.h>
#include <iomanip>
#include "geometry.h"

using namespace std;
namespace pl = polyanya;
pl::MeshPtr mp;
pl::visibleSearchInstance* vs;
pl::SearchInstance* si;

void output_graph(const std::vector<vector<int>>& visibility_graph, const std::vector<vector<double>>& graph_weight,
        const std::vector<pl::Point>& coordinate,const std::vector<int>& vertice_mapper,string output_file)
{
    std::cout<<"Saving graph to"<<output_file<<std::endl;
    std::ofstream outputFile(output_file);
    outputFile <<coordinate.size()<<"\n";
    int number_of_edges = 0 ;
    for(const vector<int>& vertices : visibility_graph){
        for(const int & edge : vertices){
            number_of_edges ++;
        }
    }
    outputFile <<number_of_edges<<"\n";
    for(int v_id = 0; v_id < visibility_graph.size(); v_id ++){
        for(int e_id= 0; e_id < visibility_graph[v_id].size(); e_id ++){
            outputFile<<std::fixed<<setprecision(8) <<v_id<<" "<<visibility_graph[v_id][e_id]<<" "<<graph_weight[v_id][e_id]<<"\n";
        }
    }
    outputFile.close();

    size_t lastindex = output_file.find_last_of(".");
    string rawname = output_file.substr(0, lastindex);

    std::ofstream coordinateFile(rawname+".cor");
    int id = 0;
    for(pl::Point cor:coordinate){
        coordinateFile<<std::fixed<<setprecision(8)<<id<<" "<<cor.x<<" "<<cor.y<<"\n";
        id++;
    }

    save_vector(rawname+".vMapper",vertice_mapper);
    std::cout<<"done"<<std::endl;
}
void build_visiblity_graph(string mesh_path, string grid_path, string output_path){
    std::cout<<"Building visibility graph ..."<<std::endl;
    ifstream meshfile(mesh_path);
    mp = new pl::Mesh(meshfile);
    // mark obstacle edge;
    mp->pre_compute_obstacle_edge_on_vertex();
    // mark valid turning point;

    mp->mark_turning_point(grid_path.c_str());
    vector<pl::Point> turning_point;
    vector<int> turning_vertices;
    vector<pl::PointLocation> turning_vertices_location;

    int id = 0;
    int poly = 0;
    for( pl::Vertex& v : mp->mesh_vertices){
        if(v.is_turning_vertex && !v.is_ambig) {
            for (int polygon: v.polygons) {
                if (polygon != -1) {
//                    p.polygons.insert(polygon);
                    poly = polygon;
                }
            }
            // there is some issue here, old implementation assume that these vertex always inside one polygon only. it doesnt
            // use the correct type actually, I fix it here manually assign a random adjacent polygon
            pl::PointLocation location = {pl::PointLocation::ON_CORNER_VERTEX_UNAMBIG, poly, -1, id, -1};
            turning_vertices_location.push_back(location);
            turning_vertices.push_back(id);
            turning_point.push_back(mp->mesh_vertices[id].p);
        }
        id ++;
    }

    const vector<pl::Vertex>& mesh_vertices = mp->mesh_vertices;

    std::vector<pl::visibleSearchInstance*> visibility_search;
    for (int i = 0; i < omp_get_max_threads(); i ++){
        visibility_search.push_back(new pl::visibleSearchInstance(mp));
    }
    std::vector<vector<int>> visibility_graph;
    int number_of_vertices = turning_vertices.size();
    {
        {
            pl::visibleSearchInstance VS(mp);
            warthog::timer t;
            t.start();
            std::vector<int> visible_vertices;
            VS.search_visible_vertices(turning_vertices[0],turning_vertices_location[0],visible_vertices);
            t.stop();
            double tots = t.elapsed_time_micro()*number_of_vertices / 1000000 /12;
            printf("Estimated sequential running time : %fmin\n", tots / 60.0);
        }

        printf("Using %d threads\n", omp_get_max_threads());
        std::vector<std::vector<vector<int>>>thread_graph(omp_get_max_threads());

        int progress = 0;

#pragma omp parallel
        {
            const int thread_count = omp_get_num_threads();
            const int thread_id = omp_get_thread_num();
//            const int begin_int  = 0;
            const int node_count =number_of_vertices;

            int node_begin = (node_count*thread_id) / thread_count ;
            int node_end = (node_count*(thread_id+1)) / thread_count ;
            pl::visibleSearchInstance& thread_dij = *visibility_search[thread_id];

            for(int source_node=node_begin; source_node < node_end; ++source_node){

                int source = turning_vertices[source_node];
                pl::Point source_point = mesh_vertices[source].p;
                auto comp = [&](int v1,int v2)-> bool {
                    double d1 =source_point.distance(mesh_vertices[v1].p);
                    double d2 = source_point.distance(mesh_vertices[v2].p);
                    if(d1<d2){
                        return true;
                    }else if(d1 == d2){
                        return v1 < v2;
                    }else{
                        return false;
                    }
                };
                std::vector<int> visible_vertices;
                thread_dij.search_visible_vertices(
                        turning_vertices[source_node],turning_vertices_location[source_node],visible_vertices);
                if(!visible_vertices.empty()) {
                    // it's possible to be empty;
                    sort(visible_vertices.begin(), visible_vertices.end(), comp);
                    for (int cur = 0; cur < visible_vertices.size() - 1; cur++) {
                        vector<int> remove_list;
                        const pl::Point &cur_point = mesh_vertices[visible_vertices[cur]].p;
                        for (int next = cur + 1; next < visible_vertices.size(); next++) {
                            //remove the collinear edges;
                            const pl::Point &next_point = mesh_vertices[visible_vertices[next]].p;
                            if (get_orientation(source_point, cur_point, next_point) == pl::Orientation::COLLINEAR &&
                                source_point.distance(next_point) > cur_point.distance(next_point)) {
                                remove_list.push_back(next);
                            }
                        }
                        int index = 0;
                        for (int rm: remove_list) {
                            visible_vertices.erase(visible_vertices.begin() + rm - index);
                            index++;
                        }
                    }
                }
                thread_graph[thread_id].push_back(visible_vertices);


#pragma omp critical
                {
                    ++progress;
                    if(progress % 100 == 0) {
                        double ratio = (double)progress / number_of_vertices * 100.0;
                        std::cout << "Progress: [" << progress << "/" << number_of_vertices  << "] "
                                  << std::setprecision(3) << ratio << "% \r";
                        std::cout.flush();
                    }
                }
            }
        }

        for(auto&x:thread_graph){
            for(auto&y:x){
                visibility_graph.push_back(y);
            }
        }

    }
    for (auto i: visibility_search) delete i;
    
    vector<int> vertice_mapper(mesh_vertices.size());
    std::fill(vertice_mapper.begin(),vertice_mapper.end(),-1);
    for(int i = 0; i < turning_vertices.size(); i ++){
        vertice_mapper[turning_vertices[i]] = i;
    }
    std::vector<vector<double>> graph_weight(visibility_graph.size());
    for(int i = 0; i < visibility_graph.size(); i++){
        graph_weight[i].resize(visibility_graph[i].size());
        for(int j = 0; j < visibility_graph[i].size(); j++){
            double distance = mesh_vertices[turning_vertices[i]].p.distance(mesh_vertices[visibility_graph[i][j]].p);
            if(distance == 0){
               std::cout<<"distance should not be 0 "<< std::endl;
            }
            graph_weight[i][j]= distance;
            visibility_graph[i][j] = vertice_mapper[visibility_graph[i][j]];
        }
    }
    std::cout<<"done"<<std::endl;
    output_graph(visibility_graph,graph_weight,turning_point, vertice_mapper,output_path);
    std::cout<<std::endl;
}



void test_visiblity_graph(string mesh_path, string grid_path){
    // test with polyanya
    std::cout<<mesh_path<<std::endl;
    ifstream meshfile(mesh_path);
    mp = new pl::Mesh(meshfile);
    std::cout<<mesh_path<<std::endl;
    mp->pre_compute_obstacle_edge_on_vertex();
    mp->mark_turning_point(grid_path.c_str());
    vector<pl::Point> turning_point;
    vector<int> turning_vertex;
    vector<pl::PointLocation> turning_vertex_location;

    int id = 0;
    int poly = 0;
    for( pl::Vertex& v : mp->mesh_vertices){
        if(v.is_turning_vertex && !v.is_ambig) {
            for (int polygon: v.polygons) {
                if (polygon != -1) {
//                    p.polygons.insert(polygon);
                    poly = polygon;
                }
            }
            // there is some issue here, old implementation assume that these vertex always inside one polygon only. it doesnt
            // use the correct type actually, I fix it here manually assign a random adjacent polygon
            pl::PointLocation location = {pl::PointLocation::ON_NON_CORNER_VERTEX, poly, -1, id, -1};
            turning_vertex_location.push_back(location);
            turning_vertex.push_back(id);
            turning_point.push_back(mp->mesh_vertices[id].p);
        }
        id ++;
    }
    vs = new pl::visibleSearchInstance(mp);
    si = new pl::SearchInstance(mp);
    for(int i =0 ; i < turning_vertex.size(); i ++) {
        std::cout<<"test: "<<i<<std::endl;
        vector<int> visible_vertices;
        int source = turning_vertex[i];
        pl::Point source_point = mp->mesh_vertices[source].p;
        //sorted based on distance
        auto comp = [&](int v1,int v2)-> bool {
            double d1 =source_point.distance(mp->mesh_vertices[v1].p);
            double d2 = source_point.distance(mp->mesh_vertices[v2].p);
            if(d1<d2){
                return true;
            }else if(d1 == d2){
                return v1 < v2;
            }else{
                return false;
            }
         };

        vs->search_visible_vertices(source, turning_vertex_location[i], visible_vertices);
        if(!visible_vertices.empty()) {
            sort(visible_vertices.begin(), visible_vertices.end(),comp);
            for(int cur = 0; cur < visible_vertices.size()-1;cur++ ){
                vector<int> remove_list;
                const pl::Point& cur_point = mp->mesh_vertices[visible_vertices[cur]].p;

                for(int next = cur+1; next < visible_vertices.size(); next ++){
                    const pl::Point& next_point = mp->mesh_vertices[visible_vertices[next]].p;

                    if(get_orientation(source_point,cur_point,next_point) == pl::Orientation::COLLINEAR && source_point.distance(next_point)>cur_point.distance(next_point)){
                        remove_list.push_back(next);
                    }
                }
                int index =0;
                for(int rm: remove_list){
                    visible_vertices.erase(visible_vertices.begin() + rm-index);
                    index++;
                }
            }
        }


        vector<int> visible_vertices2;
        for (int v_id: turning_vertex) {
            if (v_id == turning_vertex[i]) {
                continue;
            }
            si->set_start_goal(mp->mesh_vertices[source].p, mp->mesh_vertices[v_id].p);
            si->search();
            double cost = si->get_cost();
            double Euclidean_distance = mp->mesh_vertices[turning_vertex[i]].p.distance(mp->mesh_vertices[v_id].p);
            vector<pl::Point> out;
            si->get_path_points(out);
            if (cost == Euclidean_distance && out.size()==2 ) {
                visible_vertices2.push_back(v_id);
            }
        }
        if(!visible_vertices2.empty()) {
            sort(visible_vertices2.begin(), visible_vertices2.end(),comp);
            for(int cur = 0; cur < visible_vertices2.size()-1;cur++ ){
                vector<int> remove_list;
                const pl::Point& cur_point = mp->mesh_vertices[visible_vertices2[cur]].p;
                for(int next = cur+1; next < visible_vertices2.size(); next ++){
                    const pl::Point& next_point = mp->mesh_vertices[visible_vertices2[next]].p;

                    if(get_orientation(source_point,cur_point,next_point) == pl::Orientation::COLLINEAR && source_point.distance(next_point)>cur_point.distance(next_point)){
                        remove_list.push_back(next);
                    }
                }
                int index =0;
                for(int rm: remove_list){
                    visible_vertices2.erase(visible_vertices2.begin() + rm-index);
                    index++;
                }
            }
        }

        sort(visible_vertices.begin(), visible_vertices.end(),comp);
        sort(visible_vertices2.begin(), visible_vertices2.end(),comp);

        if (visible_vertices.size() != visible_vertices2.size()) {
            std::cout << "Error: size is different" << std::endl;
        }
        for (int j = 0; j < visible_vertices.size(); j++) {
            if (visible_vertices[j] != visible_vertices2[j]) {
                std::cout << "Error: vertex is different" << std::endl;
            }
        };
    }
};

int main(int argc, char*argv[]){

    try{
        string mesh_file;
        string grid_file;
        string output_file;

        if(argc != 4){
            cerr << argv[0] << "mesh_file grid_file output_file" << endl;
            return 1;
        }else{
            mesh_file = argv[1];
            grid_file = argv[2];
            output_file = argv[3];
        }
        build_visiblity_graph(mesh_file,grid_file,output_file);

    }catch(exception&err){
        cerr << "Stopped on exception : " << err.what() << endl;
    }
}