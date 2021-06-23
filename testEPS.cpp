//
// Created by Bojie Shen on 13/12/19.
//

//
// Created by Bojie Shen on 28/11/19.
//
#include <stdio.h>
#include <iostream>
#include <expansion.h>
#include <searchinstance.h>
#include <eps.h>
#include "point.h"
#include "cpd.h"
#include <iomanip>
#include "consts.h"
#include "scenario.h"

using namespace std;
namespace pl = polyanya;
pl::MeshPtr mp;
pl::CPDPtr cpd;
pl::eps* EPS;
pl::SearchInstance* si;

int benchmark_eps(  string dir_name,string map_name, const std::vector<double>& pcost){

    polyanya::vector<polyanya::Scenario> out;
    string mesh_path = "dataset/merged-mesh/"+dir_name+"/"+map_name +"-merged.mesh";
    ifstream meshfile(mesh_path);
    mp = new pl::Mesh(meshfile);


    // mark obstacle edge;
    mp->pre_compute_obstacle_edge_on_vertex();

    // We use grid map to quickly identify convex vertices. You may need other approach if you dont have grid map.
    string grid_path = "dataset/grid/"+dir_name+"/"+map_name +".map";
    mp->mark_turning_point(grid_path.c_str());

    string vertices_mapper_path ="dataset/visibility_graph/" +dir_name+"/"+map_name +".vMapper";
    string cpd_mapper_path ="dataset/cpd/" +dir_name+"/"+map_name +".mapper";


    string visibility_graph_path ="dataset/visibility_graph/" +dir_name+"/"+map_name +".vis";
    pl::Graph g = pl::Graph();
    g.load_graph(visibility_graph_path);



    vector<int> vertices_mapper = load_vector<int>(vertices_mapper_path);
    vector<int> cpd_mapper = load_vector<int>(cpd_mapper_path);
    mp->create_cpd_to_vertices_mapper(vertices_mapper,cpd_mapper);
    mp->create_cpd_out_list(g,cpd_mapper);

    cpd = new pl::cpd();
    string CPD_path = "dataset/cpd/" +dir_name+"/"+map_name +".cpd";
    FILE*f = fopen(CPD_path.c_str(), "r");
    cpd->load(f);



    int max = 0;
    for(std::vector<int> v: mp->cpd_out_vertices){
        if(v.size()>max){
            max = v.size();
        }

    }
    if(max > 4096){
        std::cout<< "Warning Size too big !: "<<max << std::endl;
    }

    string scenario_path = "dataset/scenarios/"+dir_name+"/"+map_name +".map.scen";
    ifstream scenariofile(scenario_path);
    polyanya::load_scenarios(scenariofile, out);


    EPS = new pl::eps(mp, cpd);
    std::vector<double> cpdtime ( out.size(), 0);
    std::vector<double> cost (out.size(),0);

    si = new pl::SearchInstance(mp);
    warthog::timer timer =  warthog::timer ();

    int runtimes = 5;
    for ( int i = 0; i < runtimes; i++) {
        int count =0;
//        const polyanya::Scenario& s = out[10];
        for (const polyanya::Scenario& s : out) {


            EPS->set_start_goal(s.start,s.goal);
            timer.start();
            EPS->search();
            timer.stop();
            double eps_search_cost =  EPS->get_cost();
            cost[count] = eps_search_cost;
            double polyanya_search_cost = pcost[count];

            if (fabs(eps_search_cost - polyanya_search_cost)>EPSILON) {
                // test results with polyanya;
                std::cout << "eps cost error" << std::endl;
                std::cout << setprecision(8) << eps_search_cost << std::endl;
                std::cout << setprecision(8) << polyanya_search_cost << std::endl;
                break;
            }
            cpdtime[count] =  cpdtime[count] + timer.elapsed_time_nano();
            count++;

        }
    }

    double total = 0 ;
    for(double time  :cpdtime){
        total = total +time;
    }
    std::cout<<"EPS: "<< "average time: "<<total/out.size()/runtimes<<" nano secs" <<std::endl;

    delete mp;
    delete EPS;
    delete cpd;

    return 0;
}


int benchmark_polyanya(  string dir_name, string map_name, std::vector<double>& pathcost){

    polyanya::vector<polyanya::Scenario> out;

    string mesh_path = "dataset/merged-mesh/"+dir_name+"/"+map_name +"-merged.mesh";
    ifstream meshfile(mesh_path);
    mp = new pl::Mesh(meshfile);
    string scenario_path = "dataset/scenarios/"+dir_name+"/"+map_name +".map.scen";
    ifstream scenariofile(scenario_path );
    polyanya::load_scenarios(scenariofile, out);

    si = new pl::SearchInstance(mp);

    std::vector<double> polytime;
    polytime.resize(out.size());
    fill(polytime.begin(), polytime.end(), 0);

    pathcost.resize(out.size());
    fill( pathcost.begin(),  pathcost.end(), 0);
    warthog::timer timer =  warthog::timer ();
    int runtimes = 5 ;
    for ( int i = 0; i < runtimes; i++) {
        int count = 0;
        for (const polyanya::Scenario& s : out) {
            si->set_start_goal(s.start,s.goal);
            timer.start();
            si->search();
            timer.stop();
            double poly_search_cost =  si->get_cost();
            polytime[count] = polytime[count]+ timer.elapsed_time_nano();
            pathcost[count] =  poly_search_cost;
            count ++;
        }
    }

    double total = 0 ;
    for(double time  :polytime){
        total = total +time;
    }
    std::cout<<"polyanya: average time: "<<total/out.size()/runtimes <<" nano secs" <<std::endl;

    delete mp;
    delete si;

    return 0;
}


void  benchmark_map(  string dir_name, string map_name){
    std::cout<<"benchmark: "<< dir_name <<" "<< map_name <<std::endl;
    std::vector<double> pathcost;


    benchmark_polyanya(dir_name,map_name,pathcost);
    benchmark_eps(dir_name,map_name,pathcost);
    std::cout <<std::endl;
    std::cout<<std::endl;
}


int main(int argc, char* argv[]) {
    if (argc > 1) {
        string dir_name = string (argv[1]);
        string map_name = string(argv[2]);
        benchmark_map(dir_name, map_name);

    }
}
