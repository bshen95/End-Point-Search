#pragma once
#include <vector>
#include <set>
#include <algorithm>
#include <string>
#include "binary_search.h"
#include "range.h"
#include "vec_io.h"

namespace polyanya {

//    #include "adj_graph.h"
//#include "mapper.h"
//#include "rect_wildcard.h"

    using namespace std;
//! Compressed Path database. Allows to quickly query the first out arc id of
//! any shortest source-target-path. There may be at most 15 outgoing arcs for
//! any node.
    class cpd {
    public:
        cpd(): begin{0}{}

        //! Adds a new node s to the CPD. first_move should be an array that
        //! maps every target node onto a 15-bit bitfield that has a bit set
        //! for every valid first move. get_first_move is free to return any of
        //! them.
        void append_row(int source_node, const vector<set<int>> &first_move);

        void append_rows(const cpd&other);
        //! Get the first move.
        //! An ID of 0xF means that there is no path.
        //! If source_node == target_node then return value is undefined.
        int  get_first_move(int source_node, int target_node)const{
            assert(source_node != -1);
            assert(target_node != -1);
            target_node <<= 12;
            target_node |= 0xFFF;
            return *binary_find_last_true(
                    entry.begin() + begin[source_node],
                    entry.begin() + begin[source_node+1],
                    [=](int x){return x <= target_node;}
            )&0xFFF;
        }

        int node_count() const{
            // get the number of rows
            // in inverse centroid cpd, this returns the number of centroids.
            return begin.size()-1;
        }

        int entry_count()const{
            return entry.size();
        }

        friend bool operator==(const cpd&l, const cpd&r){
            return l.begin == r.begin && l.entry == r.entry;
        }

        friend bool operator!=(const cpd&l, const cpd&r){
            return !(l == r);
        }
        void save(std::FILE*f)const{
            save_vector(f, begin);
            save_vector(f, entry);
        }

        void load(std::FILE*f){
            begin = load_vector<int>(f);
            entry = load_vector<int>(f);
        }

        int get_entry_size() {
            return entry.size();
        }
        const vector<int>& get_entry() const {
            return entry;
        }

        const vector<int>& get_begin() const {
            return begin;
        }

        pair<int,set<int>> getIntersection(set<int>& s1, const set<int>& s2) {
            set<int> intersect = set<int>();
            if(s2.empty()&& s1.empty()){
                return pair<int,set<int>>(1,intersect);
            }else if(s2.empty()){
                return pair<int,set<int>>(1,s1);
            }else if(s1.empty()){
                return pair<int,set<int>>(1,s2);
            }
            set_intersection(s1.begin(),s1.end(),s2.begin(),s2.end(),
                             std::inserter(intersect,intersect.begin()));
            return pair<int,set<int>>(0,intersect);
        }
        void append_vertices_list(const std::vector<int>& v_list){
            vertices_list.push_back(v_list);
        }
        std::vector<std::vector<int>> get_vertices_list(){
            return vertices_list;
        }

        void append_distance_list(const std::vector<double>& dis_list){
            distance_list.push_back(dis_list);
        }
        std::vector<std::vector<double>> get_distance_list(){
            return distance_list;
        }

    protected:
        std::vector<int>begin;
        std::vector<int>entry;
        std::vector<std::vector<int>>vertices_list;
        std::vector<std::vector<double>>distance_list;


        const set<int>& get_allowed(int x, int s, const vector <set<int>> &fmoves) const;
    };


}