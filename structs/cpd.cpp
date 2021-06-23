#include "cpd.h"
#include <fstream>
#include <stdexcept>
#include <cassert>

// compile with -O3 -DNDEBUG
namespace polyanya {
    void cpd::append_row(
            int source_node, const vector<set<int>> & allowed_first_move) {

        auto get_allowed_local = [&](int x) {
            return get_allowed(x, source_node, allowed_first_move);
        };

        int node_begin = 0;
        set<int> allowed_up_to_now = get_allowed_local(0);

        for (int i = 1; i < (int) allowed_first_move.size(); ++i) {
            pair<int,set<int>> allowed_next = getIntersection(allowed_up_to_now , get_allowed_local(i));
            if (allowed_next.first == 0  && allowed_next.second.empty()) {
                entry.push_back((node_begin << 12) |  *allowed_up_to_now.begin());
                node_begin = i;
                allowed_up_to_now = get_allowed_local(i);
            } else
                allowed_up_to_now = allowed_next.second;
        }
        entry.push_back((node_begin << 12) | *allowed_up_to_now.begin());

        begin.push_back(entry.size());
    }

    void cpd::append_rows(const cpd &other) {
        int offset = begin.back();
        for (auto x:make_range(other.begin.begin() + 1, other.begin.end()))
            begin.push_back(x + offset);
        std::copy(other.entry.begin(), other.entry.end(), back_inserter(entry));
    }

    const set<int>& cpd::get_allowed(int x, int s, const vector<set<int>> &fmoves) const {
        return fmoves[x];
    }

}