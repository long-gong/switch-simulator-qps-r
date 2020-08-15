#ifndef __MWFA_H_
#define __MWFA_H_

#include <boost/graph/adjacency_list.hpp>

#include "base_allocate.h"
// required BOOST VERSION AT >= 1.72.0
#include <boost/graph/maximum_weighted_matching.hpp>

using namespace boost;

typedef property<edge_weight_t, float, property<edge_index_t, int>>
    EdgeProperty;
typedef adjacency_list<vecS, vecS, undirectedS, no_property, EdgeProperty>
    w_graph;

using weight_matrix = std::vector<std::vector<unsigned>>;

struct mwfa : base_allocate {
  const weight_matrix *const weights_ptr;  // read-only pointer

  mwfa(bitmap_vector &in, bitmap_vector &out, const weight_matrix &weights)
      : base_allocate(in, out, "MWFA"), weights_ptr(&weights) {}
  std::vector<unsigned int> operator()(
      unsigned int j, const std::vector<unsigned> &proposals) override {
    const unsigned batch_size = left->front().size();
    const unsigned np = proposals.size();
    constexpr unsigned UNMATCHED = -1;

    auto &bo = right->at(j);
    std::vector<unsigned int> allocation(batch_size, UNMATCHED);
    if (bo.to_uint64() == 0ull) return allocation;

    const size_t n_vertices = batch_size + np;
    w_graph g(n_vertices);

    for (unsigned k = 0; k < batch_size; ++k) {
      if (bo[k]) {
        for (unsigned i = 0; i < np; ++i) {
          auto p = proposals[i];
          if (left->at(p)[k]) {
            add_edge(k, i + batch_size, EdgeProperty(weights_ptr->at(p)[j]), g);
          }
        }
      }
    }

    std::vector<int> mate(n_vertices, -1);
    maximum_weighted_matching(g, &mate[0]);

    for (unsigned k = 0; k < batch_size; ++k) {
      if (mate[k] >= (int)batch_size && mate[k] < (int)n_vertices) {
        auto in = proposals.at(mate[k] - batch_size);
        allocation[k] = in;
        // update bitmaps
        bo.reset(k);
        left->at(in).reset(k);
      }
    }
    return allocation;
  }
};
#endif  //__MWFA_H_
