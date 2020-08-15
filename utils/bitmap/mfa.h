#ifndef __MFA_H_
#define __MFA_H_

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/max_cardinality_matching.hpp>

#include "base_allocate.h"

using namespace boost;
typedef adjacency_list<vecS, vecS, directedS> uw_graph;

struct mfa : base_allocate {
  mfa(bitmap_vector &in, bitmap_vector &out) : base_allocate(in, out, "MFA") {}

  std::vector<unsigned int> operator()(
      unsigned int j, const std::vector<unsigned> &proposals) override {
    const unsigned batch_size = left->front().size();
    const unsigned np = proposals.size();
    constexpr unsigned UNMATCHED = -1;

    auto &bo = right->at(j);
    std::vector<unsigned int> allocation(batch_size, UNMATCHED);

    if (bo.to_uint64() == 0ull) return allocation;

    const size_t n_vertices = batch_size + np;
    uw_graph g(n_vertices);

    for (unsigned k = 0; k < batch_size; ++k) {
      if (bo[k]) {
        for (unsigned i = 0; i < np; ++i) {
          auto p = proposals[i];
          if (left->at(p)[k]) {
            add_edge(k, i + batch_size, g);
          }
        }
      }
    }

    std::vector<int> mate(n_vertices, -1);
    edmonds_maximum_cardinality_matching(g, &mate[0]);

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

#endif  //__MFA_H_
