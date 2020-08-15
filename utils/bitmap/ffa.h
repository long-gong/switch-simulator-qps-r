#ifndef __FFA_H_
#define __FFA_H_

#include "base_allocate.h"

#ifdef DEBUG
#include <iostream>
#endif

// First fit allocation
struct ffa : base_allocate {
  ffa(bitmap_vector &in, bitmap_vector &out) : base_allocate(in, out, "FFA") {}

  std::vector<unsigned int> operator()(
      unsigned int j, const std::vector<unsigned> &proposals) override {
    unsigned batch_size = left->front().size();
    constexpr unsigned UNMATCHED = -1;
    auto &bo = right->at(j);
    std::vector<unsigned int> allocation(batch_size, UNMATCHED);

    for (auto p : proposals) {
      if (bo.to_uint64() == 0ull) break;  // early stop
      auto &bi = left->at(p);
      auto b = bi & bo;
      unsigned k = b.ffs();

#ifdef DEBUG
      std::cout << bi << " & " << bo << " = " << b << " ==> " << k << "\n";
#endif
      if (k < batch_size) {
        allocation[k] = p;
        bo.reset(k);
        bi.reset(k);
      }
    }

    return allocation;
  }
};

#endif  //__FFA_H_
