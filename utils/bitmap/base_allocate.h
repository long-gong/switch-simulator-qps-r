#ifndef __BASE_ALLOCATE_H_
#define __BASE_ALLOCATE_H_

#include <algorithm>
#include <functional>
#include <memory>
#include <stdexcept>
#include <vector>

#include "bitmap.h"

using bitmap_vector = std::vector<BitMap>;

class not_implement_error : public std::logic_error {
 public:
  explicit not_implement_error(std::string name)
      : std::logic_error(std::move(name) + " was not implemented!") {}
};

struct base_allocate {
  bitmap_vector *left;
  bitmap_vector *right;
  const std::string name;

  base_allocate(bitmap_vector &in, bitmap_vector &out, std::string cls = "Base")
      : left(&in), right(&out), name(std::move(cls)) {}

  virtual std::vector<unsigned> operator()(unsigned,
                                           const std::vector<unsigned> &) {
    throw not_implement_error("Allocation method of " + name);
  }
};

#endif  //__BASE_ALLOCATE_H_
