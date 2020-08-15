#include "fair_frame.h"

#include <switch/iq_switch.h>

#include <bvd/bvd.hpp>
#include <bvd/utils.hpp>

namespace saber {
// using matrix = std::vector<std::vector<unsigned >>;

// Implementation of class FairFrame
FairFrame::FairFrame(const std::string &name, int num_inputs, int num_outputs,
                     int frame_size)
    : BatchScheduler(name, num_inputs, num_outputs, frame_size, true),
      _cf_packets_counter(num_inputs, std::vector<unsigned>(num_outputs, 0)),
      _row_sums(num_inputs, 0),
      _col_sums(num_outputs, 0),
      _w(0) {
  _cf_rel_time = 0;
  // generate initial schedulers (used for the first frame)
  for (auto &sched : _schedules) {
    std::fill(sched.begin(), sched.end(), -1);
  }
}

void FairFrame::handle_arrivals(const IQSwitch *sw) {
  assert(sw != nullptr);
  auto arrivals = sw->get_arrivals();
  for (const auto &sd : arrivals) {
    if (sd.first == -1) break;
    assert(sd.first >= 0 && sd.first < _num_inputs);
    assert(sd.second >= 0 && sd.second < _num_outputs);
    if (_row_sums[sd.first] < frame_size() &&
        _col_sums[sd.second] < frame_size()) {
      ++_row_sums[sd.first];
      ++_col_sums[sd.second];
      auto max_temp = std::max(_row_sums[sd.first], _col_sums[sd.second]);
      if (max_temp > _w) _w = max_temp;
      ++_cf_packets_counter[sd.first][sd.second];
    } else {
      _overflows.push_back(sd);
    }
  }
}

void FairFrame::init(const IQSwitch *sw) {
  // reserved
}

void FairFrame::schedule(const saber::IQSwitch *sw) {
  auto frame_id = (_cf_rel_time % _frame_size);
  // copy out scheduler for the last frame
  if (_schedules.size() > frame_id) {
    std::copy(_schedules[frame_id].begin(), _schedules[frame_id].end(),
              _in_match.begin());
  } else {
    std::fill(_in_match.begin(), _in_match.end(), -1);
  }

  handle_arrivals(sw);

  if (frame_id == _frame_size - 1) {
    _packets_counter_bk = _cf_packets_counter;
    // calculate new matchings
    auto doubly_stochastic_mat =
        DenseMatrix<unsigned>{std::move(_cf_packets_counter)};
    padding_to_doubly_stochastic<DenseMatrix<unsigned>>(
        doubly_stochastic_mat, _w, _row_sums, _col_sums);
    // NOTICE: will change size of _schedules
    //    printf("%lu: w = %d\n", _cf_rel_time, _w);
    birkhoff_von_neumann_decomposition<DenseMatrix<unsigned>>(
        doubly_stochastic_mat, _schedules, _w);

    if (_schedules.size() != _w) {
      throw std::logic_error("BvD incorrect!");
    }

    // virtual departing
    for (auto &sched : _schedules) {
      for (unsigned i = 0; i < num_inputs(); ++i) {
        auto j = sched[i];
        if (_packets_counter_bk[i][j] > 0)
          --_packets_counter_bk[i][j];
        else  // invalid
          sched[i] = -1;
      }
    }

    _cf_packets_counter.resize(num_inputs(),
                               std::vector<unsigned>(num_outputs(), 0));
    std::fill(_row_sums.begin(), _row_sums.end(), 0);
    std::fill(_col_sums.begin(), _col_sums.end(), 0);

    _w = 0;
    // handle overflows
    if (!_overflows.empty()) {
      std::vector<std::pair<int, int>> tmp_bk;
      for (const auto &sd : _overflows) {
        if (_row_sums[sd.first] < frame_size() &&
            _col_sums[sd.second] < frame_size()) {
          ++_row_sums[sd.first];
          ++_col_sums[sd.second];
          ++_cf_packets_counter[sd.first][sd.second];
          auto max_temp = std::max(_row_sums[sd.first], _col_sums[sd.second]);
          if (max_temp > _w) _w = max_temp;
        } else {
          tmp_bk.push_back(sd);
        }
      }
      _overflows = std::move(tmp_bk);
    }
  }
  ++_cf_rel_time;
}

void FairFrame::reset() {
  BatchScheduler::reset();
  for (auto &counter : _cf_packets_counter)
    std::fill(counter.begin(), counter.end(), 0);

  std::fill(_row_sums.begin(), _row_sums.end(), 0);
  std::fill(_col_sums.begin(), _col_sums.end(), 0);

  _overflows.clear();

  _cf_rel_time = 0;
  for (auto &sched : _schedules) {
    std::fill(sched.begin(), sched.end(), -1);
  }
}

void FairFrame::display(std::ostream &os) const { BatchScheduler::display(os); }

}  // end namespace saber
