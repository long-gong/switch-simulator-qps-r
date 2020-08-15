#include "sb_qps.h"

#include <bitmap/ffa.h>
#include <bitmap/mfa.h>
#include <bitmap/mwfa.h>
#include <switch/iq_switch.h>

#include <functional>
#include <memory>

namespace saber {
saber::SB_QPS::SB_QPS(const std::string &name, int num_inputs, int num_outputs,
                      int frame_size, unsigned iterations,
                      const AcceptPolicy &ap, std::mt19937::result_type seed)
    : BatchScheduler(name, num_inputs, num_outputs, frame_size, true),
      _seed(seed),
      _eng(seed),
      _bst(num_inputs),
      _match_flag_in(num_inputs, bitmap_t(_frame_size, 0ull)),
      _match_flag_out(num_outputs, bitmap_t(_frame_size, 0ull)),
      _cf_packets_counter(num_inputs, std::vector<unsigned>(num_outputs, 0)),
      _iterations(iterations),
      _accept_policy(ap),
      _schedules_pre(frame_size, std::vector<int>(num_inputs, -1)) {
  _left_start = BST::nearest_power_of_two(_num_outputs);
  for (size_t i = 0; i < num_inputs; ++i) _bst[i].resize(2 * _left_start, 0);
  _cf_rel_time = 0;
  // generate initial schedulers (used for the first frame)
  for (auto &sched : _schedules) {
    std::fill(sched.begin(), sched.end(), -1);
  }
  bitmap_reset();  // set to all available
}

void SB_QPS::bitmap_reset() {
  for (auto &mf : _match_flag_in) mf.set();
  for (auto &mf : _match_flag_out) mf.set();
}

void SB_QPS::handle_arrivals(const IQSwitch *sw) {
  assert(sw != nullptr);
  auto arrivals = sw->get_arrivals();
  for (const auto &sd : arrivals) {
    if (sd.first == -1) break;
    assert(sd.first >= 0 && sd.first < _num_inputs);
    assert(sd.second >= 0 && sd.second < _num_outputs);
    BST::update<int>(_bst[sd.first], sd.second + _left_start);
    ++_cf_packets_counter[sd.first][sd.second];
  }
}

void SB_QPS::handle_departures(
    const std::vector<std::pair<int, int>> &dep_pre) {
  for (const auto &sd : dep_pre) {
    auto s = sd.first;
    auto d = sd.second;
    assert(_cf_packets_counter[s][d] > 0);
    BST::update<int>(_bst[s], d + _left_start, -1);
    --_cf_packets_counter[s][d];
  }
}

int SB_QPS::sampling(int source) {
  assert(source >= 0 && source < _num_inputs);
  std::uniform_real_distribution<double> dist(0, _bst[source][1]);
  double r = dist(_eng);

  int out = BST::upper_bound<int>(_bst[source], r) - _left_start;
  assert(out >= 0 && out < _num_outputs);
  return out;
}

void SB_QPS::qps(const saber::IQSwitch *sw, size_t frame_id) {
  assert(_frame_size_fixed);
  // handle arrivals
  handle_arrivals(sw);

  std::unique_ptr<base_allocate> allocation;

  switch (_accept_policy) {
    case AcceptPolicy::FFA:
      allocation =
          std::unique_ptr<ffa>(new ffa(_match_flag_in, _match_flag_out));
      break;
    case AcceptPolicy::MFA:
      allocation =
          std::unique_ptr<mfa>(new mfa(_match_flag_in, _match_flag_out));
      break;
    case AcceptPolicy::MWFA:
      allocation = std::unique_ptr<mwfa>(
          new mwfa(_match_flag_in, _match_flag_out, _cf_packets_counter));
      break;
  }

  for (unsigned it = 0; it < _iterations; ++it) {
    std::vector<std::vector<unsigned>> proposals(num_inputs());

    // Step 1: Proposing
    for (int i = 0; i < _num_inputs; ++i) {
      if (queue_length(i) == 0) continue;  // no packets
      auto out = sampling(i);              // sampling an output for this input
      assert(_cf_packets_counter[i][out] > 0);
      proposals[out].push_back(i);
    }
    std::vector<std::pair<int, int>> vdep;  // virtual departures

    // Step 2: Accept
    for (unsigned out = 0; out < _num_outputs; ++out) {
      if (proposals[out].empty()) continue;
      if (proposals[out].size() > 1) {
        // sort in descending order
        std::sort(proposals[out].begin(), proposals[out].end(),
                  [out, this](unsigned lhs, unsigned rhs) {
                    return (this->_cf_packets_counter[lhs][out] >
                            this->_cf_packets_counter[rhs][out]);
                  });
      }
      auto acc = allocation->operator()(out, proposals[out]);
      for (unsigned k = 0; k < frame_size(); ++k) {
        auto in = acc[k];
        if (in < num_inputs()) {
          _schedules[k][in] = out;
          vdep.emplace_back(in, out);
        }
      }
    }
    handle_departures(vdep);
  }
}

void SB_QPS::init(const IQSwitch *sw) {
  // reserved
}

void SB_QPS::schedule(const saber::IQSwitch *sw) {
  auto frame_id = (_cf_rel_time % _frame_size);
  // copy out scheduler for the last frame
  std::copy(_schedules_pre[frame_id].begin(), _schedules_pre[frame_id].end(),
            _in_match.begin());
  std::fill(_schedules_pre[frame_id].begin(), _schedules_pre[frame_id].end(),
            -1);
  qps(sw, frame_id);
  // reset bit map, if it is the last time slot of a frame
  if (frame_id == _frame_size - 1) {
    bitmap_reset();
    _schedules_pre = _schedules;
    for (auto &sched : _schedules) {
      std::fill(sched.begin(), sched.end(), -1);
    }
  }
  ++_cf_rel_time;
}

void SB_QPS::reset() {
  BatchScheduler::reset();
  bitmap_reset();
  for (size_t i = 0; i < _num_inputs; ++i)
    std::fill(_bst[i].begin(), _bst[i].end(), 0);
  for (auto &counter : _cf_packets_counter)
    std::fill(counter.begin(), counter.end(), 0);
  _cf_rel_time = 0;
  for (auto &sched : _schedules) {
    std::fill(sched.begin(), sched.end(), -1);
  }
  for (auto &sched : _schedules_pre) {
    std::fill(sched.begin(), sched.end(), -1);
  }
}

void SB_QPS::display(std::ostream &os) const {
  BatchScheduler::display(os);
  os << "---------------------------------------------------------------------"
        "\n";
  os << "seed             : " << _seed << "\nbst              : " << _bst
     << "\naccepting policy : " << policy2name(_accept_policy)
     << "\niterations       : " << _iterations << "\n";
}

}  // namespace saber