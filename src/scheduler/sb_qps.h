#ifndef _SCHEDULER_SB_QPS_H_
#define _SCHEDULER_SB_QPS_H_

#include <bitmap/bitmap.h>

#include "scheduler/batch_scheduler.h"

namespace saber {

enum class AcceptPolicy { FFA, MFA, MWFA };

inline std::string policy2name(const AcceptPolicy &policy) {
  switch (policy) {
    case AcceptPolicy::FFA:
      return "FFA";
    case AcceptPolicy::MFA:
      return "MFA";
    case AcceptPolicy::MWFA:
      return "MWFA";
    default:
      return "";
  }
}

inline AcceptPolicy num2policy(unsigned id) {
  switch (id) {
    case 1:
      return AcceptPolicy::MFA;
    case 2:
      return AcceptPolicy::MWFA;
    default:
      return AcceptPolicy::FFA;
  }
}
/** @brief Class for SB_QPS
 *
 *  In the first half of a batch, just doing QPS-1 for each time slot,
 *  whereas in the second half, doing QPS-1 but allowing using holes left
 *  before to accept inputs' proposals.
 *
 */
class SB_QPS : public BatchScheduler {
  friend class SchedulerFactory;

 protected:
  using bst_t = std::vector<int>;
  using bitmap_t = BitMap;

  std::mt19937::result_type _seed;
  std::mt19937 _eng{std::random_device{}()};

  int _left_start{-1};
  std::vector<bst_t> _bst;

  // bitmaps for each input & output
  std::vector<bitmap_t> _match_flag_in;
  std::vector<bitmap_t> _match_flag_out;

  // counter of packets
  std::vector<std::vector<unsigned>> _cf_packets_counter;

  unsigned _iterations;
  AcceptPolicy _accept_policy;

  std::vector<std::vector<int>> _schedules_pre;

  SB_QPS(const std::string &name, int num_inputs, int num_outputs,
         int frame_size, unsigned iterations, const AcceptPolicy &ap,
         std::mt19937::result_type seed);
  void bitmap_reset();
  void handle_arrivals(const IQSwitch *sw);
  void handle_departures(const std::vector<std::pair<int, int>> &dep_pre);
  int sampling(int source);
  int queue_length(int source) {
    assert(source >= 0 && source < _num_inputs);
    return _bst[source][1];
  }
  void qps(const IQSwitch *sw, size_t current_ts);

 public:
  ~SB_QPS() override = default;
  void schedule(const IQSwitch *sw) override;
  void init(const IQSwitch *sw) override;
  void reset() override;
  void display(std::ostream &os) const override;
  //// reserved
  void dump_stats(std::ostream &os) override {}
};  // SB_QPS

}  // namespace saber

#endif  //_SCHEDULER_SB_QPS_H_