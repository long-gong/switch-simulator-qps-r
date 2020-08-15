#ifndef FAIR_FRAME_H
#define FAIR_FRAME_H

#include <utility>

#include "batch_scheduler.h"

namespace saber {
class FairFrame : public BatchScheduler {
  friend class SchedulerFactory;

 protected:
  // counter of packets
  std::vector<std::vector<unsigned>> _cf_packets_counter, _packets_counter_bk;
  // row & column sums
  std::vector<unsigned> _row_sums;
  std::vector<unsigned> _col_sums;
  int _w;

  std::vector<std::pair<int, int>> _overflows;

  FairFrame(const std::string &name, int num_inputs, int num_outputs,
            int frame_size);
  void handle_arrivals(const IQSwitch *sw);
  //  void handle_departures(const std::vector<std::pair<int, int>> &dep_pre);
 public:
  ~FairFrame() override = default;
  void schedule(const IQSwitch *sw) override;
  void init(const IQSwitch *sw) override;
  void reset() override;
  void display(std::ostream &os) const override;
  //// reserved
  void dump_stats(std::ostream &os) override {}
};  // SB_QPS_HalfHalf_Oblivious
}  // namespace saber
#endif  // FAIR_FRAME_H