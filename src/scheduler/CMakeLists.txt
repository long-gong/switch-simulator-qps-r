cmake_minimum_required(VERSION 3.0)

set(sched_hdr
        "scheduler.h"
        "batch_scheduler.h"
        "max_weight_scheduler.h"
        "maximal_scheduler.h"
        "qps.h"
        "sb_qps.h"
        "fair_frame.h"
        "miscellaneous.h"
        "scheduler_factory.h"
        )
set(sched_src
        "scheduler.cc"
        "batch_scheduler.cc"
        "max_weight_scheduler.cc"
        "maximal_scheduler.cc"
        "qps.cc"
        "sb_qps.cpp"
        "fair_frame.cc"
        "miscellaneous.cc"
        "scheduler_factory.cc")

add_library(scheduler
        ${sched_hdr}
        ${sched_src})
# add_dependencies(scheduler
#         json)
target_link_libraries(
        scheduler
        ${saber_libraries}
)
