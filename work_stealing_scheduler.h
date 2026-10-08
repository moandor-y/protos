#ifndef PROTOS_WORK_STEALING_SCHEDULER_H_
#define PROTOS_WORK_STEALING_SCHEDULER_H_

#include <memory>

#include "task.h"

namespace protos {

// Creates the cross-CPU placement and work-stealing task scheduler layer
// wrapping `inner`.
std::shared_ptr<TaskScheduler> CreateWorkStealingScheduler(
    std::shared_ptr<TaskScheduler> inner);

}  // namespace protos

#endif  // PROTOS_WORK_STEALING_SCHEDULER_H_
