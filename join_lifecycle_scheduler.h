#ifndef PROTOS_JOIN_LIFECYCLE_SCHEDULER_H_
#define PROTOS_JOIN_LIFECYCLE_SCHEDULER_H_

#include <memory>

#include "task.h"

namespace protos {

// Creates the task join synchronization and zombie lifecycle scheduler layer
// wrapping `inner`.
std::shared_ptr<TaskScheduler> CreateJoinLifecycleScheduler(
    std::shared_ptr<TaskScheduler> inner);

}  // namespace protos

#endif  // PROTOS_JOIN_LIFECYCLE_SCHEDULER_H_
