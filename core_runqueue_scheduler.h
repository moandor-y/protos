#ifndef PROTOS_CORE_RUNQUEUE_SCHEDULER_H_
#define PROTOS_CORE_RUNQUEUE_SCHEDULER_H_

#include <memory>

#include "task.h"

namespace protos {

// Creates the innermost core per-CPU weighted Red-Black Tree runqueue and
// context-switch task scheduler layer.
std::shared_ptr<TaskScheduler> CreateCoreRunqueueScheduler();

}  // namespace protos

#endif  // PROTOS_CORE_RUNQUEUE_SCHEDULER_H_
