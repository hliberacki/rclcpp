// Copyright 2015 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rcpputils/scope_exit.hpp"

#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/any_executable.hpp"

using rclcpp::executors::SingleThreadedExecutor;

SingleThreadedExecutor::SingleThreadedExecutor(const rclcpp::ExecutorOptions & options)
: rclcpp::Executor(options) {}

SingleThreadedExecutor::~SingleThreadedExecutor() {}

void
SingleThreadedExecutor::spin(std::chrono::nanoseconds timeout)
{
  auto end_time = std::chrono::steady_clock::now() + timeout;
  std::chrono::nanoseconds timeout_left = timeout;

  if (spinning.exchange(true)) {
    throw std::runtime_error("spin() called while already spinning");
  }

  const bool is_blocking = timeout < std::chrono::nanoseconds::zero();

  RCPPUTILS_SCOPE_EXIT(this->spinning.store(false); );
  while (rclcpp::ok(this->context_) && spinning.load()) {
    rclcpp::AnyExecutable any_executable;

    if (get_next_executable(any_executable, timeout_left)) {
      execute_any_executable(any_executable);
    }

    if (!is_blocking) {
      auto now = std::chrono::steady_clock::now();
      if (now >= end_time) {
        return;
      }

      // Subtract the elapsed time from the original timeout.
      timeout_left = end_time - now;
    }
  }
}
