// Copyright 2026 Open Source Robotics Foundation, Inc.
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

#ifndef RMW_FASTRTPS_SHARED_CPP__RMW_INIT_OPTIONS_IMPL_HPP_
#define RMW_FASTRTPS_SHARED_CPP__RMW_INIT_OPTIONS_IMPL_HPP_

#include <cstdint>

/// Backend used for ROS message serialization and typesupport.
enum class SerializationBackend : uint8_t
{
  /// FastCDR-based serialization (default, current ROS 2 behavior).
  FASTCDR = 0,
  /// Experimental XCDR-buffers-based serialization with zero-copy support.
  XCDR_BUFFERS = 1,
};

/// Definition of the middleware-specific init options impl.
/**
 * This struct is cast from rmw_init_options_t::impl and is allocated/freed
 * by the shared rmw_init_options_init/copy/fini helpers.
 */
struct rmw_init_options_impl_s
{
  /// Serialization backend for this context.
  SerializationBackend backend{SerializationBackend::FASTCDR};
};

#endif  // RMW_FASTRTPS_SHARED_CPP__RMW_INIT_OPTIONS_IMPL_HPP_
