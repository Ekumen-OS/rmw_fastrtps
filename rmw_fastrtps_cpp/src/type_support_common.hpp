// Copyright 2016-2018 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#ifndef TYPE_SUPPORT_COMMON_HPP_
#define TYPE_SUPPORT_COMMON_HPP_

#include <sstream>
#include <string>

#include "rcutils/error_handling.h"
#include "rcutils/logging_macros.h"

#include "rmw/error_handling.h"

#include "rmw_fastrtps_shared_cpp/TypeSupport.hpp"
#include "rmw_fastrtps_shared_cpp/rmw_init_options_impl.hpp"

#include "rmw_fastrtps_cpp/MessageTypeSupport.hpp"
#include "rmw_fastrtps_cpp/ServiceTypeSupport.hpp"

#include "rmw_fastrtps_cpp/identifier.hpp"

#include "rosidl_runtime_c/type_hash.h"

#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"

#include "rosidl_typesupport_xcdr_c/identifier.h"
#include "rosidl_typesupport_xcdr_c/message_type_support.h"
#include "rosidl_typesupport_xcdr_cpp/identifier.hpp"
#include "rosidl_typesupport_xcdr_cpp/message_type_support.hpp"

#define RMW_FASTRTPS_CPP_TYPESUPPORT_C rosidl_typesupport_fastrtps_c__identifier
#define RMW_FASTRTPS_CPP_TYPESUPPORT_CPP rosidl_typesupport_fastrtps_cpp::typesupport_identifier

using MessageTypeSupport_cpp = rmw_fastrtps_cpp::MessageTypeSupport;
using TypeSupport_cpp = rmw_fastrtps_cpp::TypeSupport;
using RequestTypeSupport_cpp = rmw_fastrtps_cpp::RequestTypeSupport;
using ResponseTypeSupport_cpp = rmw_fastrtps_cpp::ResponseTypeSupport;

inline std::string
_create_type_name(
  std::string message_namespace,
  std::string message_name)
{
  std::ostringstream ss;
  if (!message_namespace.empty()) {
    ss << message_namespace << "::";
  }
  ss << "dds_::" << message_name << "_";
  return ss.str();
}

inline std::string
_create_type_name(
  const message_type_support_callbacks_t * members)
{
  if (!members) {
    RMW_SET_ERROR_MSG("members handle is null");
    return "";
  }
  std::string message_namespace(members->message_namespace_);
  std::string message_name(members->message_name_);
  return _create_type_name(message_namespace, message_name);
}

// ---------------------------------------------------------------------------
// Backend-aware type support lookup helpers
// ---------------------------------------------------------------------------

/// Try to resolve the XCDR message typesupport handle from the dispatch tree.
/**
 * Language-agnostic: asks the dispatch function for the first typesupport
 * whose identifier matches the "rosidl_typesupport_xcdr*" family pattern.
 * The dispatch function (in rosidl_typesupport_c/_cpp) performs prefix
 * matching via rosidl_runtime_c_typesupport_identifier_matches, so no
 * specific language identifier (C, C++) is hardcoded here — the set of
 * supported languages stays open (e.g. experimental Python messages backed
 * by the C++ typesupport).
 */
inline const rosidl_message_type_support_t *
try_get_xcdr_message_typesupport(
  const rosidl_message_type_support_t * type_supports)
{
  return get_message_typesupport_handle(type_supports, "rosidl_typesupport_xcdr*");
}

/// Try to resolve the FastRTPS C message typesupport handle from the dispatch tree.
inline const rosidl_message_type_support_t *
try_get_fastrtps_message_typesupport_c(
  const rosidl_message_type_support_t * type_supports)
{
  return get_message_typesupport_handle(
    type_supports, RMW_FASTRTPS_CPP_TYPESUPPORT_C);
}

/// Try to resolve the FastRTPS C++ message typesupport handle from the dispatch tree.
inline const rosidl_message_type_support_t *
try_get_fastrtps_message_typesupport_cpp(
  const rosidl_message_type_support_t * type_supports)
{
  return get_message_typesupport_handle(
    type_supports, RMW_FASTRTPS_CPP_TYPESUPPORT_CPP);
}

/// Try to resolve any FastRTPS message typesupport, trying C then C++.
inline const rosidl_message_type_support_t *
try_get_fastrtps_message_typesupport(
  const rosidl_message_type_support_t * type_supports)
{
  const rosidl_message_type_support_t * ts = try_get_fastrtps_message_typesupport_c(type_supports);
  if (nullptr != ts) {
    return ts;
  }
  rcutils_reset_error();
  ts = try_get_fastrtps_message_typesupport_cpp(type_supports);
  if (nullptr != ts) {
    return ts;
  }
  rcutils_reset_error();
  return nullptr;
}

/// Try to resolve the FastRTPS service typesupport, trying C then C++.
inline const rosidl_service_type_support_t *
try_get_fastrtps_service_typesupport(
  const rosidl_service_type_support_t * type_supports)
{
  const rosidl_service_type_support_t * ts = get_service_typesupport_handle(
    type_supports, RMW_FASTRTPS_CPP_TYPESUPPORT_C);
  if (nullptr != ts) {
    return ts;
  }
  rcutils_error_string_t prev_error_string = rcutils_get_error_string();
  rcutils_reset_error();
  ts = get_service_typesupport_handle(
    type_supports, RMW_FASTRTPS_CPP_TYPESUPPORT_CPP);
  if (nullptr != ts) {
    return ts;
  }
  RCUTILS_LOG_DEBUG_NAMED(
    "rmw_fastrtps_cpp",
    "No FastRTPS service typesupport found. Previous error: %s",
    prev_error_string.str);
  rcutils_reset_error();
  return nullptr;
}

/// Try to extract a message type name from an XCDR typesupport handle.
/**
 * Uses the XCDR outer struct's message_namespace and message_name fields
 * (populated by the code generator).
 */
inline std::string
try_get_message_type_name_from_xcdr(
  const rosidl_message_type_support_t * xcdr_ts)
{
  if (nullptr == xcdr_ts || nullptr == xcdr_ts->data) {
    return "";
  }
  auto xcdr = static_cast<const rosidl_message_xcdr_type_support_t *>(xcdr_ts->data);
  if (nullptr != xcdr->message_namespace && nullptr != xcdr->message_name) {
    return _create_type_name(xcdr->message_namespace, xcdr->message_name);
  }
  return "";
}

/// Safe type hash retrieval: returns a zero-initialized hash when unavailable.
inline rosidl_type_hash_t
try_get_type_hash(
  const rosidl_message_type_support_t * type_supports)
{
  if (nullptr != type_supports && nullptr != type_supports->get_type_hash_func) {
    const rosidl_type_hash_t * hash = type_supports->get_type_hash_func(type_supports);
    if (nullptr != hash) {
      return *hash;
    }
    // hash function returned nullptr; clear any error it may have set.
    rcutils_reset_error();
  }
  return rosidl_get_zero_initialized_type_hash();
}

#endif  // TYPE_SUPPORT_COMMON_HPP_
