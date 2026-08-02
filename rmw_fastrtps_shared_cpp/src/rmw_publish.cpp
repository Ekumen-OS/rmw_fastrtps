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

#include "fastcdr/Cdr.h"
#include "fastcdr/FastBuffer.h"

#include "fastdds/rtps/common/Time_t.h"

#include "rmw/allocators.h"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"
#include "rmw/impl/cpp/macros.hpp"

#include "rmw_fastrtps_shared_cpp/rmw_common.hpp"
#include "rmw_fastrtps_shared_cpp/custom_publisher_info.hpp"
#include "rmw_fastrtps_shared_cpp/TypeSupport.hpp"
#include "rmw_fastrtps_shared_cpp/XcdrTypeSupport.hpp"

#include "rosidl_typesupport_xcdr_c/message_type_support.h"
#include "rosidl_typesupport_xcdr_cpp/message_type_support.hpp"

#include "rosidl_runtime_cpp/experimental/memory.hpp"

#include "tracetools/tracetools.h"

namespace rmw_fastrtps_shared_cpp
{
rmw_ret_t
__rmw_publish(
  const char * identifier,
  const rmw_publisher_t * publisher,
  const void * ros_message,
  rmw_publisher_allocation_t * allocation)
{
  RCUTILS_CAN_RETURN_WITH_ERROR_OF(RMW_RET_INVALID_ARGUMENT);
  RCUTILS_CAN_RETURN_WITH_ERROR_OF(RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RCUTILS_CAN_RETURN_WITH_ERROR_OF(RMW_RET_ERROR);

  (void) allocation;
  RMW_CHECK_FOR_NULL_WITH_MSG(
    publisher, "publisher handle is null",
    return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher, publisher->implementation_identifier, identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    ros_message, "ros message handle is null",
    return RMW_RET_INVALID_ARGUMENT);

  auto info = static_cast<CustomPublisherInfo *>(publisher->data);
  RCUTILS_CHECK_FOR_NULL_WITH_MSG(info, "publisher info pointer is null", return RMW_RET_ERROR);

  rmw_fastrtps_shared_cpp::SerializedData data;
  data.type = FASTRTPS_SERIALIZED_DATA_TYPE_ROS_MESSAGE;
  data.data = const_cast<void *>(ros_message);
  data.impl = info->type_support_impl_;
  eprosima::fastrtps::Time_t stamp;
  eprosima::fastrtps::Time_t::now(stamp);
  TRACETOOLS_TRACEPOINT(rmw_publish, publisher, ros_message, stamp.to_ns());
  if (!info->data_writer_->write_w_timestamp(&data, eprosima::fastdds::dds::HANDLE_NIL, stamp)) {
    RMW_SET_ERROR_MSG("cannot publish data");
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}

rmw_ret_t
__rmw_publish_serialized_message(
  const char * identifier,
  const rmw_publisher_t * publisher,
  const rmw_serialized_message_t * serialized_message,
  rmw_publisher_allocation_t * allocation)
{
  RCUTILS_CAN_RETURN_WITH_ERROR_OF(RMW_RET_INVALID_ARGUMENT);
  RCUTILS_CAN_RETURN_WITH_ERROR_OF(RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RCUTILS_CAN_RETURN_WITH_ERROR_OF(RMW_RET_ERROR);

  (void) allocation;
  RMW_CHECK_FOR_NULL_WITH_MSG(
    publisher, "publisher handle is null",
    return RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher, publisher->implementation_identifier, identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RMW_CHECK_FOR_NULL_WITH_MSG(
    serialized_message, "serialized message handle is null",
    return RMW_RET_INVALID_ARGUMENT);

  auto info = static_cast<CustomPublisherInfo *>(publisher->data);
  RCUTILS_CHECK_FOR_NULL_WITH_MSG(info, "publisher info pointer is null", return RMW_RET_ERROR);

  eprosima::fastcdr::FastBuffer buffer(
    reinterpret_cast<char *>(serialized_message->buffer), serialized_message->buffer_length);
  eprosima::fastcdr::Cdr ser(
    buffer, eprosima::fastcdr::Cdr::DEFAULT_ENDIAN, eprosima::fastcdr::CdrVersion::XCDRv1);
  ser.set_encoding_flag(eprosima::fastcdr::EncodingAlgorithmFlag::PLAIN_CDR);
  if (!ser.jump(serialized_message->buffer_length)) {
    RMW_SET_ERROR_MSG("cannot correctly set serialized buffer");
    return RMW_RET_ERROR;
  }

  rmw_fastrtps_shared_cpp::SerializedData data;
  data.type = FASTRTPS_SERIALIZED_DATA_TYPE_CDR_BUFFER;
  data.data = &ser;
  data.impl = nullptr;  // not used when type is FASTRTPS_SERIALIZED_DATA_TYPE_CDR_BUFFER
  eprosima::fastrtps::Time_t stamp;
  eprosima::fastrtps::Time_t::now(stamp);
  TRACETOOLS_TRACEPOINT(rmw_publish, publisher, serialized_message, stamp.to_ns());
  if (!info->data_writer_->write_w_timestamp(&data, eprosima::fastdds::dds::HANDLE_NIL, stamp)) {
    RMW_SET_ERROR_MSG("cannot publish data");
    return RMW_RET_ERROR;
  }

  return RMW_RET_OK;
}
rmw_ret_t
__rmw_publish_loaned_message(
  const char * identifier,
  const rmw_publisher_t * publisher,
  const void * ros_message,
  rmw_publisher_allocation_t * allocation)
{
  static_cast<void>(allocation);
  RCUTILS_CAN_RETURN_WITH_ERROR_OF(RMW_RET_INVALID_ARGUMENT);
  RCUTILS_CAN_RETURN_WITH_ERROR_OF(RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  RCUTILS_CAN_RETURN_WITH_ERROR_OF(RMW_RET_ERROR);

  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher, publisher->implementation_identifier, identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  if (!publisher->can_loan_messages) {
    RMW_SET_ERROR_MSG("Loaning is not supported");
    return RMW_RET_UNSUPPORTED;
  }

  RMW_CHECK_ARGUMENT_FOR_NULL(ros_message, RMW_RET_INVALID_ARGUMENT);

  auto info = static_cast<CustomPublisherInfo *>(publisher->data);

  auto * xcdr_ts = dynamic_cast<rmw_fastrtps_shared_cpp::XcdrTypeSupport *>(
    info->type_support_.get());

  if (xcdr_ts) {
    // -- XCDR backend: typed-view lifecycle --
    // Derive the blob pointer from the message view (non-destructive).
    // Use the effective handle only for backing-storage derivation;
    // the per-loan entry handle is used for compaction below.
    rosidl_runtime_cpp::MemoryRegion<void> backing =
      rosidl_typesupport_xcdr_cpp::get_backing_storage(
        xcdr_ts->get_effective_handle(), ros_message);
    void * lookup_key = backing.data();
    if (nullptr == lookup_key) {
      RMW_SET_ERROR_MSG("cannot derive blob key from message view");
      return RMW_RET_ERROR;
    }

    // Look up the per-loan entry and extract its specific handle & blob.
    void * blob = nullptr;
    std::shared_ptr<rosidl_message_type_support_t> entry_handle{nullptr};
    {
      std::lock_guard<std::mutex> lock(info->outstanding_loans_mutex_);
      auto it = info->outstanding_loans.find(lookup_key);
      if (it == info->outstanding_loans.end()) {
        RMW_SET_ERROR_MSG("missing per-loan entry for constrained message");
        return RMW_RET_INVALID_ARGUMENT;
      }
      blob = it->second.blob;
      entry_handle = it->second.handle;
      info->outstanding_loans.erase(it);
    }

    // Compact and consume the message view using the loan-specific handle
    // (which may differ from the publisher's effective handle).
    // The compacted XCDR data (its own CDR header + fields) is written at
    // the typed_view base, i.e. payload.data[0].  The wire carries
    // m_typeSize bytes (the loan is pre-sized); the receiver bounds parsing
    // by payload->length, so no size needs to be encoded.
    rosidl_memory_region_t region =
      rosidl_typesupport_xcdr_cpp::compact_message_in_place(
        entry_handle.get(), const_cast<void *>(ros_message));

    if (nullptr == region.location.address) {
      // Compaction failed — message view NOT consumed.
      info->data_writer_->discard_loan(blob);
      rcutils_reset_error();
      return RMW_RET_ERROR;
    }

    // Write the blob (repr header at [-4..-1], compacted XCDR at [0..]).
    eprosima::fastrtps::Time_t stamp;
    eprosima::fastrtps::Time_t::now(stamp);
    TRACETOOLS_TRACEPOINT(rmw_publish, publisher, ros_message, stamp.to_ns());
    if (!info->data_writer_->write_w_timestamp(
        blob, eprosima::fastdds::dds::HANDLE_NIL, stamp))
    {
      info->data_writer_->discard_loan(blob);
      RMW_SET_ERROR_MSG("cannot publish data");
      return RMW_RET_ERROR;
    }
  } else {
    // -- Non-XCDR backend: raw-blob loan --
    void * blob = const_cast<void *>(ros_message);
    eprosima::fastrtps::Time_t stamp;
    eprosima::fastrtps::Time_t::now(stamp);
    TRACETOOLS_TRACEPOINT(rmw_publish, publisher, ros_message, stamp.to_ns());
    if (!info->data_writer_->write_w_timestamp(
        blob, eprosima::fastdds::dds::HANDLE_NIL, stamp))
    {
      RMW_SET_ERROR_MSG("cannot publish data");
      return RMW_RET_ERROR;
    }
  }

  return RMW_RET_OK;
}
}  // namespace rmw_fastrtps_shared_cpp
