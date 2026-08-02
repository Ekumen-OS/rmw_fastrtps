// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include <string>

#include "rmw/allocators.h"
#include "rmw/error_handling.h"
#include "rmw/impl/cpp/macros.hpp"
#include "rmw/rmw.h"

#include "fastdds/dds/publisher/DataWriter.hpp"
#include "fastdds/dds/publisher/qos/DataWriterQos.hpp"

#include "rmw_fastrtps_shared_cpp/custom_participant_info.hpp"
#include "rmw_fastrtps_shared_cpp/custom_publisher_info.hpp"
#include "rmw_fastrtps_shared_cpp/namespace_prefix.hpp"
#include "rmw_fastrtps_shared_cpp/publisher.hpp"
#include "rmw_fastrtps_shared_cpp/qos.hpp"
#include "rmw_fastrtps_shared_cpp/rmw_common.hpp"
#include "rmw_fastrtps_shared_cpp/rmw_context_impl.hpp"
#include "rmw_fastrtps_shared_cpp/TypeSupport.hpp"
#include "rmw_fastrtps_shared_cpp/XcdrTypeSupport.hpp"

#include "rosidl_typesupport_xcdr_c/message_type_support.h"
#include "rosidl_typesupport_xcdr_cpp/message_type_support.hpp"

#include "rosidl_runtime_cpp/experimental/memory.hpp"

#include "time_utils.hpp"

namespace rmw_fastrtps_shared_cpp
{
rmw_ret_t
__rmw_destroy_publisher(
  const char * identifier,
  const rmw_node_t * node,
  rmw_publisher_t * publisher)
{
  assert(node->implementation_identifier == identifier);
  assert(publisher->implementation_identifier == identifier);

  rmw_ret_t ret = RMW_RET_OK;
  rmw_error_state_t error_state;
  auto common_context = static_cast<rmw_dds_common::Context *>(node->context->impl->common);
  auto info = static_cast<const CustomPublisherInfo *>(publisher->data);

  // Update graph
  rmw_ret_t rmw_ret = common_context->remove_publisher_graph(
    info->publisher_gid,
    node->name, node->namespace_
  );
  if (RMW_RET_OK != rmw_ret) {
    error_state = *rmw_get_error_state();
    ret = rmw_ret;
    rmw_reset_error();
  }

  auto participant_info =
    static_cast<CustomParticipantInfo *>(node->context->impl->participant_info);
  rmw_ret_t inner_ret = destroy_publisher(identifier, participant_info, publisher);
  if (RMW_RET_OK != inner_ret) {
    if (RMW_RET_OK != ret) {
      RMW_SAFE_FWRITE_TO_STDERR(rmw_get_error_string().str);
      RMW_SAFE_FWRITE_TO_STDERR(" during '" RCUTILS_STRINGIFY(__function__) "'\n");
    } else {
      error_state = *rmw_get_error_state();
      ret = inner_ret;
    }
    rmw_reset_error();
  }

  if (RMW_RET_OK != ret) {
    rmw_set_error_state(error_state.message, error_state.file, error_state.line_number);
  }
  return ret;
}

rmw_ret_t
__rmw_publisher_count_matched_subscriptions(
  const rmw_publisher_t * publisher,
  size_t * subscription_count)
{
  auto info = static_cast<CustomPublisherInfo *>(publisher->data);

  *subscription_count = info->publisher_event_->subscription_count();

  return RMW_RET_OK;
}

rmw_ret_t
__rmw_publisher_assert_liveliness(
  const char * identifier,
  const rmw_publisher_t * publisher)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher,
    publisher->implementation_identifier,
    identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  auto info = static_cast<CustomPublisherInfo *>(publisher->data);
  if (nullptr == info) {
    RMW_SET_ERROR_MSG("publisher internal data is invalid");
    return RMW_RET_ERROR;
  }

  info->data_writer_->assert_liveliness();
  return RMW_RET_OK;
}

rmw_ret_t
__rmw_publisher_wait_for_all_acked(
  const char * identifier,
  const rmw_publisher_t * publisher,
  rmw_time_t wait_timeout)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher,
    publisher->implementation_identifier,
    identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);

  auto info = static_cast<CustomPublisherInfo *>(publisher->data);

  eprosima::fastrtps::Duration_t timeout = rmw_time_to_fastrtps(wait_timeout);

  ReturnCode_t ret = info->data_writer_->wait_for_acknowledgments(timeout);
  if (ReturnCode_t::RETCODE_OK == ret) {
    return RMW_RET_OK;
  }

  return RMW_RET_TIMEOUT;
}

rmw_ret_t
__rmw_publisher_get_actual_qos(
  const rmw_publisher_t * publisher,
  rmw_qos_profile_t * qos)
{
  auto info = static_cast<CustomPublisherInfo *>(publisher->data);
  eprosima::fastdds::dds::DataWriter * fastdds_dw = info->data_writer_;
  const eprosima::fastdds::dds::DataWriterQos & dds_qos = fastdds_dw->get_qos();

  dds_qos_to_rmw_qos(dds_qos, qos);

  return RMW_RET_OK;
}

rmw_ret_t
__rmw_borrow_loaned_message(
  const char * identifier,
  const rmw_publisher_t * publisher,
  const rosidl_message_type_support_t * type_support,
  const rosidl_message_type_constraints_t * type_constraints,
  void ** ros_message)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher, publisher->implementation_identifier, identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  if (!publisher->can_loan_messages) {
    RMW_SET_ERROR_MSG("Loaning is not supported");
    return RMW_RET_UNSUPPORTED;
  }

  RMW_CHECK_ARGUMENT_FOR_NULL(type_support, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_ARGUMENT_FOR_NULL(ros_message, RMW_RET_INVALID_ARGUMENT);
  if (nullptr != *ros_message) {
    return RMW_RET_INVALID_ARGUMENT;
  }

  auto info = static_cast<CustomPublisherInfo *>(publisher->data);

  // Resolve the XCDR type support from the publisher.
  auto * xcdr_ts = dynamic_cast<rmw_fastrtps_shared_cpp::XcdrTypeSupport *>(
    info->type_support_.get());

  if (xcdr_ts) {
    // -- XCDR backend: typed-view loan --
    std::shared_ptr<rosidl_message_type_support_t> loan_handle{nullptr};

    if (type_constraints) {
      // Validate against publisher baseline (if any).
      const rosidl_message_type_support_t * ts_handle = xcdr_ts->get_effective_handle();
      const rosidl_message_type_constraints_t * baseline =
        ts_handle ? rosidl_typesupport_xcdr_c_get_constraints(ts_handle) : nullptr;
      if (!rosidl_typesupport_xcdr_cpp::compare_constraints(
          ts_handle, type_constraints, baseline))
      {
        RMW_SET_ERROR_MSG(
          "per-loan constraints exceed publisher-wide bounds; "
          "set looser constraints at create_publisher time instead");
        return RMW_RET_CONSTRAINTS_HIT;
      }

      // Build a per-loan constrained handle from the base handle.
      const rosidl_message_type_support_t * base = xcdr_ts->get_base_handle();
      if (nullptr == base) {
        RMW_SET_ERROR_MSG("no base XCDR handle for per-loan constraints");
        return RMW_RET_ERROR;
      }
      loan_handle = rosidl_typesupport_xcdr_cpp::create_constrained_message_type_support(
        base, type_constraints);
      if (!loan_handle) {
        rcutils_reset_error();
        RMW_SET_ERROR_MSG("failed to create per-loan constrained handle");
        return RMW_RET_ERROR;
      }
    } else {
      // Publisher-wide constraints (or fixed-size without explicit constraints).
      const rosidl_message_type_support_t * eff = xcdr_ts->get_effective_handle();
      loan_handle = std::shared_ptr<rosidl_message_type_support_t>(
        const_cast<rosidl_message_type_support_t *>(eff),
        [](rosidl_message_type_support_t *) {});
    }

    // Compute expected data size.  Zero means no typed view is possible.
    size_t expected_data_size =
      XcdrTypeSupport::get_expected_data_size_for_handle(loan_handle.get());
    if (0 == expected_data_size) {
      RMW_SET_ERROR_MSG(
        "XCDR type does not support typed loan (unbounded)");
      return RMW_RET_ERROR;
    }

    // Loan a raw blob from Fast DDS.
    void * blob = nullptr;
    if (!info->data_writer_->loan_sample(
        blob,
        eprosima::fastdds::dds::DataWriter::LoanInitializationKind::NO_LOAN_INITIALIZATION))
    {
      return RMW_RET_ERROR;
    }

    // Construct the typed message view at the very start of the payload
    // buffer (blob - representation_header_size).  The XCDR block (its own
    // CDR header + fields) must occupy payload.data[0..] so that regular
    // copy-take deserialization sees the same layout as the normal
    // serialize path.
    void * typed_base = static_cast<char *>(blob) -
      eprosima::fastrtps::rtps::SerializedPayload_t::representation_header_size;
    rosidl_runtime_cpp::MemoryRegion<void> storage(typed_base, expected_data_size);
    void * message = nullptr;
    rcutils_ret_t ret = rosidl_typesupport_xcdr_cpp::construct_message_at(
      loan_handle.get(), storage, &message);
    if (RCUTILS_RET_OK != ret || nullptr == message) {
      info->data_writer_->discard_loan(blob);
      rcutils_reset_error();
      return RMW_RET_ERROR;
    }
    *ros_message = message;

    // Register per-loan entry keyed by typed_base (the address that
    // get_backing_storage returns for the message view).  This ensures
    // the publish and return-loan paths can find the entry via
    // backing storage lookup.
    {
      std::lock_guard<std::mutex> lock(info->outstanding_loans_mutex_);
      info->outstanding_loans[typed_base] = {
        blob, std::move(loan_handle), expected_data_size};
    }
  } else {
    // -- Non-XCDR backend: raw-blob loan --
    void * blob = nullptr;
    if (!info->data_writer_->loan_sample(
        blob,
        eprosima::fastdds::dds::DataWriter::LoanInitializationKind::NO_LOAN_INITIALIZATION))
    {
      return RMW_RET_ERROR;
    }
    *ros_message = blob;
  }

  return RMW_RET_OK;
}

rmw_ret_t
__rmw_return_loaned_message_from_publisher(
  const char * identifier,
  const rmw_publisher_t * publisher,
  void * loaned_message)
{
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher, RMW_RET_INVALID_ARGUMENT);
  RMW_CHECK_TYPE_IDENTIFIERS_MATCH(
    publisher, publisher->implementation_identifier, identifier,
    return RMW_RET_INCORRECT_RMW_IMPLEMENTATION);
  if (!publisher->can_loan_messages) {
    RMW_SET_ERROR_MSG("Loaning is not supported");
    return RMW_RET_UNSUPPORTED;
  }

  RMW_CHECK_ARGUMENT_FOR_NULL(loaned_message, RMW_RET_INVALID_ARGUMENT);

  auto info = static_cast<CustomPublisherInfo *>(publisher->data);

  auto * xcdr_ts = dynamic_cast<rmw_fastrtps_shared_cpp::XcdrTypeSupport *>(
    info->type_support_.get());

  if (xcdr_ts) {
    // -- XCDR backend: typed-view lifecycle --
    rosidl_runtime_cpp::MemoryRegion<void> backing =
      rosidl_typesupport_xcdr_cpp::get_backing_storage(
        xcdr_ts->get_effective_handle(), loaned_message);
    void * lookup_key = backing.data();
    if (nullptr == lookup_key) {
      RMW_SET_ERROR_MSG("cannot derive blob key from message view");
      return RMW_RET_ERROR;
    }

    // Look up the per-loan entry.
    std::shared_ptr<rosidl_message_type_support_t> entry_handle{nullptr};
    void * blob = nullptr;
    {
      std::lock_guard<std::mutex> lock(info->outstanding_loans_mutex_);
      auto it = info->outstanding_loans.find(lookup_key);
      if (it == info->outstanding_loans.end()) {
        RMW_SET_ERROR_MSG("missing per-loan entry for constrained message");
        return RMW_RET_INVALID_ARGUMENT;
      }
      entry_handle = it->second.handle;
      blob = it->second.blob;
      info->outstanding_loans.erase(it);
    }

    // Release the typed view to recover the underlying blob, then discard.
    rosidl_memory_region_t storage =
      rosidl_typesupport_xcdr_c_release_message(entry_handle.get(), loaned_message);
    if (nullptr == storage.location.address) {
      RMW_SET_ERROR_MSG("failed to release loaned message");
      return RMW_RET_ERROR;
    }
    if (!info->data_writer_->discard_loan(blob)) {
      return RMW_RET_ERROR;
    }
  } else {
    // -- Non-XCDR backend: raw-blob loan --
    if (!info->data_writer_->discard_loan(loaned_message)) {
      return RMW_RET_ERROR;
    }
  }

  return RMW_RET_OK;
}
}  // namespace rmw_fastrtps_shared_cpp
