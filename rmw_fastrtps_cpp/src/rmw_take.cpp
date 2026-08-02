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

#include <exception>
#include <mutex>

#include "rmw/allocators.h"
#include "rmw/error_handling.h"
#include "rmw/serialized_message.h"
#include "rmw/rmw.h"

#include "rmw_fastrtps_shared_cpp/custom_subscriber_info.hpp"
#include "rmw_fastrtps_shared_cpp/rmw_common.hpp"
#include "rmw_fastrtps_shared_cpp/TypeSupport.hpp"
#include "rmw_fastrtps_shared_cpp/XcdrTypeSupport.hpp"

#include "rosidl_runtime_cpp/experimental/memory.hpp"

#include "rosidl_typesupport_xcdr_c/message_type_support.h"
#include "rosidl_typesupport_xcdr_cpp/message_type_support.hpp"

#include "rmw_fastrtps_cpp/identifier.hpp"

namespace
{

/// Interpret a loaned XCDR sample as a typed view.
/**
 * Bounded XCDR types pose as plain, so the stock shared
 * __rmw_take_loaned_message_internal loans the raw payload: the sample is
 * payload.data + representation_header_size (fields start) and the XCDR
 * block (its own CDR header + fields) begins 4 bytes before it.  The cast
 * runs HERE, outside the typesupport, with the constrained allocation as a
 * safe parser bound — the layout parser follows the wire format's own length
 * prefixes and never reads trailing padding.
 *
 * Unbounded XCDR types are non-plain, so the loan element is a
 * SerializedData holder tagged with FASTRTPS_SERIALIZED_DATA_TYPE_XCDR_LOAN_VIEW
 * whose data slot was set by the typesupport's deserialize() to a typed view
 * of the payload (the cast happened there, with the authoritative
 * payload->length).
 *
 * In both cases *loaned_message is replaced with the view and the view->raw
 * mapping is registered for rmw_return_loaned_message.  On failure the raw
 * sample/holder is left in *loaned_message so the caller can return it.
 *
 * Per-loan constraints (when given) are pre-checked against the subscription
 * baseline and then enforced on the view.
 */
rmw_ret_t
xcdr_maybe_view_loaned_message(
  CustomSubscriberInfo * info,
  const rosidl_message_type_constraints_t * type_constraints,
  void ** loaned_message,
  bool taken)
{
  if (!taken || nullptr == info || nullptr == loaned_message ||
    nullptr == *loaned_message)
  {
    return RMW_RET_OK;
  }

  auto * type_ptr = info->type_support_.get();
  auto * xcdr_ts =
    dynamic_cast<rmw_fastrtps_shared_cpp::XcdrTypeSupport *>(type_ptr);
  if (nullptr == xcdr_ts) {
    return RMW_RET_OK;  // not an XCDR-backed subscription
  }

  void * view = nullptr;
  std::shared_ptr<rosidl_message_type_support_t> destroy_handle(
    const_cast<rosidl_message_type_support_t *>(xcdr_ts->get_effective_handle()),
    [](rosidl_message_type_support_t *) {});

  if (xcdr_ts->is_plain()) {
    // -- Bounded (plain) loan: the sample is the raw payload; cast here. --
    void * raw_buffer = *loaned_message;
    uint8_t * repr_hdr = static_cast<uint8_t *>(raw_buffer) -
      eprosima::fastrtps::rtps::SerializedPayload_t::representation_header_size;

    // XCDR v1 only: the CDR encapsulation field (bytes 0-1) must be CDR_LE
    // (0x0001) or CDR_BE (0x0000).  Anything else — e.g. XCDR2 PL_CDR
    // (0x0002 / 0x0003) — is rejected (defensive; endpoints are pinned to
    // XCDR_DATA_REPRESENTATION via QoS).
    uint16_t encapsulation = (static_cast<uint16_t>(repr_hdr[0]) << 8) | repr_hdr[1];
    if (0x0000 != encapsulation && 0x0001 != encapsulation) {
      rcutils_reset_error();
      RMW_SET_ERROR_MSG(
        "unsupported CDR encapsulation in representation header "
        "(XCDR backend is XCDR v1 only)");
      return RMW_RET_UNSUPPORTED;
    }

    // Cast using the constrained allocation as a safe parser bound.
    size_t data_size = xcdr_ts->get_expected_data_size();
    if (0 == data_size) {
      RMW_SET_ERROR_MSG("XCDR type has no expected data size for loaned take");
      return RMW_RET_ERROR;
    }
    rosidl_runtime_cpp::MemoryRegion<void> storage(repr_hdr, data_size);
    rcutils_ret_t ret = RMW_RET_ERROR;
    try {
      // cast_message_at can throw through the generated accessor paths; the
      // loan must still be returned, so the caller relies on this function
      // never throwing.
      ret = rosidl_typesupport_xcdr_cpp::cast_message_at(
        destroy_handle.get(), storage, &view);
    } catch (const std::exception & e) {
      rcutils_reset_error();
      RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
        "exception during XCDR loan cast: %s", e.what());
      return RMW_RET_ERROR;
    }
    if (RCUTILS_RET_OK != ret || nullptr == view) {
      rcutils_reset_error();
      RMW_SET_ERROR_MSG("failed to cast XCDR loan buffer to typed message");
      return RMW_RET_ERROR;
    }
  } else {
    // -- Unbounded (non-plain) loan: the typesupport's deserialize cast the
    //    payload into the holder's data slot. --
    auto * holder = static_cast<rmw_fastrtps_shared_cpp::SerializedData *>(*loaned_message);
    if (holder->type != rmw_fastrtps_shared_cpp::FASTRTPS_SERIALIZED_DATA_TYPE_XCDR_LOAN_VIEW) {
      RMW_SET_ERROR_MSG("loaned sample is not an XCDR view holder");
      return RMW_RET_ERROR;
    }
    view = holder->data;
    if (nullptr == view) {
      // The typesupport's deserialize failed to cast the payload (e.g. an
      // XCDR2 payload rejected by the layout parser's header validation).
      RMW_SET_ERROR_MSG("XCDR loaned sample has no typed view (cast failed)");
      return RMW_RET_ERROR;
    }
  }

  if (type_constraints) {
    // Pre-check per-loan constraints as upper bounds against the
    // subscription-wide baseline (tighter-or-equal accepted, looser rejected).
    const rosidl_message_type_support_t * ts_handle = xcdr_ts->get_effective_handle();
    const rosidl_message_type_constraints_t * baseline =
      ts_handle ? rosidl_typesupport_xcdr_c_get_constraints(ts_handle) : nullptr;
    if (!rosidl_typesupport_xcdr_cpp::compare_constraints(
        ts_handle, type_constraints, baseline))
    {
      rosidl_typesupport_xcdr_cpp::destroy_message(destroy_handle.get(), view);
      RMW_SET_ERROR_MSG(
        "per-loan constraints exceed subscription-wide bounds; "
        "set looser constraints at create_subscription time instead");
      return RMW_RET_CONSTRAINTS_HIT;
    }

    // Enforce the tighter per-loan bounds on the typed view.
    const rosidl_message_type_support_t * base_handle = xcdr_ts->get_base_handle();
    if (nullptr == base_handle) {
      rosidl_typesupport_xcdr_cpp::destroy_message(destroy_handle.get(), view);
      RMW_SET_ERROR_MSG("XCDR base handle not available for subscription");
      return RMW_RET_ERROR;
    }
    auto per_loan =
      rosidl_typesupport_xcdr_cpp::create_constrained_message_type_support(
        base_handle, type_constraints);
    if (!per_loan) {
      rcutils_reset_error();
      rosidl_typesupport_xcdr_cpp::destroy_message(destroy_handle.get(), view);
      RMW_SET_ERROR_MSG("failed to create constrained handle for subscription take");
      return RMW_RET_ERROR;
    }
    const rosidl_message_type_constraints_t * per_loan_constraints =
      rosidl_typesupport_xcdr_c_get_constraints(per_loan.get());
    if (nullptr != per_loan_constraints && RCUTILS_RET_OK !=
      rosidl_typesupport_xcdr_cpp::validate_message(
          per_loan.get(), per_loan_constraints, view))
    {
      rcutils_reset_error();
      rosidl_typesupport_xcdr_cpp::destroy_message(destroy_handle.get(), view);
      RMW_SET_ERROR_MSG("loaned message violates per-loan constraints");
      return RMW_RET_CONSTRAINTS_HIT;
    }
  }

  // Record the mapping so return_loan can destroy the view and recover the
  // raw sample / holder.
  {
    std::lock_guard<std::mutex> guard(info->xcdr_loan_mutex_);
    info->xcdr_loan_map_[view] = {*loaned_message, std::move(destroy_handle)};
  }
  *loaned_message = view;
  return RMW_RET_OK;
}

}  // anonymous namespace

extern "C"
{
rmw_ret_t
rmw_take(
  const rmw_subscription_t * subscription,
  void * ros_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation)
{
  return rmw_fastrtps_shared_cpp::__rmw_take(
    eprosima_fastrtps_identifier, subscription, ros_message, taken, allocation);
}

rmw_ret_t
rmw_take_with_info(
  const rmw_subscription_t * subscription,
  void * ros_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  return rmw_fastrtps_shared_cpp::__rmw_take_with_info(
    eprosima_fastrtps_identifier, subscription, ros_message, taken, message_info, allocation);
}

rmw_ret_t
rmw_take_sequence(
  const rmw_subscription_t * subscription,
  size_t count,
  rmw_message_sequence_t * message_sequence,
  rmw_message_info_sequence_t * message_info_sequence,
  size_t * taken,
  rmw_subscription_allocation_t * allocation)
{
  return rmw_fastrtps_shared_cpp::__rmw_take_sequence(
    eprosima_fastrtps_identifier, subscription, count, message_sequence, message_info_sequence,
    taken, allocation);
}

rmw_ret_t
rmw_take_serialized_message(
  const rmw_subscription_t * subscription,
  rmw_serialized_message_t * serialized_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation)
{
  return rmw_fastrtps_shared_cpp::__rmw_take_serialized_message(
    eprosima_fastrtps_identifier, subscription, serialized_message, taken, allocation);
}

rmw_ret_t
rmw_take_serialized_message_with_info(
  const rmw_subscription_t * subscription,
  rmw_serialized_message_t * serialized_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  return rmw_fastrtps_shared_cpp::__rmw_take_serialized_message_with_info(
    eprosima_fastrtps_identifier, subscription, serialized_message, taken, message_info,
    allocation);
}

rmw_ret_t
rmw_take_loaned_message(
  const rmw_subscription_t * subscription,
  void ** loaned_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation)
{
  static_cast<void>(allocation);
  auto info = static_cast<CustomSubscriberInfo *>(subscription->data);

  auto * xcdr_ts = dynamic_cast<rmw_fastrtps_shared_cpp::XcdrTypeSupport *>(
    info->type_support_.get());
  if (nullptr == xcdr_ts) {
    // Non-XCDR backend: stock loaned take.
    return rmw_fastrtps_shared_cpp::__rmw_take_loaned_message_internal(
      eprosima_fastrtps_identifier, subscription, loaned_message, taken, nullptr);
  }

  // XCDR backend: the stock loaned take hands out the view holder; interpret
  // it as a typed view.
  void * loaned = nullptr;
  rmw_ret_t ret = rmw_fastrtps_shared_cpp::__rmw_take_loaned_message_internal(
    eprosima_fastrtps_identifier, subscription, &loaned, taken, nullptr);
  if (RMW_RET_OK != ret || !*taken) {
    return ret;
  }
  void * holder = loaned;
  rmw_ret_t view_ret = xcdr_maybe_view_loaned_message(info, nullptr, &loaned, *taken);
  if (RMW_RET_OK != view_ret) {
    // Release the raw holder loan that was registered by the shared internal take.
    rmw_fastrtps_shared_cpp::__rmw_return_loaned_message_from_subscription(
      eprosima_fastrtps_identifier, subscription, holder);
    return view_ret;
  }
  *loaned_message = loaned;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_take_loaned_message_with_info(
  const rmw_subscription_t * subscription,
  void ** loaned_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  static_cast<void>(allocation);
  RMW_CHECK_ARGUMENT_FOR_NULL(message_info, RMW_RET_INVALID_ARGUMENT);
  auto info = static_cast<CustomSubscriberInfo *>(subscription->data);

  auto * xcdr_ts = dynamic_cast<rmw_fastrtps_shared_cpp::XcdrTypeSupport *>(
    info->type_support_.get());
  if (nullptr == xcdr_ts) {
    return rmw_fastrtps_shared_cpp::__rmw_take_loaned_message_internal(
      eprosima_fastrtps_identifier, subscription, loaned_message, taken, message_info);
  }

  void * loaned = nullptr;
  rmw_ret_t ret = rmw_fastrtps_shared_cpp::__rmw_take_loaned_message_internal(
    eprosima_fastrtps_identifier, subscription, &loaned, taken, message_info);
  if (RMW_RET_OK != ret || !*taken) {
    return ret;
  }
  void * holder = loaned;
  rmw_ret_t view_ret = xcdr_maybe_view_loaned_message(info, nullptr, &loaned, *taken);
  if (RMW_RET_OK != view_ret) {
    rmw_fastrtps_shared_cpp::__rmw_return_loaned_message_from_subscription(
      eprosima_fastrtps_identifier, subscription, holder);
    return view_ret;
  }
  *loaned_message = loaned;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_take_loaned_message_with_constraints(
  const rmw_subscription_t * subscription,
  const rosidl_message_type_constraints_t * type_constraints,
  void ** loaned_message,
  bool * taken,
  rmw_subscription_allocation_t * allocation)
{
  static_cast<void>(allocation);
  auto info = static_cast<CustomSubscriberInfo *>(subscription->data);

  auto * xcdr_ts = dynamic_cast<rmw_fastrtps_shared_cpp::XcdrTypeSupport *>(
    info->type_support_.get());
  if (nullptr == xcdr_ts) {
    RMW_SET_ERROR_MSG(
      "per-loan constraints are only supported on XCDR-backed subscriptions");
    return RMW_RET_UNSUPPORTED;
  }

  void * loaned = nullptr;
  rmw_ret_t ret = rmw_fastrtps_shared_cpp::__rmw_take_loaned_message_internal(
    eprosima_fastrtps_identifier, subscription, &loaned, taken, nullptr);
  if (RMW_RET_OK != ret || !*taken) {
    return ret;
  }
  void * holder = loaned;
  rmw_ret_t view_ret = xcdr_maybe_view_loaned_message(info, type_constraints, &loaned, *taken);
  if (RMW_RET_OK != view_ret) {
    rmw_fastrtps_shared_cpp::__rmw_return_loaned_message_from_subscription(
      eprosima_fastrtps_identifier, subscription, holder);
    return view_ret;
  }
  *loaned_message = loaned;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_take_loaned_message_with_info_and_constraints(
  const rmw_subscription_t * subscription,
  const rosidl_message_type_constraints_t * type_constraints,
  void ** loaned_message,
  bool * taken,
  rmw_message_info_t * message_info,
  rmw_subscription_allocation_t * allocation)
{
  static_cast<void>(allocation);
  RMW_CHECK_ARGUMENT_FOR_NULL(message_info, RMW_RET_INVALID_ARGUMENT);
  auto info = static_cast<CustomSubscriberInfo *>(subscription->data);

  auto * xcdr_ts = dynamic_cast<rmw_fastrtps_shared_cpp::XcdrTypeSupport *>(
    info->type_support_.get());
  if (nullptr == xcdr_ts) {
    RMW_SET_ERROR_MSG(
      "per-loan constraints are only supported on XCDR-backed subscriptions");
    return RMW_RET_UNSUPPORTED;
  }

  void * loaned = nullptr;
  rmw_ret_t ret = rmw_fastrtps_shared_cpp::__rmw_take_loaned_message_internal(
    eprosima_fastrtps_identifier, subscription, &loaned, taken, message_info);
  if (RMW_RET_OK != ret || !*taken) {
    return ret;
  }
  void * holder = loaned;
  rmw_ret_t view_ret = xcdr_maybe_view_loaned_message(info, type_constraints, &loaned, *taken);
  if (RMW_RET_OK != view_ret) {
    rmw_fastrtps_shared_cpp::__rmw_return_loaned_message_from_subscription(
      eprosima_fastrtps_identifier, subscription, holder);
    return view_ret;
  }
  *loaned_message = loaned;
  return RMW_RET_OK;
}

rmw_ret_t
rmw_return_loaned_message_from_subscription(
  const rmw_subscription_t * subscription,
  void * loaned_message)
{
  auto info = static_cast<CustomSubscriberInfo *>(subscription->data);

  auto * xcdr_ts = dynamic_cast<rmw_fastrtps_shared_cpp::XcdrTypeSupport *>(
    info->type_support_.get());

  if (xcdr_ts) {
    // -- XCDR backend: typed-view lifecycle --
    void * holder = nullptr;
    {
      std::lock_guard<std::mutex> lock(info->xcdr_loan_mutex_);
      auto it = info->xcdr_loan_map_.find(loaned_message);
      if (it == info->xcdr_loan_map_.end()) {
        RMW_SET_ERROR_MSG("returned message not found in XCDR loan map");
        return RMW_RET_ERROR;
      }

      // Destroy the typed view using the handle that created it.
      rosidl_typesupport_xcdr_cpp::destroy_message(
        it->second.handle.get(), loaned_message);

      holder = it->second.holder;
      info->xcdr_loan_map_.erase(it);
    }

    // Return the holder through the shared internal return.
    return rmw_fastrtps_shared_cpp::__rmw_return_loaned_message_from_subscription(
      eprosima_fastrtps_identifier, subscription, holder);
  }

  // -- Non-XCDR backend: raw-blob return --
  return rmw_fastrtps_shared_cpp::__rmw_return_loaned_message_from_subscription(
    eprosima_fastrtps_identifier, subscription, loaned_message);
}

rmw_ret_t
rmw_take_event(
  const rmw_event_t * event_handle,
  void * event_info,
  bool * taken)
{
  return rmw_fastrtps_shared_cpp::__rmw_take_event(
    eprosima_fastrtps_identifier, event_handle, event_info, taken);
}
}  // extern "C"
