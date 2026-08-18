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

#include "rmw_fastrtps_shared_cpp/XcdrTypeSupport.hpp"

#include <cstring>
#include <memory>
#include <string>
#include <utility>

#include "fastcdr/Cdr.h"
#include "fastcdr/FastBuffer.h"

#include "rcutils/error_handling.h"

#include "rmw/error_handling.h"
#include "rmw/types.h"

#include "rosidl_runtime_cpp/experimental/memory.hpp"

#include "rosidl_typesupport_xcdr_c/identifier.h"
#include "rosidl_typesupport_xcdr_c/message_type_support.h"

#include "rosidl_typesupport_xcdr_cpp/identifier.hpp"
#include "rosidl_typesupport_xcdr_cpp/message_type_support.hpp"

#include "rmw_fastrtps_shared_cpp/TypeSupport.hpp"
#include "rmw_fastrtps_shared_cpp/custom_participant_info.hpp"

namespace rmw_fastrtps_shared_cpp
{

// ---- Static helpers (internal) ----

namespace
{

/// Walk the typesupport tree looking for an XCDR handle (any language).
const rosidl_message_type_support_t *
resolve_xcdr_handle(const rosidl_message_type_support_t * type_supports)
{
  if (nullptr == type_supports) {
    return nullptr;
  }

  // Language-agnostic family pattern: matches any XCDR typesupport
  // implementation (C, C++, CPython), regardless of which language generated
  // the callbacks.  The dispatch functions perform trailing-'*' prefix
  // matching via rosidl_runtime_c_typesupport_identifier_matches().
  return get_message_typesupport_handle(
    type_supports, "rosidl_typesupport_xcdr*");
}

}  // anonymous namespace

// ---- Constructor ----

XcdrTypeSupport::XcdrTypeSupport(
  const rosidl_message_type_support_t * type_supports,
  const rosidl_message_type_constraints_t * constraints,
  const std::string & type_name, bool assume_bounded)
{
  m_isGetKeyDefined = false;
  setName(type_name.c_str());

  // Auto-fill XTypes type information so that discovery carries
  // it for type matching, but explicitly drop type object as
  // type object comparison only works for simple dynamic types.
  auto_fill_type_object(false);
  auto_fill_type_information(true);

  // Resolve the base XCDR handle from the typesupport tree.
  base_handle_ = resolve_xcdr_handle(type_supports);

  // Create constrained handle when constraints are provided.
  // The handle itself clones and owns the constraints via clone_constraints.
  // create_constrained_message_type_support returns a shared_ptr with
  // automatic destruction.
  if (nullptr != base_handle_ && nullptr != constraints) {
    constrained_handle_ =
      rosidl_typesupport_xcdr_cpp::create_constrained_message_type_support(
        base_handle_, constraints);

  }

  // Cache the effective handle (raw pointer, valid while constrained_handle_ lives).
  cached_effective_ = constrained_handle_ ? constrained_handle_.get() : base_handle_;

  // Compute bounded / plain flags from expected sizes.  get_expected_size
  // reports 0 when the handle has no cached layout (i.e. the size is not
  // statically known), so a non-zero result means the layout is fixed.
  //
  // - An inherently fixed type (no variable-length members) reports a known
  //   expected size from its BASE handle.
  // - An UNBOUNDED type only becomes bounded via constraints; its
  //   constrained handle carries a cached layout with a known size.
  plain_ = false;
  bounded_ = assume_bounded;
  type_size_ = 0;

  if (nullptr != base_handle_) {
    size_t base_size = 0;
    if (RCUTILS_RET_OK == rosidl_typesupport_xcdr_cpp::get_expected_message_size(
        base_handle_, &base_size) && 0 < base_size)
    {
      // Inherently fixed layout.
      plain_ = true;
      bounded_ = true;
      type_size_ = static_cast<uint32_t>(base_size);
    } else if (nullptr != constrained_handle_) {
      size_t constrained_size = 0;
      if (RCUTILS_RET_OK == rosidl_typesupport_xcdr_cpp::get_expected_message_size(
          constrained_handle_.get(), &constrained_size) && 0 < constrained_size)
      {
        // Bounded via constraints.
        plain_ = true;
        bounded_ = true;
        type_size_ = static_cast<uint32_t>(constrained_size);
      }
    }
  }

  // if (0 == type_size_) {
  //   // Unbounded and unconstrained (or unresolved): use a generous default so
  //   // the DataReader can provide loaned samples of a reasonable size.  The
  //   // reader is non-plain, so the pool keeps PREALLOCATED_WITH_REALLOC and
  //   // this value only sizes the initial loan allocation.
  //   type_size_ = 4096;
  // }

  // m_typeSize includes only the representation header.  The wire layout is
  // [repr header 4B][XCDR block (its own CDR header + fields)]; the reader
  // bounds parsing by payload->length, so no in-payload size prefix or
  // representation-header options are needed.
  m_typeSize = type_size_ +
    static_cast<uint32_t>(eprosima::fastrtps::rtps::SerializedPayload_t::
    representation_header_size);
}

// ---- Destructor ----

XcdrTypeSupport::~XcdrTypeSupport()
{
  // constrained_handle_ is a shared_ptr — destroyed automatically.
  // cached_effective_ may have pointed to constrained_handle_.get().
  cached_effective_ = nullptr;
}

// ---- TopicDataType overrides ----

bool
XcdrTypeSupport::serialize(
  void * data,
  eprosima::fastrtps::rtps::SerializedPayload_t * payload)
{
  auto ser_data = static_cast<rmw_fastrtps_shared_cpp::SerializedData *>(data);

  if (ser_data->type == FASTRTPS_SERIALIZED_DATA_TYPE_CDR_BUFFER) {
    // Pre-serialized message: copy the raw CDR bytes into the payload.
    auto ser = static_cast<eprosima::fastcdr::Cdr *>(ser_data->data);
    if (nullptr == ser) {
      RMW_SET_ERROR_MSG("serialized Cdr is null");
      return false;
    }
    if (payload->max_size < ser->get_serialized_data_length()) {
      RMW_SET_ERROR_MSG("payload buffer too small");
      return false;
    }
    payload->length = static_cast<uint32_t>(ser->get_serialized_data_length());
    payload->encapsulation = ser->endianness() ==
      eprosima::fastcdr::Cdr::BIG_ENDIANNESS ? CDR_BE : CDR_LE;
    memcpy(payload->data, ser->get_buffer_pointer(), payload->length);
    return true;
  }

  if (ser_data->type != FASTRTPS_SERIALIZED_DATA_TYPE_ROS_MESSAGE) {
    RMW_SET_ERROR_MSG(
      "XcdrTypeSupport only handles ROS_MESSAGE and CDR_BUFFER data types");
    return false;
  }

  const void * ros_message = ser_data->data;
  if (nullptr == ros_message) {
    RMW_SET_ERROR_MSG("ros_message is null");
    return false;
  }

  const rosidl_message_type_support_t * handle = cached_effective_;
  if (nullptr == handle) {
    RMW_SET_ERROR_MSG("no XCDR typesupport handle available");
    return false;
  }

  // Compute serialized size.
  size_t serialized_size = 0;
  if (constrained_handle_) {
    if (RCUTILS_RET_OK != rosidl_typesupport_xcdr_cpp::get_expected_message_size(
        handle, &serialized_size))
    {
      RMW_SET_ERROR_MSG("failed to get expected message size");
      return false;
    }
  } else {
    if (RCUTILS_RET_OK != rosidl_typesupport_xcdr_cpp::get_message_size(
        handle, ros_message, &serialized_size))
    {
      RMW_SET_ERROR_MSG("failed to get message size");
      return false;
    }
  }

  // Validate when the handle has owned baseline constraints.
  // The trampoline encapsulates strict vs non-strict policy etc.
  const rosidl_message_type_constraints_t * baseline =
    rosidl_typesupport_xcdr_c_get_constraints(handle);
  if (nullptr != baseline && RCUTILS_RET_OK !=
    rosidl_typesupport_xcdr_cpp::validate_message(
        handle, baseline, ros_message))
  {
    RMW_SET_ERROR_MSG("validation failed for message");
    return false;
  }

  // Ensure payload has enough capacity.
  if (payload->max_size < serialized_size) {
    RMW_SET_ERROR_MSG("payload buffer too small");
    return false;
  }

  // Serialize into payload buffer.
  rosidl_runtime_cpp::MemoryRegion<void> storage(
    payload->data, serialized_size);
  if (RCUTILS_RET_OK != rosidl_typesupport_xcdr_cpp::serialize_message_into(
      handle, ros_message, storage))
  {
    RMW_SET_ERROR_MSG("XCDR serialization failed");
    return false;
  }

  // XCDR_BUFFERS uses XCDRv1 (PLAIN_CDR), little-endian by default.
  // This is the same wire format as the legacy FastCDR path.
  payload->encapsulation = CDR_LE;
  payload->length = static_cast<uint32_t>(serialized_size);
  return true;
}

bool
XcdrTypeSupport::serialize(
  void * data,
  eprosima::fastrtps::rtps::SerializedPayload_t * payload,
  eprosima::fastdds::dds::DataRepresentationId_t data_representation)
{
  // XCDR_BUFFERS is XCDRv1 (PLAIN_CDR) only.  Refuse to serialize XCDR2
  // payloads; endpoints are pinned to XCDR_DATA_REPRESENTATION via QoS.
  if (data_representation != eprosima::fastdds::dds::XCDR_DATA_REPRESENTATION) {
    RMW_SET_ERROR_MSG(
      "XCDR backend only supports XCDR v1 data representation");
    return false;
  }
  return serialize(data, payload);
}

bool
XcdrTypeSupport::deserialize(
  eprosima::fastrtps::rtps::SerializedPayload_t * payload,
  void * data)
{
  auto ser_data = static_cast<rmw_fastrtps_shared_cpp::SerializedData *>(data);

  if (ser_data->type == FASTRTPS_SERIALIZED_DATA_TYPE_ROS_MESSAGE_LOAN) {
    // Non-plain loaned take (unbounded types): cast the payload to a typed
    // view and store it in the holder's data slot.  payload->length is the
    // authoritative received size — a safe upper bound for the parser.  The
    // holder and the payload are owned by the same outstanding loan item, so
    // the view outlives this call until the loan is returned.
    const rosidl_message_type_support_t * handle = cached_effective_;
    if (nullptr == handle) {
      RMW_SET_ERROR_MSG("no XCDR typesupport handle available");
      ser_data->data = nullptr;
      return false;
    }

    void * view = nullptr;
    rosidl_runtime_cpp::MemoryRegion<void> storage(payload->data, payload->length);
    rcutils_ret_t ret = rosidl_typesupport_xcdr_cpp::cast_message_at(
      handle, storage, &view);
    if (RCUTILS_RET_OK != ret || nullptr == view) {
      rcutils_reset_error();
      RMW_SET_ERROR_MSG("failed to cast XCDR loan buffer to typed message");
      ser_data->data = nullptr;
      return false;
    }
    ser_data->data = view;
    return true;
  }

  if (ser_data->type == FASTRTPS_SERIALIZED_DATA_TYPE_CDR_BUFFER) {
    // Pre-serialized take: copy the raw CDR bytes out of the payload.
    auto buffer = static_cast<eprosima::fastcdr::FastBuffer *>(ser_data->data);
    if (nullptr == buffer) {
      RMW_SET_ERROR_MSG("serialized FastBuffer is null");
      return false;
    }
    if (!buffer->reserve(payload->length)) {
      RMW_SET_ERROR_MSG("cannot reserve serialized buffer");
      return false;
    }
    memcpy(buffer->getBuffer(), payload->data, payload->length);
    return true;
  }

  if (ser_data->type != FASTRTPS_SERIALIZED_DATA_TYPE_ROS_MESSAGE) {
    RMW_SET_ERROR_MSG(
      "XcdrTypeSupport only handles ROS_MESSAGE, CDR_BUFFER and ROS_MESSAGE_LOAN data types");
    return false;
  }

  void * ros_message = ser_data->data;
  if (nullptr == ros_message) {
    RMW_SET_ERROR_MSG("ros_message is null");
    return false;
  }

  const rosidl_message_type_support_t * handle = cached_effective_;
  if (nullptr == handle) {
    RMW_SET_ERROR_MSG("no XCDR typesupport handle available");
    return false;
  }

  rosidl_runtime_cpp::MemoryRegion<void> storage(
    payload->data, payload->length);
  if (RCUTILS_RET_OK != rosidl_typesupport_xcdr_cpp::deserialize_message_from(
      handle, storage, ros_message))
  {
    RMW_SET_ERROR_MSG("XCDR deserialization failed");
    return false;
  }

  // Validate when the handle has owned baseline constraints.
  const rosidl_message_type_constraints_t * baseline =
    rosidl_typesupport_xcdr_c_get_constraints(handle);
  if (nullptr != baseline && RCUTILS_RET_OK !=
    rosidl_typesupport_xcdr_cpp::validate_message(
        handle, baseline, ros_message))
  {
    RMW_SET_ERROR_MSG("validation failed for received message");
    return false;
  }

  return true;
}

std::function<uint32_t()>
XcdrTypeSupport::getSerializedSizeProvider(void * data)
{
  auto ser_data = static_cast<rmw_fastrtps_shared_cpp::SerializedData *>(data);

  if (ser_data->type == FASTRTPS_SERIALIZED_DATA_TYPE_CDR_BUFFER) {
    auto ser = static_cast<eprosima::fastcdr::Cdr *>(ser_data->data);
    return [ser]() -> uint32_t
           {
             if (nullptr != ser) {
               return static_cast<uint32_t>(ser->get_serialized_data_length());
             }
             return 0;
           };
  }

  const rosidl_message_type_support_t * handle = cached_effective_;
  const void * ros_message = (ser_data->type == FASTRTPS_SERIALIZED_DATA_TYPE_ROS_MESSAGE) ?
    ser_data->data : nullptr;
  bool is_constrained = (nullptr != constrained_handle_);

  return [handle, ros_message, is_constrained]() -> uint32_t
         {
           if (is_constrained) {
             size_t size = 0;
             if (RCUTILS_RET_OK == rosidl_typesupport_xcdr_cpp::get_expected_message_size(
                 handle, &size))
             {
               return static_cast<uint32_t>(size);
             }
             return 0;
           }
           if (nullptr != ros_message) {
             size_t size = 0;
             if (RCUTILS_RET_OK == rosidl_typesupport_xcdr_cpp::get_message_size(
                 handle, ros_message, &size))
             {
               return static_cast<uint32_t>(size);
             }
           }
           return 0;
         };
}

void *
XcdrTypeSupport::createData()
{
  // Non-plain loan-sample holder: deserialize() casts the payload and stores
  // the typed view pointer in SerializedData::data.
  auto * sd = new rmw_fastrtps_shared_cpp::SerializedData();
  sd->type = FASTRTPS_SERIALIZED_DATA_TYPE_ROS_MESSAGE_LOAN;
  sd->data = nullptr;
  sd->impl = nullptr;
  return sd;
}

void
XcdrTypeSupport::deleteData(void * data)
{
  if (data) {
    delete static_cast<rmw_fastrtps_shared_cpp::SerializedData *>(data);
  }
}

bool
XcdrTypeSupport::construct_sample(void * memory) const
{
  // Only constrained types with fixed layout support in-place construction.
  // Note: This is a TopicDataType virtual called by Fast DDS in non-loaning
  // contexts as well; the XCDR block (its own CDR header + fields) is
  // written at memory[0].
  if (!constrained_handle_ || !memory) {
    return false;
  }

  // The data area after the representation header has
  // m_typeSize - representation_header_size bytes.
  size_t data_size = static_cast<size_t>(m_typeSize) -
    eprosima::fastrtps::rtps::SerializedPayload_t::representation_header_size;
  void * message = nullptr;
  rosidl_runtime_cpp::MemoryRegion<void> storage(
    memory, data_size);
  if (RCUTILS_RET_OK != rosidl_typesupport_xcdr_cpp::construct_message_at(
      get_effective_handle(), storage, &message))
  {
    rcutils_reset_error();
    return false;
  }
  return message != nullptr;
}

bool
XcdrTypeSupport::cast_sample(
  void * buffer, size_t size, void ** message) const
{
  if (!buffer || !message) {
    return false;
  }

  rosidl_runtime_cpp::MemoryRegion<void> storage(buffer, size);
  return RCUTILS_RET_OK == rosidl_typesupport_xcdr_cpp::cast_message_at(
    get_effective_handle(), storage, message);
}

bool
XcdrTypeSupport::compact_loaned_message(
  void * message,
  void ** blob_out,
  size_t * actual_data_size) const
{
  if (!constrained_handle_ || !message || !blob_out || !actual_data_size) {
    return false;
  }

  const rosidl_message_type_support_t * handle = get_effective_handle();
  if (nullptr == handle) {
    RMW_SET_ERROR_MSG("no effective handle for compaction");
    return false;
  }

  *blob_out = nullptr;
  *actual_data_size = 0;

  rosidl_memory_region_t region =
    rosidl_typesupport_xcdr_cpp::compact_message_in_place(
      handle, message);

  if (nullptr == region.location.address) {
    rcutils_reset_error();
    return false;
  }

  *blob_out = region.location.address;
  *actual_data_size = region.size;
  return true;
}

bool
XcdrTypeSupport::getKey(
  void * data,
  eprosima::fastrtps::rtps::InstanceHandle_t * ihandle,
  bool force_md5)
{
  (void)data;
  (void)ihandle;
  (void)force_md5;
  return false;
}

bool
XcdrTypeSupport::is_bounded() const
{
  return bounded_;
}

bool
XcdrTypeSupport::is_plain() const
{
  return plain_;
}

bool
XcdrTypeSupport::is_plain(
  eprosima::fastdds::dds::DataRepresentationId_t) const
{
  return plain_;
}

bool
XcdrTypeSupport::supports_loans() const
{
  if (nullptr == cached_effective_ || nullptr == cached_effective_->data) {
    return false;
  }
  auto * xcdr = static_cast<const rosidl_message_xcdr_type_support_t *>(
    cached_effective_->data);
  if (nullptr == xcdr || nullptr == xcdr->inner) {
    return false;
  }
  auto * impl =
    static_cast<const rosidl_typesupport_xcdr_cpp::rosidl_message_xcdr_cpp_type_support_t *>(
    xcdr->inner);
  return nullptr != impl->cast_message;
}

const rosidl_message_type_support_t *
XcdrTypeSupport::get_base_handle() const
{
  return base_handle_;
}

size_t
XcdrTypeSupport::get_expected_data_size_for_handle(
  const rosidl_message_type_support_t * handle)
{
  if (nullptr == handle) {
    return 0;
  }
  size_t expected_size = 0;
  if (RCUTILS_RET_OK != rosidl_typesupport_xcdr_cpp::get_expected_message_size(
      handle, &expected_size))
  {
    return 0;
  }
  return expected_size;
}

const rosidl_message_type_support_t *
XcdrTypeSupport::get_effective_handle() const
{
  return cached_effective_;
}

size_t
XcdrTypeSupport::get_expected_data_size() const
{
  // m_typeSize = type_size_ + representation_header_size for all types
  // (no in-payload size prefix).  The XCDR data area in a loaned sample
  // (after the representation header) is exactly type_size_.
  // type_size_ == 0 means no resolved handle / no typed view possible.
  return static_cast<size_t>(type_size_);
}

bool
resolve_constrained_endpoint(
  CustomParticipantInfo * participant_info,
  const std::string & topic_name_mangled,
  const std::string & base_type_name,
  const rosidl_message_type_constraints_t * constraints,
  std::string * topic_type_name)
{
  eprosima::fastdds::dds::TopicDescription * topic =
    participant_info->participant_->lookup_topicdescription(topic_name_mangled);

  if (nullptr == topic) {
    // First endpoint on this topic.  Constrained variants get a unique local
    // type name so different topics can carry different constrained variants
    // of the same message type without colliding in the participant's type
    // registry; unconstrained endpoints keep the base name (matching by name
    // until type objects are registered).
    if (nullptr == constraints) {
      *topic_type_name = base_type_name;
    } else {
      size_t n = ++participant_info->type_name_counter_;
      *topic_type_name = base_type_name + "#" + std::to_string(n);
    }
    return true;
  }

  // Existing topic: the endpoint shares the registered type for this topic.
  const std::string topic_type = topic->get_type_name();
  eprosima::fastdds::dds::TypeSupport registered =
    participant_info->participant_->find_type(topic_type);
  if (registered.empty()) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "existing topic %s has no registered type %s",
      topic_name_mangled.c_str(), topic_type.c_str());
    return false;
  }

  // The new endpoint's constraints must be compatible with the registered
  // type support; otherwise creating it would silently use the wrong bounds.
  bool compatible = true;
  auto * xcdr_ts = dynamic_cast<XcdrTypeSupport *>(registered.get());
  if (nullptr != xcdr_ts) {
    const rosidl_message_type_support_t * handle = xcdr_ts->get_effective_handle();
    const rosidl_message_type_constraints_t * baseline =
      handle ? rosidl_typesupport_xcdr_c_get_constraints(handle) : nullptr;
    if (nullptr != baseline && nullptr != constraints) {
      compatible = rosidl_typesupport_xcdr_cpp::compare_constraints(
        handle, constraints, baseline);
    } else if (nullptr == baseline && nullptr != constraints) {
      // A bounded endpoint on an unbounded registered type: the registered
      // type cannot provide proper loan sizing for the bounded endpoint.
      compatible = false;
    }
    // Registered bounded + unconstrained endpoint: allowed (the endpoint
    // reuses the bounded registered type; its data must fit the bound).
    // Both unconstrained: allowed.
  } else {
    // Non-XCDR registered type: fall back to the type-name check.
    compatible = (topic_type == base_type_name);
  }

  if (!compatible) {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "existing topic %s is already registered with incompatible constraints "
      "for type %s; cannot create another constrained endpoint with different bounds",
      topic_name_mangled.c_str(), base_type_name.c_str());
    return false;
  }

  // Compatible: share the topic's registered type name.
  *topic_type_name = topic_type;
  return true;
}

}  // namespace rmw_fastrtps_shared_cpp
