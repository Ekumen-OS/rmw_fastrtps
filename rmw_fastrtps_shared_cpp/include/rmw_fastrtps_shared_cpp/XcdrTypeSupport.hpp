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

#ifndef RMW_FASTRTPS_SHARED_CPP__XCDRTYPESUPPORT_HPP_
#define RMW_FASTRTPS_SHARED_CPP__XCDRTYPESUPPORT_HPP_

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "fastdds/dds/topic/TopicDataType.hpp"
#include "fastdds/rtps/common/InstanceHandle.h"
#include "fastdds/rtps/common/SerializedPayload.h"

#include "rosidl_runtime_c/message_type_support_struct.h"

#include "rmw_fastrtps_shared_cpp/TypeSupport.hpp"
#include "rmw_fastrtps_shared_cpp/visibility_control.h"

// CustomParticipantInfo is a C-style struct typedef in the global namespace
// (see custom_participant_info.hpp); forward-declare it the same way so the
// declaration below matches the real type.
typedef struct CustomParticipantInfo CustomParticipantInfo;

namespace rmw_fastrtps_shared_cpp
{

/// TopicDataType implementation backed by the XCDR typesupport callbacks.
/**
 * A separate, parallel TypeSupport class for the XCDR_BUFFERS backend.
 * Uses rosidl_typesupport_xcdr_cpp trampolines (serialize_message_into,
 * deserialize_message_from, cast_message_at, etc.) instead of FastCDR.
 *
 * The XCDR backend works with raw loaned samples on the take side:
 *
 * - The type poses as plain only when bounded (is_plain(rep) == true for
 *   XCDR when the type is bounded).  This unlocks DataWriter::loan_sample
 *   for loaned publish, and on the reader it makes Fast DDS loan raw
 *   payloads (pointer arithmetic, no deserialization).  The loaned-take
 *   path casts the payload with cast_message_at using the constrained
 *   allocation as a safe parser bound — the layout parser follows the
 *   wire format's own length prefixes and never reads trailing padding,
 *   so no size needs to be encoded in the CDR representation header.
 *
 * - Unbounded (unconstrained) types never pose as plain and are never
 *   loan-taken: the codegen emits cast_message only for types with
 *   constraints support, so unbounded types cannot produce a typed view.
 *   Their subscriptions advertise can_loan_messages = false and the
 *   executor falls back to copy take (deserialize_message_from).
 *
 * The type support is shared per type name across endpoints (Fast DDS
 * registers one TopicDataType per type), so its plainness cannot depend on
 * the endpoint role; it depends only on whether the type is bounded.
 */
class XcdrTypeSupport : public eprosima::fastdds::dds::TopicDataType
{
public:
  /// Construct an XCDR-backed TopicDataType.
  /**
   * \param[in] type_supports  Full typesupport handle tree (as passed to
   *                           rmw_create_publisher / rmw_create_subscription).
   *                           Walked internally to find the XCDR identifier.
   * \param[in] constraints    Optional per-message-type constraints for
   *                           bounded message sizes.  May be nullptr.
   * \param[in] type_name      Fully-qualified type name for Fast DDS
   *                           registration (e.g. "my_pkg::msg::dds_::MyMsg_").
   */
  RMW_FASTRTPS_SHARED_CPP_PUBLIC
  XcdrTypeSupport(
    const rosidl_message_type_support_t * type_supports,
    const rosidl_message_type_constraints_t * constraints,
    const std::string & type_name);

  RMW_FASTRTPS_SHARED_CPP_PUBLIC
  ~XcdrTypeSupport() override;

  // Non-copyable, non-movable (owns constrained handle).
  XcdrTypeSupport(const XcdrTypeSupport &) = delete;
  XcdrTypeSupport & operator=(const XcdrTypeSupport &) = delete;

  // ---- TopicDataType interface ----

  RMW_FASTRTPS_SHARED_CPP_PUBLIC
  bool serialize(void * data, eprosima::fastrtps::rtps::SerializedPayload_t * payload) override;

  RMW_FASTRTPS_SHARED_CPP_PUBLIC
  bool serialize(
    void * data,
    eprosima::fastrtps::rtps::SerializedPayload_t * payload,
    eprosima::fastdds::dds::DataRepresentationId_t data_representation) override;

  RMW_FASTRTPS_SHARED_CPP_PUBLIC
  bool deserialize(
    eprosima::fastrtps::rtps::SerializedPayload_t * payload,
    void * data) override;

  RMW_FASTRTPS_SHARED_CPP_PUBLIC
  std::function<uint32_t()> getSerializedSizeProvider(void * data) override;

  RMW_FASTRTPS_SHARED_CPP_PUBLIC
  void * createData() override;

  RMW_FASTRTPS_SHARED_CPP_PUBLIC
  void deleteData(void * data) override;

  RMW_FASTRTPS_SHARED_CPP_PUBLIC
  bool getKey(
    void * data,
    eprosima::fastrtps::rtps::InstanceHandle_t * ihandle,
    bool force_md5) override;

  RMW_FASTRTPS_SHARED_CPP_PUBLIC
  bool is_bounded() const override;

  /// Pose as plain only when the type is bounded (XCDR v1).
  RMW_FASTRTPS_SHARED_CPP_PUBLIC
  bool is_plain() const override;

  /// Pose as plain only when the type is bounded and the representation is XCDR v1.
  RMW_FASTRTPS_SHARED_CPP_PUBLIC
  bool is_plain(eprosima::fastdds::dds::DataRepresentationId_t rep) const override;

  /// Return the effective typesupport handle (constrained if available).
  RMW_FASTRTPS_SHARED_CPP_PUBLIC
  const rosidl_message_type_support_t *
  get_effective_handle() const;

  /// Whether loaned takes can produce typed views for this type.
  /**
   * The codegen emits the cast_message callback only for types with
   * constraints support; without it, the loan path cannot create a typed
   * view of the payload, so the subscription must not advertise loan
   * capability (the executor falls back to copy take).
   */
  RMW_FASTRTPS_SHARED_CPP_PUBLIC
  bool supports_loans() const;

  /// Return the base XCDR typesupport handle (before constraints).
  /**
   * This is the handle resolved from the typesupport tree, before applying
   * publisher-wide constraints.  Used as the starting point for creating
   * per-loan constrained handles.
   *
   * The returned pointer is valid for the lifetime of this XcdrTypeSupport object.
   * Returns nullptr if XCDR handle resolution failed.
   */
  RMW_FASTRTPS_SHARED_CPP_PUBLIC
  const rosidl_message_type_support_t *
  get_base_handle() const;

  /// Compute the expected data area size from an arbitrary handle.
  /**
   * Wrapper around rosidl_typesupport_xcdr_cpp::get_expected_message_size.
   * Returns 0 on failure or for unconstrained handles.
   */
  RMW_FASTRTPS_SHARED_CPP_PUBLIC
  static size_t get_expected_data_size_for_handle(
    const rosidl_message_type_support_t * handle);

  /// Return the expected data area size (usable bytes after the repr header).
  /**
   * This is the number of bytes available after the Fast DDS representation
   * header in a loaned sample, equal to the XCDR layout size computed from
   * constraints (or the default pool size for unconstrained types).
   * Returns 0 for unresolved handles.
   */
  RMW_FASTRTPS_SHARED_CPP_PUBLIC
  size_t get_expected_data_size() const;

  /// Construct a message in-place on a loaned sample buffer.
  /**
   * Only available when the type has a constrained handle (bounded layout).
   * For unconstrained types, returns false (the caller falls back to
   * default loan initialization).
   *
   * \param[in] memory  Pointer to the memory where the sample should be
   *                    constructed.  Guaranteed by the Fast DDS loan
   *                    infrastructure to be at least m_typeSize bytes.
   *                    The compacted XCDR data (with its own CDR header)
   *                    is written starting at this pointer.
   * \return true if the sample was successfully constructed in-place.
   */
  RMW_FASTRTPS_SHARED_CPP_PUBLIC
  bool construct_sample(void * memory) const override;

  /// Cast a loaned buffer to a typed message pointer (take-side zero-copy).
  /**
   * Interprets the raw XCDR layout data in the buffer as a typed ROS
   * message, returning a pointer suitable for the caller to read fields
   * from.  The returned pointer is a lightweight wrapper whose lifetime
   * is tied to the buffer — it does not own the data.
   *
   * Only available when the type has a constrained handle.
   *
   * \param[in]  buffer   Pointer to the XCDR data area (payload.data).
   * \param[in]  size     Size of the XCDR data area in bytes (payload->length).
   * \param[out] message  On success, set to a typed message pointer
   *                      backed by buffer.
   * \return true if the buffer was successfully cast to a typed message.
   */
  RMW_FASTRTPS_SHARED_CPP_PUBLIC
  bool cast_sample(
    void * buffer, size_t size, void ** message) const;

  /// Consume a loaned message view by compacting it in-place.
  /**
   * Validates the message against baseline constraints, re-writes compact
   * XCDR encoding into the backing buffer, and destroys the message view.
   * On success, *blob_out is set to the blob pointer (same value that
   * loan_sample returned) and *actual_data_size to the compact payload
   * length (including the compacted data's own CDR header).
   *
   * Only available when the type has a constrained handle.
   *
   * \param[in]  message   Message view to consume and compact.
   * \param[out] blob_out  On success, set to blob pointer for Fast DDS.
   * \param[out] actual_data_size  On success, set to compact payload bytes.
   * \return true on success, false on constraint violation or error.
   */
  RMW_FASTRTPS_SHARED_CPP_PUBLIC
  bool compact_loaned_message(
    void * message,
    void ** blob_out,
    size_t * actual_data_size) const;

private:
  /// Base XCDR typesupport handle (not owned).  Null if resolution failed.
  const rosidl_message_type_support_t * base_handle_{nullptr};

  /// Constrained handle (owned via shared_ptr, auto-destroyed).  Null if none.
  std::shared_ptr<rosidl_message_type_support_t> constrained_handle_{nullptr};

  /// Cached effective handle for fast access (constrained_ or base_).
  /// Set once in the constructor before the object is registered with Fast DDS.
  const rosidl_message_type_support_t * cached_effective_{nullptr};

  /// Type size hint.  0 means unbounded/unresolved.
  uint32_t type_size_{0};

  /// Whether this type is fully bounded (fixed layout known, from base or constraints).
  bool bounded_{false};
};

/// Resolve the type name and registration strategy for a constrained XCDR
/// endpoint (publisher or subscription) on a given topic.
///
/// Fast DDS manages a single (topic name, type name) tuple per topic: a topic
/// has exactly one type name, and a participant cannot register two type
/// supports under the same name.  Constrained variants of the same message
/// type are different types for serialization / deserialization / loaning but
/// equivalent for matching, so a constrained endpoint registers under a unique
/// local type name (base name + "#" + per-participant counter); an
/// unconstrained endpoint keeps the base name.  Two endpoints on the same
/// topic must use compatible constraints (the first registration wins); an
/// existing topic with incompatible constraints fails explicitly.
///
/// - Topic does not exist (first endpoint on it): `*topic_type_name` is the
///   base name (unconstrained) or base name + "#" + counter (constrained).
///   The caller must register its own TypeSupport under this name and create
///   the topic with it.
/// - Topic exists and the endpoint's constraints are compatible with the
///   registered type support: `*topic_type_name` is the existing topic's type
///   name.  The caller reuses the already-registered TypeSupport.
/// - Topic exists with incompatible constraints: returns false with an RMW
///   error set (explicit failure, per design).
///
/// Must be called with `participant_info->entity_creation_mutex_` held.
///
/// @param[out] topic_type_name  Type name to use for registration and for
///                              creating/using the topic.
RMW_FASTRTPS_SHARED_CPP_PUBLIC
bool resolve_constrained_endpoint(
  CustomParticipantInfo * participant_info,
  const std::string & topic_name_mangled,
  const std::string & base_type_name,
  const rosidl_message_type_constraints_t * constraints,
  std::string * topic_type_name);

}  // namespace rmw_fastrtps_shared_cpp

#endif  // RMW_FASTRTPS_SHARED_CPP__XCDRTYPESUPPORT_HPP_
