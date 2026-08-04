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

#include "fastdds/dds/core/policy/QosPolicies.hpp"
#include "fastdds/dds/domain/DomainParticipant.hpp"
#include "fastdds/dds/publisher/Publisher.hpp"
#include "fastdds/dds/publisher/qos/DataWriterQos.hpp"
#include "fastdds/dds/topic/TypeSupport.hpp"
#include "fastdds/dds/topic/Topic.hpp"
#include "fastdds/dds/topic/TopicDescription.hpp"
#include "fastdds/dds/topic/qos/TopicQos.hpp"
#include "fastdds/rtps/resources/ResourceManagement.h"

#include "rcutils/error_handling.h"
#include "rcutils/macros.h"

#include "rmw/allocators.h"
#include "rmw/error_handling.h"
#include "rmw/rmw.h"
#include "rmw/validate_full_topic_name.h"

#include "rcpputils/scope_exit.hpp"

#include "rmw_fastrtps_shared_cpp/create_rmw_gid.hpp"
#include "rmw_fastrtps_shared_cpp/custom_participant_info.hpp"
#include "rmw_fastrtps_shared_cpp/custom_publisher_info.hpp"
#include "rmw_fastrtps_shared_cpp/names.hpp"
#include "rmw_fastrtps_shared_cpp/namespace_prefix.hpp"
#include "rmw_fastrtps_shared_cpp/qos.hpp"
#include "rmw_fastrtps_shared_cpp/rmw_common.hpp"
#include "rmw_fastrtps_shared_cpp/utils.hpp"
#include "rmw_fastrtps_shared_cpp/XcdrTypeSupport.hpp"

#include "rmw_fastrtps_cpp/identifier.hpp"
#include "rmw_fastrtps_cpp/publisher.hpp"

#include "tracetools/tracetools.h"

#include "type_support_common.hpp"

rmw_publisher_t *
rmw_fastrtps_cpp::create_publisher(
  CustomParticipantInfo * participant_info,
  const rosidl_message_type_support_t * type_supports,
  const rosidl_message_type_constraints_t * constraints,
  const char * topic_name,
  const rmw_qos_profile_t * qos_policies,
  const rmw_publisher_options_t * publisher_options)
{
  /////
  // Check input parameters
  RCUTILS_CAN_RETURN_WITH_ERROR_OF(nullptr);

  RMW_CHECK_ARGUMENT_FOR_NULL(participant_info, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(type_supports, nullptr);
  RMW_CHECK_ARGUMENT_FOR_NULL(topic_name, nullptr);
  if (0 == strlen(topic_name)) {
    RMW_SET_ERROR_MSG("create_publisher() called with an empty topic_name argument");
    return nullptr;
  }
  RMW_CHECK_ARGUMENT_FOR_NULL(qos_policies, nullptr);
  if (!qos_policies->avoid_ros_namespace_conventions) {
    int validation_result = RMW_TOPIC_VALID;
    rmw_ret_t ret = rmw_validate_full_topic_name(topic_name, &validation_result, nullptr);
    if (RMW_RET_OK != ret) {
      return nullptr;
    }
    if (RMW_TOPIC_VALID != validation_result) {
      const char * reason = rmw_full_topic_name_validation_result_string(validation_result);
      RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
        "create_publisher() called with invalid topic name: %s", reason);
      return nullptr;
    }
  }
  RMW_CHECK_ARGUMENT_FOR_NULL(publisher_options, nullptr);

  if (RMW_UNIQUE_NETWORK_FLOW_ENDPOINTS_STRICTLY_REQUIRED ==
    publisher_options->require_unique_network_flow_endpoints)
  {
    RMW_SET_ERROR_MSG("Unique network flow endpoints not supported on publishers");
    return nullptr;
  }

  /////
  // Check RMW QoS
  if (!is_valid_qos(*qos_policies)) {
    RMW_SET_ERROR_MSG("create_publisher() called with invalid QoS");
    return nullptr;
  }

  /////
  // Get RMW Type Support (backend-aware: select appropriate typesupport before extraction)

  const rosidl_message_type_support_t * type_support = nullptr;
  const void * ts_impl = nullptr;
  std::string type_name;
  const char * typesupport_identifier = nullptr;

  if (participant_info->backend_mode == SerializationBackend::XCDR_BUFFERS) {
    // XCDR backend: try XCDR typesupport first, fall back to FastRTPS.
    type_support = try_get_xcdr_message_typesupport(type_supports);
    if (type_support) {
      typesupport_identifier = type_support->typesupport_identifier;
      type_name = try_get_message_type_name_from_xcdr(type_support);
      if (type_name.empty()) {
        RMW_SET_ERROR_MSG("Could not determine type name from XCDR typesupport");
        return nullptr;
      }
      ts_impl = nullptr;  // XcdrTypeSupport does not use impl field
    } else {
      // No XCDR typesupport for this type; fall back to FastRTPS.
      rcutils_reset_error();
      RCUTILS_LOG_DEBUG_NAMED(
        "rmw_fastrtps_cpp",
        "XCDR typesupport not available for type, falling back to FastRTPS");
    }
  }
  if (!type_support) {
    // FastCDR backend (default), or XCDR backend fallback
    const rosidl_message_type_support_t * fastrtps_ts = try_get_fastrtps_message_typesupport_c(
      type_supports);
    if (!fastrtps_ts) {
      rcutils_error_string_t prev_error_string = rcutils_get_error_string();
      rcutils_reset_error();
      fastrtps_ts = try_get_fastrtps_message_typesupport_cpp(type_supports);
      if (!fastrtps_ts) {
        rcutils_error_string_t error_string = rcutils_get_error_string();
        rcutils_reset_error();
        RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
          "Type support not from this implementation. Got:\n"
          "    %s\n"
          "    %s\n"
          "while fetching it",
          prev_error_string.str, error_string.str);
        return nullptr;
      }
    }
    type_support = fastrtps_ts;
    typesupport_identifier = type_support->typesupport_identifier;
    auto callbacks = static_cast<const message_type_support_callbacks_t *>(type_support->data);
    type_name = _create_type_name(callbacks);
    ts_impl = callbacks;
  }

  std::lock_guard<std::mutex> lck(participant_info->entity_creation_mutex_);

  // Create Topic names
  auto topic_name_mangled =
    _create_topic_name(qos_policies, ros_topic_prefix, topic_name).to_string();

  /////
  // Resolve the registration strategy for XCDR endpoints.
  //
  // Constrained variants of the same message type are different types for
  // serialization / deserialization / loaning but equivalent for matching, so
  // each constrained endpoint registers under a unique local type name.
  // Different topics can then carry different constrained variants; an
  // existing topic with incompatible constraints fails explicitly.
  bool is_constrained = (nullptr == ts_impl && nullptr != constraints);
  std::string endpoint_type_name = type_name;
  std::string topic_type_name = type_name;
  bool register_own_type = true;
  if (nullptr == ts_impl) {
    if (!rmw_fastrtps_shared_cpp::resolve_constrained_endpoint(
        participant_info, topic_name_mangled, type_name, constraints,
        &endpoint_type_name, &topic_type_name, &register_own_type))
    {
      return nullptr;
    }
  }

  /////
  // Find and check existing topic and type

  eprosima::fastdds::dds::TypeSupport fastdds_type;
  eprosima::fastdds::dds::TopicDescription * des_topic;
  if (!rmw_fastrtps_shared_cpp::find_and_check_topic_and_type(
      participant_info,
      topic_name_mangled,
      topic_type_name,
      &des_topic,
      &fastdds_type))
  {
    RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
      "create_publisher() called for existing topic name %s with incompatible type %s",
      topic_name_mangled.c_str(), topic_type_name.c_str());
    return nullptr;
  }

  /////
  // Get Participant and Publisher
  eprosima::fastdds::dds::DomainParticipant * dds_participant = participant_info->participant_;
  eprosima::fastdds::dds::Publisher * publisher = participant_info->publisher_;

  /////
  // Create the custom Publisher struct (info)
  auto info = new (std::nothrow) CustomPublisherInfo();
  if (!info) {
    RMW_SET_ERROR_MSG("create_publisher() failed to allocate CustomPublisherInfo");
    return nullptr;
  }

  auto cleanup_info = rcpputils::make_scope_exit(
    [info, participant_info]() {
      rmw_fastrtps_shared_cpp::remove_topic_and_type(
        participant_info, info->publisher_event_, info->topic_, info->type_support_);
      delete info->data_writer_listener_;
      delete info->publisher_event_;
      delete info;
    });

  info->typesupport_identifier_ = typesupport_identifier;
  info->type_support_impl_ = ts_impl;

  /////
  // Create the Type Support struct
  if (nullptr == ts_impl) {
    // XCDR-backed type support (ts_impl == nullptr when XCDR handle was resolved).
    // Constrained endpoints always get their own instance (carrying their own
    // constraints and a unique local type name); unconstrained endpoints reuse
    // an already-registered type when one exists.
    if (is_constrained || !fastdds_type) {
      auto tsupport = new (std::nothrow) rmw_fastrtps_shared_cpp::XcdrTypeSupport(
        type_supports, constraints, endpoint_type_name);
      if (!tsupport) {
        RMW_SET_ERROR_MSG("create_publisher() failed to allocate XcdrTypeSupport");
        return nullptr;
      }
      fastdds_type.reset(tsupport);
    }
  } else if (!fastdds_type) {
    // FastCDR-backed type support
    auto callbacks = static_cast<const message_type_support_callbacks_t *>(ts_impl);
    auto tsupport = new (std::nothrow) MessageTypeSupport_cpp(callbacks);
    if (!tsupport) {
      RMW_SET_ERROR_MSG("create_publisher() failed to allocate MessageTypeSupport");
      return nullptr;
    }
    fastdds_type.reset(tsupport);
  }

  // Constrained endpoints register only when they are the first on their
  // topic (their own type support is otherwise not registered; the topic uses
  // the shared registration).  Unconstrained endpoints always register (a
  // re-registration of an already-registered type is a no-op).
  if (!is_constrained || register_own_type) {
    if (ReturnCode_t::RETCODE_OK != fastdds_type.register_type(dds_participant)) {
      RMW_SET_ERROR_MSG("create_publisher() failed to register type");
      return nullptr;
    }
  }
  info->type_support_ = fastdds_type;

  if (!is_constrained || register_own_type) {
    if (!rmw_fastrtps_shared_cpp::register_type_object(type_supports, topic_type_name)) {
      // Type object registration fails when no introspection typesupport is available
      // (e.g., XCDR-only experimental messages).  This is non-fatal — the metadata
      // is used for type hash discovery during matching but isn't required for communication.
      RCUTILS_LOG_WARN_NAMED(
        "rmw_fastrtps_cpp",
        "Failed to register type object for type %s; "
        "type hash discovery may be degraded (non-fatal)",
        topic_type_name.c_str());
      rcutils_reset_error();
    }
  }

  /////
  // Create Listener
  info->publisher_event_ = new (std::nothrow) RMWPublisherEvent(info);
  if (!info->publisher_event_) {
    RMW_SET_ERROR_MSG("create_publisher() could not create publisher event");
    return nullptr;
  }

  info->data_writer_listener_ = new (std::nothrow) CustomDataWriterListener(info->publisher_event_);
  if (!info->data_writer_listener_) {
    RMW_SET_ERROR_MSG("create_publisher() could not create publisher data writer listener");
    return nullptr;
  }

  /////
  // Create and register Topic
  eprosima::fastdds::dds::TopicQos topic_qos = dds_participant->get_default_topic_qos();
  if (!get_topic_qos(*qos_policies, topic_qos)) {
    RMW_SET_ERROR_MSG("create_publisher() failed setting topic QoS");
    return nullptr;
  }

  info->topic_ = participant_info->find_or_create_topic(
    topic_name_mangled, topic_type_name, topic_qos, info->publisher_event_);
  if (!info->topic_) {
    RMW_SET_ERROR_MSG("create_publisher() failed to create topic");
    return nullptr;
  }

  /////
  // Create DataWriter

  // If the user defined an XML file via env "FASTRTPS_DEFAULT_PROFILES_FILE", try to load
  // datawriter which profile name matches with topic_name. If such profile does not exist,
  // then use the default Fast DDS QoS.
  eprosima::fastdds::dds::DataWriterQos writer_qos = publisher->get_default_datawriter_qos();

  // Try to load the profile with the topic name
  // It does not need to check the return code, as if the profile does not exist,
  // the QoS is already the default
  publisher->get_datawriter_qos_from_profile(topic_name, writer_qos);

  // Modify specific DataWriter Qos
  if (!participant_info->leave_middleware_default_qos) {
    if (participant_info->publishing_mode == publishing_mode_t::ASYNCHRONOUS) {
      writer_qos.publish_mode().kind = eprosima::fastrtps::ASYNCHRONOUS_PUBLISH_MODE;
    } else if (participant_info->publishing_mode == publishing_mode_t::SYNCHRONOUS) {
      writer_qos.publish_mode().kind = eprosima::fastrtps::SYNCHRONOUS_PUBLISH_MODE;
    }

    writer_qos.endpoint().history_memory_policy =
      eprosima::fastrtps::rtps::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;

    writer_qos.data_sharing().off();
  }

  // Get QoS from RMW
  if (!get_datawriter_qos(
      *qos_policies, try_get_type_hash(type_supports),
      writer_qos))
  {
    RMW_SET_ERROR_MSG("create_publisher() failed setting data writer QoS");
    return nullptr;
  }

  // Creates DataWriter with a mask enabling publication_matched calls for the listener
  info->data_writer_ = publisher->create_datawriter(
    info->topic_,
    writer_qos,
    info->data_writer_listener_,
    eprosima::fastdds::dds::StatusMask::publication_matched());

  if (!info->data_writer_) {
    RMW_SET_ERROR_MSG("create_publisher() could not create data writer");
    return nullptr;
  }

  // Set the StatusCondition to none to prevent triggering via WaitSets
  info->data_writer_->get_statuscondition().set_enabled_statuses(
    eprosima::fastdds::dds::StatusMask::none());

  // lambda to delete datawriter
  auto cleanup_datawriter = rcpputils::make_scope_exit(
    [publisher, info]() {
      publisher->delete_datawriter(info->data_writer_);
    });

  /////
  // Create RMW GID
  info->publisher_gid = rmw_fastrtps_shared_cpp::create_rmw_gid(
    eprosima_fastrtps_identifier, info->data_writer_->guid());

  /////
  // Allocate publisher
  rmw_publisher_t * rmw_publisher = rmw_publisher_allocate();
  if (!rmw_publisher) {
    RMW_SET_ERROR_MSG("create_publisher() failed to allocate rmw_publisher");
    return nullptr;
  }
  auto cleanup_rmw_publisher = rcpputils::make_scope_exit(
    [rmw_publisher]() {
      rmw_free(const_cast<char *>(rmw_publisher->topic_name));
      rmw_publisher_free(rmw_publisher);
    });

  // XCDR publishers can loan only when the type is bounded (constrained
  // layout or inherently fixed): unbounded unconstrained types cannot
  // provide a typed loan (borrow would fail).  FastCDR publishers loan
  // when the type is plain.
  auto * type_ptr = info->type_support_.get();
  auto * xcdr_ts = dynamic_cast<rmw_fastrtps_shared_cpp::XcdrTypeSupport *>(type_ptr);
  if (xcdr_ts) {
    rmw_publisher->can_loan_messages = xcdr_ts->is_bounded();
  } else {
    rmw_publisher->can_loan_messages = type_ptr->is_plain();
  }
  rmw_publisher->implementation_identifier = eprosima_fastrtps_identifier;
  rmw_publisher->data = info;

  rmw_publisher->topic_name = static_cast<char *>(rmw_allocate(strlen(topic_name) + 1));
  if (!rmw_publisher->topic_name) {
    RMW_SET_ERROR_MSG("create_publisher() failed to allocate memory for rmw_publisher topic name");
    return nullptr;
  }
  memcpy(const_cast<char *>(rmw_publisher->topic_name), topic_name, strlen(topic_name) + 1);

  rmw_publisher->options = *publisher_options;

  cleanup_rmw_publisher.cancel();
  cleanup_datawriter.cancel();
  cleanup_info.cancel();

  TRACETOOLS_TRACEPOINT(
    rmw_publisher_init,
    static_cast<const void *>(rmw_publisher),
    info->publisher_gid.data);
  return rmw_publisher;
}
