# rcl API coverage: rclgo compared with rclcpp and rclpy

How much of the rcl API rclgo uses, compared with rclcpp and rclpy, and which features the missing functions belong
to. Measured on ROS 2 Jazzy for `jazzy` before #55–#61 and for the code with all of #55–#61.

## How it is measured

- **The API:** the 397 public functions declared in ROS 2 Jazzy's headers for `rcl`, `rcl_action`,
  `rcl_yaml_param_parser`, `rcl_lifecycle` and `rcl_logging_interface`.
- **rclcpp:** the rcl functions its installed libraries import (`librclcpp*.so`, undefined dynamic symbols), plus
  those called inline from the rclcpp, rclcpp_action and rclcpp_lifecycle headers.
- **rclpy:** the functions its `_rclpy_pybind11` module imports.
- **rclgo:** the functions called in its Go and C sources (not tests, generated messages or the fault-injection
  wrappers).
- The binary symbol lists are exact. The header scan may catch a name mentioned in a comment, so rclcpp's count is
  approximate by a few.

## Result

138 of the 397 functions are used by **neither** rclcpp nor rclpy: lexer, YAML struct internals, type-hash
conversion, rmw handles, `*_is_valid` and so on. That leaves **259** that a client library actually uses, of which
**198** both rclcpp and rclpy use (the "core").

| | rcl functions used | Share of the 259 (rclcpp ∪ rclpy) | Share of the 198 core | rcl / rcl_action / rcl_lifecycle |
|---|---|---|---|---|
| rclcpp (with action + lifecycle) | 248 | 96 % | 100 % | 174 / 57 / 13 |
| rclpy | 209 | 81 % | 100 % | 151 / 48 / 8 |
| **rclgo `jazzy`** (before #55–#61) | 126 | 48 % | 60 % | 84 / 41 / 0 |
| **rclgo with #55–#61** | **132** | **50 %** | **62 %** | 89 / 42 / 0 |

- **What #55–#61 added:** six calls: `rcl_timer_call` (timers keep their schedule), `rcl_init_options_fini` (a
  leak), `rcl_set_ros_time_override`/`rcl_enable_…`/`rcl_disable_ros_time_override` (timers follow simulated time),
  and `rcl_action_goal_handle_get_info`. Those PRs fix defects rather than add features, so the share barely moves.
- **Function counts don't measure features well:**
  - Parameters and executors barely touch rcl in any client library. rclgo's parameters now match rclcpp's,
    services included, which the table doesn't show.
  - Multi-threaded executors and callback groups are missing, and the table doesn't show that either.
  - About 17 of the missing core functions are trivial accessors (`*_get_default_options`, `*_get_topic_name`,
    `context_is_valid`, …). Without them rclgo covers roughly 70 % of the core that matters.

## The 76 core functions rclgo doesn't use, by feature

| Area | Missing (used by both rclcpp and rclpy) | What it means for rclgo users | Tracked in |
|---|---|---|---|
| **Lifecycle nodes** | `rcl_lifecycle_state_machine_init/fini/is_initialized`, `…get_default_state_machine_options`, `…get_zero_initialized_state_machine`, `…trigger_transition_by_id/label`, `…get_transition_by_label` (8; rclcpp uses 13) | No managed nodes, no `ros2 lifecycle` | #5 |
| **QoS events** | `rcl_publisher_event_init`, `rcl_subscription_event_init`, `rcl_take_event`, `rcl_event_fini`, `rcl_get_zero_initialized_event`, `rcl_wait_set_add_event` (+ `rcl_event_set_callback`) | No deadline-missed, liveliness-lost or incompatible-QoS callbacks | #7 |
| **Waiting for servers, graph** | `rcl_service_server_is_available`, `rcl_action_server_is_available`, `rcl_count_publishers/subscribers/clients/services`, `rcl_get_service_names_and_types`, `rcl_get_node_names_with_enclaves` | No `WaitForService`/`WaitForActionServer`; no global service list; no per-topic counts | #10 |
| **Timer controls** | `rcl_timer_cancel`, `…is_canceled`, `…is_ready`, `…get_next_call_time`, `…call_with_info`, `…clock`, `…set_on_reset_callback` | Only Reset and time-until-next-call exist | |
| **Clock** | `rcl_clock_add_jump_callback`, `rcl_clock_remove_jump_callback`, `rcl_is_enabled_ros_time_override` | No user jump callbacks; rclgo detects backward jumps only inside `Rate`/`Sleep` | |
| **Logging** | `rcl_logging_rosout_add_sublogger/remove_sublogger`, `rcl_logging_configure_with_output_handler`, `rcl_logging_get_logging_directory` | Child loggers don't reach `/rosout` as in rclcpp; no `logger_level` parameter | #9 |
| **Content-filtered topics** | `rcl_subscription_set/get_content_filter`, `…is_cft_enabled`, `…content_filter_options_init/fini`, `…options_set_content_filter_options`, `rcl_get_zero_initialized_subscription_content_filter_options` (8) | Feature missing | |
| **Event-driven callbacks** | `rcl_subscription_set_on_new_message_callback`, `rcl_service_set_on_new_request_callback`, `rcl_client_set_on_new_response_callback`, `rcl_event_set_callback` | Only needed for an EventsExecutor-style executor | #6 (executor design) |
| **Names and arguments** | `rcl_arguments_get_unparsed_ros`, `…get_count_unparsed_ros`, `rcl_remove_ros_arguments`, `rcl_node_resolve_name`, `rcl_remap_topic_name`, `rcl_validate_topic_name`, `rcl_topic_name_validation_result_string`, `rcl_get_default_topic_name_substitutions` | Small conveniences (strip ROS arguments from a program's own flags, resolve names) | |
| **Jazzy extras** | `rcl_client/service_configure_service_introspection`, `rcl_node_type_description_service_init/handle_request` | No service introspection; `ros2 interface` can't query a running rclgo node | |
| **Publisher** | `rcl_publisher_assert_liveliness`, `rcl_publisher_wait_for_all_acked` | Manual liveliness; wait until reliable data is acknowledged | |
| **Trivial accessors** | `rcl_*_get_default_options` (5), `rcl_*_get_topic_name`/`get_service_name` (4), `rcl_context_is_valid`, `rcl_context_get_domain_id`, `rcl_node_get_options`, `rcl_action_server_goal_exists`, `rcl_action_goal_handle_is_active`, `rcl_action_*_wait_set_get_num_entities` (2) | No behaviour attached; easy to add | |

## Missing features that aren't rcl functions

- **Executors and callback groups:** a multi-threaded executor and MutuallyExclusive/Reentrant groups (#6). This is
  design work in Go, not rcl calls.
- **Async calls and cancellation for service clients** (#8).
- **Examples** matching rclcpp/rclpy's service, action and lifecycle examples.
