find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TemperatureHumidity.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)