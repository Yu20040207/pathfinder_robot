cmake_minimum_required(VERSION 3.8)
project(fwr_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FaceObservation.msg"
  "msg/FaceObservationArray.msg"
  "msg/AttendanceEvent.msg"
  "msg/VoiceTranscript.msg"
  "msg/VoiceResponse.msg"
  "action/ExecuteRobotTask.action"
  "action/TurnRelative.action"
  "srv/RecoveryPermit.srv"
  DEPENDENCIES std_msgs)
ament_package()
