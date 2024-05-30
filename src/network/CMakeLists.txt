add_library(websocket_utils STATIC
  websocket/WebSocketServer.cpp
  websocket/WebSocketProtocol.cpp)
target_link_libraries(websocket_utils
  ${BOOST_LIBRARIES}
  websocketpp::websocketpp
  nlohmann_json::nlohmann_json)

add_library(mission_control_interface STATIC
  MissionControlProtocol.cpp
  MissionControlTasks.cpp)
target_link_libraries(mission_control_interface utils video autonomous rover_common control)
set_target_properties(mission_control_interface PROPERTIES POSITION_INDEPENDENT_CODE ON)

