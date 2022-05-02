add_library(websocket_utils STATIC
  websocket/WebSocketServer.cpp
  websocket/WebSocketProtocol.cpp)
target_link_libraries(websocket_utils
  ${BOOST_LIBRARIES}
  websocketpp::websocketpp
  nlohmann_json::nlohmann_json)

add_library(mission_control_interface STATIC
  MissionControlProtocol.cpp)
set_target_properties(mission_control_interface PROPERTIES POSITION_INDEPENDENT_CODE ON)
if(WITH_CV)
  target_sources(mission_control_interface PRIVATE
	${CMAKE_CURRENT_LIST_DIR}/MissionControlStreaming.cpp)
  target_link_libraries(mission_control_interface ${OpenCV_LIBRARIES})
else()
  target_sources(mission_control_interface PRIVATE
	${CMAKE_CURRENT_LIST_DIR}/MissionControlStreamingStub.cpp)
endif()

