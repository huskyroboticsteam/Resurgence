// for WS message schema

#pragma once

namespace net::odrive {

// request types
constexpr const char* ODRIVE_READ_REQ_TYPE = "odriveReadRequest";
constexpr const char* ODRIVE_WRITE_REQ_TYPE = "odriveWriteRequest";
constexpr const char* ODRIVE_LIST_REQ_TYPE = "odriveListRequest";

// response types
constexpr const char* ODRIVE_READ_REP_TYPE = "odriveReadReport";
constexpr const char* ODRIVE_WRITE_REP_TYPE = "odriveWriteReport";
constexpr const char* ODRIVE_LIST_REP_TYPE = "odriveListReport";

} // namespace net::odrive