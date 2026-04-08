#pragma once

namespace net {
namespace odrive {

class ODriveProtocol : public WebSocketProtocol {
public:
    ODriveProtocol(SingleClientWSServer& server);
    ~ODriveProtocol();
    ODriveProtocol(const ODriveProtocol& other) = delete;
    ODriveProtocol& operator=(const ODriveProtocol& other) = delete;

private:
    SingleClientWSServer& _server;
    // tasks:: ?

};

}; // namespace odrive
} // namespace net