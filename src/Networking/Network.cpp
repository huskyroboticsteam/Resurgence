#include "Network.h"

#include "../Globals.h"
#include "../log.h"
#include "NetworkConstants.h"
#include "ParseCAN.h"

#include <cassert>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>

int base_station_fd = -1;
bool connected = false;
bool failed_once = false;

bool InitializeBaseStationSocket() {
	if (connected)
		return connected;

	char* ssh_client = std::getenv("SSH_CLIENT");
	const char* server_ip;
	if (ssh_client == nullptr) {
		log(LOG_WARN,
			"SSH_CLIENT environment variable is not set: defaulting server IP to 127.0.0.1\n");
		server_ip = "127.0.0.1";
	} else {
		int i = 0;
		log(LOG_DEBUG, "Got ssh client %s\n", ssh_client);
		char c;
		while ((c = ssh_client[i++]) != '\0') {
			if (c == ' ') {
				ssh_client[i - 1] = '\0';
				break;
			}
		}
		server_ip = ssh_client;
		log(LOG_INFO, "Attempting to connect to server IP %s...\n", server_ip);
	}

	if ((base_station_fd = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
		if (!failed_once) {
			perror("Base station socket creation failed");
			failed_once = true;
		}
		return false;
	}

	struct sockaddr_in servaddr;
	bzero(&servaddr, sizeof(servaddr));

	servaddr.sin_family = AF_INET;
	servaddr.sin_port = htons(PORT);
	servaddr.sin_addr.s_addr = inet_addr(server_ip);

	if (connect(base_station_fd, (struct sockaddr*)&servaddr, sizeof(servaddr)) < 0) {
		if (!failed_once) {
			perror("Base station connect failed");
			failed_once = true;
		}
		return false;
	}

	log(LOG_INFO, "Connected to base station.\n");
	return (connected = true);
}

int abort(const std::string& msg) {
	perror(msg.c_str());
	if (connected) {
		close(base_station_fd);
		base_station_fd = -1;
		connected = false;
		failed_once = false;
	}
	return 0;
}

int recvBaseStationPacket(char* buffer) {
	if (!connected)
		return 0;

	// TODO: How do we split base station packets? If we split on newlines
	// this will require more complicated buffer management.
	// If we split by requiring the base station to send a four-byte length
	// before sending each packet, we might need to dynamically allocate
	// memory. Either way we may need to handle blocking reads more carefully.
	uint8_t len_buffer[5];
	len_buffer[4] = '\0';
	int ret = recv(base_station_fd, &len_buffer, 4, MSG_DONTWAIT);
	if (ret < 0) {
		if (errno != EAGAIN && errno != EWOULDBLOCK) {
			return abort("Failed to read from base station");
		}
		return 0;
	} else if (ret == 0) {
		return abort("Base station disconnected");
	} else if (ret != 4) {
		log(LOG_ERROR, "Could not read exactly four bytes from base station (got %d)", ret);
		return abort("Disconnecting");
	}
	uint8_t b0 = len_buffer[0];
	uint8_t b1 = len_buffer[1];
	uint8_t b2 = len_buffer[2];
	uint8_t b3 = len_buffer[3];
	log(LOG_TRACE, "received bytes from base station: %x %x %x %x (%s)\n", b0, b1, b2, b3,
		len_buffer);
	int length = b0 | (b1 << 8) | (b2 << 16) | (b3 << 24);
	log(LOG_TRACE, "parsed this to packet length: %d (MAXLINE is %d)\n", length, MAXLINE);

	bool invalid = (length > (MAXLINE - 1)) || (length < 0);
	if (invalid) {
		log(LOG_ERROR, "Message length from base station is invalid: %d\n", length);
		return abort("Disconnecting");
	}

	int len_to_recv = MAXLINE - 1;
	if (length < len_to_recv)
		len_to_recv = length;

	ret = recv(base_station_fd, buffer, len_to_recv, MSG_DONTWAIT);
	buffer[len_to_recv] = '\0';

	if (ret < 0) {
		if (errno != EAGAIN && errno != EWOULDBLOCK) {
			return abort("Failed to read from base station");
		}
		return 0;
	} else if (ret == 0) {
		return abort("Base station disconnected");
	} else if (ret != length) {
		log(LOG_ERROR, "Could not read %d bytes from the base station (got %d)", length, ret);
		return abort("Disconnecting");
	}
	log(LOG_TRACE, "final two characters of message: %d %d '%s'", buffer[length - 2],
		buffer[length - 1], buffer + (length - 2));

	return 1;
}

void sendBaseStationPacket(const std::string& packet) {
	if (write(base_station_fd, packet.c_str(), packet.length()) < 0) {
		perror("Failed to send to base station");
	}
}
