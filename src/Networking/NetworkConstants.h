#pragma once

#include <iostream>
#include <unistd.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>

// so linux won't shit itself
#include <cstdlib>
#include <cstring>

#define PORT 3001
#define MAXLINE 1024
#define CLOSE_TCP "~~closetcp~~"
#define SERVER_IP "127.0.0.1"
