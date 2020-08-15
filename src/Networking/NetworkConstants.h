#pragma once

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>

// so linux won't shit itself
#include <cstring>
#include <cstdlib>

#define PORT 3001
#define MAXLINE 1024
#define CLOSE_TCP "~~closetcp~~"
#define SERVER_IP "127.0.0.1"
