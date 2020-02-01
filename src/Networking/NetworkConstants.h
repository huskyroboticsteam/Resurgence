#pragma once

#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <iostream>
#include <cstring>

#define PORT 5000 
#define MAXLINE 1024
#define CLOSE_TCP "~~closetcp~~"
