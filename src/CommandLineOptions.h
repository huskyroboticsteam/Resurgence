#pragma once

struct CommandLineOptions {
	const char* can_name;
};

CommandLineOptions ParseCommandLineOptions(int argc, char** argv);
