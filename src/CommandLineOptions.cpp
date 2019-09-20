#include "CommandLineOptions.h"

#include <string>

CommandLineOptions GetDefaultCommandLineOptions()
{
    CommandLineOptions result;
    result.can_name = "can0";
    // Fill in defaults here
    return result;
}

void ParseCommandLineOption(CommandLineOptions &result, std::string option)
{
    // Parse command line options here
}

CommandLineOptions ParseCommandLineOptions(int argc, char **argv)
{
    CommandLineOptions result = GetDefaultCommandLineOptions();
    for(int i = 1; i < argc; i++)
    {
        ParseCommandLineOption(result, argv[i]);
    }
}
