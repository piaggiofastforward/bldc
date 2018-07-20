#include <thread>
#include <unistd.h>
#include <iostream>
#include <string.h>

class VescReader
{
  public:
    char * server_hostname_;

    VescReader(char * server_hostname)
    {
      server_hostname_ = server_hostname;
    }

    void ThreadEntry()
    {
      // Run the ros node
      std::string server_arg(server_hostname_);

      // TODO Why does this work? What is the point of arg1 because it doesn't show up in bash
      execl("vesc_start.sh", "arg1", server_arg.c_str(), NULL);

      return;
    }
};