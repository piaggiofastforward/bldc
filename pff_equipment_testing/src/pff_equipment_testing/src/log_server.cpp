#include <log_server.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "log_server", ros::init_options::NoSigintHandler);
  LogServer server = LogServer();

  // Read data
  server.run();
}