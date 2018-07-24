#include <arduino_reader.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arduino_reader", ros::init_options::NoSigintHandler);
  ArduinoReader reader = ArduinoReader();

  // Check if the deivce was found
  if (!reader.connected_)
  {
    ros::shutdown();
    return 0;
  }

  // Read data
  reader.run();
}