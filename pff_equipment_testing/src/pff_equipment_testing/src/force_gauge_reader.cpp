#include <force_gauge_reader.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "arduino_reader", ros::init_options::NoSigintHandler);
  ForceGaugeReader reader = ForceGaugeReader();

  // Check if the deivce was found
  if (!reader.connected_)
  {
    ros::shutdown();
    return 0;
  }

  // Read data
  reader.run();
}