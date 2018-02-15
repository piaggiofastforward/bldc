#include "ros/ros.h"
#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <dynamic_reconfigure/server.h>
#include <vesc_driver/vescDriverConfig.h>
#include "vesc_driver/DynamicReconfigure.h"


typedef dynamic_reconfigure::Server<vesc_driver::vescDriverConfig> DynamicServer;
typedef boost::function< void(vesc_driver::DynamicReconfigure) > DynamicCallback;

class DynamicReconfigure {
  public:
    DynamicReconfigure(ros::NodeHandle *nh, std::string dyn_conf_topic, float kp, float ki, float kd);
    void setUp();
    DynamicCallback callback_;

  private:
    void reconfigureCallback(vesc_driver::vescDriverConfig &msg, uint32_t lvl);
    vesc_driver::DynamicReconfigure extractMsg(
        vesc_driver::vescDriverConfig &srvMsg,
        uint32_t level
    );
    bool publish_reconfig_;
    ros::NodeHandle *nh_;
    DynamicServer server_;
    ros::Publisher dyn_pub_;
    float kp_;
    float ki_;
    float kd_;
};
