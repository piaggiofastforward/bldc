#include "ros/ros.h"
#include <string>
#include <signal.h>
#include <ros/xmlrpc_manager.h>
#include <ros/callback_queue.h>
#include <map>
#include <memory> // For this to work need to edit CMakeLists.txt compiler options to (preferably) --std=c++17
#include <vector>
#include <iostream>
#include <chrono>

sig_atomic_t volatile signal_shutdown = 0;

class PFFNode
{
  public:

    struct timer_container
    {
      ros::Timer * timer;
      ros::Duration period;

      ~ timer_container()
      {
        if (timer)
          delete timer;
      }
    };

    // Analysis
    std::chrono::high_resolution_clock::time_point time_1_;
    std::chrono::high_resolution_clock::time_point time_2_;

    // Constructor
    PFFNode()
    {
      signal(SIGINT, PFFNode::sigint_handler);
      ros::XMLRPCManager::instance()->unbind("shutdown");
      ros::XMLRPCManager::instance()->bind("shutdown", PFFNode::shutdown_handler);
      get_params();

      subscriber_spinner_ = new ros::AsyncSpinner(queue_thread_count_); // Default subscription queue
      service_queue_ = new ros::CallbackQueue();        // Provide separate queue for services
      service_spinner_ = new ros::AsyncSpinner(queue_thread_count_, service_queue_);
      subscriber_spinner_->start();
      service_spinner_->start();

      start_time_ = ros::Time::now();
    }

    ~PFFNode()
    {
      try
      {
        subscriber_spinner_->stop();
        service_spinner_->stop();

        if (service_queue_)
          delete service_queue_;
        if (subscriber_spinner_)
          delete subscriber_spinner_;
        if (service_spinner_)
          delete service_spinner_;
      }
      catch(std::exception exception)
      {
        std::cout << "An error occurred cleaning up resourcs of PFFNode: " << exception.what() << std::endl;
      }
    }

    // Execute program
    void run()
    {
      ROS_INFO("Running");
      ros::Rate rate = ros::Rate(loop_rate_);

      while (!signal_shutdown)
      {
        try
        {
          update();
          try
          {
            rate.sleep();
          }
          catch(ros::Exception time_moved_backwards)
          {
            if (use_sim_time_)
            {
              // Reset time parameters?
              ROS_WARN("Sim time has reset");
            }
            else
            {
               ROS_ERROR("Time has moved backwards, shutting down");
               ros::shutdown();
            }
          }
        }
        catch(ros::Exception exception)
        {
          ROS_ERROR("Node update has failed, shutting down");
          ros::shutdown();
        }
      }

      shutdown_callback();
    }

  protected:

    int loop_rate_; // hz

  private:

    // Variables
    bool queue_threading_;
    int queue_thread_count_;
    bool use_sim_time_;
    ros::AsyncSpinner * subscriber_spinner_;
    ros::AsyncSpinner * service_spinner_;
    ros::CallbackQueue * service_queue_;        // Provide separate queue for services
    std::map <std::string, std::shared_ptr<timer_container>> heartbeat_subscriptions_;
    std::vector <std::shared_ptr<timer_container>> async_publishers_;

    // Get parameters
    void get_params()
    {
      ros::param::param<int>("~loop_rate", loop_rate_, 10);
      ROS_INFO("Loop rate: %i", loop_rate_);
      ros::param::param<bool>("~use_sim_time", use_sim_time_, "false");
      if (use_sim_time_)
        ROS_INFO("Using sim time");
      ros::param::param<int>("~queue_threads", queue_thread_count_, 1);
      ROS_INFO("Subscribe queue threads: %i", queue_thread_count_);
      if (queue_thread_count_ > 0)
        queue_threading_ = true;
      else
        queue_threading_ = false;
    }

    // SIGINT handler
    static void sigint_handler(int signal)
    {
      signal_shutdown = 1;
    }

    // Shutdown handler
    static void shutdown_handler(XmlRpc::XmlRpcValue &params, XmlRpc::XmlRpcValue &result)
    {
      int num_params = 0;
      if (params.getType() == XmlRpc::XmlRpcValue::TypeArray)
        num_params = params.size();
      if (num_params > 1)
      {
        std::string reason = params[1];
        ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
        signal_shutdown = 1; // Set flag
      }
      result = ros::xmlrpc::responseInt(1, "", 0);
    }

    // Shutdown callback
    void shutdown_callback()
    {
      ROS_WARN("Shutting down...");

      // Stop any registered publishers
      for (auto iter = async_publishers_.begin(); iter < async_publishers_.end(); iter++)
      {
        auto ptr = iter->get();
        ptr->timer->stop();
      }
      on_shutdown();
    }

    // Get heartbeat info
    std::shared_ptr<timer_container> get_heartbeat_info(std::string topic)
    {
      try
      {
        auto ptr = heartbeat_subscriptions_[topic];
        return ptr;
      }
      catch(std::exception exception)
      {
        return NULL;
      }
    }

  protected:

    // Variables
    ros::NodeHandle nh_;
    ros::Time start_time_;

    // Asyncronous publishing
    template <typename class_type_t>
    void add_async_publisher(ros::Rate rate, void (class_type_t::*callback)(const ros::TimerEvent&), class_type_t * parent_object)
    {
      ros::Duration timer_delay = rate.expectedCycleTime();
      ros::Timer * timer = new ros::Timer;
      *timer = nh_.createTimer(timer_delay, callback, parent_object);

      auto ptr = std::make_shared<timer_container>();
      ptr->timer = timer;

      async_publishers_.push_back(ptr);
      ROS_INFO("Registered async publisher");
    }

    // Heartbeating
    template <typename class_type_t>
    void add_heartbeat(std::string topic, ros::Duration timeout, void (class_type_t::*callback)(const ros::TimerEvent&), class_type_t * parent_object)
    {
      ros::Timer * timer = new ros::Timer;
      *timer = nh_.createTimer(timeout, callback, parent_object);

      auto ptr = std::make_shared<timer_container>();
      ptr->timer = timer;
      ptr->period = timeout;

      heartbeat_subscriptions_[topic] = ptr;
      ROS_INFO("Registered topic: %s with heartbeat monitor", topic.c_str());
    }

    // Heartbeat
    void heartbeat(std::string topic)
    {
      auto ptr = get_heartbeat_info(topic);
      if (ptr)
      {
        ptr->timer->setPeriod(ptr->period, true);
      }
    }

    // Main loop
    virtual void update() = 0;

    // Shutdown callback
    virtual void on_shutdown() = 0;

};