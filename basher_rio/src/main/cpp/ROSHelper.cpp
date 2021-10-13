#include "RosHelper.h"
#include <frc/DriverStation.h>
#include <iostream>

std::string getErrorStr(rcl_ret_t ret)
{
    switch (ret)
    {
    case RCL_RET_ERROR:
        return "Generic error";
    case RCL_RET_EVENT_INVALID:
        return "Event failure";
    case RCL_RET_NOT_INIT:
        return "RCL Has not init yet";
    case RCL_RET_INVALID_ARGUMENT:
        return "Function was given invalid args";
    case RCL_RET_PUBLISHER_INVALID:
        return "Publisher is invalid";
    case RCL_RET_NODE_INVALID:
        return "Invalid node context";
    case RCL_RET_ALREADY_INIT:
        return "Mode has already been initalized";
    case RCL_RET_BAD_ALLOC:
        return "Failed to allocate memeory";
    case RCL_RET_TOPIC_NAME_INVALID:
        return "Invalid topic name passed";
    default:
        return "Unknown RCL error code: " + std::to_string((int)ret);
    }
}

namespace ros
{

    bool checkRetOk(rcl_ret_t ret, const std::string &msg)
    {
        if (ret != RCL_RET_OK)
        {
            frc::DriverStation::GetInstance().ReportError(msg + " with error " + getErrorStr(ret));
            return false;
        }
        return true;
    }

    RosHelper::RosHelper(const std::string &hostAddress, const std::string &hostPort)
    {
        allocator = rcl_get_default_allocator();
        rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
        rcl_context_t context = rcl_get_zero_initialized_context();
        rcl_init_options_init(&init_options, allocator);

        // Take RMW options from RCL options
        rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

        // TCP/UDP case: Set RMW IP parameters
        rcl_ret_t ret = rmw_uros_options_set_udp_address(hostAddress.c_str(), hostPort.c_str(), rmw_options);
        checkRetOk(ret, "Failed to config IP params");

        ret = rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);
        checkRetOk(ret, "Failed to init support");
    }

    RosHelper::~RosHelper()
    {
        for (auto pub : publishers)
        {
            // finalize publishers
            rcl_publisher_fini(pub, &node);
            delete pub;
        }
        for (auto sub : subscribers)
        {
            // finalize subscribers
            rcl_subscription_fini(sub, &node);
            delete sub;
        }
        for (auto serv : services)
        {
            // finalize serivices
            rcl_service_fini(serv, &node);
        }

        //cleanup node context
        rcl_node_fini(&node);
    }

    bool RosHelper::initAgentConnect()
    {
        // Wait for agent successful ping for 2 minutes.
        const int timeout_ms = 1000;
        const uint8_t attempts = 120;

        rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);
        if (!checkRetOk(ret, "Failed to connect with agent"))
            return false;

        //frc::DriverStation::GetInstance().ReportWarning("Agent discovered");

        ret = rclc_node_init_default(&node, "rio_node", "", &support);
        if (!checkRetOk(ret, "Failed to init node"))
            return false;

        //frc::DriverStation::GetInstance().ReportWarning("Node initalized");

        ret = rclc_executor_init(&executor, &support.context, 1, &allocator);
        if (!checkRetOk(ret, "Failed to init executor"))
            return false;

        //frc::DriverStation::GetInstance().ReportWarning("Created executor");

        // Succesfull init of micro_ros
        return true;
    }

    // Because pointers arent confusing enough, we have to pass the pointer by refrence.....
    bool RosHelper::registerPublisher(rcl_publisher_t *&publisher, const std::string &topicName, const rosidl_message_type_support_t *typeSupport)
    {
        publisher = new rcl_publisher_t();
        rcl_ret_t ret = rclc_publisher_init_default(publisher, &node, typeSupport, topicName.c_str());
        if (!checkRetOk(ret, "Failed to init publisher"))
            return false;

        if (!rcl_publisher_is_valid(publisher))
        {
            frc::DriverStation::GetInstance().ReportError("Created invalid publisher");
            return false;
        }

        publishers.push_back(publisher);
        return true;
    }

    // Because pointers arent confusing enough, we have to pass the pointer by refrence.....
    bool RosHelper::registerSubscriber(rcl_subscription_t *&subscriber, const std::string &topicName, const rosidl_message_type_support_t *typeSupport, void (*subscription_callback)(const void *msgin), void *msg)
    {
        subscriber = new rcl_subscription_t();
        rcl_ret_t ret = rclc_subscription_init_default(subscriber, &node, typeSupport, topicName.c_str());
        rclc_executor_add_subscription(&executor, subscriber, &msg, subscription_callback, ON_NEW_DATA);
        if(!checkRetOk(ret, "Failed to init subscriber"))
            return false;

        if(!rcl_subscription_is_valid(subscriber)){
            frc::DriverStation::GetInstance().ReportError("Created invalid subscriber");
            return false;
        }

        subscribers.push_back(subscriber);
        return true;
    }

    bool RosHelper::createService(rcl_service_t *&service, const std::string &serviceName, const rosidl_service_type_support_t *typeSupport /*, callback */)
    {
    }

    void RosHelper::spinOnce()
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }

    void RosHelper::spinForever()
    {
    }

} // namespace ros