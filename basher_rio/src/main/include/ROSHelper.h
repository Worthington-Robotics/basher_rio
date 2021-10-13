#pragma once

#include <string>
#include <vector>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

//Have to include all used message types here
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <geometry_msgs/msg/twist.h>

namespace ros
{
    class RosHelper
    {
    public:
        RosHelper(const std::string &hostAddress, const std::string &hostPort);

        ~RosHelper();

        /**
         * Creates the connection to the micro_ros agent as specified by the initial configuration
         * @returns true if connection and helper init was succesful
         **/
        bool initAgentConnect();

        /**
         * Creates a publisher with the specified topic and message type.
         * The publisher itself is stored in an internal list for deletion later;
         * @param publisher the pointer to the publisher to be added to the node. The pointer should be null when passed in, and will become the pointer to the true publisher after the function finishes
         * @param topicName the name of the topic to publish to
         * @param typeSupport the type support object to use in the message sending
         * @returns true if the operation was succesful
         **/
        bool registerPublisher(rcl_publisher_t * &publisher, const std::string &topicName, const rosidl_message_type_support_t *typeSupport);

        /**
         * Creates a subscriber with the specified topic and message type as well as callback function and message.
         * The publisher itself is stored in an internal list for deletion later;
         * @param subscriber the pointer to the subscriber to be added to the node. The pointer should be null when passed in, and will become the pointer to the true subscriber after the function finishes
         * @param topicName the name of the topic to subscribe to
         * @param typeSupport the type support object to use in the message sending
         * @param subscription_callback a function pointer to the callback function
         * @param msg the message to use internally for data. This should be an already constructed object when passed
         * @returns true if the operation was succesful
         **/
        bool registerSubscriber(rcl_subscription_t *&subscriber, const std::string &topicName, const rosidl_message_type_support_t *typeSupport, void (*subscription_callback)(const void *msgin), void *msg);

        /**
         * TODO
         **/
        bool createService(rcl_service_t *&service, const std::string &serviceName, const rosidl_service_type_support_t * /*, callback */);

        /**
         * Causes RCLC and the associated executor to spin once
         **/
        void spinOnce();

        /**
         * Causes RCLC and the associated executor to spin until halted forcefully
         **/
        void spinForever();

    private:
        rcl_node_t node;
        rcl_allocator_t allocator;
        rclc_support_t support;
        rclc_executor_t executor;

        rcl_init_options_t init_options;

        // lists of current topics
        std::vector<rcl_publisher_t *> publishers;
        std::vector<rcl_subscription_t *> subscribers;
        std::vector<rcl_service_t *> services;
    };

    bool checkRetOk(rcl_ret_t ret, const std::string &msg);
} // namespace ros