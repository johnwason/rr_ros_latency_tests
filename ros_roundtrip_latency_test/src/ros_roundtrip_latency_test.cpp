#include "ros/ros.h"
#include "ros_roundtrip_latency_test/LatencyTestPayload.h"

#include <sstream>
#include <chrono>

using namespace std::chrono;

int main(int argc, char **argv)
{  

    std::vector<std::string> args(argv, argv+argc);

    if (args.size() < 4)
    {
        std::cout << "requires \"command\", \"buffer_size\", and \"iters\"" << std::endl;
        return 1;
    }

    std::string command = args.at(1);
    if (command != "client" && command != "server")
    {
        std::cout << "invalid command, expects \"client\" or \"server\"" << std::endl;
    }

    

    size_t payload_size = boost::lexical_cast<size_t>(args.at(2));
    size_t iters = boost::lexical_cast<size_t>(args.at(3));
    std::vector<uint8_t> payload(payload_size);
    for (size_t i=0; i<payload_size; i++)
    {
        payload[i] = (uint8_t)(i % 256);
    }

    if (command == "client")
    {
        ros::init(argc, argv, "ros_latency_test_client");

        ros::NodeHandle n;

        uint32_t count = 0;
        bool done = false;

        ros::Publisher ping_pub = n.advertise<ros_roundtrip_latency_test::LatencyTestPayload>("latencytest/ping", 1000);
        ros::Subscriber pong_sub = n.subscribe<ros_roundtrip_latency_test::LatencyTestPayload>("latencytest/pong", 1000, 
            [&](const ros_roundtrip_latency_test::LatencyTestPayload::ConstPtr& msg)
            {
                if (msg->seqno >= count)
                {
                    ++ count;
                    if (count >= iters)
                    {
                        done = true;
                    }
                    ros_roundtrip_latency_test::LatencyTestPayload msg2;
                    msg2.seqno = count;
                    msg2.payload = payload;
                    ping_pub.publish(msg2);                    
                }
                    
            }
        );

        ros::Duration(5).sleep();
        
        high_resolution_clock::time_point t1 = high_resolution_clock::now();
        ros_roundtrip_latency_test::LatencyTestPayload msg;
        msg.seqno = count;
        msg.payload = payload;

        ping_pub.publish(msg);

        while (!done && ros::ok())
        {
            ros::spinOnce();
        }

        high_resolution_clock::time_point t2 = high_resolution_clock::now();
        duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

        std::cout << "It took me " << time_span.count() << " seconds.";

        return 0;
    }

    if (command == "server")
    {

        ros::init(argc, argv, "ros_latency_test_server");

        ros::NodeHandle n;

        ros::Publisher pong_pub = n.advertise<ros_roundtrip_latency_test::LatencyTestPayload>("latencytest/pong", 1000);
        ros::Subscriber ping_sub = n.subscribe<ros_roundtrip_latency_test::LatencyTestPayload>("latencytest/ping", 1000, 
            [&](const ros_roundtrip_latency_test::LatencyTestPayload::ConstPtr& msg)
            {
                auto msg2 = *msg;
                pong_pub.publish(msg2);
            }
        );

        ros::spin();

        return 0;
    }



    return 2;
}