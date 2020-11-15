#include "rclcpp/rclcpp.hpp"
#include "ros_roundtrip_latency_test/msg/latency_test_payload.hpp"

#include <sstream>
#include <chrono>

#include <boost/lexical_cast.hpp>
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
        rclcpp::init(argc, argv);

        auto n = rclcpp::Node::make_shared("ros_latency_test_client");

        uint32_t count = 0;
        bool done = false;

        auto ping_pub = n->create_publisher<ros_roundtrip_latency_test::msg::LatencyTestPayload>("latencytest/ping", 10);
        auto pong_sub = n->create_subscription<ros_roundtrip_latency_test::msg::LatencyTestPayload>("latencytest/pong", 10, 
            [&](const ros_roundtrip_latency_test::msg::LatencyTestPayload::SharedPtr msg)
            {
                if (msg->seqno >= count)
                {
                    ++ count;
                    if (count >= iters)
                    {
                        done = true;
                    }
                    ros_roundtrip_latency_test::msg::LatencyTestPayload msg2;
                    msg2.seqno = count;
                    msg2.payload = payload;
                    ping_pub->publish(msg2);                    
                }
                    
            }
        );

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        
        high_resolution_clock::time_point t1 = high_resolution_clock::now();
        ros_roundtrip_latency_test::msg::LatencyTestPayload msg;
        msg.seqno = count;
        msg.payload = payload;

        ping_pub->publish(msg);

        while (!done && rclcpp::ok())
        {
            rclcpp::spin_some(n);
        }

        high_resolution_clock::time_point t2 = high_resolution_clock::now();
        duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

        std::cout << "It took me " << time_span.count() << " seconds.";

        return 0;
    }

    if (command == "server")
    {

        rclcpp::init(argc, argv);

        auto n = rclcpp::Node::make_shared("ros_latency_test_server");

        auto pong_pub = n->create_publisher<ros_roundtrip_latency_test::msg::LatencyTestPayload>("latencytest/pong", 10);
        auto ping_sub = n->create_subscription<ros_roundtrip_latency_test::msg::LatencyTestPayload>("latencytest/ping", 10, 
            [&](const ros_roundtrip_latency_test::msg::LatencyTestPayload::SharedPtr msg)
            {
                ros_roundtrip_latency_test::msg::LatencyTestPayload msg2;
                msg2 = *msg;
                pong_pub->publish(msg2);
            }
        );

        rclcpp::spin(n);

        return 0;
    }



    return 2;
}