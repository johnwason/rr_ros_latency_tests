#include "rclcpp/rclcpp.hpp"
#include "ros_roundtrip_latency_test/msg/latency_test_payload.hpp"
#include <boost/program_options.hpp>

#include <sstream>
#include <chrono>

#include <boost/lexical_cast.hpp>
#include <chrono>

using namespace std::chrono;
namespace po = boost::program_options;

int run_client(int argc, char *argv[], const po::variables_map& vm);
int run_server(int argc, char *argv[], const po::variables_map& vm);

int main(int argc, char **argv)
{
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help","produce help message")
        ("client","run client")
        ("server","run server")
        ("single-thread","run in single thread mode")
        ("disable-asyncio","disable async-io")
        ("iters", po::value<uint32_t>(), "number of iterations to run (default 100000) - client only")
        ("payload-size", po::value<uint32_t>(), "size of \"payload\" field in bytes (default 8) - client only")
        ("url", po::value<std::string>(), "client connection URL")
    ;

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).options(desc).allow_unregistered().run(), vm);
    po::notify(vm);

    if (vm.count("help")) {
        std::cout << desc << "\n";
        return 1;
    }

    if (vm.count("client"))
    {
        return run_client(argc,argv,vm);
    }

    if (vm.count("server"))
    {
        return run_server(argc,argv,vm);
    }

    std::cout << "error: must specify --client or --server" << std::endl;
    return 1;
}

int run_client(int argc, char *argv[], const po::variables_map& vm)
{

    uint32_t iters = 100000;
    uint32_t payload_size = 8;
    
    if (vm.count("iters"))
    {
        iters = vm["iters"].as<uint32_t>();
    }

    if (vm.count("payload-size"))
    {
        payload_size = vm["payload-size"].as<uint32_t>();
    }

    std::cout << "Running latency test client payload-size: " << payload_size << " iters: " << iters << std::endl;

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
                assert(msg->payload.front() == (uint8_t)(count % 256));
                assert(msg->payload.back() == (uint8_t)(count % 256));
                assert(msg->payload.size() == payload_size);
                ++ count;
                if (count >= iters)
                {
                    done = true;
                }
                ros_roundtrip_latency_test::msg::LatencyTestPayload msg2;
                msg2.seqno = count;
                msg2.payload.resize(payload_size);
                msg2.payload.front() = (uint8_t)(count % 256);
                msg2.payload.back() = (uint8_t)(count % 256);
                ping_pub->publish(msg2);                    
            }
                
        }
    );

    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    ros_roundtrip_latency_test::msg::LatencyTestPayload msg;
    msg.seqno = count;
    msg.payload.resize(payload_size);
    msg.payload.front() = (uint8_t)(count % 256);
    msg.payload.back() = (uint8_t)(count % 256);

    ping_pub->publish(msg);

    while (!done && rclcpp::ok())
    {
        rclcpp::spin_some(n);
    }

    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);

    std::cout << "It took me " << time_span.count() << " seconds." << std::endl;

    return 0;
}

int run_server(int argc, char *argv[], const po::variables_map& vm)
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