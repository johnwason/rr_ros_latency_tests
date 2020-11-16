#include "ros/ros.h"
#include "ros_roundtrip_latency_test/LatencyTestPayload.h"

#include <sstream>
#include <chrono>
#include <boost/program_options.hpp>

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

    std::vector<uint8_t> payload;
    payload.resize(payload_size);

    std::cout << "Running latency test client payload-size: " << payload_size << " iters: " << iters << std::endl;

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
                assert(msg.payload.front() == (uint8_t)(count % 256));
                assert(msg.payload.back() == (uint8_t)(count % 256));
                assert(msg.payload.size() == payload_size);
                ++ count;
                if (count >= iters)
                {
                    done = true;
                }
                ros_roundtrip_latency_test::LatencyTestPayload msg2;
                msg2.seqno = count;
                msg2.payload.resize(payload_size);
                msg2.payload.front() = (uint8_t)(count % 256);
                msg2.payload.back() = (uint8_t)(count % 256);
                ping_pub.publish(msg2);                    
            }
                
        }
    );

    ros::Duration(0.5).sleep();
    
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

    std::cout << "It took me " << time_span.count() << " seconds." << std::endl;

    return 0;
}


int run_server(int argc, char *argv[], const po::variables_map& vm)
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