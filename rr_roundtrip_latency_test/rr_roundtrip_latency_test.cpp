#include <RobotRaconteur.h>
#include <boost/chrono.hpp>
#include "robotraconteur_generated.h"
#include <boost/program_options.hpp>

using namespace RobotRaconteur;
namespace po = boost::program_options;
namespace lt = experimental::latency_test;
using namespace std::chrono;

int run_client(int argc, char *argv[], const po::variables_map& vm);
int run_server(int argc, char *argv[], const po::variables_map& vm);

int main(int argc, char *argv[])
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

class latency_test_impl : public lt::latency_test_obj_default_impl, public IRRServiceObject, public RR_ENABLE_SHARED_FROM_THIS<latency_test_impl>
{
public:
    virtual void RRServiceObjectInit(boost::weak_ptr<ServerContext> ctx, const std::string& service_path)
    {
        RR_WEAK_PTR<latency_test_impl> weak_this = shared_from_this();
        RR_WEAK_PTR<WireBroadcaster<lt::PayloadPtr> > weak_pong = rrvar_pong;
        rrvar_ping->InValueChanged.connect(
            [weak_this, weak_pong](const lt::PayloadPtr& payload, const TimeSpec& ts, uint32_t ep)
            {
                auto pong = weak_pong.lock();
                if (!pong)
                {
                    return;
                }

                lt::PayloadPtr ret(new lt::Payload());
                ret->seqno = payload->seqno;
                ret->payload = AttachRRArrayCopy(payload->payload->data(), payload->payload->size());
                pong->SetOutValue(ret);
            }
        );
    }
};


int run_server(int argc, char *argv[], const po::variables_map& vm)
{
    boost::asio::io_context my_io_context;
	boost::asio::io_context::work work(my_io_context);
    RR_SHARED_PTR<IOContextThreadPool> io_context_thread_pool;
    if (vm.count("single-thread"))
    {
        io_context_thread_pool = RR_MAKE_SHARED<IOContextThreadPool>(RobotRaconteurNode::sp(), boost::ref(my_io_context), false);
        RobotRaconteurNode::s()->SetThreadPool(io_context_thread_pool);
    }
    
    ServerNodeSetup node_setup(ROBOTRACONTEUR_SERVICE_TYPES,"experimental.latency_test", 56767, argc, argv);
    if (vm.count("disable-asyncio"))
    {
        node_setup.GetTcpTransport()->SetDisableAsyncMessageIO(true);
        node_setup.GetLocalTransport()->SetDisableAsyncMessageIO(true);
    }

    auto o = RR_MAKE_SHARED<latency_test_impl>();

	RobotRaconteurNode::s()->RegisterService("latency_test_service", "experimental.latency_test", o);

	std::cout << "Running" << std::endl;
	
	my_io_context.run();


    return 0;
}

int run_client(int argc, char *argv[], const po::variables_map& vm)
{

    if (!vm.count("url"))
    {
        std::cout << "URL not specified" << std::endl;
        return 1;
    }
    std::string url = vm["url"].as<std::string>();

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

    boost::asio::io_context my_io_context;
	boost::asio::io_context::work work(my_io_context);
    RR_SHARED_PTR<IOContextThreadPool> io_context_thread_pool;
    bool single_thread = vm.count("single-thread") != 0;
    if (single_thread)
    {
        io_context_thread_pool = RR_MAKE_SHARED<IOContextThreadPool>(RobotRaconteurNode::sp(), boost::ref(my_io_context), false);
        RobotRaconteurNode::s()->SetThreadPool(io_context_thread_pool);
    }
    
    ClientNodeSetup node_setup(ROBOTRACONTEUR_SERVICE_TYPES, argc, argv);
    if (vm.count("disable-asyncio"))
    {
        node_setup.GetTcpTransport()->SetDisableAsyncMessageIO(true);
        node_setup.GetLocalTransport()->SetDisableAsyncMessageIO(true);
    }

    lt::latency_test_objPtr c1;
    WireConnectionPtr<lt::PayloadPtr> ping;
    WireConnectionPtr<lt::PayloadPtr> pong;

    if (!single_thread)
    {
        c1 = rr_cast<lt::latency_test_obj>(RobotRaconteurNode::s()->ConnectService(url));
        ping = c1->get_ping()->Connect();
        pong = c1->get_pong()->Connect();
    }
    else
    {
        IOContextThreadPool_AsyncResultAdapter<RR_SHARED_PTR<RRObject> > connect_res(my_io_context);
	    RobotRaconteurNode::s()->AsyncConnectService(url, "", RR_INTRUSIVE_PTR<RRMap<std::string,RRValue> >(), NULL, "", connect_res, 5000);
	    RR_SHARED_PTR<RRObject> c0 = connect_res.GetResult();
        c1 = rr_cast<lt::latency_test_obj>(c0);

        IOContextThreadPool_AsyncResultAdapter<WireConnectionPtr<lt::PayloadPtr> > ping_res(my_io_context);
        c1->get_ping()->AsyncConnect(ping_res,5000);
        ping = ping_res.GetResult();

        IOContextThreadPool_AsyncResultAdapter<WireConnectionPtr<lt::PayloadPtr> > pong_res(my_io_context);
        c1->get_pong()->AsyncConnect(pong_res,5000);
        pong = pong_res.GetResult();

    }
    
    bool done = false;

    uint32_t count = 0;
    
    high_resolution_clock::time_point t1;
    high_resolution_clock::time_point t2;

    pong->WireValueChanged.connect(
        [&](const WireConnectionPtr<lt::PayloadPtr>& source, const lt::PayloadPtr payload, const TimeSpec& ts)
        {
            assert(val->payload->front() == (uint8_t)(count % 256));
            assert(val->payload->back() == (uint8_t)(count % 256));
            assert(val->payload->size() == payload_size);
            count++;
            if (count >= iters)
            {
                t2 = high_resolution_clock::now();
                done = true;
                return;
            }

            lt::PayloadPtr val2(new lt::Payload());
            val2->seqno = payload->seqno;
            val2->payload = AllocateEmptyRRArray<uint8_t>(payload_size);
            val2->payload->front() = (uint8_t)(count % 256);
            val2->payload->back() = (uint8_t)(count % 256);
            ping->SetOutValue(val2);
        }
    );

    lt::PayloadPtr val(new lt::Payload());
    val->seqno = count;
    val->payload = AllocateEmptyRRArray<uint8_t>(payload_size);
    val->payload->front() = (uint8_t)(count % 256);
    val->payload->back() = (uint8_t)(count % 256);
    ping->SetOutValue(val);
    t1 = high_resolution_clock::now();

    while (!done)
    {
        my_io_context.run_one_for(std::chrono::milliseconds(100));
    }

    duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
    std::cout << "It took me " << time_span.count() << " seconds." << std::endl;

    return 0;
}

