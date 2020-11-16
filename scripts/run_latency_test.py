import argparse
import subprocess
import os
import copy
import re
import time

rr_executable = "./rr_roundtrip_latency_test"
ros_executable = "./ros_roundtrip_latency_test"
ros2_executable = "./ros_roundtrip_latency_test_p"

N=5
payload_sizes_iters = [(8,10000),(64,10000),(1024,10000),(1048576,100)]

def main():
    parser = argparse.ArgumentParser(description = "Run Robot Raconteur/ROS latency tests")
    parser.add_argument("--client",action='store_true', help="Run a client")
    parser.add_argument("--server",action='store_true', help="Run a server")
    parser.add_argument("--config",type=str,help="Configuration to run")
    parser.add_argument("--url",type=str,help="URL for Robot Raconteur client",default="rr+tcp://localhost:56767?service=latency_test_service")

    args = parser.parse_args()

    if args.client:
        run_client(args.config,args.url)
    elif args.server:
        run_server(args.config)
    else:
        assert False, "Must specify --client or --server"

def run_client(config,url):
    timestr = time.strftime("%Y-%m-%d--%H-%M-%S")
    fname = config + "-" + timestr
    if (config == "rr_default"):
        run_client_args([rr_executable, "--client", f"--url={url}"],fname)
    elif (config == "rr_single_thread"):
        run_client_args([rr_executable, "--client", f"--url={url}", "--single-thread"],fname)
    elif (config == "rr_single_thread_disable_asyncio"):
        run_client_args([rr_executable, "--client", f"--url={url}", "--single-thread", "--disable-asyncio"],fname)
    elif (config == "ros_default"):
        run_client_args([ros_executable, "--client"],fname)
    elif (config == "ros2_default"):
        run_client_args([ros2_executable, "--client"],fname)                


def run_client_args(client_args, fname):
    results = ""
    for payload,iters in payload_sizes_iters:
        exec_times = []
        for i in range(N):
            res = subprocess.check_output(client_args + [f"--iters={iters}", f"--payload-size={payload}"])
            #print(res)
            exec_time = str(extract_time(res))
            #print(f"Parsed time of: {exec_time}")
            exec_times.append(exec_time)
        result = " ".join([str(x) for x in [payload,iters] + exec_times]) + "\n"
        print(result)
        results += result
    print(results)

def run_server(config):
    if (config == "rr_default"):
        subprocess.check_call([rr_executable, "--server"])
    elif (config == "rr_single_thread"):
        subprocess.check_call([rr_executable, "--server", "--single-thread"])
    elif (config == "rr_single_thread_disable_asyncio"):
        subprocess.check_call([rr_executable, "--server", "--single-thread", "--disable-asyncio"])
    elif (config == "ros_default"):
        subprocess.check_call([ros_executable, "--server"])
    elif (config == "ros2_default"):
        subprocess.check_call([ros2_executable, "--server"])
    else:
        assert False, "Invalid configuration"

def extract_time(res):
    match = re.search("It took me ([\\d\\.]+) seconds.",res.decode('ascii'))
    #print(match)
    return float(match.group(1))

if __name__ == "__main__":
    main()