#!/usr/bin/env python3

import argparse
import re
import socket
import statistics
import subprocess
import time
from typing import List, Optional


def strip_user(host: str) -> str:
    """
    Convert rbansal66@sky1.cc.gatech.edu -> sky1.cc.gatech.edu
    """
    return host.split("@")[-1]


def run_cmd(cmd: List[str], timeout: float = 10.0) -> Optional[str]:
    try:
        result = subprocess.run(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=timeout,
        )
        if result.returncode == 0:
            return result.stdout.strip()
        return (result.stdout + "\n" + result.stderr).strip()
    except subprocess.TimeoutExpired:
        return None
    except FileNotFoundError:
        return None


def summarize(values: List[float], unit: str = "ms") -> None:
    if not values:
        print("  No successful measurements.")
        return

    print(f"  Count: {len(values)}")
    print(f"  Min:   {min(values):.2f} {unit}")
    print(f"  Mean:  {statistics.mean(values):.2f} {unit}")
    print(f"  Med:   {statistics.median(values):.2f} {unit}")
    print(f"  Max:   {max(values):.2f} {unit}")

    if len(values) > 1:
        print(f"  Std:   {statistics.stdev(values):.2f} {unit}")


def profile_ping(hostname: str, count: int) -> None:
    print("\n=== ICMP ping latency ===")

    output = run_cmd(["ping", "-c", str(count), hostname], timeout=count + 5)

    if output is None:
        print("  ping command failed or timed out.")
        return

    times = []
    for line in output.splitlines():
        match = re.search(r"time[=<]([\d.]+)\s*ms", line)
        if match:
            times.append(float(match.group(1)))

    summarize(times)


def profile_tcp_connect(hostname: str, port: int, count: int, timeout: float) -> None:
    print(f"\n=== TCP connect latency to {hostname}:{port} ===")

    latencies = []

    for i in range(count):
        start = time.perf_counter()
        try:
            with socket.create_connection((hostname, port), timeout=timeout):
                elapsed_ms = (time.perf_counter() - start) * 1000
                latencies.append(elapsed_ms)
        except Exception as e:
            print(f"  Attempt {i + 1}: failed ({e})")

        time.sleep(0.2)

    summarize(latencies)


def profile_ssh_roundtrip(host: str, count: int, timeout: float) -> None:
    print("\n=== SSH round-trip latency ===")
    print("  This measures time to run `true` over SSH.")
    print("  Assumes SSH keys or cached auth; password prompts will break timing.")

    latencies = []

    for i in range(count):
        start = time.perf_counter()

        try:
            result = subprocess.run(
                [
                    "ssh",
                    "-o", "BatchMode=yes",
                    "-o", f"ConnectTimeout={int(timeout)}",
                    host,
                    "true",
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=timeout + 5,
            )

            elapsed_ms = (time.perf_counter() - start) * 1000

            if result.returncode == 0:
                latencies.append(elapsed_ms)
            else:
                print(f"  Attempt {i + 1}: failed")
                if result.stderr.strip():
                    print(f"    {result.stderr.strip()}")

        except subprocess.TimeoutExpired:
            print(f"  Attempt {i + 1}: timed out")

        time.sleep(0.2)

    summarize(latencies)


def profile_dns(hostname: str, count: int) -> None:
    print("\n=== DNS lookup latency ===")

    latencies = []

    for i in range(count):
        start = time.perf_counter()
        try:
            socket.getaddrinfo(hostname, None)
            elapsed_ms = (time.perf_counter() - start) * 1000
            latencies.append(elapsed_ms)
        except Exception as e:
            print(f"  Attempt {i + 1}: failed ({e})")

        time.sleep(0.1)

    summarize(latencies)


def run_trace(hostname: str) -> None:
    print("\n=== Route trace ===")

    output = run_cmd(["tracepath", hostname], timeout=15)

    if output is None:
        output = run_cmd(["traceroute", hostname], timeout=15)

    if output is None:
        print("  tracepath/traceroute not available.")
    else:
        print(output)


def main():
    parser = argparse.ArgumentParser(
        description="Profile network latency to a remote host."
    )

    parser.add_argument(
        "host",
        help="Remote host, e.g. rbansal66@sky1.cc.gatech.edu or sky1.cc.gatech.edu",
    )

    parser.add_argument(
        "-n",
        "--count",
        type=int,
        default=10,
        help="Number of samples per test. Default: 10",
    )

    parser.add_argument(
        "-p",
        "--port",
        type=int,
        default=22,
        help="TCP port to test. Default: 22",
    )

    parser.add_argument(
        "--timeout",
        type=float,
        default=5.0,
        help="Timeout per attempt in seconds. Default: 5",
    )

    parser.add_argument(
        "--no-ssh",
        action="store_true",
        help="Skip SSH round-trip test.",
    )

    parser.add_argument(
        "--trace",
        action="store_true",
        help="Run tracepath/traceroute at the end.",
    )

    args = parser.parse_args()

    full_host = args.host
    hostname = strip_user(args.host)

    print(f"Profiling latency to: {full_host}")
    print(f"Resolved hostname:     {hostname}")

    profile_dns(hostname, args.count)
    profile_ping(hostname, args.count)
    profile_tcp_connect(hostname, args.port, args.count, args.timeout)

    if not args.no_ssh:
        profile_ssh_roundtrip(full_host, args.count, args.timeout)

    if args.trace:
        run_trace(hostname)


if __name__ == "__main__":
    main()
