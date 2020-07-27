#!/usr/bin/env python3

import pickle
import sys

def main():
    results_file = sys.argv[1]
    output_file = sys.argv[2]

    data = pickle.load(open(results_file, "rb"))

    with open(output_file, "w") as output:
        output.write("numunique,time\n")
        for (num, time) in data[0]["data"]["alloc_timings"]:
            output.write(f"{num},{time}\n")

if __name__ == "__main__":
    main()
