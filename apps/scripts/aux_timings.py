#!/usr/bin/env python3

import pickle
import sys

def main():
    results_file = sys.argv[1]
    output_file = sys.argv[2]

    data = pickle.load(open(results_file, "rb"))

    with open(output_file, "w") as output:
        output.write("idx,time\n")
        idx = 0
        for time in data[0]["data"]["aux_timings"]:
            output.write(f"{idx},{time}\n")
            idx = idx + 1

if __name__ == "__main__":
    main()
