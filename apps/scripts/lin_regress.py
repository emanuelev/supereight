#!/usr/bin/env python3

import numpy as np
import scipy.stats as st

import sys

def main():
    file = sys.argv[1]
    data = np.loadtxt(file, delimiter=",", skiprows=1)

    if len(sys.argv) > 2:
        max_size = int(sys.argv[2])
        data = data[data[:,0] < max_size]

    res = st.linregress(data)
    print(f"{res.slope}*x + {res.intercept}")

if __name__ == "__main__":
    main()
