#!/usr/bin/env python3

from _run import *
from systemsettings import *
from datasets import *
import numpy as np

import pickle
import time
import subprocess
import os
import sys

TUM_RGB_FR1 = [TUM_RGB_FR1_DESK, TUM_RGB_FR1_ROOM, TUM_RGB_FR1_PLANT]
TUM_RGB_FR2 = []
TUM_RGB_FR3 = [TUM_RGB_FR3_DESK]
ICL = [ICL_NUIM_LIV_1, ICL_NUIM_LIV_2] # + \
      # [ICL_NUIM_OFF_0, ICL_NUIM_OFF_1, ICL_NUIM_OFF_2, ICL_NUIM_OFF_3]

def run_kf(results_dir, version, env = {}, nsight_cu = ''):
    algorithm = KinectFusion(BIN_PATH)
    # -q --fps 10 --block-read 1

    algorithm.impl = version

    # Find the best alignment between the gt and the computed trajectory.
    # It influences results a lot, we should really discuss this.
    algorithm.ate_align = True

    # All must be true for ICL-NUIM
    algorithm.ate_associate_identity = False  # 1to1 association gt-tra

    # When true the trajectory is offset by the first position.
    algorithm.ate_remove_offset = False
    algorithm.voxel_block = '8'
    algorithm.rendering_rate = '1'
    algorithm.bilateralFilter = False

    algorithm.volume_resolution = "1024"
    algorithm.volume_size = '10.24'
    algorithm.compute_size_ratio = 1
    algorithm.integration_rate = 1
    algorithm.mu = 0.1

    algorithm.dump_volume = ".vtk"

    algorithm.nsight_cu = nsight_cu

    min_ate = 100.0
    results = []

    # Warmup
    # algorithm.init_pose = TUM_RGB_FR1_PLANT.init_pose
    # algorithm.run(TUM_RGB_FR1_PLANT, env)

    for sequence in ICL + TUM_RGB_FR1 + TUM_RGB_FR2 + TUM_RGB_FR3:
    # for sequence in [TUM_RGB_FR3_DESK]:
        for num in range(2):
            algorithm.init_pose = sequence.init_pose
            res = algorithm.run(sequence, env)

            res['sequence'] = sequence.descr
            res['noise_factor'] = algorithm.mu

            if num > 0:
                results.append(res)

            # Cooling break
            time.sleep(5)


    return results


def make_resume(file, results):
    file.write('{:>20}\t{:>10}\t{:>10}\t{:>10}\t{:>10}\t{:>10}\t{:>10}\t{:>10}\n'\
            .format('dataset', 'noise_factor', 'ATE',
            'preprocessing',
            'tracking',
            'integration',
            'raycasting',
            'computation'))

    for d in results:
        data = d['data']
        file.write('{:>20}\t{:>10.4f}\t{:>10.4f}\t{:>10.4f}\t{:>10.4f}\t{:>10.4f}\t{:>10.4f}\t{:>10.4f}\n' \
            .format(
                d['sequence'],
                float(d['noise_factor']),
                float(d['ate_mean']),
                float(data['preprocessing']['mean']),
                float(data['tracking']['mean']),
                float(data['integration']['mean']),
                float(data['raycasting']['mean']),
                float(data['computation']['mean'])))


def run(name, version, env = {}, nsight_cu = ''):
    results_dir = gen_results_dir(RESULTS_PATH, name)
    results = run_kf(results_dir, version, env, nsight_cu)

    with open(results_dir + '/resume.log', 'w') as file:
        make_resume(file, results)

    with open(results_dir + '/results.pkl', 'wb') as file:
        pickle.dump(results, file)


def compile(defs = {}):
    env = {**os.environ, **defs}
    print(defs)

    subprocess.check_call("make clean", shell=True, cwd="../..", env=env)
    subprocess.check_call("make", shell=True, cwd="../..", env=env)

    time.sleep(2)


def compile_inc(defs = {}, targets = []):
    env = {**os.environ, **defs}
    print(defs)

    subprocess.check_call("cmake -GNinja -DCMAKE_BUILD_TYPE=Release ..", shell=True, cwd="../../build", env=env)
    subprocess.check_call("ninja " + " ".join(targets), shell=True, cwd="../../build")

    time.sleep(2)


def bench_raycast():
    nsight_cu = "nv-nsight-cu-cli --kernel-id ::regex:raycastKernel: --metrics gpu__time_duration"
    compile("initial", {"BLOCK_ALIGN": "0", "USE_VECTOR_FIELDS": "false"})

    for tile_size in [2, 4, 8, 16, 24, 32]:
        compile_inc({"BLOCK_ALIGN": "0", "RAYCAST_THREAD_DIM": str(tile_size), "RAYCAST_THREAD_TILING": "true"},
                    ["supereight-denseslam-cuda-sdf-main", "supereight-denseslam-cuda-ofusion-main"])

        run("cuda-sdf-tiled-" + str(tile_size), "cuda-sdf", {"CUDA_VISIBLE_DEVICES": "0"}, nsight_cu);
        run("cuda-ofusion-tiled-" + str(tile_size), "cuda-ofusion", {"CUDA_VISIBLE_DEVICES": "0"}, nsight_cu);


    for dim in [32, 48, 64, 80, 128, 160, 256, 320, 512]:
        compile_inc({"BLOCK_ALIGN": "0", "RAYCAST_THREAD_DIM": str(dim), "RAYCAST_THREAD_TILING": "false"},
                    ["supereight-denseslam-cuda-sdf-main", "supereight-denseslam-cuda-ofusion-main"])

        run("cuda-sdf-none-" + str(dim), "cuda-sdf", {"CUDA_VISIBLE_DEVICES": "0"}, nsight_cu);
        run("cuda-ofusion-none-" + str(dim), "cuda-ofusion", {"CUDA_VISIBLE_DEVICES": "0"}, nsight_cu);


    # run("cuda-sdf-740", "cuda-sdf", {"CUDA_VISIBLE_DEVICES": "1"});
    # run("cuda-ofusion-740", "cuda-ofusion", {"CUDA_VISIBLE_DEVICES": "1"});

    # run("openmp-sdf", "openmp-sdf");
    # run("openmp-ofusion", "openmp-ofusion");

def bench_allocate():
    bins = ["supereight-denseslam-cuda-sdf-main", "supereight-denseslam-cuda-ofusion-main"]
    mk_env = lambda p, s, f: \
        {"USE_VECTOR_FIELDS": "false", "BLOCK_ALIGN": "0",
         "PARALLEL_ALLOCATE": p, "SORT_ALLOCATION_LIST": s, "FILTER_ALLOCATION_LIST": f}

    def compile_and_run(n, p, s, f):
        compile_inc(mk_env(p, s, f), bins)
        run("cuda-sdf-alloc-" + str(n), "cuda-sdf", {"CUDA_VISIBLE_DEVICES": "0"})
        run("cuda-ofusion-alloc-" + str(n), "cuda-ofusion", {"CUDA_VISIBLE_DEVICES": "0"})


    compile(mk_env("true", "true", "1"))

    compile_and_run(0, "false", "false", "0")

    compile_and_run(1, "false", "true", "0")
    compile_and_run(2, "false", "true", "1")
    compile_and_run(3, "false", "true", "2")

    compile_and_run(4, "true", "true", "0")
    compile_and_run(5, "true", "true", "1")
    compile_and_run(6, "true", "true", "2")

def bench_allocate2():
    bins = ["supereight-denseslam-cuda-sdf-main", "supereight-denseslam-cuda-ofusion-main"]
    mk_env = lambda pa: \
        {"USE_VECTOR_FIELDS": "false", "BLOCK_ALIGN": "0", "CPU_ALLOCATE": "false",
         "PARALLEL_ALLOCATE": pa}

    compile(mk_env("false"))

    compile_inc(mk_env("false"), bins)
    run("cuda-sdf-alloc-0", "cuda-sdf", {"CUDA_VISIBLE_DEVICES": "0"})
    run("cuda-ofusion-alloc-0", "cuda-ofusion", {"CUDA_VISIBLE_DEVICES": "0"})

    compile_inc(mk_env("true"), bins)
    run("cuda-sdf-alloc-1", "cuda-sdf", {"CUDA_VISIBLE_DEVICES": "0"})
    run("cuda-ofusion-alloc-1", "cuda-ofusion", {"CUDA_VISIBLE_DEVICES": "0"})

def bench_allocate3():
    bins = ["supereight-denseslam-cuda-sdf-main", "supereight-denseslam-cuda-ofusion-main"]
    mk_env = lambda ca: \
        {"USE_VECTOR_FIELDS": "true", "BLOCK_ALIGN": "0", "CPU_ALLOCATE": ca}

    compile(mk_env("true"))

    compile_inc(mk_env("true"), bins)
    run("cuda-sdf-alloc-0", "cuda-sdf", {"CUDA_VISIBLE_DEVICES": "0"})
    run("cuda-ofusion-alloc-0", "cuda-ofusion", {"CUDA_VISIBLE_DEVICES": "0"})

    compile_inc(mk_env("false"), bins)
    run("cuda-sdf-alloc-1", "cuda-sdf", {"CUDA_VISIBLE_DEVICES": "0"})
    run("cuda-ofusion-alloc-1", "cuda-ofusion", {"CUDA_VISIBLE_DEVICES": "0"})

def bench_alignment():
    bins = ["supereight-denseslam-cuda-sdf-main", "supereight-denseslam-cuda-ofusion-main"]
    mk_env = lambda a: {"USE_VECTOR_FIELDS": "true", "BLOCK_ALIGN": str(a)}

    compile(mk_env(0))
    for a in [0, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096]:
        compile_inc(mk_env(a), bins)
        run("cuda-sdf-" + str(a), "cuda-sdf", {"CUDA_VISIBLE_DEVICES": "0"})
        run("cuda-ofusion-" + str(a), "cuda-ofusion", {"CUDA_VISIBLE_DEVICES": "0"})

def bench_final():
    env = {"USE_VECTOR_FIELDS": "true", "BLOCK_ALIGN": "128"}
    compile(env)

    run("cuda-sdf-1080ti", "cuda-sdf", {"CUDA_VISIBLE_DEVICES": "0"})
    run("cuda-ofusion-1080ti", "cuda-ofusion", {"CUDA_VISIBLE_DEVICES": "0"})

    run("cuda-sdf-740", "cuda-sdf", {"CUDA_VISIBLE_DEVICES": "1"})
    run("cuda-ofusion-740", "cuda-ofusion", {"CUDA_VISIBLE_DEVICES": "1"})

    run("openmp-sdf", "openmp-sdf")
    run("openmp-ofusion", "openmp-ofusion")


if __name__ == "__main__":
    bench_final()
