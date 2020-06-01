#!/usr/bin/env python3

from _run import *
from systemsettings import *
from datasets import *
import numpy as np
import pickle

def dup(l):
    return [x for x in l for _ in (0, 1)]

TUM_RGB_FR1 = dup([TUM_RGB_FR1_XYZ, TUM_RGB_FR1_DESK, TUM_RGB_FR1_ROOM, TUM_RGB_FR1_PLANT])
TUM_RGB_FR2 = []
TUM_RGB_FR3 = dup([TUM_RGB_FR3_DESK, TUM_RGB_FR3_CABINET])
ICL = dup([ICL_NUIM_LIV_1, ICL_NUIM_LIV_2]) # + \
      # [ICL_NUIM_OFF_0, ICL_NUIM_OFF_1, ICL_NUIM_OFF_2, ICL_NUIM_OFF_3]

def run_kf(results_dir, version):
    algorithm = KinectFusion(BIN_PATH)
    # -q --fps 10 --block-read 1

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

    min_ate = 100.0
    results = []

    # for sequence in ICL + TUM_RGB_FR1 + TUM_RGB_FR2 + TUM_RGB_FR3:
    for sequence in [TUM_RGB_FR1_DESK, TUM_RGB_FR1_PLANT]:
        for num in range(2):
            algorithm.impl = version
            algorithm.init_pose = sequence.init_pose

            res = algorithm.run(sequence)
            res['sequence'] = sequence.descr
            res['noise_factor'] = algorithm.mu

            if num > 0:
                results.append(res)

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

if __name__ == "__main__":
    for version in ['cuda-sdf', 'cuda-ofusion', 'openmp-sdf', 'openmp-ofusion']:
        results_dir = gen_results_dir(RESULTS_PATH, version)
        results = run_kf(results_dir, version)

        with open(results_dir + '/resume.log', 'w') as file:
            make_resume(file, results)

        with open(results_dir + '/results.pkl', 'wb') as file:
            pickle.dump(results, file)
