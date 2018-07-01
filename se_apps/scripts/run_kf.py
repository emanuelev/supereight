#!/usr/bin/python2
from _run import * 
from systemsettings import *
from datasets import *
import numpy as np 

TUM_RGB_FR1 = [TUM_RGB_FR1_XYZ, TUM_RGB_FR1_FLOOR, TUM_RGB_FR1_PLANT, TUM_RGB_FR1_DESK]
TUM_RGB_FR2 = [TUM_RGB_FR2_DESK]
TUM_RGB_FR3 = [TUM_RGB_FR3_DESK]
ICL = [ICL_NUIM_LIV_0, ICL_NUIM_LIV_1, ICL_NUIM_LIV_2, ICL_NUIM_LIV_3]

if __name__ == "__main__":
    results_dir = gen_results_dir(RESULTS_PATH)
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
    min_ate = 100.0
    run_results = {}
    # for mu in [0.1, 0.05]:
    run_counter = 0
    # for sequence in ICL + TUM_RGB_FR1 + TUM_RGB_FR2 + TUM_RGB_FR3:
    for sequence in [ICL_NUIM_LIV_2, TUM_RGB_FR3_DESK]:
        for resol in [512]:
            for version in ['sdf']:
                kernel_data = []
                mu = 0.1
                algorithm.impl = version
                algorithm.volume_resolution = str(resol)
                algorithm.volume_size = '5'
                algorithm.compute_size_ratio = 2
                algorithm.integration_rate = 1
                algorithm.mu = mu
                algorithm.init_pose = sequence.init_pose
                algorithm.dump_volume = ".vtk"
                res = algorithm.run(sequence)
                res['sequence'] = sequence.descr
                res['noise_factor'] = mu
                run_results[run_counter] = res
                run_counter += 1
                kernel_data.append(res)

    with open(results_dir + '/resume.log', 'w') as f:
        f.write('{:>10}\t{:>10}\t{:>10}\t{:>10}\t{:>10}\t{:>10}\t{:>10}\n'\
                .format('dataset', 'noise_factor', 'ATE',
                'preprocessing',
                'integration',
                'raycasting',
                'computation'))
        for k, d in run_results.iteritems():
            data = d['data']
            f.write('{:>10}\t{:>10.4f}\t{:>10.4f}\t{:>10.4f}\t{:>10.4f}\t{:>10.4f}\t{:>10.4f}\n'.format(d['sequence'], 
                        float(d['noise_factor']),
                        float(d['ate_mean']),
                        float(data['preprocessing']['mean']),
                        float(data['integration']['mean']),
                        float(data['raycasting']['mean']),
                        float(data['computation']['mean'])))
