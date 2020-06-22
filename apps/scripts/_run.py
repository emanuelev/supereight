import subprocess
import math
import datetime
import os
import sys
import json

import errno
import os
import tempfile
import subprocess

from evaluate_ate import EvaluateATE
import _util

import csv
from os import listdir
from os.path import isfile, join

_RESULTS_DIR = ''
_RES_COUNTER = 0


def _time():
    return datetime.datetime.now().strftime('%Y-%m-%d_%H:%M:%S')

#
# Generate a results directory using the current time.
#


def gen_results_dir(path, name = ''):
    global _RESULTS_DIR

    _RESULTS_DIR = os.path.join(path, name + '-' + _time())
    os.makedirs(_RESULTS_DIR)
    return _RESULTS_DIR


def _get_next_res_file():
    global _RES_COUNTER

    _RES_COUNTER += 1
    return os.path.join(_RESULTS_DIR, str(_RES_COUNTER))


class SLAMAlgorithm:
    """ A general SLAM algorithm evaluator.
    """

    def __init__(self, bin_path):
        self.failed = False
        self.evalATE = None

        self.data = {}
        self.bin_path = bin_path

    """ Must be called before every run (it is called inside run to make sure)
  """

    def reset_evaluation(self):
        self.evalATE = None
        self.failed = False

        self._reset_evaluation()

    def _dump_config(self, results_path, dataset):
        config_path = results_path + '_config'
        res = {}
        with open(config_path, 'w') as file:
            res = self._store_variables(res)
            res['trajectory'] = dataset.descr
            for k in sorted(res):
                file.write('{}: {}\n'.format(k, res[k]))

    def run(self, dataset, env = {}):
        res = {}

        self.reset_evaluation()
        self.evalATE = None

        results_path = _get_next_res_file()
        self._dump_config(results_path, dataset)
        self._setup_from_dataset(dataset)

        if dataset.use_ground_truth:
            self._run_internal(dataset.camera_file,
                               dataset.dataset_path, results_path, dataset.ground_truth_assoc, env)
        else:
            self._run_internal(dataset.camera_file,
                               dataset.dataset_path, results_path, env=env)


        if self.failed:
            return res

        self._calculate_ate(dataset.ground_truth,
                            results_path, dataset.pre_assoc_file_path)

        res = self._store_variables(res)

        res['data'] = self._parse_stdio_log(results_path)
        res['ate_rmse'] = str(self._latest_ate_rmse())
        res['ate_mean'] = float(self._latest_ate_mean())
        res['ate_median'] = float(self._latest_ate_median())
        res['ate_std'] = float(self._latest_ate_std())
        res['ate_min'] = float(self._latest_ate_min())
        res['ate_max'] = float(self._latest_ate_max())
        with open(results_path + '_log', 'a') as logfile:
            logfile.write('ate: {}'.format(res['ate_mean']))
        return res

    def _calculate_ate(self, ground_truth, results_path, pre_assoc_file):
        """ Calculate the ATE, comments inside
        """
        if self.evalATE is None:
            if pre_assoc_file is not None:
                results_path_orig = results_path
                results_path = results_path + '_assoc'
                _pre_assoc_results(
                    pre_assoc_file, results_path_orig, results_path)

            self.evalATE = EvaluateATE(ground_truth, results_path)

            # Do we need to search or not for the ground truth trajectory?
            if self.ate_associate_identity:
                self.evalATE.associate_identitiy()
            else:
                offset = 0.0
                self.evalATE.associate(offset)

            # Remove constant offset checkPos.py for ICL-NUIM
            # Multiplier to fix ICL-NUIM camera thing
            if self.ate_remove_offset:
                multiplier = [1.0, 1.0, 1.0]

                # if self.ate_is_icl_nuim:
                #  multiplier = [1.0, -1.0, 1.0]

                self.evalATE.remove_const_offset(multiplier)

            # Align using a method by Horn or just compare directly
            # DEBUG
            #for i in range(0, 10):
            #    indexF, indexS = self.evalATE.matches[i]
            #    print self.evalATE.first_list[indexF]
            #    print self.evalATE.second_list[indexS]
            #    print "\n\n"
            if self.ate_align:
                self._calculate_ate_for_scale()
                self._plotTraj(results_path)
            else:
                self.evalATE.calculate_no_align()

        # What to do to print the ate error (for the histograms)
        # for t in self.evalATE.trans_error:
        #   print t

        return float(self.evalATE.latest_ate())

    def _logpath(self, results_path):
        return results_path + '_log'

    def _run_internal(self, camera_calib_path, dataset_path, results_path,
                      ground_truth_path = '', env = {}):
        """ Generate the run command and run
        """
        stdio_log_path = results_path + '_log'
        with open(stdio_log_path, 'w') as stdio_log:
            cmd = self._generate_run_command(
                camera_calib_path, dataset_path, results_path, ground_truth_path)

            print(' '.join(cmd))

            try:
                # Doesn't work without shell=True??
                subprocess.check_call(
                    ' '.join(cmd), shell=True, stdout=stdio_log, stderr=stdio_log, env=env)
            except Exception:
                pass
                #self.failed = True

    #
    # Find the first line which appears to be json
    #
    def _parse_stdio_log(self, results_path):
        stdio_log_path = self._logpath(results_path)

        alloc_timings = []
        aux_timings = []
        res = {}

        with open(stdio_log_path, 'r') as stdio_log:
            lines = stdio_log.readlines()

            for line in lines:
                if line.startswith('$'):
                    dat = line.split();
                    alloc_timings.append((int(dat[2]), float(dat[4])))
                if line.startswith('%'):
                    dat = line.split();
                    aux_timings.append(float(dat[1]))
                elif line.startswith('{'):
                    res = json.loads(line)

        if alloc_timings != {}:
            res['alloc_timings'] = alloc_timings

        if aux_timings != {}:
            res['aux_timings'] = aux_timings

        return res

    #
    # Extract the ATE statistics
    #

    def _latest_ate_rmse(self):
        return float(self.evalATE.latest_ate())

    def _latest_ate_mean(self):
        return float(self.evalATE.latest_ate_mean())

    def _latest_ate_median(self):
        return float(self.evalATE.latest_ate_median())

    def _latest_ate_std(self):
        return float(self.evalATE.latest_ate_std())

    def _latest_ate_min(self):
        return float(self.evalATE.latest_ate_min())

    def _latest_ate_max(self):
        return float(self.evalATE.latest_ate_max())

    #
    # Plot trajs
    #
    def _plotTraj(self, filepath):
        self.evalATE.latest_plot(filepath + '.png')
        #xs = []
        #ys = []
        #gtxs = []
        #gtys = []
        #for i in range(0, len(self.evalATE.matches)):
        #    indexF, indexS = self.evalATE.matches[i]
        #    xs.append(self.evalATE.first_list[indexF][0])
        #    ys.append(self.evalATE.first_list[indexF][2])
        #    gtxs.append(self.evalATE.second_list[indexS][0])
        #    gtys.append(self.evalATE.second_list[indexS][1])
        #fig = plt.figure()
        #ax = fig.add_subplot(111)
        #ax.set_xlabel("x (meters)")
        #ax.set_ylabel("y (meters)")

        #line1 = ax.plot(xs, ys, 'r--', label='Estimated')
        #line2 = ax.plot(gtxs, gtys, 'b', label='Ground truth')
        #handles, labels = ax.get_legend_handles_labels()
        #ax.legend(handles, labels, loc='upper left')
        #pdf = PdfPages(filepath + '.pdf')
        #plt.savefig(pdf, format='pdf')
        #pdf.close()


class KinectFusion(SLAMAlgorithm):

    def __init__(self, bin_path):
        SLAMAlgorithm.__init__(self, bin_path)

        self.fps = 0

        self.compute_size_ratio = 2
        self.icp_threshold = math.pow(10, -5)
        self.mu = 0.1
        self.init_pose = '0.5,0.5,0'

        self.volume_size = '9.6'

        self.integration_rate = 1
        self.volume_resolution = '512'
        self.pyramid_levels = '10,5,4'
        self.rendering_rate = 4
        self.tracking_rate = 1

        self.blocking = False
        self.bilateralFilter = True

        self.camera = ''
        self.quat = None

        self.impl = 'cuda-sdf'
        self.dump_volume = ''
        #self.ate_remove_offset = True
        #self.ate_align = True

        self.nsight_cu = ''

    def reset_evaluation(self):
        pass

    def _calculate_ate_for_scale(self):
        self.evalATE.calculate_for_scale()

    def _setup_from_dataset(self, dataset):
        if dataset.quat:
            self.quat = dataset.quat

        if dataset.init_pose:
            self.init_pose = dataset.init_pose

        self.camera = dataset.camera
        #self.ate_associate_identity = dataset.ate_associate_identity

    def _generate_run_command(self, camera_calib_path, dataset_path, results_path,
                              ground_truth_path = ''):
        args = []
        args.extend(['--compute-size-ratio', str(self.compute_size_ratio)])
        args.extend(['--fps', str(self.fps)])
        # args.extend(['--block-read', str(self.blocking)])
        args.extend(['--input-file', dataset_path])
        args.extend(['--icp-threshold', str(self.icp_threshold)])
        args.extend(['--log-file', str(results_path)])
        args.extend(['--mu', str(self.mu)])
        args.extend(['--init-pose', str(self.init_pose)])
        args.extend(['--no-gui'])
        args.extend(['--integration-rate', str(self.integration_rate)])
        args.extend(['--volume-size', str(self.volume_size)])
        args.extend(['--tracking-rate', str(self.tracking_rate)])
        args.extend(['--volume-resolution', str(self.volume_resolution)])
        args.extend(['--pyramid-levels', str(self.pyramid_levels)])
        args.extend(['--rendering-rate', str(self.rendering_rate)])
        if self.dump_volume != "":
            args.extend(['--dump-volume', str(results_path) +
                str(self.dump_volume)])
        args.extend(['-k', str(self.camera)])

        if ground_truth_path != '':
            args.extend(['--ground-truth', ground_truth_path])

        if self.quat:
            args.extend(['-a', str(self.quat)])

        if self.bilateralFilter:
            args.extend(['-F', ''])

        nsight_cu = []
        if self.nsight_cu != '':
            nsight_cu.extend([self.nsight_cu])
            nsight_cu.extend(["-o", str(results_path)])

        return nsight_cu + [self.bin_path + 'supereight-denseslam-' + self.impl + '-main'] + (args)

    def _store_variables(self, res):
        res['compute-size-ratio'] = str(self.compute_size_ratio)
        res['fps'] = str(self.fps)
        # res['block-read'] = str(self.blocking)
        res['icp-threshold'] = str(self.icp_threshold)
        res['mu'] = str(self.mu)
        res['init-pose'] = str(self.init_pose)
        res['integration-rate'] = str(self.integration_rate)
        res['volume-size'] = str(self.volume_size)
        res['tracking-rate'] = str(self.tracking_rate)
        res['volume-resolution'] = str(self.volume_resolution)
        res['pyramid-levels'] = str(self.pyramid_levels)
        res['rendering-rate'] = str(self.rendering_rate)
        res['version'] = str(self.impl)

        return res


class LSDSLAM(SLAMAlgorithm):

    def __init__(self, bin_path):
        SLAMAlgorithm.__init__(self, bin_path)

        self.scale = 1.0
        self.evalATE = None

        self.kfusage = 5.0
        self.kfdist = 5.0

        self.blocking = True
        self.fps = 30

        self.process_every_frame = True
        self.minUseGrad = 5.0

        self.kfreactive = False
        self.subpixelstereo = False
        self.maxLoopClosureCandidates = 20

        self.pose_optimise = True

        self.randomSeed = 1
        self.useFabMap = False
        self.ate_remove_offset = True
        self.ate_align = False

    def _calculate_ate_for_scale(self):
        s, self.ate = _util.golden_section_search(
            0.0, 4.0, 0.01, lambda scale: self.__calculate_ate_for_scale(scale))

        # Print out the ATE ground-truth path
        # for t in self.evalATE.first_xyz_full_aligned.transpose().A:
        #          print str(t[0]) + ' ' + str(t[1]) + ' ' + str(t[2])

    def __calculate_ate_for_scale(self, scale=None):
        if scale is None:
            scale = self.scale

        self.evalATE.calculate_for_scale(scale)

        return float(self.evalATE.latest_ate())

    def _setup_from_dataset(self, dataset):
        pass

    def _reset_evaluation(self):
        self.scale = 1.0

    def _generate_run_command(self, camera_calib_path, dataset_path, results_path):
        args = []
        args.extend(['--kfusage', str(self.kfusage)])
        args.extend(['--kfdist', str(self.kfdist)])
        args.extend(['--camera', camera_calib_path])
        args.extend(['--raw-image-file', dataset_path])
        args.extend(['--log', str(results_path)])
        args.extend(['--fps', str(self.fps)])
        args.extend(['--blocking', str(self.blocking)])
        args.extend(['--perf', str(self.process_every_frame)])
        args.extend(['--minUseGrad', str(self.minUseGrad)])
        args.extend(['--kfreactive', str(self.kfreactive)])
        args.extend(['--subpixelstereo', str(self.subpixelstereo)])
        args.extend(['--maxloopcand', str(self.maxLoopClosureCandidates)])
        args.extend(['--randomSeed', str(self.randomSeed)])
        args.extend(['--fabmap', str(self.useFabMap)])
        args.extend(['--use-pose-optim', str(self.pose_optimise)])

        return [self.bin_path + 'lsdslam/lsdslam-benchmark-cpp'] + (args)

    def _store_variables(self, res):
        res['kfusage'] = str(self.kfusage)
        res['kfdist'] = str(self.kfdist)
        res['fps'] = str(self.fps)
        res['blocking'] = str(self.blocking)
        res['perf'] = str(self.process_every_frame)
        res['minUseGrad'] = str(self.minUseGrad)
        res['kfreactive'] = str(self.kfreactive)
        res['subpixelstereo'] = str(self.subpixelstereo)
        res['maxloopcand'] = str(self.maxLoopClosureCandidates)
        res['randomSeed'] = str(self.randomSeed)
        res['fabmap'] = str(self.useFabMap)

        return res

#
# Connect the real times (the times of the RGB frames) to the calculated trajectory so then they can be associated.
#


def _pre_assoc_results(pre_assoc_file, path_to_traj, output_path):
    traj = []

    # Extract, tab separated format
    with open(path_to_traj) as ate_file:
        traj = [line.split() for line in ate_file]

    # remove blanks
    traj = [[j for j in i if j != ''] for i in traj]

    # remove counter
    traj = [i[1:] for i in traj]

    assoc_times = []

    with open(pre_assoc_file) as ate_file:
        assoc_times = [line.rstrip('\n').split()[2] for line in ate_file]

    with open(output_path, 'w') as new_traj:
        for i in range(0, min(len(traj), len(assoc_times))):
            new_traj.write(assoc_times[i] + ' ' + ' '.join(traj[i]) + '\n')


#
# Set the CPU Frequency on a Linux machine (assumes hardware support)
# Need to install cpufreq tools
#
def set_freq(freq):
    for proc in range(0, 8):
        subprocess.check_call("cpufreq-set -c " + str(proc) +
                              " -g userspace -u " + freq + " -d " + freq, shell=True)
