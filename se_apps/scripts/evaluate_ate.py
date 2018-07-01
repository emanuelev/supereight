#!/usr/bin/python
# Software License Agreement (BSD License)
#
# Copyright (c) 2013, Juergen Sturm, TUM
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of TUM nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Requirements: 
# sudo apt-get install python-argparse

#
# FYI - Modified to add functionality. 
#

"""
This script computes the absolute trajectory error from the ground truth
trajectory and the estimated trajectory.
"""

import sys
import numpy
import argparse
import associate

class EvaluateATE:

    def __init__(self, groundtruthfile, resultsfile):
        self.resultsfile = resultsfile
        self.groundtruthfile = groundtruthfile


    def associate(self, offset = 0.0, max_difference = 0.02):
        self.first_list = associate.read_file_list(self.groundtruthfile)
        self.second_list = associate.read_file_list(self.resultsfile)

        self.matches = associate.associate(self.first_list, self.second_list,float(offset),float(max_difference))    
        if len(self.matches)<2:
            sys.exit("Couldn't find matching timestamp pairs between groundtruth and estimated trajectory! Did you choose the correct sequence?")


    def associate_identitiy(self):
        self.first_list = associate.read_file_list(self.resultsfile)
        self.second_list = associate.read_file_list(self.groundtruthfile)

        self.matches = zip(self.first_list, self.second_list)

    #
    # Remove constant offset, uses first point to provide the offset. 
    # Multiplier is the hack to fix the ICL-NUIM inverse y axis problem.
    #
    def remove_const_offset(self, multiplier):
        firstGround = self.second_list[self.matches[0][1]]

        for i in range(0, len(self.matches)):
            indexF, indexS = self.matches[i]
            self.first_list[indexF] = (multiplier[0] * (float(self.first_list[indexF][0]) + float(firstGround[0])), multiplier[1] * (float(self.first_list[indexF][1]) + float(firstGround[1]) ), multiplier[2] * (float(self.first_list[indexF][2]) + float(firstGround[2]) ))

    #
    # Direct pairwise ATE evaluation. 
    #
    def calculate_no_align(self):
        self.trans_error = []

        for i in range(0, len(self.matches)):
            indexF, indexS = self.matches[i]

            calc_traj = ((float(self.first_list[indexF][0])), (float(self.first_list[indexF][1])), (float(self.first_list[indexF][2])))
            ground_traj = ((float(self.second_list[indexS][0])), (float(self.second_list[indexS][1])), (float(self.second_list[indexS][2])))

            err = (abs(calc_traj[0] - ground_traj[0]), abs(calc_traj[1] - ground_traj[1]), abs(calc_traj[2] - ground_traj[2]))

            err_2 = (err[0] * err[0], err[1] * err[1],err[2] * err[2])
            self.trans_error.append(numpy.sqrt(sum(err_2)))

    def calculate_for_scale(self, scale = 1.0):
        self.first_xyz = numpy.matrix([[float(value) for value in self.first_list[a][0:3]] for a,b in self.matches]).transpose()
        second_xyz = numpy.matrix([[float(value)*float(scale) for value in self.second_list[b][0:3]] for a,b in self.matches]).transpose()
        rot,trans,self.trans_error = align(second_xyz,self.first_xyz)
        self.second_xyz_aligned = rot * second_xyz + trans

        self.first_stamps = self.first_list.keys()
        self.first_stamps.sort()
        self.first_xyz_full = numpy.matrix([[float(value) for value in self.first_list[b][0:3]] for b in self.first_stamps]).transpose()
        
        self.second_stamps = self.second_list.keys()
        self.second_stamps.sort()
        second_xyz_full = numpy.matrix([[float(value)*float(scale) for value in self.second_list[b][0:3]] for b in self.second_stamps]).transpose()
        self.second_xyz_full_aligned = rot * second_xyz_full + trans

    def latest_ate(self):
        return numpy.sqrt(numpy.dot(self.trans_error,self.trans_error) / len(self.trans_error))

    def latest_print(self, verbose = False):
        if verbose:
            print "compared_pose_pairs %d pairs"%(len(self.trans_error))

            print "absolute_translational_error.rmse %f m"%self.latest_ate()
            print "absolute_translational_error.mean %f m"%numpy.mean(self.trans_error)
            print "absolute_translational_error.median %f m"%numpy.median(self.trans_error)
            print "absolute_translational_error.std %f m"%numpy.std(self.trans_error)
            print "absolute_translational_error.min %f m"%numpy.min(self.trans_error)
            print "absolute_translational_error.max %f m"%numpy.max(self.trans_error)
        else:
            print "%f"%self.latest_ate()

    def latest_save_associations(self, path):
        file = open(path,"w")
        file.write("\n".join(["%f %f %f %f %f %f %f %f"%(a,x1,y1,z1,b,x2,y2,z2) for (a,b),(x1,y1,z1),(x2,y2,z2) in zip(self.matches,self.first_xyz.transpose().A,self.second_xyz_aligned.transpose().A)]))
        file.close()

    def latest_save(self, path):
        file = open(path,"w")
        file.write("\n".join(["%f "%stamp+" ".join(["%f"%d for d in line]) for stamp,line in zip(self.second_stamps,self.second_xyz_full_aligned.transpose().A)]))
        file.close()

    def latest_plot(self, path):
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig = plt.figure()
        ax = fig.add_subplot(111)
        plot_traj(ax, self.first_stamps,
                  self. first_xyz_full.transpose().A, '-',
                  "black", "ground truth")
        
        plot_traj(ax, self.second_stamps,
                  self.second_xyz_full_aligned.transpose().A, '-',
                  "blue", "estimated")

        label="difference"
        for (a,b),(x1,y1,z1),(x2,y2,z2) in zip(self.matches,self.first_xyz.transpose().A,self.second_xyz_aligned.transpose().A):
            ax.plot([x1,x2],[y1,y2],'-',color="red",label=label)
            label=""
            
        ax.legend()
            
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        plt.savefig(path,dpi=90)


    def latest_ate_mean(self):
        return numpy.mean(self.trans_error)

    def latest_ate_median(self):
        return numpy.median(self.trans_error)

    def latest_ate_std(self):
        return numpy.std(self.trans_error)

    def latest_ate_min(self):
        return numpy.min(self.trans_error)

    def latest_ate_max(self):
        return numpy.max(self.trans_error)

def align(model,data):
    """Align two trajectories using the method of Horn (closed-form).
    
    Input:
    model -- first trajectory (3xn)
    data -- second trajectory (3xn)
    
    Output:
    rot -- rotation matrix (3x3)
    trans -- translation vector (3x1)
    trans_error -- translational error per point (1xn)
    
    """
    numpy.set_printoptions(precision=3,suppress=True)
    model_zerocentered = model - model.mean(1)
    data_zerocentered = data - data.mean(1)

    W = numpy.zeros( (3,3) )
    for column in range(model.shape[1]):
        W += numpy.outer(model_zerocentered[:,column],data_zerocentered[:,column])
    U,d,Vh = numpy.linalg.linalg.svd(W.transpose())
    S = numpy.matrix(numpy.identity( 3 ))
    if(numpy.linalg.det(U) * numpy.linalg.det(Vh)<0):
        S[2,2] = -1
    rot = U*S*Vh
    trans = data.mean(1) - rot * model.mean(1)
    
    model_aligned = rot * model + trans
    alignment_error = model_aligned - data

    # for i in numpy.transpose(alignment_error):
    #     print i

    trans_error = numpy.sqrt(numpy.sum(numpy.multiply(alignment_error,alignment_error),0)).A[0]
        
    return rot,trans,trans_error

def plot_traj(ax,stamps,traj,style,color,label):
    """
    Plot a trajectory using matplotlib. 
    
    Input:
    ax -- the plot
    stamps -- time stamps (1xn)
    traj -- trajectory (3xn)
    style -- line style
    color -- line color
    label -- plot legend
    
    """
    stamps.sort()
    interval = numpy.median([s-t for s,t in zip(stamps[1:],stamps[:-1])])
    x = []
    y = []
    last = stamps[0]
    for i in range(len(stamps)):
        if stamps[i]-last < 2*interval:
            x.append(traj[i][0])
            y.append(traj[i][1])
        elif len(x)>0:
            ax.plot(x,y,style,color=color,label=label)
            label=""
            x=[]
            y=[]
        last= stamps[i]
    if len(x)>0:
        ax.plot(x,y,style,color=color,label=label)
            

if __name__=="__main__":
    # parse command line
    parser = argparse.ArgumentParser(description='''
    This script computes the absolute trajectory error from the ground truth trajectory and the estimated trajectory. 
    ''')
    parser.add_argument('first_file', help='ground truth trajectory (format: timestamp tx ty tz qx qy qz qw)')
    parser.add_argument('second_file', help='estimated trajectory (format: timestamp tx ty tz qx qy qz qw)')
    parser.add_argument('--offset', help='time offset added to the timestamps of the second file (default: 0.0)',default=0.0)
    parser.add_argument('--scale', help='scaling factor for the second trajectory (default: 1.0)',default=1.0)
    parser.add_argument('--max_difference', help='maximally allowed time difference for matching entries (default: 0.02)',default=0.02)
    parser.add_argument('--save', help='save aligned second trajectory to disk (format: stamp2 x2 y2 z2)')
    parser.add_argument('--save_associations', help='save associated first and aligned second trajectory to disk (format: stamp1 x1 y1 z1 stamp2 x2 y2 z2)')
    parser.add_argument('--plot', help='plot the first and the aligned second trajectory to an image (format: png)')
    parser.add_argument('--verbose', help='print all evaluation data (otherwise, only the RMSE absolute translational error in meters after alignment will be printed)', action='store_true')
    args = parser.parse_args()

    evaluated = EvaluateATE(args.first_file, args.second_file)
    evaluated.associate(offset = 0.0, max_difference = 0.02)
    evaluated.calculate_for_scale(args.scale)

    evaluated.latest_print(args.verbose)

    if args.save_associations:
        evaluated.latest_save_associations(args.save_associations)
        
    if args.save:
        evaluated.latest_save(args.save)

    if args.plot:
        evaluated.latest_plot(args.plot)
        
