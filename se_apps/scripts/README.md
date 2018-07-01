# Scripts #
The sripts in this directory are to help with the automation of running tests.

They provide a python'ic OO interface. 

These scripts assume the datasets are contained in a single directory
E.g.

* ~/datasets/icl-nuim_living_room_1/...
* ~/datasets/icl-nuim_living_room_2/...

and so on


# Initialisation #
On the first use there is some setup to be performed. 

## systemsettings.py ##
Setup the correct paths

* BIN_PATH : Complete path to binaries (probabily $clone_dir$/build
* RESULTS_PATH : Path to store intermediate results
* DATASETS_PATH : Path to folder containing datasets (in the example above this would be ~/datasets)


## datasets.py ##
Most of the settings in here are fixed for a particular dataset. You may need to change the paths to the datasets. The paths are relative to DATASETS_PATH.



# Examples in this directory #

## run_lsd_* ##
LSD-SLAM examples:

* run_lsd_def.py - Default LSD-SLAM settings (as in my report)
* run_lsd_var_kfusage_kfdist.py - LSD-SLAM varying the thresholds for determining when to create a new key-frame



## run_kf_* ##
KFusion examples:

* run_kf_def.py - Default KFusion settings (as in my report)
* run_kf_vary_icp.py - KFusion varying the ICP threshold


# Using the evaluate stuff #

For the algorithm (either KFusion or LSD-SLAM) you need to specify how it is going to calculate the ATE. This does depend a little on the dataset. 

## Pose Optimise using Horn method ##
```
#!
lsdslam.pose_optimise = True
```

## Associate on the identity (True for ICL-NUIM, false for everything else) ## 
```
#!
lsdslam.ate_associate_identitiy = False
```

## Do what the original checkPos.py did (True iff using ICL-NUIM) ## 
```
#!
lsdslam.ate_remove_offset = False
```

## If using an ICL-NUIM dataset (invert the y axis) ## 
```
#!
lsdslam.ate_is_icl_nuim = False
```