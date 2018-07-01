# Observations and to-do list

* TUM trajectories has a different coordinate frame from what we
  output with KF. In particular y and z axis are swapped.
* Aligning trajectories via HORN gives MUCH better results with KinectFusion.
  In fact, now our ATE is at par with other solutions such as InfiniTAM or 
  ElasticFusion.
* Shall we give up and always realign? If it's true that horn doens't
  compute just mirroring and translation, then are we doing a fair thing?

## TODO
* Clean the code to effectively separate data-set dependent stuff from
  algorithm. TUM works with timestamps, ICL-NUIM doesn't, we should 
  CLEARLY separate those stages. 


