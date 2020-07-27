import os

from _dataset import *
from systemsettings import *

#
# TUM RGB-D fr2/xyz Settings
#
TUM_RGB_FR2_XYZ = Dataset()
TUM_RGB_FR2_XYZ.dataset_path = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg2_xyz/scene.raw')
TUM_RGB_FR2_XYZ.ground_truth = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg2_xyz/groundtruth.txt')
TUM_RGB_FR2_XYZ.ground_truth_assoc = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg2_xyz/groundtruth_assoc.txt')
TUM_RGB_FR2_XYZ.camera_file = os.path.join(DATASETS_PATH, 'freiburg2.txt')
TUM_RGB_FR2_XYZ.camera = '520.9,521.0,325.1,249.7'
TUM_RGB_FR2_XYZ.quat = '-0.5721,0.6521,-0.3565,0.3469'
TUM_RGB_FR2_XYZ.init_pose = '0.5,0.5,0.5'
TUM_RGB_FR2_XYZ.rgb_image = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg2_xyz/rgb/')
TUM_RGB_FR2_XYZ.pre_assoc_file_path = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg2_xyz/associations.txt')
TUM_RGB_FR2_XYZ.ate_associate_identity = False
TUM_RGB_FR2_XYZ.descr = 'fr2_xyz'

#
# TUM RGB-D fr1/xyz Settings
#

TUM_RGB_FR1_XYZ = Dataset()
TUM_RGB_FR1_XYZ.dataset_path = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg1_xyz/scene.raw')
TUM_RGB_FR1_XYZ.ground_truth = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg1_xyz/groundtruth.txt')
TUM_RGB_FR1_XYZ.ground_truth_assoc = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg1_xyz/groundtruth_assoc.txt')
TUM_RGB_FR1_XYZ.camera_file = os.path.join(DATASETS_PATH, 'freiburg1.txt')
TUM_RGB_FR1_XYZ.camera = '517.3,516.5,318.6,255.3'
TUM_RGB_FR1_XYZ.quat = '0.6132,0.5962,-0.3311,-0.3986'
TUM_RGB_FR1_XYZ.init_pose = '0.5,0.5,0.5'
TUM_RGB_FR1_XYZ.rgb_image = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg1_xyz/rgb/')
TUM_RGB_FR1_XYZ.pre_assoc_file_path = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg1_xyz/associations.txt')
TUM_RGB_FR1_XYZ.ate_associate_identity = False
TUM_RGB_FR1_XYZ.descr = 'fr1_xyz'

#
# TUM RGB-D fr2/desk Settings
#
TUM_RGB_FR2_DESK = Dataset()
TUM_RGB_FR2_DESK.dataset_path = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg2_desk/scene.raw')
TUM_RGB_FR2_DESK.ground_truth = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg2_desk/groundtruth.txt')
TUM_RGB_FR2_DESK.ground_truth_assoc = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg2_desk/groundtruth_assoc.txt')
TUM_RGB_FR2_DESK.use_ground_truth = True
TUM_RGB_FR2_DESK.camera_file = os.path.join(DATASETS_PATH, 'freiburg2.txt')
TUM_RGB_FR2_DESK.camera = '520.9,521.0,325.1,249.7'
TUM_RGB_FR2_DESK.quat = '0.6529,-0.5483,0.3248,-0.4095'
TUM_RGB_FR2_DESK.init_pose = '0.5,0.5,0.3'
TUM_RGB_FR2_DESK.pre_assoc_file_path = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg2_desk/associations.txt')
TUM_RGB_FR2_DESK.descr = 'fr2_desk'
TUM_RGB_FR2_DESK.ate_associate_identity = False

#
# TUM RGB-D fr1/desk Settings
#
TUM_RGB_FR1_DESK = Dataset()
TUM_RGB_FR1_DESK.dataset_path = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg1_desk/scene.raw')
TUM_RGB_FR1_DESK.ground_truth = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg1_desk/groundtruth.txt')
TUM_RGB_FR1_DESK.ground_truth_assoc = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg1_desk/groundtruth_assoc.txt')
TUM_RGB_FR1_DESK.use_ground_truth = True
TUM_RGB_FR1_DESK.camera = '517.3,516.5,318.6,255.3'
TUM_RGB_FR1_DESK.quat = '0.6529,-0.5483,0.3248,-0.4095'
TUM_RGB_FR1_DESK.init_pose = '0.5,0.5,0.2'
TUM_RGB_FR1_DESK.pre_assoc_file_path = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg1_desk/associations.txt')
TUM_RGB_FR1_DESK.descr = 'fr1_desk'
TUM_RGB_FR1_DESK.ate_associate_identity = False

#
# TUM RGB-D fr1/room Settings
#
TUM_RGB_FR1_ROOM = Dataset()
TUM_RGB_FR1_ROOM.dataset_path = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg1_room/scene.raw')
TUM_RGB_FR1_ROOM.ground_truth = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg1_room/groundtruth.txt')
TUM_RGB_FR1_ROOM.ground_truth_assoc = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg1_room/groundtruth_assoc.txt')
TUM_RGB_FR1_ROOM.use_ground_truth = True
TUM_RGB_FR1_ROOM.camera = '517.3,516.5,318.6,255.3'
TUM_RGB_FR1_ROOM.quat = '0.6529,-0.5483,0.3248,-0.4095'
TUM_RGB_FR1_ROOM.init_pose = '0.5,0.5,0.5'
TUM_RGB_FR1_ROOM.pre_assoc_file_path = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg1_room/associations.txt')
TUM_RGB_FR1_ROOM.descr = 'fr1_room'
TUM_RGB_FR1_ROOM.ate_associate_identity = False

#
# TUM RGB-D fr1/plant Settings
#
TUM_RGB_FR1_PLANT = Dataset()
TUM_RGB_FR1_PLANT.dataset_path = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg1_plant/scene.raw')
TUM_RGB_FR1_PLANT.ground_truth = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg1_plant/groundtruth.txt')
TUM_RGB_FR1_PLANT.ground_truth_assoc = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg1_plant/groundtruth_assoc.txt')
TUM_RGB_FR1_PLANT.use_ground_truth = True
TUM_RGB_FR1_PLANT.camera = '517.3,516.5,318.6,255.3'
TUM_RGB_FR1_PLANT.quat = '0.6529,-0.5483,0.3248,-0.4095'
TUM_RGB_FR1_PLANT.init_pose = '0.5,0.5,0.5'
TUM_RGB_FR1_PLANT.pre_assoc_file_path = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg1_plant/associations.txt')
TUM_RGB_FR1_PLANT.descr = 'fr1_plant'
TUM_RGB_FR1_PLANT.ate_associate_identity = False

#
# TUM RGB-D fr1/floor Settings
#
TUM_RGB_FR1_FLOOR = Dataset()
TUM_RGB_FR1_FLOOR.dataset_path = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg1_floor/scene.raw')
TUM_RGB_FR1_FLOOR.ground_truth = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg1_floor/groundtruth.txt')
TUM_RGB_FR1_FLOOR.ground_truth_assoc = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg1_floor/groundtruth_assoc.txt')
TUM_RGB_FR1_FLOOR.use_ground_truth = True
TUM_RGB_FR1_FLOOR.camera = '517.3,516.5,318.6,255.3'
TUM_RGB_FR1_FLOOR.quat = '0.6529,-0.5483,0.3248,-0.4095'
TUM_RGB_FR1_FLOOR.init_pose = '0.5,0.5,0.5'
TUM_RGB_FR1_FLOOR.pre_assoc_file_path = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg1_floor/associations.txt')
TUM_RGB_FR1_FLOOR.descr = 'fr1_floor'
TUM_RGB_FR1_FLOOR.ate_associate_identity = False

#
# TUM RGB-D fr3/desk Settings
#
TUM_RGB_FR3_DESK = Dataset()
TUM_RGB_FR3_DESK.dataset_path = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg3_long_office_household_validation/scene.raw')
TUM_RGB_FR3_DESK.ground_truth = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg3_long_office_household_validation/groundtruth.txt')
TUM_RGB_FR3_DESK.ground_truth_assoc = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg3_long_office_household_validation/groundtruth_assoc.txt')
TUM_RGB_FR3_DESK.camera = '535.4,539.2,320.1,247.6'
TUM_RGB_FR3_DESK.init_pose = '0.5,0.5,0.3'
TUM_RGB_FR3_DESK.pre_assoc_file_path = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg3_long_office_household_validation/associations.txt')
TUM_RGB_FR3_DESK.descr = 'fr3_long_office'
TUM_RGB_FR3_DESK.ate_associate_identity = False

#
# TUM RGB-D fr3/cabinet Settings
#
TUM_RGB_FR3_CABINET = Dataset()
TUM_RGB_FR3_CABINET.dataset_path = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg3_cabinet/scene.raw')
TUM_RGB_FR3_CABINET.ground_truth = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg3_cabinet/groundtruth.txt')
TUM_RGB_FR3_CABINET.ground_truth_assoc = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg3_cabinet/groundtruth_assoc.txt')
TUM_RGB_FR3_CABINET.camera = '535.4,539.2,320.1,247.6'
TUM_RGB_FR3_CABINET.init_pose = '0.5,0.5,0'
TUM_RGB_FR3_CABINET.pre_assoc_file_path = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg3_cabinet/associations.txt')
TUM_RGB_FR3_CABINET.descr = 'fr3_cabinet'
TUM_RGB_FR3_CABINET.ate_associate_identity = False

#
# TUM RGB-D fr3/cabinet Settings
#
TUM_RGB_FR3_LARGE_CABINET = Dataset()
TUM_RGB_FR3_LARGE_CABINET.dataset_path = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg3_large_cabinet/scene.raw')
TUM_RGB_FR3_LARGE_CABINET.ground_truth = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg3_large_cabinet/groundtruth.txt')
TUM_RGB_FR3_LARGE_CABINET.ground_truth_assoc = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg3_large_cabinet/groundtruth_assoc.txt')
TUM_RGB_FR3_LARGE_CABINET.use_ground_truth = True
TUM_RGB_FR3_LARGE_CABINET.camera = '535.4,539.2,320.1,247.6'
TUM_RGB_FR3_LARGE_CABINET.init_pose = '0.5,0.5,0'
TUM_RGB_FR3_LARGE_CABINET.pre_assoc_file_path = os.path.join(DATASETS_PATH, 'rgbd_dataset_freiburg3_large_cabinet/associations.txt')
TUM_RGB_FR3_LARGE_CABINET.descr = 'fr3_large_cabinet'
TUM_RGB_FR3_LARGE_CABINET.ate_associate_identity = False


#
# ICL-NUIM Living Room 0
#
ICL_NUIM_LIV_0 = Dataset()
ICL_NUIM_LIV_0.dataset_path = os.path.join(DATASETS_PATH, 'living_room_traj0_loop/scene.raw')
ICL_NUIM_LIV_0.ground_truth = os.path.join(DATASETS_PATH, 'living_room_traj0_loop/livingRoom0.gt.freiburg')
ICL_NUIM_LIV_0.ground_truth_assoc = ICL_NUIM_LIV_0.ground_truth
ICL_NUIM_LIV_0.camera_file = os.path.join(DATASETS_PATH, 'living_room_traj0_loop/camera.txt')
ICL_NUIM_LIV_0.camera = '481.2,-480,320,240'
ICL_NUIM_LIV_0.init_pose = '0.34,0.5,0.24'
ICL_NUIM_LIV_0.ate_associate_identity = True
ICL_NUIM_LIV_0.descr = 'liv_traj_0'


#
# ICL-NUIM Living Room 1
#
ICL_NUIM_LIV_1 = Dataset()
ICL_NUIM_LIV_1.dataset_path = os.path.join(DATASETS_PATH, 'living_room_traj1_loop/scene.raw')
ICL_NUIM_LIV_1.ground_truth = os.path.join(DATASETS_PATH, 'living_room_traj1_loop/livingRoom1.gt.freiburg')
ICL_NUIM_LIV_1.ground_truth_assoc = ICL_NUIM_LIV_1.ground_truth
ICL_NUIM_LIV_1.camera_file = os.path.join(DATASETS_PATH, 'living_room_traj1_loop/camera.txt')
ICL_NUIM_LIV_1.camera = '481.2,-480,320,240'
ICL_NUIM_LIV_1.init_pose = '0.485,0.5,0.55'
ICL_NUIM_LIV_1.ate_associate_identity = True
ICL_NUIM_LIV_1.descr = 'liv_traj_1'


#
# ICL-NUIM Living Room 2
#
ICL_NUIM_LIV_2 = Dataset()
ICL_NUIM_LIV_2.dataset_path = os.path.join(DATASETS_PATH, 'living_room_traj2_loop/scene.raw')
ICL_NUIM_LIV_2.ground_truth = os.path.join(DATASETS_PATH, 'living_room_traj2_loop/livingRoom2.gt.freiburg')
ICL_NUIM_LIV_2.ground_truth_assoc = ICL_NUIM_LIV_2.ground_truth
ICL_NUIM_LIV_2.camera_file = os.path.join(DATASETS_PATH, 'living_room_traj2_loop/camera.txt')
ICL_NUIM_LIV_2.camera = '481.2,-480,320,240'
ICL_NUIM_LIV_2.init_pose = '0.34,0.5,0.24'
ICL_NUIM_LIV_2.ate_associate_identity = True
ICL_NUIM_LIV_2.descr = 'liv_traj_2'

#
# ICL-NUIM Living Room 3
#
ICL_NUIM_LIV_3 = Dataset()
ICL_NUIM_LIV_3.dataset_path = os.path.join(DATASETS_PATH, 'living_room_traj3_loop/scene.raw')
ICL_NUIM_LIV_3.ground_truth = os.path.join(DATASETS_PATH, 'living_room_traj3_loop/livingRoom3.gt.freiburg')
ICL_NUIM_LIV_3.ground_truth_assoc = ICL_NUIM_LIV_3.ground_truth
ICL_NUIM_LIV_3.use_ground_truth = True
ICL_NUIM_LIV_3.camera_file = os.path.join(DATASETS_PATH, 'living_room_traj3_loop/camera.txt')
ICL_NUIM_LIV_3.camera = '481.2,-480,320,240'
ICL_NUIM_LIV_3.init_pose = '0.2685,0.5,0.4'
ICL_NUIM_LIV_3.ate_associate_identity = True
ICL_NUIM_LIV_3.descr = 'liv_traj_3'

#
# ICL-NUIM Office 0
#
ICL_NUIM_OFF_0 = Dataset()
ICL_NUIM_OFF_0.dataset_path = os.path.join(DATASETS_PATH, 'office_room_traj0_loop/scene.raw')
ICL_NUIM_OFF_0.ground_truth = os.path.join(DATASETS_PATH, 'office_room_traj0_loop/traj0.gt.freiburg')
ICL_NUIM_OFF_0.ground_truth_assoc = ICL_NUIM_OFF_0.ground_truth
ICL_NUIM_OFF_0.use_ground_truth = True
ICL_NUIM_OFF_0.camera_file = os.path.join(DATASETS_PATH, 'office_room_traj0_loop/camera.txt')
ICL_NUIM_OFF_0.camera = '481.2,480,320,240'
ICL_NUIM_OFF_0.init_pose = '0.5,0.5,0.5'
ICL_NUIM_OFF_0.ate_associate_identity = True
ICL_NUIM_OFF_0.descr = 'off_traj_0'

#
# ICL-NUIM Office 1
#
ICL_NUIM_OFF_1 = Dataset()
ICL_NUIM_OFF_1.dataset_path = os.path.join(DATASETS_PATH, 'office_room_traj1_loop/scene.raw')
ICL_NUIM_OFF_1.ground_truth = os.path.join(DATASETS_PATH, 'office_room_traj1_loop/traj1.gt.freiburg')
ICL_NUIM_OFF_1.ground_truth_assoc = ICL_NUIM_OFF_1.ground_truth
ICL_NUIM_OFF_1.use_ground_truth = True
ICL_NUIM_OFF_1.camera_file = os.path.join(DATASETS_PATH, 'office_room_traj1_loop/camera.txt')
ICL_NUIM_OFF_1.camera = '481.2,480,320,240'
ICL_NUIM_OFF_1.init_pose = '0.5,0.5,0.5'
ICL_NUIM_OFF_1.ate_associate_identity = True
ICL_NUIM_OFF_1.descr = 'off_traj_1'


#
# ICL-NUIM Office 2
#
ICL_NUIM_OFF_2 = Dataset()
ICL_NUIM_OFF_2.dataset_path = os.path.join(DATASETS_PATH, 'office_room_traj2_loop/scene.raw')
ICL_NUIM_OFF_2.ground_truth = os.path.join(DATASETS_PATH, 'office_room_traj2_loop/traj2.gt.freiburg')
ICL_NUIM_OFF_2.ground_truth_assoc = ICL_NUIM_OFF_2.ground_truth
ICL_NUIM_OFF_2.use_ground_truth = True
ICL_NUIM_OFF_2.camera_file = os.path.join(DATASETS_PATH, 'office_room_traj2_loop/camera.txt')
ICL_NUIM_OFF_2.camera = '481.2,480,320,240'
ICL_NUIM_OFF_2.init_pose = '0.5,0.5,0.5'
ICL_NUIM_OFF_2.ate_associate_identity = True
ICL_NUIM_OFF_2.descr = 'off_traj_2'

#
# ICL-NUIM Office 3
#
ICL_NUIM_OFF_3 = Dataset()
ICL_NUIM_OFF_3.dataset_path = os.path.join(DATASETS_PATH, 'office_room_traj3_loop/scene.raw')
ICL_NUIM_OFF_3.ground_truth = os.path.join(DATASETS_PATH, 'office_room_traj3_loop/traj3.gt.freiburg')
ICL_NUIM_OFF_3.ground_truth_assoc = ICL_NUIM_OFF_3.ground_truth
ICL_NUIM_OFF_3.camera_file = os.path.join(DATASETS_PATH, 'office_room_traj3_loop/camera.txt')
ICL_NUIM_OFF_3.camera = '481.2,480,320,240'
ICL_NUIM_OFF_3.init_pose = '0.5,0.5,0.5'
ICL_NUIM_OFF_3.ate_associate_identity = True
ICL_NUIM_OFF_3.descr = 'off_traj_3'

