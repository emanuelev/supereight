class Dataset:

    def __init__(self):
        self.dataset = None
        self.ground_truth = None
        self.ground_truth_assoc = None
        self.use_ground_truth = False
        self.camera_file = None
        self.camera = None
        self.quat = None
        self.init_pose = None
        self.rgb_image = None
        self.pre_assoc_file_path = None
        self.descr = None
