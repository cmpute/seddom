from d3d.dataset.kitti import KittiOdometryLoader
import pcl
from map import SeddoMap
import tqdm
import numpy as np

i = 0
loader = KittiOdometryLoader("/media/jacobz/BtrfsTest", inzip=True)
cloud = pcl.load_pcd("/home/jacobz/Datasets/kitti-parsed/kitti-odometry_04.bag.%04d.pcd" % i)
dmap = SeddoMap("/home/jacobz/temp_map.db3")
pred = dmap.query(cloud.xyz)
print(np.unique(pred))
gt = loader.annotation_3dpoints((4, i))['semantic']
print(np.unique(gt))
