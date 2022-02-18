from d3d.dataset.kitti import KittiOdometryLoader
import pcl
from map import SeddoMap
import tqdm
import numpy as np

loader = KittiOdometryLoader("/media/jacobz/BtrfsTest", inzip=True)
dmap = SeddoMap("/home/jacobz/temp_map.db3")

seq_id = 4
tp = fn = 0
for i in tqdm.trange(loader.sequence_sizes[seq_id]):
    cloud = pcl.load_pcd("/home/jacobz/Datasets/kitti-parsed/kitti-odometry_04.bag.%04d.pcd" % i)
    pred = dmap.query(cloud.xyz)
    anno = loader.annotation_3dpoints((4, i))
    moving = anno['moving']
    gt = anno['semantic']

    tp += np.sum(moving & (pred == 0))
    fn += np.sum(moving & (pred != 0))

print(tp, fn, tp/(tp+fn))
