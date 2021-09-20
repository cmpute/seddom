# this script add noise points to sim_unstructured dataset

import pcl
import numpy as np
import os

N = 100
input_dir = "data/sim_unstructured_annotated/"
output_dir = "data/sim_unstructured_annotated_noisy/"

if not os.path.exists(output_dir):
    os.mkdir(output_dir)

for f in os.listdir(input_dir):
    cloud = pcl.load_pcd(input_dir + f)

    a = np.random.choice(len(cloud), 2 * N)
    b = np.random.choice(len(cloud), 2 * N)
    valid = (a != b) & (cloud['label'][a] != cloud['label'][b])
    va = a[valid][:N]
    vb = b[valid][:N]

    count = len(va)
    p1 = cloud.xyz[va]
    p2 = cloud.xyz[vb]
    r = np.clip(np.random.rand(count, 1), 0.2, 0.8)
    noise = p1 * r + p2 * (1-r)

    new_xyz = np.vstack([cloud.xyz, noise])
    new_label = np.concatenate([cloud['label'], np.full(count, 4)])
    new_cloud = pcl.create_xyzl(np.hstack([new_xyz, new_label[:, np.newaxis]]))

    pcl.save_pcd(output_dir + f, new_cloud)
    print("Processed " + f)
