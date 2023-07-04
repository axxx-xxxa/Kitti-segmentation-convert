import numpy as np
import os
from glob import glob
from KITTI_unit import *
import tqdm
root = "/media/ubuntu/G/dataset/data_odometry_velodyne/dataset/sequences"
# sequence = "04"
save = "/media/ubuntu/G/dataset_KITTI/data_32_velodyne/dataset/sequences"
_bin = "velodyne"
_lab = "labels"

def dataprocess(bin, label):
    bin, label = KITTI_del_outsize_ring(bin.reshape(-1,4), label)
    bin, label = KITTI_add_ring(bin, label)
    bin, label = KITTI_divid_2_to_KITTI(bin, label)
    return bin, label

def convert(path, sequence):
    # print(os.path.join(save, sequence, _bin))
    # assert (os.path.join(save, sequence, _bin) == False)
    # assert (os.path.join(save, sequence, _lab) == False)
    os.makedirs(os.path.join(save, sequence, _bin))
    os.makedirs(os.path.join(save, sequence, _lab))

    bins = glob(os.path.join(path, sequence, _bin, "*"))
    labs = glob(os.path.join(path, sequence, _lab, "*"))
    assert (len(bins) == len(labs))

    tq = tqdm.tqdm(bins, desc=f"[{sequence}] is processing")

    for i, bin in enumerate(tq):
        id = bin.split("/")[-1].split(".")[0]
        lab = os.path.join(path, sequence, _lab, (id + ".label"))

        bin64 = np.fromfile(bin, dtype=np.float32)
        lab64 = np.fromfile(lab, dtype=np.int32)
        bin32, lab32 = dataprocess(bin64, lab64)

        bin32.tofile(os.path.join(save, sequence, _bin, (id + ".bin")))
        lab32.tofile(os.path.join(save, sequence, _lab, (id + ".label")))



if __name__ == '__main__':
    # all
    # seqs = ["00","01","02","03","04","05","06","07","08","09","10"]
    # 00,02,05,08 4000+
    # 01 03 04 06 07 OK
    # seqs = ["09","10"]
    seqs = ["00","02","05","08"]
    for seq in seqs:
        convert(root, seq)