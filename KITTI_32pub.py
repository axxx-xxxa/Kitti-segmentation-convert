import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import numpy as np
import os
from glob import glob
import time
_bin = "velodyne"
from tqdm import tqdm


def create_msg(points):
    msg = PointCloud2()
    msg.header.stamp = rospy.Time().now()
    msg.header.frame_id = "velo_link"

    if len(points.shape) == 3:
        msg.height = points.shape[1]
        msg.width = points.shape[0]
    else:
        msg.height = 1
        msg.width = len(points)

    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1)]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * points.shape[0]
    msg.is_dense = False
    msg.data = np.asarray(points, np.float32).tostring()
    return msg

def pub(root32, root64, sequence):
    rospy.init_node('pointcloud_publisher_node', anonymous=True)
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'base_link'
    pub64 = rospy.Publisher('bin64', PointCloud2, queue_size=5)
    pub32 = rospy.Publisher('bin32', PointCloud2, queue_size=5)
    rate = rospy.Rate(10)

    fields = [pc2.PointField(name='x', offset=0, datatype=pc2.PointField.FLOAT32, count=1),
              pc2.PointField(name='y', offset=4, datatype=pc2.PointField.FLOAT32, count=1),
              pc2.PointField(name='z', offset=8, datatype=pc2.PointField.FLOAT32, count=1),
              pc2.PointField(name='i', offset=12, datatype=pc2.PointField.FLOAT32, count=1)]

    bins32 = glob(os.path.join(root32, sequence, _bin, "*"))
    bins64 = glob(os.path.join(root64, sequence, _bin, "*"))
    tq = tqdm(bins32, desc="loading ")
    for i, bin in enumerate(tq):
        bin_ = "{:06d}.bin".format(i)
        bin_32 = os.path.join(root32, sequence, _bin, bin_)
        bin_64 = os.path.join(root64, sequence, _bin, bin_)

        points32 = np.fromfile(bin_32, dtype=np.float32).reshape(-1, 4)  # 读取bin文件reshape成Nx4
        points64 = np.fromfile(bin_64, dtype=np.float32).reshape(-1, 4)  # 读取bin文件reshape成Nx4

    # points64 = np.fromfile('test64.bin',dtype=np.float32).reshape(-1,4)  #读取bin文件reshape成Nx4


        # msg64 = create_msg(points64)
        # msg32 = create_msg(points32)
        # msg64 = pc2.create_cloud(header, fields, points64)
        msg32 = pc2.create_cloud(header, fields, points32)
        msg64 = pc2.create_cloud(header, fields, points64)
        # msg32 = pc2.create_cloud_xyz32(header, points32)
        if i > 0:
            bin_debug = "{:06d}.bin".format(i-1)
            bin_debug_32 = os.path.join(root32, sequence, _bin, bin_debug)
            bin_debug_64 = os.path.join(root64, sequence, _bin, bin_)

            bin_debug_32 = np.fromfile(bin_debug_32, dtype=np.float32).reshape(-1, 4)  # 读取bin文件reshape成Nx4
            bin_debug_64 = np.fromfile(bin_debug_64, dtype=np.float32).reshape(-1, 4)  # 读取bin文件reshape成Nx4
            if bin_debug_64.shape[0] / bin_debug_32.shape[0] > 3:
                print(f"[Warning] {bin_debug} 64:{bin_debug_64.shape[0]} 32:{bin_debug_32.shape[0]}")
                time.sleep(3)
        pub64.publish(msg64)
        pub32.publish(msg32)



        # print("published...")
        # rate.sleep()
        time.sleep(0.1)

if __name__ == '__main__':
    root32 = "/media/ubuntu/G/dataset_KITTI/data_32_velodyne/dataset/sequences"
    root64 = "/media/ubuntu/G/dataset/data_odometry_velodyne/dataset/sequences"
    pub(root32, root64, "04")