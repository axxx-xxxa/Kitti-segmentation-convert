import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import PointField
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Header
import numpy as np

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

def talker():
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

    points64 = np.fromfile('test64.bin',dtype=np.float32).reshape(-1,4)  #读取bin文件reshape成Nx4
    points32 = np.fromfile('test32.bin',dtype=np.float32).reshape(-1,4)  #读取bin文件reshape成Nx4


    while not rospy.is_shutdown():
        # msg64 = create_msg(points64)
        # msg32 = create_msg(points32)
        msg64 = pc2.create_cloud(header, fields, points64)
        msg32 = pc2.create_cloud(header, fields, points32)
        # msg64 = pc2.create_cloud_xyz32(header, points64)
        # msg32 = pc2.create_cloud_xyz32(header, points32)

        pub64.publish(msg64)
        pub32.publish(msg32)
        print("published...")
        rate.sleep()

if __name__ == '__main__':

    talker()