# sensors/pointcloud2_utils.py
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import rclpy

def make_pointcloud2(points_xyz: np.ndarray, frame_id: str = "lidar_link",
                     stamp=None) -> PointCloud2:
    """
    points_xyz: shape (N,3) float32 (x,y,z)
    """
    msg = PointCloud2()
    msg.header = Header()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp if stamp is not None else rclpy.time.Time().to_msg()

    msg.height = 1
    msg.width = int(points_xyz.shape[0])

    msg.fields = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = points_xyz.astype(np.float32).tobytes()
    return msg
