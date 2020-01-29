import transforms3d as t3
import numpy as np
import geometry_msgs.msg as geometry_msgs

def _to_trans(position, orientation):
  return t3.affines.compose(
    [position.x, position.y, position.z],
    t3.quaternions.quat2mat([
      orientation.w, orientation.x, orientation.y, orientation.z
    ]), 
    [1, 1, 1]
  )

def pose_msg_to_trans(msg):
    return _to_trans(msg.position, msg.orientation)

def trans_to_pose_msg(trans):
    T, R, _, _ = t3.affines.decompose(trans)
    return geometry_msgs.Pose(
      position=geometry_msgs.Point(*T),
      orientation=geometry_msgs.Quaternion(*np.roll(t3.quaternions.mat2quat(R), -1))
    )

def tf_to_trans(tf):
    return _to_trans(*tf)