import rospy
import actionlib

from rv_msgs.msg import MoveToPoseAction, MoveToPoseGoal
from rv_msgs.srv import GetRelativePose

from geometry_msgs.msg import PoseStamped

# initialise ros node
rospy.init_node('move_to_points_example')

# Create a ros action client to communicate with the controller
client = actionlib.SimpleActionClient('/arm/cartesian/pose', MoveToPoseAction)
client.wait_for_server()

# Create a ros service client
# TODO probably move this out to somewhere more general
get_pose = rospy.ServiceProxy('/arm/get_link_position', GetRelativePose)
get_pose.wait_for_service()

# Get the current position of panda hand w.r.t. the panda base
current = get_pose('panda_hand', 'panda_link0')
print(current)
# Create a target pose based on our current position but moving up 10cm on the z axis
target = current.relative_pose
target.pose.position.z += 0.1
print(target)
# Adjust target to move 10cm above ready pose and create new goal
goal = MoveToPoseGoal(goal_pose=target)

# Seng goal and wait for it to finish
client.send_goal(goal)
client.wait_for_result()

# Adjust target to original position and create new goal
target = current.relative_pose
target.pose.position.z -= 0.1

# Seng goal and wait for it to finish
client.send_goal(goal)
client.wait_for_result()
