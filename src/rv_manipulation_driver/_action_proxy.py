
import rospy
from actionlib_msgs.msg import GoalID, GoalStatusArray

class ActionProxy(object):
  def __init__(self, topic_in, topic_out, topic_type_name, controller_cb):
    module, typename = topic_type_name.split('/')
    topic_type = getattr(importlib.import_module(module + '.msg'), typename)

    self.controller_cb = controller_cb

    self.goal_type = getattr(importlib.import_module(module + '.msg'), '{}Goal'.format(typename))
    self.feedback_type = getattr(importlib.import_module(module + '.msg'), '{}Feedback'.format(typename))
    self.result_type = getattr(importlib.import_module(module + '.msg'), '{}Result'.format(typename))

    self.goal_subscriber = rospy.Subscriber('{}/goal'.format(topic_in), self.goal_type, self.goal_cb)
    self.republish = topic_out is not None

    if self.republish:
      # CLIENT
      self.cancel_subscriber = rospy.Subscriber('{}/cancel'.format(topic_in), GoalID, queue_size=1)
      self.feedback_publisher = rospy.Publisher('{}/feedback'.format(topic_in), self.feedback_type, queue_size=1)
      self.result_publisher = rospy.Publisher('{}/result'.format(topic_in), self.result_type, queue_size=1)

      # SERVER
      self.cancel_publisher = rospy.Publisher('{}/cancel'.format(topic_out), GoalID, queue_size=1)      
      self.feedback_subscriber = rospy.Subscriber('{}/feedback'.format(topic_out), self.feedback_type, self.feedback_cb)
      self.goal_publisher = rospy.Publisher('{}/goal'.format(topic_out), self.goal_type, queue_size=1)
      self.result_subscriber = rospy.Subscriber('{}/result'.format(topic_out), self.result_type, self.result_cb)

  def cancel_cb(self, msg):
    self.cancel_publisher.publish(msg)

  def feedback_cb(self, msg):
    self.feedback_publisher.publish(msg)
  
  def goal_cb(self, msg):
    self.controller_cb()
    if self.republish:
      self.goal_publisher.publish(msg)

  def result_cb(self, msg):
    self.result_publisher.publish(msg)
 
  def status_cb(self, msg):
    self.status_publisher.publish(msg)