import rospy
import controller_manager_msgs.srv as cm_srv

class ControlSwitcher(object):
  def __init__(self, controller_manager_node='/controller_manager', robot_resource_name='panda'):
    rospy.wait_for_service(controller_manager_node + '/switch_controller')
    rospy.wait_for_service(controller_manager_node + '/list_controllers')
    self.switcher_srv = rospy.ServiceProxy(controller_manager_node + '/switch_controller', cm_srv.SwitchController)
    self.lister_srv = rospy.ServiceProxy(controller_manager_node + '/list_controllers', cm_srv.ListControllers)

    self.robot_resource_name = robot_resource_name
    
  def get_current(self):
    controllers = self.lister_srv().controller
    for controller in controllers:
        if controller.state != 'running':
          continue
        
        resources = [item for claimed in controller.claimed_resources for item in claimed.resources]
        
        if len(list(resource for resource in resources if resource.startswith(self.robot_resource_name))):
          return controller

    return None

  def get_current_name(self):
    current = self.get_current()
    return current.name if current else ''

  def switch_controller(self, controller_name):
      controllers = self.lister_srv().controller
      selected = None
      
      for controller in controllers:
        if controller.name == controller_name:
          selected = controller
          break

      required = [item for claimed in controller.claimed_resources for item in claimed.resources]

      start_controllers = [controller_name] if controller_name else []
      stop_controllers = [self.get_current_name()] if not controller_name else []

      for controller in controllers:
          if controller.name == controller_name:
            continue

          resources = [item for claimed in controller.claimed_resources for item in claimed.resources]
          
          if len(list(resource for resource in resources if resource.startswith(self.robot_resource_name))):
            stop_controllers.append(controller.name)
      

      controller_switch_msg = cm_srv.SwitchControllerRequest()
      controller_switch_msg.strictness = 1
      controller_switch_msg.start_controllers = start_controllers
      controller_switch_msg.stop_controllers = stop_controllers

      res = self.switcher_srv(controller_switch_msg).ok
      if res:
          rospy.loginfo('Successfully switched to controller %s' % (controller_name))
          return res
      else:
          return False