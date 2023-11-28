import rospy
import smach
import smach_ros
from std_msgs.msg import Bool
# ... [other imports] ...

# Define the CollisionDetection state
class CollisionDetection(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['aborted', 'running'])
        self.publisher = rospy.Publisher('/collision_status', Bool, queue_size=10)

    def execute(self, userdata):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # Check for collision
            collision_detected = False  # Replace with actual collision detection logic
            if collision_detected:
                self.publisher.publish(True)
                return 'aborted'
            self.publisher.publish(False)
            rate.sleep()
        return 'running'

# ... [other state definitions] ...

def main():
    rospy.init_node('feeding_task_state_machine')

    # Create a SMACH state machine
    sm = CustomStateMachine(outcomes_sm, input_keys_sm, output_keys=input_keys_sm)

    # ... [state machine setup] ...

    # Create a Concurrence container
    cc = smach.Concurrence(outcomes=['succeeded', 'aborted'],
                           default_outcome='succeeded',
                           outcome_map={'aborted': {'COLLISION_DETECTION': 'aborted'}})

    # Add states to the Concurrence container
    with cc:
        smach.Concurrence.add('MAIN_STATE_MACHINE', sm)
        smach.Concurrence.add('COLLISION_DETECTION', CollisionDetection())

    # Execute the Concurrence container
    outcome = cc.execute()

    rospy.spin()

if __name__ == '__main__':
    main()
    
    
# conversion of reach food items
import rospy
import tf2_ros
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
from moveit_commander import MoveGroupCommander

class Manipulation:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('manipulation_node', anonymous=True)

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        # MoveIt! commander
        self.move_group = MoveGroupCommander("arm")

        # Constants
        self.BASE_LINK_NAME = "base_link"
        self.GRASP_LINK_NAME = "grasp_link"
        self.TOOL_LINK_NAME = "tool_link"

    def reach_food_item(self):
        self.move_group.go(joint_group_positions, wait=True)
        rospy.sleep(2)

        # Get transform from base to grasp
        t_base_grasp = self.tf_buffer.lookup_transform(self.BASE_LINK_NAME, self.GRASP_LINK_NAME, rospy.Time(0))

        # Additional transform from anygrasp to kinova tool frame
        q_base_grasp = quaternion_from_euler(0.7071068, 0.7071068, 0, 0)

        # Grasp pose in Euler angles
        grasp_pose = geometry_msgs.msg.Vector3()
        grasp_pose.x = 180 / 57.3
        grasp_pose.y = 0 / 57.3
        grasp_pose.z = 90 / 57.3

        # T_tool_grasp
        T_tool_grasp = geometry_msgs.msg.Transform()
        T_tool_grasp.translation.x = 0.0
        T_tool_grasp.translation.y = 0.0
        T_tool_grasp.translation.z = 0.175
        T_tool_grasp.rotation.x = 0.00
        T_tool_grasp.rotation.y = 0.00
        T_tool_grasp.rotation.z = 0.00
        T_tool_grasp.rotation.w = 1.00

        # T_grasp_tool
        T_grasp_tool = T_tool_grasp
        T_grasp_tool.rotation = T_tool_grasp.rotation
        T_grasp_tool.translation = T_tool_grasp.translation

        # Determine EE (end effector) pose
        T_base_tool = geometry_msgs.msg.Transform()
        T_base_tool.translation.x = t_base_grasp.transform.translation.x + T_grasp_tool.translation.x
        T_base_tool.translation.y = t_base_grasp.transform.translation.y + T_grasp_tool.translation.y
        T_base_tool.translation.z = t_base_grasp.transform.translation.z + T_grasp_tool.translation.z
        T_base_tool.rotation = q_base_grasp

        # Pre-grasp
        self.move_group.set_pose_target(T_base_tool)
        self.move_group.go(wait=True)
        rospy.sleep(1)

        # Adjust position for grasp
        T_base_tool.translation.z += 0.08
        self.move_group.set_pose_target(T_base_tool)
        self.move_group.go(wait=True)
        rospy.sleep(1)

        # Additional steps like goTop() can be added here

if __name__ == "__main__":
    manipulation = Manipulation()
    manipulation.reach_food_item()
    
    
import tf.transformations
import geometry_msgs.msg

def invert_transform(transform):
    """
    Inverts a geometry_msgs.msg.Transform object.
    
    Args:
        transform (geometry_msgs.msg.Transform): The transform to be inverted.
    
    Returns:
        geometry_msgs.msg.Transform: The inverted transform.
    """
    # Convert Transform to a 4x4 matrix
    matrix = tf.transformations.compose_matrix(
        translate=[transform.translation.x, transform.translation.y, transform.translation.z],
        angles=tf.transformations.euler_from_quaternion([
            transform.rotation.x, 
            transform.rotation.y, 
            transform.rotation.z, 
            transform.rotation.w
        ])
    )

    # Invert the matrix
    inverse_matrix = tf.transformations.inverse_matrix(matrix)

    # Convert the inverted matrix back to a Transform object
    inverse_transform = geometry_msgs.msg.Transform()
    inverse_transform.translation.x, inverse_transform.translation.y, inverse_transform.translation.z = tf.transformations.translation_from_matrix(inverse_matrix)
    quat = tf.transformations.quaternion_from_matrix(inverse_matrix)
    inverse_transform.rotation.x, inverse_transform.rotation.y, inverse_transform.rotation.z, inverse_transform.rotation.w = quat

    return inverse_transform

# Example usage
# Assume T_tool_grasp is a properly defined Transform object
# T_grasp_tool = invert_transform(T_tool_grasp)


import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros

# define state Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        if self.counter < 3:
            self.counter += 1
            return 'outcome1'
        else:
            return 'outcome2'


# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        return 'outcome1'
        


# define state Bas
class Bas(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome3'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAS')
        return 'outcome3'




def main():
    rospy.init_node('smach_example_state_machine')

    # Create the top level SMACH state machine
    sm_top = smach.StateMachine(outcomes=['outcome6'])
    
    # Open the container
    with sm_top:

        smach.StateMachine.add('BAS', Bas(),
                               transitions={'outcome3':'CON'})

        # Create the sub SMACH state machine
        sm_con = smach.Concurrence(outcomes=['outcome4','outcome5'],
                                   default_outcome='outcome4',
                                   outcome_map={'outcome5':
                                       { 'FOO':'outcome2',
                                         'BAR':'outcome1'}})

        # Open the container
        with sm_con:
            # Add states to the container
            smach.Concurrence.add('FOO', Foo())
            smach.Concurrence.add('BAR', Bar())

        smach.StateMachine.add('CON', sm_con,
                               transitions={'outcome4':'CON',
                                            'outcome5':'outcome6'})

    # Execute SMACH plan
    outcome = sm_top.execute()


if __name__ == '__main__':
    main()
    
import smach

# Define your Concurrence container
cc = smach.Concurrence(
    outcomes=['succeeded', 'aborted'],
    default_outcome='succeeded',
    outcome_map={'aborted': {'collision_detection': 'aborted'}}
)

# Assuming you're adding the Concurrence container to your main state machine 'sm'
with sm:
    # Add states to the Concurrence container 'cc' as needed
    # ...

    # Add the Concurrence container 'cc' to the main state machine 'sm'
    smach.StateMachine.add('CONCURRENT_STATE', cc, 
                           transitions={'succeeded': 'NEXT_STATE',  # Define the next state for 'succeeded'
                                        'aborted': 'aborted'})     # Map 'aborted' to the 'aborted' outcome of the main state machine
    
import rospy
from smach import State
from std_msgs.msg import String  # or any other message type you need

class TopicState(State):
    def __init__(self, topic_name, message_type, timeout=None):
        State.__init__(self, outcomes=['succeeded', 'aborted', 'preempted'])
        self.topic_name = topic_name
        self.message_type = message_type
        self.timeout = timeout
        self.message_received = None
        self.subscriber = rospy.Subscriber(self.topic_name, self.message_type, self.message_callback)

    def message_callback(self, msg):
        self.message_received = msg

    def execute(self, userdata):
        start_time = rospy.Time.now()
        self.message_received = None
        while not rospy.is_shutdown():
            if self.message_received is not None:
                return 'succeeded'
            if self.timeout and rospy.Time.now() - start_time > rospy.Duration(self.timeout):
                return 'aborted'
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            rospy.sleep(0.1)
        return 'aborted'

    def request_preempt(self):
        super(TopicState, self).request_preempt()
        self.subscriber.unregister()


sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
with sm:
    smach.StateMachine.add('SUBSCRIBE_TOPIC', TopicState('/my_topic', String, timeout=5.0),
                           transitions={'succeeded': 'NEXT_STATE', 
                                        'aborted': 'ERROR_STATE', 
                                        'preempted': 'PREEMPTED_STATE'})  
    # replace the original collision_detection class




# // Assuming these are defined somewhere in your code
# ros::Publisher collision_status_pub;
# std_msgs::Bool collision_state;
# const double TAU_THRESHOLD = ...;  // Your threshold value

# // ... inside your loop or function where you check for collision ...

# if (delta_tau.maxCoeff() >= TAU_THRESHOLD) {
#     ROS_INFO(YELLOW "Excessive joint torque detected! Stopping!" RESET);

#     if (stop_client.call(stop_action)) {
#         ROS_INFO(GREEN "Succeeded to Stop!" RESET);
#     } else {
#         ROS_INFO(RED "Failed to stop! Press emergency stop!" RESET);
#         // return 1; // Depending on your application, you might want to uncomment this
#     }

#     // Set collision state to true and publish for 3 seconds
#     collision_state.data = true;
#     ros::Time start_time = ros::Time::now();
#     while (ros::Time::now() - start_time < ros::Duration(3.0)) {
#         collision_status_pub.publish(collision_state);
#         ros::spinOnce();
#         ros::Duration(0.1).sleep();  // Small sleep to avoid flooding the topic
#     }

#     // Optionally, reset collision_state to false after 3 seconds
#     collision_state.data = false;
#     collision_status_pub.publish(collision_state);
# }
# ros::spinOnce();


import threading

# Shared variable for collision status
collision_detected = False

def collision_monitor():
    global collision_detected
    while not rospy.is_shutdown():
        # Check for collision (e.g., by subscribing to a collision topic)
        # Set collision_detected to True if a collision occurs
        if check_collision():
            collision_detected = True
        rospy.sleep(0.1)

# Start the collision monitor in a separate thread
collision_thread = threading.Thread(target=collision_monitor)
collision_thread.start()

# In your state machine states, check for collision
class SomeState(smach.State):
    def execute(self, userdata):
        global collision_detected
        if collision_detected:
            # Handle collision
            return 'collision_detected'
        # Normal state execution logic
        # ...

    def execute(self, userdata):
        global global_collision_detected
        # Your state execution logic...
        if global_collision_detected:
            rospy.logwarn("Collision detected in FoodItemSelector, aborting.")
            return 'aborted'
        # Continue with the state logic...
        return 'next_state'


class move_to_feeding_initial_position(smach_ros.ServiceState):
    def __init__(self, target_positions_, input_keys_sm, outcomes_sm):
        super(move_to_feeding_initial_position, self).__init__(
            service_name='/kortex_simple_joint_motion_service',
            service_spec=kortex_motion_planning.srv.KortexSimpleJmpe,
            request=kortex_motion_planning.srv.KortexSimpleJmpeRequest(target_positions_),
            response_slots=['success'],
            outcomes=outcomes_sm,
            input_keys=input_keys_sm,
            output_keys=input_keys_sm,
            response_cb=move_to_feeding_initial_position_callback
        )

    def execute(self, userdata):
        outcome = super(move_to_feeding_initial_position, self).execute(userdata)
        global collision_detected
        class_name = self.__class__.__name__  # Get the class name

        if collision_detected:
            striking_loginfo(f"Collision detected in state: {class_name}, aborting...")
            return 'aborted'
        return outcome


def execute(self, ud):
    """Execute service with integrated collision detection."""
    # Initialize collision detection flag
    global collision_detected
    collision_detected = False

    # Function to check for collision
    def check_collision():
        # Implement your collision detection logic here
        # Update the global collision_detected variable as needed
        pass

    # Rest of the existing code for preemption and service connection...

    # Grab request key if set
    # Existing code to grab the request key...

    # Call user-supplied callback, if set, to get a request
    # Existing code for request callback...

    if self._request is None:
        rospy.logerr("Attempting to call service "+self._service_name+" with no request")
        return 'aborted'

    # Call service with collision detection
    try:
        rospy.logdebug("Calling service %s with request:\n%s" % (self._service_name, str(self._request)))
        self._response_future = self._proxy.call_async(self._request)
        while not self._response_future.done():
            # Check for collision
            check_collision()
            if collision_detected:
                rospy.loginfo("Collision detected during service execution")
                return 'aborted'

            # Sleep for a short duration to avoid busy waiting
            rospy.sleep(0.1)
    except rospy.ServiceException as ex:
        rospy.logerr("Exception when calling service '%s': %s" % (self._service_name, str(ex)))
        return 'aborted'

    # Retrieve the response
    try:
        self._response = self._response_future.result()
    except Exception as ex:
        rospy.logerr("Exception when getting the response from service '%s': %s" % (self._service_name, str(ex)))
        return 'aborted'

    # Call response callback if it's set
    # Existing code for response callback...

    if self._response_key is not None:
        ud[self._response_key] = self._response

    for key in self._response_slots:
        ud[key] = getattr(self._response,key)

    if response_cb_outcome is not None:
        return response_cb_outcome

    return 'succeeded'
  
  
def execute(self, ud):
    """Execute service with integrated collision detection."""
    global collision_detected
    collision_detected = False


    # Make sure we're connected to the service
    try:
        while self._proxy is None:
            if self.preempt_requested():
                rospy.loginfo("Preempting while waiting for service '%s'." % self._service_name)
                self.service_preempt()
                return 'preempted'
            if rospy.is_shutdown():
                rospy.loginfo("Shutting down while waiting for service '%s'." % self._service_name)
                return 'aborted'
            try:
                rospy.wait_for_service(self._service_name,1.0)
                self._proxy = rospy.ServiceProxy(self._service_name, self._service_spec)
                rospy.logdebug("Connected to service '%s'" % self._service_name)
            except rospy.ROSException as ex:
                rospy.logwarn("Still waiting for service '%s'..." % self._service_name)
    except:
        rospy.logwarn("Terminated while waiting for service '%s'." % self._service_name)
        return 'aborted'
    # Rest of the existing code for preemption and service connection...

    # Grab request key if set
    # Existing code to grab the request key...
        # Grab request key if set
    if self._request_key is not None:
        if self._request_key in ud:
            self._request = ud[self._request_key]
        else:
            rospy.logerr("Requested request key '%s' not in userdata struture. Available keys are: %s" % (self._request_key, str(list(ud.keys()))))
            return 'aborted'
    # Write request fields from userdata if set
    for key in self._request_slots:
        if key in ud:
            setattr(self._request,key,ud[key])
        else:
            rospy.logerr("Requested request slot key '%s' is not in userdata strcture. Available keys are: %s" % (key, str(list(ud.keys()))))
            return 'aborted'

    # Call user-supplied callback, if set, to get a request
    # Existing code for request callback...
    if self._request_cb is not None:
        try:
            request_update = self._request_cb(
                    smach.Remapper(
                            ud,
                            self._request_cb_input_keys,
                            self._request_cb_output_keys,
                            []),
                    self._request,
                    *self._request_cb_args,
                    **self._request_cb_kwargs)
            if request_update is not None:
                self._request = request_update
        except:
            rospy.logerr("Could not execute request callback: "+traceback.format_exc())
            return 'aborted'

    if self._request is None:
        rospy.logerr("Attempting to call service "+self._service_name+" with no request")
        return 'aborted'

    # Call service with collision detection
    try:
        rospy.logdebug("Calling service %s with request:\n%s" % (self._service_name, str(self._request)))
        self._response_future = self._proxy.call_async(self._request)
        while not self._response_future.done():
            # Check for collision
            if collision_detected:
                rospy.loginfo("Collision detected during service execution")
                return 'aborted'

            # Sleep for a short duration to avoid busy waiting
            rospy.sleep(0.1)
    except rospy.ServiceException as ex:
        rospy.logerr("Exception when calling service '%s': %s" % (self._service_name, str(ex)))
        return 'aborted'

    # Retrieve the response
    try:
        self._response = self._response_future.result()
    except Exception as ex:
        rospy.logerr("Exception when getting the response from service '%s': %s" % (self._service_name, str(ex)))
        return 'aborted'

    # Call response callback if it's set
    # Existing code for response callback...
    response_cb_outcome = None
    if self._response_cb is not None:
        try:
            response_cb_outcome = self._response_cb(
                    smach.Remapper(
                            ud,
                            self._response_cb_input_keys,
                            self._response_cb_output_keys,
                            []),
                    self._response,
                    *self._response_cb_args,
                    **self._response_cb_kwargs)
            if response_cb_outcome is not None and response_cb_outcome not in self.get_registered_outcomes():
                rospy.logerr("Result callback for service "+self._service_name+", "+str(self._response_cb)+" was not registered with the response_cb_outcomes argument. The response callback returned '"+str(response_cb_outcome)+"' but the only registered outcomes are: "+str(self.get_registered_outcomes()))
                return 'aborted'
        except:
            rospy.logerr("Could not execute response callback: "+traceback.format_exc())
            return 'aborted'

    if self._response_key is not None:
        ud[self._response_key] = self._response

    for key in self._response_slots:
        ud[key] = getattr(self._response,key)

    if response_cb_outcome is not None:
        return response_cb_outcome

    return 'succeeded'
  
