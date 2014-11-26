#!/usr/bin/env python

import roslib; roslib.load_manifest('smach_scout')
import rospy
import smach
import smach_ros








# define state start
class start(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_mapping','to_moving','error'])

    def execute(self, userdata):
        rospy.loginfo('Executing state start')
	
	print('\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n')
	x = input("Choose Operation Mode:\n  1 - Mapping\n  2 - Moving\n")
	
        if x == 1:
            return 'to_mapping'
	elif x==2:
	    return 'to_moving'
        else:
            return 'error'



# define state mapping
class mapping(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_start','reset'])

    def execute(self, userdata):
        rospy.loginfo('Executing state mapping')
	
	print('\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n')
	x = input("!! Mapping !! \n  0 - to quit\n")
	if x==0:
           return 'to_start'
	else:
	   return 'reset'




# define state moving
class moving(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['to_start','reset'])

    def execute(self, userdata):
        rospy.loginfo('Executing state moving')
	
	print('\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n')
	x = input("Select the way to move the robot \n  0 - exit\n  1 - Arrow Keys\n  2 - Predefined Positions\n  3 - Kinect Navigation\n")
	if x==0:
           return 'to_start'

	#elif x==1:
	 #  return 'to_arrow_keys'

	#elif x==2:
	 #  return 'to_predefined_position'

	#elif x==3:
	 #  return 'to_kinect_navigation'

	else:
	   return 'reset'
        


# define state arrow_keys
#class arrow_keys(smach.State):
#    def __init__(self):
#        smach.State.__init__(self, outcomes=['to_moving'])
#
#    def execute(self, userdata):
#        rospy.loginfo('Executing state arrow_keys')

#	print('\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n')
#	x = input("Arrow keys driving\n  0 - exit")
#
#	if x==0:
#           return 'to_moving'
	

# define state predefined_position
#class predefined_position(smach.State):
#    def __init__(self):
#        smach.State.__init__(self, outcomes=['to_moving'])

#    def execute(self, userdata):
#        rospy.loginfo('Executing state arrow_keys')

#	print('\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n')
#	x = input("pre-defined position navigation\n  0 - exit")

#	if x==0:
#           return 'to_moving'


# define state predefined_position
#class kinect_navigation(smach.State):
#    def __init__(self):
#        smach.State.__init__(self, outcomes=['to_moving'])

#    def execute(self, userdata):
#        rospy.loginfo('Executing state arrow_keys')

#	print('\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n')
#	x = input("kinect navigation\n  0 - exit")

#	if x==0:
#           return 'to_moving'






# main
def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('start', start(), 
                               transitions={'to_mapping':'mapping',
					    'to_moving':'moving',
                                            'error':'start'}) 
                                           
        smach.StateMachine.add('mapping', mapping(), 
                               transitions={'to_start':'start',
					    'reset':'mapping'})


	smach.StateMachine.add('moving', moving(), 
                               transitions={'to_start':'start',
					    'reset':'moving'})

#					    'to_arrow_keys':'arrow_keys',
#					    'to_predefined_position':'predefined_position',
#					    'to_kinect_navigation':'kinect_navigation',
					    
#	smach.StateMachine.add('arrow_keys', moving(), 
#                               transitions={'to_moving':'moving'})


#	smach.StateMachine.add('predefined_position', moving(), 
#                               transitions={'to_moving':'moving'})


#	smach.StateMachine.add('kinect_navigation', moving(), 
#                               transitions={'to_moving':'moving'})
					    


    # Create and start the introspection server
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # Execute SMACH plan
    outcome = sm.execute()
   
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()







