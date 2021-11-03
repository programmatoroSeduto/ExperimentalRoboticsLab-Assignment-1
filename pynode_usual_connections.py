


# === service connection === #
rospy.loginfo( "[%s] asking for service [%s] ...", test_name,  )
rospy.wait_for_service( client_name_random_room )
client_random_room = rospy.ServiceProxy( client_name_random_room, RandomRoom )
rospy.loginfo( "[%s] OK!", test_name )



