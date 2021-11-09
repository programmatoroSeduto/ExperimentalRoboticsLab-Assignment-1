


# === service connection === #
rospy.loginfo( "[%s] asking for service [%s] ...", test_name,  )
rospy.wait_for_service( client_name_random_room )
client_random_room = rospy.ServiceProxy( client_name_random_room, RandomRoom )
rospy.loginfo( "[%s] OK!", test_name )



# publisher : hint signal
rospy.loginfo( "[%s] opening publisher to topic [%s] ...", test_name, publisher_name_hint_sig )
publisher_hint_sig = rospy.Publisher( publisher_name_hint_sig, Empty, queue_size=1 )
rospy.loginfo( "[%s] OK!", test_name )




# subscriber : hint
rospy.loginfo( "[%s] subscribing to topic [%s] ...", test_name, subscriber_name_hint )
rospy.wait_for_message( subscriber_name_hint )
rospy.Subscriber( subscriber_name_hint, Hint, callback_hint )
rospy.loginfo( "[%s] OK!", test_name )
