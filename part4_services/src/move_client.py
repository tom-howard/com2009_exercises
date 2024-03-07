#!/usr/bin/env python3

import rospy 
from tuos_msgs.srv import SetBool, SetBoolRequest

service_name = "move_service" 

rospy.init_node(f"{service_name}_client") 

rospy.wait_for_service(service_name) 

service = rospy.ServiceProxy(service_name, SetBool) 

request_to_server = SetBoolRequest() 
request_to_server.request_signal = True 

response_from_server = service(request_to_server) 
print(response_from_server)