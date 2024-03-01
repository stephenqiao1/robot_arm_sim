'''
This script is a client node. Use this to change the cubes' position in Gazebo.
The server is '/gazebo/set_entity_state', and it runs automatically when gazebo starts
because I include the gazebo_ros_state plugin in my world file.

This client sends a request to the service to change the position on the sphere.

Basically, it sends the position in X, Y, Z where I want to put the cubes.
For now every time that the node is running sent random positions and wait for the confirmation.

Executable name in the setup file: my_client_node
'''

import sys
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SetEntityState

import random

class MyNodeClient(Node):
    
    def __init__(self):
        super().__init__('my_client_cubes_node_positions')
        
        self.client_ = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        
        # Check if the a service is available
        
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
    def send_request(self, name, x, y, z): 
        req = SetEntityState.Request() # Creates a new service request
        req.state.name = name
        req.state.reference_frame= 'world'
        req.state.pose.position.x = x
        req.state.pose.position.y = y
        req.state.pose.position.z = z
        
        future = self.client_.call_async(req) # sends the request asynchronously and returns a 'future' object
        
def main(args=None):
    rclpy.init(args=args)
    node_client = MyNodeClient()
    
    futures = []
    cube_names = ['green_cube', 'blue_cube', 'red_cube']
    for name in cube_names:
        x = random.uniform(-2.0, 2.0)
        y = random.uniform(-2.0, 2.0)
        z = random.uniform(0.1, 2.0)
        future = node_client.send_request(name, x, y, z)
        futures.append(future)
        
    while rclpy.ok(): # keeps running until all service requests have been processed
        rclpy.spin_once(node_client)
        all_done = all([future.done() for future in futures])
        if all_done:
            for name, future in zip(cube_names, futures):
                try:
                    response = future.result()
                except Exception as e:
                    node_client.get_logger().info('Service call failed %r' % (e,))
                else:
                    node_client.get_logger().info('Moved &s: status: %s' % (name, response.success))
            break
        
    node_client.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()