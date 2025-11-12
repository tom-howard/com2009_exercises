#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from part4_services_jazzy.srv import MyNumberGame

class NumberGameClient(Node):

    def __init__(self):
        super().__init__('number_game_client')

        self.client = self.create_client(
            srv_type=MyNumberGame, 
            srv_name='guess_the_number'
        ) 

        self.declare_parameters(
            namespace='',
            parameters=[
                ('guess', 0),
                ('cheat', False)
            ]
        ) 

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Waiting for service..."
            ) 

    def send_request(self): 
        guess_input = self.get_parameter('guess').get_parameter_value().integer_value
        cheat_input = self.get_parameter('cheat').get_parameter_value().bool_value

        self.get_logger().info(
            f"Sending the request:\n"
            f" - guess: {guess_input}\n"
            f" - cheat: {cheat_input}\n"
            f"   Awaiting response..."
        )

        request = MyNumberGame.Request()
        request.guess = guess_input
        request.cheat = cheat_input

        return self.client.call_async(request)

def main():
    rclpy.init()
    client = NumberGameClient()

    future = client.send_request() 
    rclpy.spin_until_future_complete(client, future) 
    response = future.result() 

    client.get_logger().info(
        f"The server responded with:\n"
        f" - {'You guessed correctly! :)' if response.correct else 'Incorrect guess :('}\n"
        f" - Number of attempts so far: {response.num_guesses}\n"
        f" - A hint: '{response.hint}'."
    ) 

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()