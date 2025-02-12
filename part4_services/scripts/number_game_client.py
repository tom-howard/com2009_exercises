#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from part4_services.srv import MyNumberGame

import argparse

class NumberGameClient(Node):

    def __init__(self):
        super().__init__('number_game_client')
        
        self.client = self.create_client(
            srv_type=MyNumberGame, 
            srv_name='guess_the_number'
        )
        
        cli = argparse.ArgumentParser()
        cli.add_argument(
            "-g", "--guess", default=0, type=int
        )
        cli.add_argument(
            "-c", "--cheat", action="store_true"
        )
        self.args = cli.parse_args()
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "Waiting for service..."
            )

    def send_request(self, guess, cheat):
        request = MyNumberGame.Request()
        request.guess = guess
        request.cheat = cheat
        
        return self.client.call_async(request)

def main():
    rclpy.init()
    client = NumberGameClient()

    client.get_logger().info(
        f"Sending the request:\n"
        f" - guess: {client.args.guess}\n"
        f" - cheat: {client.args.cheat}\n"
        f"   Awaiting response..."
    )
    
    future = client.send_request(client.args.guess, client.args.cheat)
    rclpy.spin_until_future_complete(client, future)
    response = future.result()
    
    client.get_logger().info(
        f"The server has responded with:\n"
        f" - {'Correct :)' if response.correct else 'Incorrect :('}\n"
        f" - Number of attempts so far: {response.num_guesses}\n"
        f" - A hint for your next guess: '{response.hint}'."
    )
    
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()