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
        cli = argparse.ArgumentParser(...)
        cli.add_argument(
            "--guess",
            default=0,
            type=int,
        )
        cli.add_argument(
            "--cheat",
            default=False,
            type=bool,
        )
        self.args = cli.parse_args()
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                'service not available, waiting again...'
            )
        self.request = MyNumberGame.Request()

    def send_request(self, guess, cheat):
        self.request.guess = guess
        self.request.cheat = cheat
        return self.client.call_async(self.request)

def main():
    rclpy.init()

    client = NumberGameClient()
    future = client.send_request(client.args.guess, client.args.cheat)
    rclpy.spin_until_future_complete(client, future)
    response = future.result()
    client.get_logger().info(
        f"Result:\n"
        f" - {response.hint}"
    )
    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()