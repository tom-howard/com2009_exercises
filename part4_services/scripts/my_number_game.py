#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from part4_services_jazzy.srv import MyNumberGame

from random import random

class NumberGameService(Node):

    def __init__(self):
        super().__init__('number_game_service')
        
        service_name = 'guess_the_number'
        self.srv = self.create_service(
            srv_type=MyNumberGame, 
            srv_name=service_name,
            callback=self.srv_callback
        )

        self.get_logger().info(
            f"The '/{service_name}' service is active."
        )

        self.reset_magic_number()

    def reset_magic_number(self):
        self.magic_number = int(random() * 100)
        self.num_guesses = 0
        self.get_logger().info(
            "A secret number has been set... Game on!"
        )

    def srv_callback(self, request, response):
        success = False
        hint = ""
        self.num_guesses += 1
        response.num_guesses = self.num_guesses
        guess = int(request.guess)
        if request.cheat:
            hint = f"The answer is {self.magic_number}!"
        elif guess < 0 or guess > 100:
            hint = "Error"
            self.get_logger().warning(
                f"Invalid request '{guess}'. Guess must be between 0 and 100."
            )
        elif guess == self.magic_number:
            success = True
            hint = "Winner"
            self.reset_magic_number()
        elif guess < self.magic_number:
            hint = "Higher"
        elif guess > self.magic_number:
            hint = "Lower"

        response.hint = hint
        response.correct = success
        
        return response

def main():
    rclpy.init()
    node = NumberGameService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
