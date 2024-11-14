import rclpy
from std_msgs.msg import String
from loguru import logger
import asyncio
from llm_request_handler import LLMRequestHandler
from ros_node_publisher import RosNodePublisher
from json_processor import JsonProcessor
from config import MODE_CONFIG, ROS_MESSAGE_MODE
import json

class RosMessageServer(RosNodePublisher):
    def __init__(self):
        super().__init__(ROS_MESSAGE_MODE)
        self.mode_config = MODE_CONFIG[ROS_MESSAGE_MODE]
        self._subscriptions = []
        self.received_tasks = []
        self.llm_handler = LLMRequestHandler(
            model_version=self.mode_config["model_version"],
            max_tokens=self.mode_config["max_tokens"],
            temperature=self.mode_config["temperature"],
            frequency_penalty=self.mode_config["frequency_penalty"],
            list_navigation_once=True
        )
        self.initial_messages = self.llm_handler.build_initial_messages(self.mode_config["prompt_file"], ROS_MESSAGE_MODE)
        self.json_processor = JsonProcessor()

        # Initialize subscriptions based on input topics
        for input_topic in self.mode_config["input_topics"]:
            subscription = self.create_subscription(
                String,
                input_topic,
                self.listener_callback,
                10)
            self._subscriptions.append(subscription)

        logger.info(f"{self.mode_config['display_name']} node initialized.")
    
    def listener_callback(self, msg):
        task_data = msg.data
        logger.debug(f"Received raw task data: {task_data}")
        try:
            # Parse the task data from the message
            task_data_clean = task_data.strip('"')
            self.received_tasks.append({"task": task_data_clean})
            asyncio.run(self.process_tasks())
        except Exception as e:
            logger.error(f"Unexpected error: {e}")

    async def process_tasks(self):
        while self.received_tasks:
            task = self.received_tasks.pop(0)
            logger.debug(f"Processing task: {task}")
            await self.send_to_gpt(task)

    async def send_to_gpt(self, task):
        prompt = f"# Task: {task}"
        messages = self.initial_messages + [{"role": "user", "content": prompt}]
        response = await self.llm_handler.make_completion(messages)
        if response:
            self.publish_response(response)
        else:
            self.publish_response("Error: Unable to get response.")

    def publish_response(self, response):
        response_json = self.json_processor.process_response(response)
        super().publish_response(response_json)

def main(args=None):
    rclpy.init(args=args)
    ros_message_server = RosMessageServer()
    rclpy.spin(ros_message_server)
    ros_message_server.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
