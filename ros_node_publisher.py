import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from loguru import logger
import json
from collections import OrderedDict
from config import MODE_CONFIG

class RosNodePublisher(Node):
    def __init__(self, node_type=None):
        if not rclpy.ok():
            rclpy.init()
        super().__init__(f'robovla_{node_type}_node')
        self.node_type = node_type
        self.llm_publishers = {}
        self.mode_config = MODE_CONFIG[node_type] if node_type else None
        if node_type:
            self.initialize_node(node_type)

    def initialize_node(self, node_type):
        self.node_type = node_type
        self.mode_config = MODE_CONFIG[node_type]
        for output_topic in self.mode_config['output_topics']:
            self.llm_publishers[output_topic] = self.create_publisher(String, output_topic, 10)
        logger.info(f"robovla {self.node_type} node initialized.")

    def destroy_node(self):
        super().destroy_node()
        rclpy.shutdown()
        self.node_type = None
        logger.info("ROS node destroyed.")

    def get_node_type(self):
        return self.node_type

    def is_initialized(self):
        return self.node_type is not None and rclpy.ok()

    def publish_response(self, response_json):
        try:
            for json_key, topic in self.mode_config['json_keys'].items():
                # Handle top-level key
                if json_key in response_json:
                    msg = String()
                    msg.data = json.dumps(response_json[json_key])
                    self.llm_publishers[topic].publish(msg)
                    logger.info(f"Published {json_key} to {topic}: {msg.data}")
                # Handle nested structure logic
                else:
                    # Iterate through all tasks to check for nested keys
                    for task in response_json['tasks']:
                        if json_key in task:
                            if json_key == 'instruction_function':
                                # Merge task content into instruction_function and rename key to task_name
                                task_copy = OrderedDict()
                                task_copy['task_name'] = task['task']
                                for key, value in task[json_key].items():
                                    task_copy[key] = value
                                msg = String()
                                msg.data = json.dumps(task_copy)
                                logger.info(f"Merged task_name into instruction_function: {msg.data}")
                            else:
                                msg = String()
                                msg.data = json.dumps(task[json_key])
                            self.llm_publishers[topic].publish(msg)
                            logger.info(f"Published {json_key} to {topic}: {msg.data}")
        except Exception as e:
            logger.error(f"Unexpected error: {e}")



