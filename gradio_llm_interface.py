import rclpy
import gradio as gr
from loguru import logger
from llm_request_handler import LLMRequestHandler
from ros_node_publisher import RosNodePublisher
from json_processor import JsonProcessor
from config import ROBOTS_CONFIG, MODEL_CONFIG, MODE_CONFIG

class GradioLlmInterface:
    def __init__(self):
        self.node_publisher = None
        self.received_tasks = []
        self.json_processor = JsonProcessor()

    def initialize_interface(self, mode):
        if not rclpy.ok():
            rclpy.init()
        self.node_publisher = RosNodePublisher(mode)
        mode_config = MODE_CONFIG[mode]
        model_version = MODEL_CONFIG["default_model"]
        llm_handler = LLMRequestHandler(
            model_version=model_version,
            max_tokens=MODEL_CONFIG["max_tokens"], 
            temperature=MODEL_CONFIG["temperature"], 
            frequency_penalty=MODEL_CONFIG["frequency_penalty"], 
            list_navigation_once=True,
            model_type=MODEL_CONFIG["model_type"]
        )
        file_path = mode_config["prompt_file"]
        initial_messages = llm_handler.build_initial_messages(file_path, mode)
        chatbot = gr.Chatbot(label="DART-LLM", type="messages")
        state = gr.State({
            "file_path": file_path, 
            "initial_messages": initial_messages, 
            "mode": mode, 
            "llm_config": llm_handler.get_config_dict()  # Store the config dict instead of the handler itself
        })
        return chatbot, state

    async def predict(self, input, state):
        if not self.node_publisher.is_initialized():
            mode = state.get('mode')
            self.node_publisher.initialize_node(mode)
        
        initial_messages = state['initial_messages']
        full_history = initial_messages + state.get('history', [])
        user_input = f"# Query: {input}"
        full_history.append({"role": "user", "content": user_input})

        mode_config = MODE_CONFIG[state.get('mode')]
        if mode_config["type"] == 'complex' and self.received_tasks:
            for task in self.received_tasks:
                task_prompt = f"# Task: {task}"
                full_history.append({"role": "user", "content": task_prompt})
            self.received_tasks = []

        # Create a new LLMRequestHandler instance for each request
        llm_config = state['llm_config']
        llm_handler = LLMRequestHandler.create_from_config_dict(llm_config)

        response = await llm_handler.make_completion(full_history)
        if response:
            full_history.append({"role": "assistant", "content": response})
        else:
            response = "Error: Unable to get response."
            full_history.append({"role": "assistant", "content": response})
        
        response_json = self.json_processor.process_response(response)
        self.node_publisher.publish_response(response_json)

        # Modify the messages format to match the "messages" type
        messages = [{"role": message["role"], "content": message["content"]} for message in full_history[len(initial_messages):]]
        updated_history = state.get('history', []) + [{"role": "user", "content": input}, {"role": "assistant", "content": response}]
        state.update({'history': updated_history})
        return messages, state
    
    def clear_chat(self, state):
        state['history'] = []

    def update_chatbot(self, mode, state):
        # Destroy and reinitialize the ROS node
        self.node_publisher.destroy_node()
        if not rclpy.ok():
            rclpy.init()
        self.node_publisher = RosNodePublisher(mode)
        self.json_processor = JsonProcessor()

        # Update llm_handler with the new model settings
        mode_config = MODE_CONFIG[mode]
        model_version = mode_config["model_version"]
        model_type = mode_config.get("model_type", "openai")  # Ensure the correct model_type is used

        # Re-instantiate LLMRequestHandler with the new model_version and model_type
        llm_handler = LLMRequestHandler(
            model_version=model_version,
            max_tokens=mode_config.get("max_tokens", MODEL_CONFIG["max_tokens"]),
            temperature=mode_config.get("temperature", MODEL_CONFIG["temperature"]),
            frequency_penalty=mode_config.get("frequency_penalty", MODEL_CONFIG["frequency_penalty"]),
            list_navigation_once=True,
            model_type=model_type
        )

        # Update the prompt file and initial messages
        file_path = mode_config["prompt_file"]
        initial_messages = llm_handler.build_initial_messages(file_path, mode)

        # Update state with the new handler and reset history
        logger.info(f"Updating chatbot with {file_path} and model {model_version}")
        state['file_path'] = file_path
        state['initial_messages'] = initial_messages
        state['history'] = []
        state['mode'] = mode
        state['llm_handler'] = llm_handler  # Update the state with the new handler

        logger.info(f"\033[33mMode updated to {mode}\033[0m")
        return gr.update(value=[]), state

