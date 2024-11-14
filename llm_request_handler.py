import os
from typing import List, Optional
from openai import AsyncOpenAI
import anthropic
from groq import Groq
import asyncio
from loguru import logger
from pydantic import BaseModel
from config import MODEL_CONFIG, INITIAL_MESSAGES_CONFIG, MODE_CONFIG, NAVIGATION_FUNCTIONS, ROBOT_SPECIFIC_FUNCTIONS, ROBOT_NAMES
import json

class LLMRequestHandler:

    class Message(BaseModel):
        role: str
        content: str

    def __init__(self, model_version: str = MODEL_CONFIG["default_model"], max_tokens: int = MODEL_CONFIG["max_tokens"], temperature: float = MODEL_CONFIG["temperature"], frequency_penalty: float = MODEL_CONFIG["frequency_penalty"], list_navigation_once: bool = True, model_type: str = "openai"):
        # Initialization of model parameters and API keys
        self.model_version = model_version
        self.max_tokens = max_tokens
        self.temperature = temperature
        self.frequency_penalty = frequency_penalty
        self.list_navigation_once = list_navigation_once
        self.model_type = model_type

        # Store API keys for different models
        self.openai_api_key = os.getenv("OPENAI_API_KEY")
        self.anthropic_api_key = os.getenv("ANTHROPIC_API_KEY")
        self.groq_api_key = os.getenv("GROQ_API_KEY")
            
    def load_object_data(self):
        # Load environment information (E) from a JSON file
        json_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'ros2_ws', 'src', 'breakdown_function_handler', 'object_database', 'object_database.json'))
        
        with open(json_path, 'r') as json_file:
            data = json.load(json_file)
        
        # Format the environment data
        return self.format_env_object(data)
    
    def format_env_object(self, data):
        # Format the environment data (E) for use in the prompt
        formatted_env_object = {}
        for obj in data:
            object_name = obj['object_name']
            target_position = obj['target_position']
            shape = obj['shape']
            formatted_env_object[object_name] = {
                "position": {
                    "x": target_position["x"],
                    "y": target_position["y"]
                },
                "shape": shape
            }
        return formatted_env_object

    def build_initial_messages(self, file_path: str, mode: str) -> List[dict]:
        # Build the initial prompt (P = (I, E, R, S))
        with open(file_path, 'r', encoding='utf-8') as file:
            user1 = file.read()  # Example user instructions for few-shot learning (optional)

        system = INITIAL_MESSAGES_CONFIG["system"]

        # Load environment information (E)
        env_objects = self.load_object_data()

        # Create the user introduction with robot set (R), skills (S), and environment (E)
        user_intro = INITIAL_MESSAGES_CONFIG["user_intro"]["default"] + INITIAL_MESSAGES_CONFIG["user_intro"].get(mode, "")
        functions_description = MODE_CONFIG[mode].get("functions_description", "")

        # Format user introduction with the instruction (I), robot set (R), skills (S), and environment (E)
        user_intro = user_intro.format(library=NAVIGATION_FUNCTIONS+ROBOT_SPECIFIC_FUNCTIONS, env_objects=env_objects, robot_names=ROBOT_NAMES, fewshot_examples=user1, functions_description=functions_description)

        assistant1 = INITIAL_MESSAGES_CONFIG["assistant"]

        # Construct the messages (system, user, assistant)
        messages = [
            {"role": "system", "content": system},
            {"role": "user", "content": user_intro},
            {"role": "assistant", "content": assistant1}
        ]
        return messages

    def add_user_message(self, messages: List[dict], content: str) -> None:
        # Add a user message with natural language instruction (I)
        user_message = self.Message(role="user", content=content)
        messages.append(user_message.dict())

    async def make_completion(self, full_history: List[dict]) -> Optional[str]:
        # Make a completion request to the selected model
        logger.debug(f"Using model: {self.model_version}")
        try:
            if "claude" in self.model_version:
                # Use Anthropic Claude model
                client = anthropic.Anthropic(api_key=self.anthropic_api_key)
                system_message = next((msg for msg in full_history if msg["role"] == "system"), None)
                user_messages = [msg for msg in full_history if msg["role"] != "system"]
                response = client.messages.create(
                    model=self.model_version,
                    max_tokens=self.max_tokens,
                    temperature=self.temperature,
                    system=system_message["content"] if system_message else "",
                    messages=user_messages
                )
                if response and hasattr(response, 'content') and response.content:
                    return response.content[0].text if response.content else None
                return None
            
            elif "llama" in self.model_version:
                # Use LLaMA model
                client = Groq(api_key=self.groq_api_key)
                prompt = "\n".join([msg["content"] for msg in full_history if msg["role"] != "system"])
                chat_completion = client.chat.completions.create(
                    messages=[
                        {"role": "user", "content": prompt},
                    ],
                    model=self.model_version,
                    max_tokens=self.max_tokens,
                    temperature=self.temperature,
                    frequency_penalty=self.frequency_penalty
                )
                return chat_completion.choices[0].message.content if chat_completion else None
            
            else:
                # Use OpenAI GPT model
                client = AsyncOpenAI(api_key=self.openai_api_key)
                response = await client.chat.completions.create(
                    model=self.model_version,
                    messages=full_history,
                    max_tokens=self.max_tokens,
                    temperature=self.temperature,
                    frequency_penalty=self.frequency_penalty
                )
                return response.choices[0].message.content if response else None

        except Exception as e:
            logger.error(f"Error code: {e}")
            return None


if __name__ == "__main__":
    async def main():
        selected_model_index = 2  # 0 for OpenAI, 1 for Anthropic, 2 for LLaMA

        model_options = MODEL_CONFIG["model_options"]

        # Choose the model based on selected_model_index
        if selected_model_index == 0:
            model = model_options[0]
            model_type = "openai"
        elif selected_model_index == 1:
            model = model_options[4]
            model_type = "anthropic"
        elif selected_model_index == 2:
            model = model_options[6]
            model_type = "llama"
        else:
            raise ValueError("Invalid selected_model_index")

        logger.debug("Starting test llm_request_handler...")
        handler = LLMRequestHandler(model_version=model, list_navigation_once=True, model_type=model_type)

        # Build initial messages based on the selected model
        if selected_model_index == 0:
            messages = handler.build_initial_messages("/root/share/gradio_gpt_interface/prompts/swarm/dart.txt", "dart_gpt_4o")
        elif selected_model_index == 1:
            messages = handler.build_initial_messages("/root/share/gradio_gpt_interface/prompts/swarm/dart.txt", "dart_claude_3_sonnet")
        elif selected_model_index == 2:
            messages = handler.build_initial_messages("/root/share/gradio_gpt_interface/prompts/swarm/dart.txt", "dart_llama_3_1_70b")

        # Add a natural language instruction (I) to the prompt
        handler.add_user_message(messages, "Excavator 1 performs excavation, then excavator 2 performs, then dump 1 performs unload.")

        # Request completion from the model
        response = await handler.make_completion(messages)
        logger.debug(f"Response from make_completion: {response}")

    asyncio.run(main())
