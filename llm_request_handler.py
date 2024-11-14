import os
import json
import asyncio
from typing import List, Optional, Dict, Any
from loguru import logger
from pydantic import BaseModel

from langchain_core.prompts import ChatPromptTemplate
from langchain_core.messages import SystemMessage, HumanMessage, AIMessage
from langchain_openai import ChatOpenAI
from langchain_anthropic import ChatAnthropic
from langchain_groq import ChatGroq

from config import MODEL_CONFIG, INITIAL_MESSAGES_CONFIG, MODE_CONFIG, NAVIGATION_FUNCTIONS, ROBOT_SPECIFIC_FUNCTIONS, ROBOT_NAMES

class LLMRequestConfig(BaseModel):
    model_name: str = MODEL_CONFIG["default_model"]
    max_tokens: int = MODEL_CONFIG["max_tokens"]
    temperature: float = MODEL_CONFIG["temperature"]
    frequency_penalty: float = MODEL_CONFIG["frequency_penalty"]
    list_navigation_once: bool = True
    provider: str = "openai"
    
    # Resolve Pydantic namespace conflicts
    model_config = {"protected_namespaces": ()}

    def to_dict(self):
        return {
            "model_name": self.model_name,
            "max_tokens": self.max_tokens,
            "temperature": self.temperature,
            "frequency_penalty": self.frequency_penalty,
            "list_navigation_once": self.list_navigation_once,
            "provider": self.provider
        }

    @classmethod
    def from_dict(cls, config_dict):
        return cls(**config_dict)

class LLMRequestHandler:
    class Message(BaseModel):
        role: str
        content: str

    def __init__(self, 
                 # Support both old and new parameter names for backward compatibility
                 model_version: str = None, 
                 model_name: str = None,
                 max_tokens: int = None, 
                 temperature: float = None, 
                 frequency_penalty: float = None, 
                 list_navigation_once: bool = None, 
                 model_type: str = None,
                 provider: str = None,
                 config: Optional[LLMRequestConfig] = None):
                 
        # Initialize with config or from individual parameters
        if config:
            self.config = config
        else:
            # Create config from individual parameters, giving priority to new names
            self.config = LLMRequestConfig(
                model_name=model_name or model_version or MODEL_CONFIG["default_model"],
                max_tokens=max_tokens or MODEL_CONFIG["max_tokens"],
                temperature=temperature or MODEL_CONFIG["temperature"],
                frequency_penalty=frequency_penalty or MODEL_CONFIG["frequency_penalty"],
                list_navigation_once=list_navigation_once if list_navigation_once is not None else True,
                provider=provider or model_type or "openai"
            )
        
        # Store parameters for easier access
        self.model_name = self.config.model_name
        self.model_version = self.model_name  # Alias for backward compatibility
        self.max_tokens = self.config.max_tokens
        self.temperature = self.config.temperature
        self.frequency_penalty = self.config.frequency_penalty
        self.list_navigation_once = self.config.list_navigation_once
        self.provider = self.config.provider
        self.model_type = self.provider  # Alias for backward compatibility

        # Store API keys
        self.openai_api_key = os.getenv("OPENAI_API_KEY")
        self.anthropic_api_key = os.getenv("ANTHROPIC_API_KEY")
        self.groq_api_key = os.getenv("GROQ_API_KEY")
        
        # Create the appropriate LangChain LLM based on provider
        self._setup_llm()
    
    def _setup_llm(self):
        """Initialize the appropriate LangChain LLM based on provider."""
        if "anthropic" in self.provider or "claude" in self.model_name:
            self.llm = ChatAnthropic(
                api_key=self.anthropic_api_key,
                model_name=self.model_name,
                max_tokens=self.max_tokens,
                temperature=self.temperature
            )
        elif "groq" in self.provider or "llama" in self.model_name:
            self.llm = ChatGroq(
                api_key=self.groq_api_key,
                model_name=self.model_name,
                max_tokens=self.max_tokens,
                temperature=self.temperature,
                frequency_penalty=self.frequency_penalty
            )
        else:  # Default to OpenAI
            self.llm = ChatOpenAI(
                api_key=self.openai_api_key,
                model_name=self.model_name,
                max_tokens=self.max_tokens,
                temperature=self.temperature,
                frequency_penalty=self.frequency_penalty
            )

    def get_config_dict(self):
        """Get a serializable configuration dictionary"""
        return self.config.to_dict()

    @staticmethod
    def create_from_config_dict(config_dict):
        """Create a new handler instance from a config dictionary"""
        config = LLMRequestConfig.from_dict(config_dict)
        return LLMRequestHandler(config=config)
        
    def load_object_data(self) -> Dict[str, Any]:
        """Load environment information (E) from a JSON file"""
        json_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'ros2_ws', 'src', 'breakdown_function_handler', 'object_database', 'object_database.json'))
        
        with open(json_path, 'r') as json_file:
            data = json.load(json_file)
        
        return self.format_env_object(data)
    
    def format_env_object(self, data: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Format the environment data (E) for use in the prompt"""
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

    def build_initial_messages(self, file_path: str, mode: str) -> List[Dict[str, str]]:
        """Build the initial prompt (P = (I, E, R, S))"""
        with open(file_path, 'r', encoding='utf-8') as file:
            user1 = file.read()  # Example user instructions for few-shot learning (optional)

        system = INITIAL_MESSAGES_CONFIG["system"]

        # Load environment information (E)
        env_objects = self.load_object_data()

        # Create the user introduction with robot set (R), skills (S), and environment (E)
        user_intro = INITIAL_MESSAGES_CONFIG["user_intro"]["default"] + INITIAL_MESSAGES_CONFIG["user_intro"].get(mode, "")
        functions_description = MODE_CONFIG[mode].get("functions_description", "")

        # Format user introduction with the instruction (I), robot set (R), skills (S), and environment (E)
        user_intro = user_intro.format(
            library=NAVIGATION_FUNCTIONS+ROBOT_SPECIFIC_FUNCTIONS, 
            env_objects=env_objects, 
            robot_names=ROBOT_NAMES, 
            fewshot_examples=user1, 
            functions_description=functions_description
        )

        assistant1 = INITIAL_MESSAGES_CONFIG["assistant"]

        # Construct the messages (system, user, assistant)
        messages = [
            {"role": "system", "content": system},
            {"role": "user", "content": user_intro},
            {"role": "assistant", "content": assistant1}
        ]
        return messages

    def add_user_message(self, messages: List[Dict[str, str]], content: str) -> None:
        """Add a user message with natural language instruction (I)"""
        user_message = self.Message(role="user", content=content)
        messages.append(user_message.dict())

    def _convert_to_langchain_messages(self, full_history: List[Dict[str, str]]):
        """Convert traditional message format to LangChain message objects"""
        lc_messages = []
        for msg in full_history:
            if msg["role"] == "system":
                lc_messages.append(SystemMessage(content=msg["content"]))
            elif msg["role"] == "user":
                lc_messages.append(HumanMessage(content=msg["content"]))
            elif msg["role"] == "assistant":
                lc_messages.append(AIMessage(content=msg["content"]))
        return lc_messages

    async def make_completion(self, full_history: List[Dict[str, str]]) -> Optional[str]:
        """Make a completion request to the selected model using LangChain"""
        logger.debug(f"Using model: {self.model_name}")
        try:
            # Convert traditional messages to LangChain message format
            lc_messages = self._convert_to_langchain_messages(full_history)
            
            # Create a chat prompt template
            chat_prompt = ChatPromptTemplate.from_messages(lc_messages)
            
            # Get the response
            chain = chat_prompt | self.llm
            response = await chain.ainvoke({})
            
            # Extract the content from the response
            return response.content if hasattr(response, 'content') else str(response)
            
        except Exception as e:
            logger.error(f"Error making completion: {e}")
            return None


if __name__ == "__main__":
    async def main():
        selected_model_index = 2  # 0 for OpenAI, 1 for Anthropic, 2 for LLaMA

        model_options = MODEL_CONFIG["model_options"]

        # Choose the model based on selected_model_index
        if selected_model_index == 0:
            model = model_options[0]
            provider = "openai"
        elif selected_model_index == 1:
            model = model_options[4]
            provider = "anthropic"
        elif selected_model_index == 2:
            model = model_options[6]
            provider = "groq"
        else:
            raise ValueError("Invalid selected_model_index")

        logger.debug("Starting test llm_request_handler with LangChain...")
        config = LLMRequestConfig(
            model_name=model,
            list_navigation_once=True,
            provider=provider
        )
        handler = LLMRequestHandler(config=config)

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