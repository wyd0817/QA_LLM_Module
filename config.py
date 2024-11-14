# Description of robots
DESCRIPTION_ROBOT = {
    "excavator": {'width': 100, 'height': 100, 'functions': ["Excavation", "Unloading"]},
    "dump_truck": {'width': 50, 'height': 100, 'functions': ["Loading", "Unloading"]},
}

# Navigation functions
NAVIGATION_FUNCTIONS = [
    "avoid_areas_for_all_robots",
    "avoid_areas_for_specific_robots",
    "target_area_for_all_robots",
    "target_area_for_specific_robots",
    "allow_areas_for_all_robots",
    "allow_areas_for_specific_robots",
    "return_to_start_for_all_robots",
    "return_to_start_for_specific_robots"
]

ROBOT_SPECIFIC_FUNCTIONS = [
    "Excavation", 
    "ExcavatorUnloading", 
    "DumpUnloading", 
    "DumpLoading"
]


# Robot names
ROBOT_NAMES = {
    "robot_dump_truck_01": {"id": "robot_dump_truck_01", "type": "dump_truck"},
    "robot_dump_truck_02": {"id": "robot_dump_truck_02", "type": "dump_truck"},
    # "robot_dump_truck_03": {"id": "robot_dump_truck_03", "type": "dump_truck"},
    # "robot_dump_truck_04": {"id": "robot_dump_truck_04", "type": "dump_truck"},
    # "robot_dump_truck_05": {"id": "robot_dump_truck_05", "type": "dump_truck"},
    # "robot_dump_truck_06": {"id": "robot_dump_truck_06", "type": "dump_truck"},
    "robot_excavator_01": {"id": "robot_excavator_01", "type": "excavator"},
    # "robot_excavator_02": {"id": "robot_excavator_02", "type": "excavator"},
}

# Get robot functions
def get_robot_functions(robot_name):
    robot_type = ROBOT_NAMES[robot_name]["type"]
    specific_functions = DESCRIPTION_ROBOT[robot_type]["functions"]
    return NAVIGATION_FUNCTIONS + specific_functions

# Get all functions description for planner mode
def get_all_functions_description():
    description = "Here are the common navigation functions for all robots:\n\n"
    description += ", ".join(NAVIGATION_FUNCTIONS) + "\n\n"
    description += "Here are the robots in the system and their specific functions:\n\n"
    for robot_name, robot_info in ROBOTS_CONFIG['robot_names'].items():
        specific_functions = [func for func in ROBOTS_CONFIG["get_robot_functions"](robot_name) if func not in NAVIGATION_FUNCTIONS]
        functions = ", ".join(specific_functions)
        description += f"- {robot_name}: {robot_info['type']} ({functions})\n"
    return description

# Robots configuration
ROBOTS_CONFIG = {
    "description_robot": DESCRIPTION_ROBOT,
    "robot_names": ROBOT_NAMES,
    "get_robot_functions": get_robot_functions,
    "get_all_functions_description": get_all_functions_description
}

# Model configuration
MODEL_CONFIG = {
    "model_options": ["gpt-4o", "gpt-3.5-turbo", "gpt-4-turbo", "claude-3-haiku-20240307", "claude-3-5-sonnet-20240620", "claude-3-opus-20240229", "llama-3.1-70b-versatile", "llama-3.1-8b-instant"],
    "default_model": "gpt-4o",
    "model_type": "openai",
    "max_tokens": 2048,
    "temperature": 0,
    "frequency_penalty": 0,
}

# Flag to determine whether confirmation is required
CONFIRMATION_REQUIRED = False  # Set to False if confirmation is not required

# Initial messages configuration
if CONFIRMATION_REQUIRED:
    INITIAL_MESSAGES_CONFIG = {
        "system": (
            "You are a confident and pattern-following assistant that assists the operator in managing multiple construction robots on a construction site. "
            "In regular conversations, do not generate or send JSON commands. "
            "When it becomes necessary to control the robot swarm, generate the appropriate JSON command but do not send it immediately. "
            "You must ask the operator for confirmation before sending the JSON command. "
            "When asking for confirmation, provide a detailed summary of the command's purpose and expected actions, but **do not include the actual JSON code**. "
            "Use the following format for confirmation: "
            "'I am ready to send a command to [target] on the construction site. The command will [brief description of action]. "
            "Key details: [list important parameters or actions]. Do you agree to proceed with this command?' "
            "It is crucial that you follow this instruction strictly to avoid including any JSON in the confirmation message, "
            "while still providing a clear and detailed description of the command's intent and effects. "
            "After receiving confirmation (e.g., 'yes', 'proceed', 'agreed'), immediately send the JSON command without further questions or explanations. "
            "If the confirmation is negative or unclear, ask for clarification or await further instructions without sending the command."
        ),
        "user_intro": {
            "default": (
                "I would like you to assist in managing multiple construction robots on a construction site. "
                "In most cases, you should engage in regular conversation without generating or sending JSON commands. "
                "However, when it becomes necessary to control the robot swarm, you should write JSON to do so, but only after confirming with me. "
                "When confirming, **do not include the JSON code in your confirmation response under any circumstances**. "
                "Instead, provide a detailed summary of the command's purpose and expected actions using the following format: "
                "'I am ready to send a command to [target] on the construction site. The command will [brief description of action]. "
                "Key details: [list important parameters or actions]. Do you agree to proceed with this command?' "
                "It is essential to adhere to this format, providing a clear description of the command's intent and effects, "
                "while avoiding the inclusion of any JSON in the confirmation message. "
                "Once I confirm (e.g., by saying 'yes', 'proceed', or 'agreed'), immediately send the JSON command without asking any more questions. "
                "If I don't confirm or my response is unclear, ask for clarification or wait for further instructions. "
                "Pay attention to patterns that appear in the given context code. "
                "Be thorough and thoughtful in your JSON. Do not include any import statements. Do not repeat my question. Do not provide any text explanation. "
                "Note that x is back to front, y is left to right, and z is bottom to up.\n\n"
                "Only use functions from the following library:\n\n\n{library}\n\n\n"
                "Consider the following environmental objects in the scene:\n\n\n```{env_objects}```\n\n\n"
                "The available robots on the construction site are:\n\n\n```{robot_names}```\n\n\n"
                "Here are some examples of how to format the JSON based on previous queries:\n\n\n```{fewshot_examples}```\n\n\n"
            ),
            "task_2_commannd_prompt": "You are working on decomposing tasks for the robots.",
            "dart": "You are working on decomposing tasks for the robots.",
            "task_decomposer": "You are working on decomposing tasks for the robots.",
            "composer": "You are composing high-level tasks for the robots.",
            "instruction_translator": "You are translating instructions for the robots.",
            "planner": "You are planning tasks for the robots. Please use both navigation functions and robot-specific functions from the following list:\n\n{functions_description}\n\n"
        },
        "assistant": (
            "Understood. I will generate the JSON and seek your confirmation by providing a detailed summary of the command's purpose and expected actions, without including the actual JSON code. "
            "I will use the format: 'I am ready to send a command to [target] on the construction site. The command will [brief description of action]. Key details: [list important parameters or actions]. Do you agree to proceed with this command?' "
            "This confirmation message will not include any JSON code but will give you a clear understanding of the command's intent and effects. "
            "Once you confirm with a positive response like 'yes', 'proceed', or 'agreed', I will immediately send the JSON command without asking any further questions. "
            "If your response is negative or unclear, I will seek clarification or await further instructions before proceeding."
        )
    }
else:
    INITIAL_MESSAGES_CONFIG = {
        "system": (
            "You are a confident and pattern-following assistant that pays attention to the user's instructions and writes good JSON for controlling multiple construction robots in a construction site. "
            "Please ensure that the JSON code is properly formatted with consistent indentation and alignment for better readability and ease of use when copying."

        ),
        "user_intro": {
            "default": (
                "I would like you to help me write JSON to control multiple construction robots in a construction site. "
                "Please complete the JSON code every time when I give you a new query. Pay attention to patterns that appear in the given context code. "
                "Be thorough and thoughtful in your JSON. Do not include any import statement. Do not repeat my question. Do not provide any text explanation. "
                "Note that x is back to front, y is left to right, and z is bottom to up.\n\n"
                "Only use functions from the following library:\n\n\n{library}\n\n\n"
                "Consider the following environmental objects in the scene:\n\n\n```{env_objects}```\n\n\n"
                "The available robots in the construction site are:\n\n\n```{robot_names}```\n\n\n"
                "Here are some examples of how to format the JSON based on previous queries:\n\n\n```{fewshot_examples}```\n\n\n"
            ),
            "task_2_commannd_prompt": "You are working on decomposing tasks for the robots.",
            "dart": "You are working on decomposing tasks for the robots.",
            "task_decomposer": "You are working on decomposing tasks for the robots.",
            "composer": "You are composing high-level tasks for the robots.",
            "instruction_translator": "You are translating instructions for the robots.",
            "planner": "You are planning tasks for the robots. Please use both navigation functions and robot-specific functions from the following list:\n\n{functions_description}\n\n"
        },
        "assistant": "Got it. I will complete the JSON you provide next."
    }


# Mode configuration
MODE_CONFIG = {
    "task_2_commannd_prompt": {
        "display_name": "Task to Command",
        "prompt_file": "./prompts/swarm/task_2_commannd_prompt.txt",
        "type": "GRADIO_MESSAGE_MODES",
        "model_version": "gpt-4o",
        # "model_version": "claude-3-haiku-20240307",
        "max_tokens": 2048,
        "temperature": 0,
        "frequency_penalty": 0,
        "input_topics": [],
        "output_topics": [
            "instruction_topic",
            "keywords_topic"
        ],
        "json_keys": {
            "instruction_function": "instruction_topic",
            "clip_keywords": "keywords_topic"
        },
        "functions_description": ""
    },
    "dart_gpt_4o": {
        "display_name": "gpt-4o",
        "prompt_file": "./prompts/swarm/dart.txt",
        "type": "GRADIO_MESSAGE_MODES",
        "model_version": "gpt-4o",
        "max_tokens": 2048,
        "temperature": 0,
        "frequency_penalty": 0,
        "input_topics": [],
        "output_topics": [
            "instruction_topic",
            "keywords_topic"
        ],
        "json_keys": {
            "instruction_function": "instruction_topic",
            "clip_keywords": "keywords_topic"
        },
        "functions_description": ""
    },
    "dart_gpt_4_turbo": {
        "display_name": "gpt-4-turbo",
        "prompt_file": "./prompts/swarm/dart.txt",
        "type": "GRADIO_MESSAGE_MODES",
        "model_version": "gpt-4-turbo",
        "max_tokens": 2048,
        "temperature": 0,
        "frequency_penalty": 0,
        "input_topics": [],
        "output_topics": [
            "instruction_topic",
            "keywords_topic"
        ],
        "json_keys": {
            "instruction_function": "instruction_topic",
            "clip_keywords": "keywords_topic"
        },
        "functions_description": ""
    },
    "dart_gpt_3_5_turbo": {
        "display_name": "gpt-3.5-turbo",
        "prompt_file": "./prompts/swarm/dart.txt",
        "type": "GRADIO_MESSAGE_MODES",
        "model_version": "gpt-3.5-turbo",
        "max_tokens": 2048,
        "temperature": 0,
        "frequency_penalty": 0,
        "input_topics": [],
        "output_topics": [
            "instruction_topic",
            "keywords_topic"
        ],
        "json_keys": {
            "instruction_function": "instruction_topic",
            "clip_keywords": "keywords_topic"
        },
        "functions_description": ""
    },
    "dart_claude_3_haiku": {
        "display_name": "claude-3-haiku",
        "prompt_file": "./prompts/swarm/dart.txt",
        "type": "GRADIO_MESSAGE_MODES",
        "model_version": "claude-3-haiku-20240307",
        "max_tokens": 2048,
        "temperature": 0,
        "frequency_penalty": 0,
        "input_topics": [],
        "output_topics": [
            "instruction_topic",
            "keywords_topic"
        ],
        "json_keys": {
            "instruction_function": "instruction_topic",
            "clip_keywords": "keywords_topic"
        },
        "functions_description": ""
    },
    "dart_claude_3_sonnet": {
        "display_name": "claude-3-5-sonnet",
        "prompt_file": "./prompts/swarm/dart.txt",
        "type": "GRADIO_MESSAGE_MODES",
        "model_version": "claude-3-5-sonnet-20240620",
        "max_tokens": 2048,
        "temperature": 0,
        "frequency_penalty": 0,
        "input_topics": [],
        "output_topics": [
            "instruction_topic",
            "keywords_topic"
        ],
        "json_keys": {
            "instruction_function": "instruction_topic",
            "clip_keywords": "keywords_topic"
        },
        "functions_description": ""
    }, 
    "dart_claude_3_opus": {
        "display_name": "claude-3-opus",
        "prompt_file": "./prompts/swarm/dart.txt",
        "type": "GRADIO_MESSAGE_MODES",
        "model_version": "claude-3-opus-20240229",
        "max_tokens": 2048,
        "temperature": 0,
        "frequency_penalty": 0,
        "input_topics": [],
        "output_topics": [
            "instruction_topic",
            "keywords_topic"
        ],
        "json_keys": {
            "instruction_function": "instruction_topic",
            "clip_keywords": "keywords_topic"
        },
        "functions_description": ""
    }, 
    "dart_llama_3_1_70b": {
        "display_name": "llama-3.1-70b-versatile",
        "prompt_file": "./prompts/swarm/dart.txt",
        "type": "GRADIO_MESSAGE_MODES",
        "model_version": "llama-3.1-70b-versatile",
        "max_tokens": 2048,
        "temperature": 0,
        "frequency_penalty": 0,
        "input_topics": [],
        "output_topics": [
            "instruction_topic",
            "keywords_topic"
        ],
        "json_keys": {
            "instruction_function": "instruction_topic",
            "clip_keywords": "keywords_topic"
        },
        "functions_description": ""
    }, 
    "dart_llama_3_1_8b": {
        "display_name": "llama-3.1-8b-instant",
        "prompt_file": "./prompts/swarm/dart.txt",
        "type": "GRADIO_MESSAGE_MODES",
        "model_version": "llama-3.1-8b-instant",
        "max_tokens": 2048,
        "temperature": 0,
        "frequency_penalty": 0,
        "input_topics": [],
        "output_topics": [
            "instruction_topic",
            "keywords_topic"
        ],
        "json_keys": {
            "instruction_function": "instruction_topic",
            "clip_keywords": "keywords_topic"
        },
        "functions_description": ""
    },

    "task_decomposer": {
        "display_name": "Task Decomposer",
        "prompt_file": "./prompts/swarm/task_decomposer_prompt.txt",
        "type": "GRADIO_MESSAGE_MODES",
        "model_version": "gpt-4o",
        "max_tokens": 2048,
        "temperature": 0,
        "frequency_penalty": 0,
        "input_topics": [],
        "output_topics": ["tasks_topic"],
        "json_keys": {"tasks": "tasks_topic"},
        "functions_description": ""
    },
    "composer": {
        "display_name": "Composer",
        "prompt_file": "./prompts/swarm/composer_prompt.txt",
        "type": "GRADIO_MESSAGE_MODES",
        "model_version": "gpt-4o",
        "max_tokens": 2048,
        "temperature": 0,
        "frequency_penalty": 0,
        "input_topics": [],
        "output_topics": ["tasks_topic"],
        "json_keys": {"tasks": "tasks_topic"},
        "functions_description": ""
    },
    "instruction_translator": {
        "display_name": "Instruction Translator (Developer Mode)",
        "prompt_file": "./prompts/swarm/instruction_translator_prompt.txt",
        "type": "ROS_MESSAGE_MODE",
        "model_version": "gpt-4o",
        "max_tokens": 2048,
        "temperature": 0,
        "frequency_penalty": 0,
        "input_topics": ["tasks_topic"],
        "output_topics": [
            "robovla_instruction_translator_out",
            "instruction_topic",
            "keywords_topic"
        ],
        "json_keys": {
            "instruction_function": "instruction_topic",
            "clip_keywords": "keywords_topic"
        },
        "functions_description": ""
    },
    "planner": {
        "display_name": "Planner (Developer Mode)",
        "prompt_file": "./prompts/swarm/planner_prompt.txt",
        "type": "ROS_MESSAGE_MODE",
        "model_version": "gpt-4o",
        "max_tokens": 2048,
        "temperature": 0,
        "frequency_penalty": 0,
        "input_topics": ["tasks_topic"],
        "output_topics": [
            "robovla_instruction_translator_out",
            "instruction_topic",
            "keywords_topic"
        ],
        "json_keys": {
            "instruction_function": "instruction_topic",
            "clip_keywords": "keywords_topic"
        },
        "functions_description": ROBOTS_CONFIG["get_all_functions_description"]()
    }
}

# Default modes to display in the UI
# GRADIO_MESSAGE_MODES = ["task_2_commannd_prompt", "task_decomposer", "instruction_translator", "composer", "planner"]
GRADIO_MESSAGE_MODES = ["dart_gpt_4o", "dart_gpt_3_5_turbo", "dart_gpt_4_turbo", "dart_claude_3_haiku", "dart_claude_3_sonnet", "dart_claude_3_opus", "dart_llama_3_1_70b","dart_llama_3_1_8b", "task_2_commannd_prompt"]
ROS_MESSAGE_MODE = "instruction_translator"
