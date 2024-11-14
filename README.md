# QA_LLM_Module

QA_LLM_Module is a core component of the DART-LLM (Dependency-Aware Multi-Robot Task Decomposition and Execution) system. It provides an intelligent interface for parsing natural language instructions into structured, dependency-aware task sequences for multi-robot systems. The module supports multiple LLM providers including OpenAI GPT, Anthropic Claude, and LLaMA models, enabling sophisticated task decomposition with explicit handling of dependencies between subtasks.

## Features

- **Multiple LLM Support**: Compatible with OpenAI GPT-4, GPT-3.5-turbo, Anthropic Claude, and LLaMA models
- **Dependency-Aware Task Decomposition**: Breaks down complex tasks into subtasks with explicit dependencies
- **Structured Output Format**: Generates standardized JSON output for consistent task parsing
- **Real-time Processing**: Supports real-time task decomposition and execution

## Installation



## Installation

You can choose one of the following ways to set up the module:

### Option 1: Install Dependencies Directly (Recommended for Linux/Mac)

1. Clone the repository and install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

2. Configure API keys in environment variables:
   ```bash
   export OPENAI_API_KEY="your_openai_key"
   export ANTHROPIC_API_KEY="your_anthropic_key" 
   export GROQ_API_KEY="your_groq_key"
   ```

### Option 2: Use Docker (Recommended for Windows)

For Windows users, it is recommended to use the pre-configured Docker environment to avoid compatibility issues.

1. Clone the Docker repository:
   ```bash
   git clone https://github.com/wyd0817/DART_LLM_Docker.git
   ```

2. Follow the instructions in the [DART_LLM_Docker](https://github.com/wyd0817/DART_LLM_Docker) repository to build and run the container.

3. Configure API keys in environment variables:
   ```bash
   export OPENAI_API_KEY="your_openai_key"
   export ANTHROPIC_API_KEY="your_anthropic_key" 
   export GROQ_API_KEY="your_groq_key"
   ```

## Usage

### Example Usage:

Run the main interface with Gradio:
   ```bash
   gradio main.py
   ```

### Output Format:

The module processes natural language instructions into structured JSON output following this format:
```json
{
    "instruction_function": {
        "name": "<breakdown function 1>",
        "dependencies": ["<dep 1>", "<dep 2>", "...", "<dep n>"]
    },
    "object_keywords": ["<key 1>", "<key 2>", "...", "<key n>"],
    ...
}

```

## Configuration

The module can be configured through:

- `config.py`: Model settings and API configurations
- `prompts/`: Directory containing prompt templates
- `llm_request_handler.py`: Core logic for handling LLM requests