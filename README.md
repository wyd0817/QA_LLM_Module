# QA_LLM_Module

## Overview

QA_LLM_Module is a Python library for integrating Large Language Models (LLMs) into multi-robot task management systems, focusing on question-answering and task decomposition. This module leverages natural language inputs to parse complex instructions, enabling multi-robot systems to coordinate tasks effectively with dependency-aware scheduling.

## Features

- **Natural Language Parsing**: Converts instructions into actionable steps for multi-robot systems.
- **Dependency Management**: Handles task dependencies to enable parallel and coordinated robot actions.
- **ROS Integration**: Supports communication within multi-robot systems using the Robot Operating System (ROS).

## Installation

1. Clone the repository:
   ```bash
   git clone https://github.com/wyd0817/QA_LLM_Module.git
   cd QA_LLM_Module
   ```
2. Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

## Usage

1. Configure settings in `config.py`.
2. Run the main interface with Gradio:
   ```bash
   python gradio_gpt_interface.py
   ```

This launches an interactive interface for entering instructions and observing task decomposition.