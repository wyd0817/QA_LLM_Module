import gradio as gr
from loguru import logger
from gradio_llm_interface import GradioLlmInterface
from config import GRADIO_MESSAGE_MODES, MODE_CONFIG
import speech_recognition as sr

# Speech-to-text function
def audio_to_text(audio):
    recognizer = sr.Recognizer()
    with sr.AudioFile(audio) as source:
        audio_data = recognizer.record(source)
    try:
        return recognizer.recognize_google(audio_data)
    except sr.UnknownValueError:
        return "Sorry, I could not understand the audio."
    except sr.RequestError:
        return "Sorry, there was an error with the speech recognition service."

def main():
    gradio_ros_interface = GradioLlmInterface()

    title_markdown = ("""
    # 🌋 DART-LLM: Dependency-Aware Multi-Robot Task Decomposition and Execution using Large Language Models
    [[Project Page](https://wyd0817.github.io/project-dart-llm/)] [[Code](https://github.com/wyd0817/gradio_gpt_interface)] [[Model](https://artificialanalysis.ai/)] | 📚 [[RoboQA](https://www.overleaf.com/project/6614a987ae2994cae02efcb2)] 
    """)

    with gr.Blocks(css="""
        #text-input, #audio-input {
            height: 100px; /* Unified height */
            max-height: 100px;
            width: 100%; /* Full container width */
            margin: 0;
        }
        .input-container {
            display: flex; /* Flex layout */
            gap: 10px; /* Spacing */
            align-items: center; /* Vertical alignment */
        }
    """) as demo:
        gr.Markdown(title_markdown)

        mode_choices = [MODE_CONFIG[mode]["display_name"] for mode in GRADIO_MESSAGE_MODES]
        mode_selector = gr.Radio(choices=mode_choices, label="Backend model", value=mode_choices[0])
        clear_button = gr.Button("Clear Chat")

        logger.info("Starting Gradio GPT Interface...")
        
        initial_mode = GRADIO_MESSAGE_MODES[0]
        chatbot, state = gradio_ros_interface.initialize_interface(initial_mode)
        
        def update_mode(selected_mode, state):
            mode_key = [key for key, value in MODE_CONFIG.items() if value["display_name"] == selected_mode][0]
            return gradio_ros_interface.update_chatbot(mode_key, state)

        mode_selector.change(update_mode, inputs=[mode_selector, state], outputs=[chatbot, state])
        clear_button.click(gradio_ros_interface.clear_chat, inputs=[state], outputs=[chatbot])

        # Text and audio input with corresponding handling
        with gr.Row(elem_id="input-container"):
            txt = gr.Textbox(show_label=False, placeholder="Enter text and press enter", elem_id="text-input", container=False)
            audio_input = gr.Audio(sources=["microphone"], type="filepath", elem_id="audio-input", show_label=True)

        # Handle text input submission
        txt.submit(gradio_ros_interface.predict, [txt, state], [chatbot, state])

        # Handle audio input submission
        def record_and_predict(audio, state):
            if audio is not None:
                text = audio_to_text(audio)
                return gradio_ros_interface.predict(text, state)
            else:
                return "No audio detected", state

        audio_input.change(record_and_predict, inputs=[audio_input, state], outputs=[chatbot, state])

        # Example prompts
        examples = [
            "Move half of the soil from the soil pile to the pit.",
            "Move half of the soil from the soil pile to the puddle."
        ]
        gr.Examples(examples=examples, inputs=txt)

    demo.launch(server_port=8080)

if __name__ == "__main__":
    main()