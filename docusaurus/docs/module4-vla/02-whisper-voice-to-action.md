# Voice-to-Action with OpenAI Whisper

Natural language understanding is a critical component for intuitive human-robot interaction. Enabling robots to accurately transcribe spoken commands and then act upon them bridges the gap between human intent and robotic execution. OpenAI Whisper, a robust automatic speech recognition (ASR) system, provides an excellent foundation for converting human voice commands into text that can then be processed by Large Language Models (LLMs) and translated into robot actions.

## Concepts

### Automatic Speech Recognition (ASR)

ASR systems convert spoken language into written text. This is the first crucial step in a voice-to-action pipeline. Key challenges for ASR in robotics include:
- Noise Robustness: Robots often operate in noisy environments.
- Accent and Dialect Variation: ASR needs to be robust to diverse speech patterns.
- Domain-Specific Vocabulary: Robotics often involves specialized terminology.

OpenAI Whisper stands out for its strong performance across these challenges, having been trained on a massive and diverse dataset.

### OpenAI Whisper

Whisper is an open-source ASR model released by OpenAI. It can transcribe audio into text in multiple languages and even translate spoken language into English. Its key features include:
- High Accuracy: Achieves state-of-the-art performance on various benchmarks.
- Multilingual: Supports transcription in numerous languages.
- Robustness: Performs well even with background noise or varied speaking styles.
- Open Source: Available for free use, allowing for local deployment or cloud-based API access.

### From Text to Action

Once a voice command is transcribed into text by Whisper, the next steps in the VLA pipeline typically involve:
1. Large Language Model (LLM) Processing: The transcribed text is fed to an LLM (e.g., GPT-4, Llama 2). The LLM's role is to:
    * Interpret Intent: Understand the underlying goal of the human command.
    * Disambiguate: Ask clarifying questions if the command is vague.
    * Decompose Task: Break down complex commands into a sequence of simpler, actionable sub-tasks.
    * Generate Robot-Executable Commands: Output a structured representation of actions that the robot's control system can understand (e.g., a JSON object defining a navigation goal, a manipulation command).
2. ROS 2 Action Pipeline: The LLM's output is then translated into ROS 2 messages, services, or action goals that trigger the robot's low-level controllers.

## Working Code Examples

### Example 1: Transcribing Audio with OpenAI Whisper (Python)

This example demonstrates how to use the OpenAI Whisper model (local or via API) to transcribe an audio file.

#### **Prerequisites:**
- Install `openai` Python package: `pip install openai`
- For local Whisper models, install `whisper` Python package: `pip install -U openai-whisper`
- For local models, you also need `ffmpeg` installed on your system.

#### `whisper_transcriber.py`

```python
import openai
import os
import io
import wave
import numpy as np

# Option 1: Using OpenAI API (Requires API key)
# openai.api_key = os.getenv("OPENAI_API_KEY")

def transcribe_audio_api(audio_file_path):
    """Transcribes an audio file using OpenAI Whisper API."""
    try:
        with open(audio_file_path, "rb") as audio_file:
            transcript = openai.audio.transcriptions.create(
                model="whisper-1", 
                file=audio_file, 
                response_format="text"
            )
            return transcript
    except Exception as e:
        print(f"Error during API transcription: {e}")
        return None

# Option 2: Using local Whisper model (CPU/GPU)
try:
    import whisper
    # Load a smaller model for faster inference if needed (tiny, base, small, medium, large)
    local_model = whisper.load_model("base") 
except ImportError:
    local_model = None
    print("whisper Python package not found. Install with 'pip install openai-whisper' for local transcription.")


def transcribe_audio_local(audio_file_path):
    """Transcribes an audio file using a local Whisper model."""
    if local_model is None:
        return "Local Whisper model not available. Cannot transcribe locally."
    try:
        result = local_model.transcribe(audio_file_path)
        return result["text"]
    except Exception as e:
        print(f"Error during local transcription: {e}")
        return None

# Example Usage
if __name__ == "__main__":
    # Create a dummy audio file for testing
    dummy_audio_path = "dummy_audio.wav"
    with wave.open(dummy_audio_path, 'w') as wf:
        wf.setnchannels(1) # mono
        wf.setsampwidth(2) # 16-bit
        wf.setframerate(16000) # 16kHz
        # Generate a simple tone
        frequency = 440  # Hz
        duration = 1  # seconds
        amplitude = 32767 # Max 16-bit amplitude
        t = np.linspace(0, duration, int(16000 * duration), endpoint=False)
        data = amplitude * np.sin(2 * np.pi * frequency * t)
        wf.writeframes(data.astype(np.int16).tobytes())

    print(f"Dummy audio file created at {dummy_audio_path}")

    # Use local transcription if available
    if local_model:
        print("\n--- Local Whisper Transcription ---")
        transcription_local = transcribe_audio_local(dummy_audio_path)
        print(f"Transcription: {transcription_local}")
    else:
        print("\n--- Local Whisper not configured ---")

    # Use API transcription if API key is set
    # if openai.api_key:
    #     print("\n--- OpenAI API Whisper Transcription ---")
    #     transcription_api = transcribe_audio_api(dummy_audio_path)
    #     print(f"Transcription: {transcription_api}")
    # else:
    #     print("\n--- OPENAI_API_KEY environment variable not set. Skipping API transcription. ---")

    os.remove(dummy_audio_path)
```

### Example 2: Bridging Whisper Text to LLM for Robot Action (Conceptual Python/ROS 2)

This example demonstrates how to pass the transcribed text from Whisper to an LLM and then interpret the LLM's response to trigger a ROS 2 action.

#### `whisper_llm_robot_bridge.py`

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import openai # Assuming API or a wrapper for local LLM

# --- ROS 2 Action Message (Conceptual, replace with actual) ---
# For example, a custom action for 'NavigateToPose'
# class NavigateToPose_SendGoal(Action):
#     target_pose: geometry_msgs.msg.PoseStamped
# --- End of Conceptual ROS 2 Action Message ---

class WhisperLLMRobotBridge(Node):
    def __init__(self):
        super().__init__('whisper_llm_robot_bridge')
        self.transcribed_text_sub = self.create_subscription(
            String,
            'whisper_text_output', # Topic where Whisper publishes transcribed text
            self.text_callback,
            10)
        self.get_logger().info('WhisperLLMRobotBridge node started. Waiting for transcribed text...')

        # --- Conceptual LLM Client ---
        # Assuming an OpenAI client or a custom service client to your local LLM
        # self.llm_client = openai.OpenAI() # For OpenAI API
        # Or self.llm_service_client = self.create_client(LLMQuery, 'llm_inference_service')
        # --- End Conceptual LLM Client ---
        self.llm_response_pub = self.create_publisher(String, 'llm_robot_commands', 10) # For robot commands

    def text_callback(self, msg):
        transcribed_text = msg.data
        self.get_logger().info(f"Received transcribed text: '{transcribed_text}'")

        # --- Step 1: Send transcribed text to LLM ---
        # Conceptual LLM Prompt
        prompt = f"""
The user said: "{transcribed_text}"
Based on this, what is the most appropriate robot command?
Respond with a JSON object containing 'command' and 'parameters'.
Example: {{"command": "navigate_to_pose", "parameters": {{"x": 1.0, "y": 0.5, "theta": 0.0}}}}
Example: {{"command": "pick_object", "parameters": {{"object_id": "red_block", "location": "table"}}}}
Example: {{"command": "say_response", "parameters": {{"text": "I did not understand. Can you rephrase?"}}}}
"""
        
        try:
            # --- Conceptual LLM Call ---
            # response = self.llm_client.chat.completions.create(
            #     model="gpt-3.5-turbo",
            #     messages=[{"role": "user", "content": prompt}]
            # )
            # llm_output_text = response.choices[0].message.content
            # For demonstration, simulate LLM response:
            if "go forward" in transcribed_text.lower():
                llm_output_text = '{{"command": "navigate_forward", "parameters": {{"distance": 1.0}}}}'
            elif "pick up the block" in transcribed_text.lower():
                llm_output_text = '{{"command": "pick_object", "parameters": {{"object_id": "block"}}}}'
            else:
                llm_output_text = '{{"command": "say_response", "parameters": {{"text": "I did not understand. Can you rephrase?"}}}}'
            # --- End Conceptual LLM Call ---

            self.get_logger().info(f"LLM generated command: {llm_output_text}")
            
            # --- Step 2: Parse LLM response and trigger ROS 2 action ---
            robot_command = json.loads(llm_output_text)
            self._execute_robot_command(robot_command)

        except json.JSONDecodeError as e:
            self.get_logger().error(f"LLM response was not valid JSON: {e}")
        except Exception as e:
            self.get_logger().error(f"Error processing LLM command: {e}")

    def _execute_robot_command(self, command_data):
        command_type = command_data.get("command")
        parameters = command_data.get("parameters", {})

        if command_type == "navigate_forward":
            self.get_logger().info(f"Triggering navigation action: forward by {parameters.get('distance')}m")
            # --- Conceptual ROS 2 Action Client Call ---
            # goal_msg = NavigateToPose_SendGoal.Goal()
            # goal_msg.target_pose.pose.position.x = parameters.get('distance')
            # self.nav_action_client.send_goal_async(goal_msg)
            # --- End Conceptual ROS 2 Action Client Call ---
        elif command_type == "pick_object":
            self.get_logger().info(f"Triggering manipulation action: pick {parameters.get('object_id')}")
            # --- Conceptual ROS 2 Action Client Call ---
            # goal_msg = PickAndPlace_SendGoal.Goal()
            # goal_msg.object_id = parameters.get('object_id')
            # self.manipulation_action_client.send_goal_async(goal_msg)
            # --- End Conceptual ROS 2 Action Client Call ---
        elif command_type == "say_response":
            self.get_logger().info(f"Robot says: {parameters.get('text')}")
            # --- Conceptual Text-to-Speech (TTS) call ---
            # self.tts_publisher.publish(String(data=parameters.get('text')))
            # --- End Conceptual TTS Call ---
        else:
            self.get_logger().warn(f"Unknown robot command: {command_type}")
        
        # Publish the command for other nodes to execute
        msg = String()
        msg.data = json.dumps(command_data)
        self.llm_response_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = WhisperLLMRobotBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('WhisperLLMRobotBridge node stopped cleanly.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### How to Run

1.  **Set up OpenAI API Key (if using API):** Set `OPENAI_API_KEY` environment variable.
2.  **Audio Input:** You'll need to feed audio to Whisper. For testing, you can use pre-recorded audio files. For real-time, you'd integrate with a microphone.
3.  **ROS 2 Topic for Whisper Output:** A separate node (or a modified version of `whisper_transcriber.py`) would need to publish the transcribed text to the `whisper_text_output` ROS 2 topic.
4.  **Simulate LLM and Robot Actions:** For `whisper_llm_robot_bridge.py`, if you're not using the actual OpenAI API, the provided code includes a simple `if/elif` structure to simulate LLM responses for basic commands. For robot actions, you would typically have actual ROS 2 action servers or service servers that subscribe to `/llm_robot_commands` or are called directly by the bridge node.
5.  **Build and Source ROS 2 Workspace:** (refer to Module 1 for setup).
6.  **Run the Bridge Node:** `ros2 run <your_package_name> whisper_llm_robot_bridge`.
7.  **Publish Test Text:** In another terminal, simulate Whisper output:
    ```bash
    ros2 topic pub --once /whisper_text_output std_msgs/msg/String "data: 'go forward one meter'"
    ```
    Observe the bridge node's interpretation and conceptual action.

## Diagrams (Text-based)

### Diagram 1: Voice-to-Action Pipeline with Whisper and LLM

```
+----------------+       +-----------------------+       +-----------------------+       +---------------------+
| Human Voice    |----->|  Microphone /         |----->|  OpenAI Whisper       |----->|  Large Language     |
|  Command       |       |  Audio Input          |       |  (ASR)                |       |  Model (LLM)        |
+----------------+       +-----------------------+       |                       |       | (Intent, Planning)  |
                                                         |  (Audio to Text)      |       +----------+----------+
                                                         +-----------------------+                  |
                                                                                                    | (Structured Robot Command)
                                                                                                    V
                                                             +---------------------+
                                                             |  ROS 2 Action       |
                                                             |  Pipeline /         |
                                                             |  Robot Controllers  |
                                                             +---------------------+
```

## Exercises

1.  **Real-time Whisper Integration:** Integrate a microphone input with the Whisper transcription. Create a ROS 2 node that continuously listens to microphone audio, transcribes it using Whisper (local or API), and publishes the text to a ROS 2 topic.
2.  **Enhanced LLM Prompting:** Experiment with different LLM prompts to improve the interpretation of complex or ambiguous commands. How can you guide the LLM to output more reliable structured robot commands? (Hint: The LLM could ask for clarification).
3.  **Implement a Simple Action Server:** Create a dummy ROS 2 action server for a "navigate_forward" action. Connect it to the `whisper_llm_robot_bridge.py` so that a successful navigation command triggers the dummy action.

## Summary

This chapter delved into the crucial initial step of a VLA pipeline: converting human voice commands into actionable text. You've learned about the power of OpenAI Whisper for accurate speech-to-text transcription and how its output can be fed into an LLM for intent interpretation and task decomposition. The conceptual code examples demonstrated the flow from transcribed text to structured robot commands, highlighting the integration points with ROS 2 action pipelines. This foundation in voice-to-action is essential for building intuitive and natural human-robot interfaces, bringing us closer to truly conversational robotics.
