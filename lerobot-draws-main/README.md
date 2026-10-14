# LeRobot Draws 🤖🎨

**Voice-Controlled Robot Artist**

Transform your voice commands into beautiful drawings with our SO-100 robot arm that listens, understands, and creates art through intelligent collaboration.

Developed at the LeRobot Munich hackathon by the LeBrahDor team:
- Shukrullo Nazirjonov
- Muhammad Sohaib
- Devang Darode
- Artur Adam Habuda
- Philipp Schmid

## 🎥 Project Demo


https://github.com/user-attachments/assets/97043a56-5747-4484-bec1-d5e07ccaf443

---


## 🎯 Project Overview

LeRobot Draws enables natural voice interaction with a robot artist. Users speak their drawing requests, and the system generates images, converts them to robot trajectories, and executes precise drawings on paper.

### Key Features
- **Voice Interface**: Speech-to-text using OpenAI Whisper
- **AI Image Generation**: GPT-4.1 creates drawable line art
- **Trajectory Planning**: Convert images to precise robot movements  
- **Real-time Execution**: SO-100 robot arm draws with millimeter precision
- **Imitation Learning**: GROOT model for advanced manipulation tasks

---

## 🏗️ System Architecture

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│  Voice Input    │    │   AI Processing  │    │ Robot Execution │
│                 │    │                  │    │                 │
│ 🎤 Whisper STT  │───▶│ 🧠 GPT-4.1      │───▶│ 🤖 SO-100 Arm  │
│ 🔊 TTS Response │◀───│ 🖼️  Image Gen    │    │ ✏️  Drawing     │
│ 💬 Web Interface│    │ 📐 Trajectory    │    │ 🎯 Precision    │
└─────────────────┘    └──────────────────┘    └─────────────────┘
         │                       │                       │
         │              ┌─────────────────┐              │
         └─────────────▶│ Function Calling │◀─────────────┘
                        │                 │
                        │ • draw_object() │
                        │ • erase_board() │
                        │ • respond_user()│
                        └─────────────────┘
                                 │
                        ┌─────────────────┐
                        │ Imitation       │
                        │ Learning        │
                        │ (GROOT/Nvidia)  │
                        └─────────────────┘
```

---

## 🚀 Quick Start

### Prerequisites
- SO-100 Robot Arm with USB connection
- Python 3.8+
- OpenAI API key
- Web browser with microphone access

### Installation
```bash
# Clone repository
git clone https://github.com/your-repo/lerobot-draws
cd lerobot-draws

# Install dependencies
pip install -r requirements.txt

# Set environment variables
export OPENAI_API_KEY="your-api-key"
```

### Running the System
```bash
# Start the web interface
cd voice_interface/app
python app.py

# Connect robot (separate terminal)
cd phospho_api
python robot_control.py --robot_port /dev/tty.usbmodem*

# Open browser
open http://localhost:5000
```

---

## 📁 Project Structure

```
lerobot-draws/
├── image_to_trajectory/     # Image processing & path generation
│   ├── api.py              # Main drawing pipeline
│   └── outputs/            # Generated images & trajectories
├── phospho_api/            # Robot control & communication
│   ├── robot_control.py    # Core robot movement via Phospho API
│   └── lerobot_control.py  # LeRobot library-based inverse kinematics control
├── voice_interface/        # Web-based voice interface
│   ├── app/app.py         # Flask backend
│   └── templates/         # Frontend HTML/JS
├── imitation_learning/     # GROOT model integration
└── README.md
```

---

## 🔧 Core Components

### 1. **Voice Processing Pipeline**
- **Input**: Microphone audio through web interface
- **Processing**: OpenAI Whisper converts speech to text
- **Output**: Structured drawing commands

### 2. **AI Image Generation**
- **Model**: GPT-4.1 with optimized prompts for line art
- **Style**: High-contrast, minimalistic drawings suitable for robots
- **Format**: Black and white vector-style illustrations

### 3. **Trajectory Generation**
- **Edge Detection**: Canny algorithm finds drawable contours
- **Path Optimization**: Douglas-Peucker simplification
- **Coordinate Mapping**: 2D image coordinates → 3D robot workspace
- **Output Formats**: JSON (API integration) + TXT (direct commands)

### 4. **Robot Control**
- **Hardware**: SO-100 6-DOF robot arm
- **Kinematics**: PyRoki IK solver for precise positioning
- **Execution**: Smooth trajectory following with pen control
- **Workspace**: 200mm × 200mm drawing area

### 5. **Imitation Learning (GROOT)**
- **Purpose**: Advanced manipulation tasks (erasing, tool handling)
- **Model**: NVIDIA GROOT N1.5 vision-language-action model
- **Training**: Custom dataset collected via phosphobot software
- **Capabilities**: Context-aware board erasing and drawing enhancement

---

## 🎮 Function Calling Interface

The system provides intelligent function calling for seamless interaction:

| Function | Purpose | Implementation |
|----------|---------|----------------|
| `ask_question_from_user()` | Interactive dialogue | Voice recording + STT |
| `respond_to_user()` | Contextual responses | GPT-4.1 + TTS synthesis |
| `draw_object()` | Execute drawing | Image generation → trajectory → robot |
| `erase_the_board()` | Clean workspace | GROOT imitation learning |

---

## 📊 Technical Specifications

### Drawing Precision
- **Workspace**: 200mm × 200mm active area
- **Speed**: 25mm/s drawing, 80mm/s travel
- **Coordinate System**: Bottom-left origin, millimeter units

### AI Models
- **Speech Recognition**: OpenAI Whisper-1
- **Image Generation**: GPT-4.1-mini (optimized for speed)
- **Text-to-Speech**: OpenAI TTS-1 with German accent support
- **Imitation Learning**: NVIDIA GROOT N1.5

### Output Formats
```json
// JSON Format (API Integration)
{
  "paths": [
    {
      "points": [{"x": 99.2, "y": 189.3}, {"x": 96.7, "y": 188.1}],
      "pen_down": true
    }
  ]
}
```

```
# TXT Format (Direct Robot Commands)
MOVE 99.2 189.3 0    # Position without drawing
PEN_DOWN             # Lower pen
DRAW 96.7 188.1 0    # Draw to point
PEN_UP               # Lift pen
```

---

## 🚧 Development Status

- ✅ **Voice Interface**: Complete web-based recording and playback
- ✅ **Image Generation**: Optimized prompts for drawable content  
- ✅ **Trajectory Planning**: Full 2D→3D coordinate conversion
- ✅ **Robot Control**: Kinematics and smooth motion execution
- 🔄 **GROOT Integration**: Training data collection in progress
- 🔄 **Multi-language Support**: German accent TTS implemented

---
