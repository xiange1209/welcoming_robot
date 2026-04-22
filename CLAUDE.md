# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**Smart Bank AI Service Robot** (智慧銀行 VIP 迎賓與安全通報系統) - A comprehensive AI system integrating face recognition, speech processing, and HMI on a Raspberry Pi 4 + TurtleBot3 platform.

**Key Constraint**: Raspberry Pi 4 with 8GB RAM, 32GB SD card. All models are INT8 quantized, inference latency target is <500ms @ 10 FPS.

**Phase Status**:
- ✅ Phase 1 (environment setup) completed
- ✅ Phase 2a (real-time face detection) **VERIFIED** - 12-13 FPS with InsightFace buffalo_sc on RPi4, 512D embeddings generated
- 🔄 Phase 2b (face recognition + liveness detection) **CODE READY** - Awaiting hardware testing. Single-sample VIP/blacklist matching with gender field support
- 🔄 Phase 3+ (speech, HMI, ROS2 integration) planning

**Recent Update**: Phase 2b codebase enhanced with gender field (M/F/Other) for VIP/blacklist records. Gender parameter now required during registration but NOT auto-detected. Ready for testing on Raspberry Pi 4.

---

## Architecture Overview

### Current Module Structure (Python Single-Process → ROS 2 Migration)

**Phase 2b** (Current): Single-process Python execution on Raspberry Pi OS.

**Phase 3+** (In Progress): Multi-process ROS 2 architecture with unified src/ directory:

```
專題/
├── src/
│   ├── ai_vision/           # Face detection, recognition, liveness (陳佳憲)
│   ├── database/            # SQLite schema for VIP/blacklist management
│   ├── scripts/             # Testing, benchmarking, database management
│   └── smartnav_ros2/       # ROS 2 multi-package system (孫瑋廷)
│       ├── smartnav_msgs/    # Message definitions
│       ├── smartnav_vision/  # Vision node (InsightFace + face recognition)
│       ├── smartnav_audio/   # Speech recognition/synthesis (Phase 3)
│       ├── smartnav_brain/   # Decision logic orchestrator
│       └── smartnav_bringup/ # Launch configurations
├── docs/                    # Documentation & planning
├── config/                  # System configuration
├── README.md
├── requirements_minimal.txt # Curated dependencies
└── setup_rpi4_quick.sh     # One-command deployment
```

### Design Principles

- **Edge-first**: All face recognition inference runs locally on Raspberry Pi (privacy + latency)
- **Configuration-driven**: `config/inference_config.yaml` controls platform (RPi4 vs Jetson Orin), precision (INT8 vs FP16), model paths
- **Queue-based multi-threading**: Separate threads for face detection, recognition, decision logic, I/O (prevents bottlenecks)
- **ROS 2 Ready**: Single-process code can be deployed as-is now; ROS 2 packages in `src/smartnav_ros2/` provide alternative multi-process architecture
- **Graceful degradation**: Core packages always install; optional Whisper may fail without blocking setup; Whisper can be installed later


### Model Specifications

- **Face Detection**: InsightFace RetinaFace (buffalo_sc model, ~50MB, produces 512D embeddings)
- **Face Recognition**: Embedding-based similarity matching (cosine distance, no additional model needed)
- **Liveness Detection**: MediaPipe Face Landmarks (when available on RPi4)
- **Emotion Analysis**: Not yet implemented (Phase 3+)

### Database

- **Local**: SQLite (edge storage, fast queries)
  - `vip_members`: VIP list + face embeddings + **gender field** (M/F/Other)
  - `blacklist`: Blacklisted persons + **gender field** (M/F/Other)
  - `visit_logs`: Visitor history
  - `security_alerts`: Safety incidents
- **Cloud**: PostgreSQL (via MQTT sync) - Phase 5+
- **Sync Strategy**: Hourly auto-sync; immediate sync for security alerts

---

## Phase 2b: Single-Sample Face Recognition (Current)

### Recognition Flow

```
User Registration (Backend):
  Photo + Name + Gender (M/F/Other) → Face embedding extraction → SQLite storage

Runtime Detection (Real-time on RPi4):
  Camera frame → Face detection → Extract face embedding
                              ↓
                    Check distance to VIP embeddings
                              ↓
  distance < threshold → Match VIP → Display: VIP_<name>_<gender> (confidence)
  distance < threshold (blacklist) → Alert: 黑名單_<name>_<gender>
  distance > threshold → Display: 訪客 (confidence)
```

### Key Implementation Details

**Gender Field (Manual Input, Not Auto-Detected)**:
- Format: 'M' (男), 'F' (女), 'Other'
- Entered during backend VIP/blacklist registration
- Stored in database alongside 512D face embedding
- Displayed in GUI label for human verification

**VIP Matching Algorithm**:
- Similarity: `1.0 - (euclidean_distance / max_distance)` → 0-1 score
- VIP threshold: 0.65 (tunable via `face_recognizer.py`)
- Blacklist threshold: 0.60 (check blacklist first for security)
- Search order: Blacklist (safety-first) → VIP → Else visitor

**GUI Label Format**:
```
黃色框 (VIP):     VIP_<name>_<gender> (<confidence>)
紅色框 (黑名單):   黑名單_<name>_<gender> (<confidence>)
綠色框 (訪客):     訪客 (<confidence>)
```

### VIP Management CLI

```bash
# Initialize database
python3 src/scripts/manage_vip_database.py init

# Add VIP from photo (interactive)
python3 src/scripts/manage_vip_database.py add-vip-interactive

# Add VIP from image (CLI)
python3 src/scripts/manage_vip_database.py add-vip \
  --name "陳佳憲" --gender M --level platinum \
  --phone "0900123456" --email "user@bank.com" \
  path/to/photo.jpg

# Add blacklist
python3 src/scripts/manage_vip_database.py add-blacklist \
  --name "李三" --gender F --risk high \
  --reason "Failed background check" path/to/photo.jpg

# List registered VIPs
python3 src/scripts/manage_vip_database.py list-vips

# List blacklisted persons
python3 src/scripts/manage_vip_database.py list-blacklist
```

### Testing Phase 2b

```bash
# Test VIP recognition on current frame
python3 src/scripts/test_vip_recognition.py path/to/test/photo.jpg

# Run real-time detection with VIP overlay
export DISPLAY=:1
python3 src/scripts/realtime_detection_insightface.py
```

---

## Quick Development Setup

### For Rapid Prototyping (Recommended)

Use the minimal requirements file for faster installation:

```bash
# Install minimal dependencies (20 packages, ~1.5GB)
pip install --no-cache-dir -r requirements_minimal.txt

# Takes ~5-10 minutes vs ~30 minutes for full setup
```

### For Complete Environment

Use the automated setup script:

```bash
# One-command 7-step deployment
bash setup_rpi4_quick.sh

# Handles: system packages, venv, GTK+ libs, disk cleanup, pip packages
# Takes ~20-30 minutes total
```

### Files Reference for Development

- **requirements_minimal.txt** - Curated list of only essential packages for Phase 2b
- **setup_rpi4_quick.sh** - Fully automated 7-step deployment script
- **vibe_coding流程.md** - Complete development workflow guide (Chinese)

---

## Common Development Commands

### Running Face Detection

```bash
# Activate environment
source ~/ai_bank_robot_env/bin/activate
cd ~/ai_bank_robot

# Set display for VNC/HDMI
export DISPLAY=:1

# Run InsightFace detection (working, 12-13 FPS)
python3 src/scripts/realtime_detection_insightface.py

# Controls
# Q - Quit
# S - Save current frame to /tmp/
# Space - Pause/Resume
```

### Testing and Verification

```bash
# Test InsightFace import
python3 -c "from insightface.app import FaceAnalysis; print('✓ InsightFace ready')"

# Test all core packages
python3 << 'EOF'
import cv2, numpy, insightface, onnxruntime, picamera2
print(f'✓ OpenCV {cv2.__version__}')
print('✓ All packages installed')
EOF

# Performance monitoring (during detection run)
watch -n 1 'free -h; echo "---"; vcgencmd measure_temp'

# Check disk space (need >5GB for installation)
df -h
```

### Database Testing

```bash
# Initialize SQLite schema
python3 -c "from src.database.schema import DatabaseSchema; DatabaseSchema().initialize(); print('✓ Database ready')"

# Test database operations
python3 -c "
from src.database.schema import DatabaseSchema
schema = DatabaseSchema()
conn = schema.connect()
schema.log_visit(conn, 'test_visitor', 'visitor_1', confidence=0.95)
conn.close()
print('✓ Database operations working')
"
```

---

## Raspberry Pi Specific Notes

### Storage Constraints
- **Total**: 32GB SD card
- **Used by OS+System**: ~7.9GB
- **Available for dev**: ~22GB (after setup_rpi4_dev.sh cleanup)
- **Core packages size**: ~3-4GB (numpy, OpenCV, ONNX, FastAPI, etc.)
- **Models size**: ~100MB (RetinaFace + ArcFace INT8 combined)
- **Whisper optional**: ~1GB (installs if disk space available after core packages)

### Performance Targets
- **Face detection + recognition**: <500ms total @ 10 FPS
- **Memory peak during inference**: <2GB (with INT8 quantized models)
- **CPU usage**: ~60-80% during active processing (4 cores, ARM architecture)

### Known Issues & Workarounds

1. **OpenCV GUI Error**: `The function is not implemented. Rebuild with GTK+ support`
   - **Cause**: `opencv-python-headless` was installed instead of full `opencv-python`
   - **Fix**:
   ```bash
   pip uninstall opencv-python-headless -y
   pip install --no-cache-dir opencv-python
   ```

2. **InsightFace Model Download Fails**: HTTP 404 errors
   - **Cause**: GitHub release URLs are outdated
   - **Fix**: Models auto-download from HuggingFace on first run (more reliable)

3. **Picamera2 Not Found**: `ModuleNotFoundError: No module named 'picamera2'`
   - **Cause**: Virtual environment created without `--system-site-packages`
   - **Fix**:
   ```bash
   python3 -m venv --system-site-packages ~/ai_bank_robot_env
   source ~/ai_bank_robot_env/bin/activate
   # Must install libcap-dev first: sudo apt-get install -y libcap-dev
   ```

4. **Low FPS** (<5 FPS)
   - **Cause**: CPU overload or thermal throttling
   - **Check**:
   ```bash
   vcgencmd measure_temp  # Should be <70°C
   top -b -n 1 | head -15  # Check CPU usage
   ```
   - **Fix**: Increase `detect_interval` in script (detect every 4-5 frames instead of 3)

5. **Disk Space Issues**: `[Errno 28] No space left on device`
   - **Fix**:
   ```bash
   sudo apt-get clean
   rm -rf ~/.cache/pip/*
   sudo journalctl --vacuum-size=50M
   df -h  # Verify >5GB available
   ```

---

---

---

## Development Workflow with Claude Code

### Quick Overview

For detailed step-by-step guide, see **`vibe_coding流程.md`** (in Chinese)

```
Windows (Code changes)
         ↓ (SCP transfer)
Raspberry Pi (Test & validate)
         ↓ (Report results)
Claude (Analyze & improve)
         ↓ (Loop back)
```

### Typical Development Cycle

1. **You describe a problem** (in Windows or feedback from RPi)
   - Specific error messages
   - Performance metrics (CPU, memory, latency)
   - Test results and observations

2. **Claude reads code and designs solution**
   - Analyzes CLAUDE.md and architecture
   - Reads relevant files
   - Generates/modifies code in Windows directory

3. **You transfer updated code to Raspberry Pi**
   ```bash
   cd C:\Users\陳佳憲\Desktop\專題
   scp -r . xiange@192.168.1.125:~/ai_bank_robot/
   ```

4. **You test on Raspberry Pi**
   ```bash
   ssh xiange@192.168.1.125
   cd ~/ai_bank_robot && source ~/ai_bank_robot_env/bin/activate
   python3 scripts/realtime_detection_insightface.py
   ```

5. **You report results back to Claude**
   - Copy-paste test output
   - Mention performance improvements or issues
   - Request next iteration

### Key Files for Development

- **docs/vibe_coding流程.md**: Complete Chinese workflow guide
- **docs/計畫書_2026-04-22.md**: Current project plan with technical challenges
- **docs/專題計畫.md**: Complete 8-phase specification
- **CLAUDE.md** (this file): Architecture for Claude Code

---

## Development Phases & Roadmap

### Current Status (Phase 2b)
- ✅ **Phase 1**: Environment setup (Raspberry Pi OS, dependencies)
- ✅ **Phase 2a**: Real-time face detection (InsightFace, 12-13 FPS verified)
- 🔄 **Phase 2b**: Face recognition + gender field (code complete, awaiting RPi4 verification)
  - Face embedding similarity matching (512D vectors)
  - Single-sample VIP/blacklist matching
  - Gender field support (M/F/Other)
- 🔄 **Phase 3**: ROS 2 integration (smartnav_ros2 packages prepared)
  - Multi-process architecture in `src/smartnav_ros2/`
  - Vision node with face recognition
  - Audio node (Phase 3+)
  - Brain/orchestrator node (Phase 3+)
- ⏳ **Phase 4+**: LoRa alerts, cloud sync, HMI UI

### Critical Testing Needed Before RPi4 Deployment

1. **VIP Recognition Verification**: Can a single employee photo reliably match 40+ times?
   - Test with same person, different poses/lighting
   - Verify gender field displays correctly
   - Measure confidence scores

2. **Cross-device Recognition**: Recognition model trained on iPhone camera
   - Must work on RPi camera module, USB cameras, IP cameras
   - May need feature normalization (ISP differences)

3. **Single-Sample Learning**: Improve from 40% to 75%+ recognition rate
   - Test with multi-scale features
   - Consider feature augmentation (rotation, scaling)
   - Dynamic threshold tuning

---

---

## Configuration System

**File**: `config/inference_config.yaml`

```yaml
platform: "rpi4"              # or "jetson_orin" (easy platform switch)
model_precision: "int8"       # or "fp16" for Jetson
batch_size: 1
num_threads: 4
use_gpu: false                # true only for Jetson
model_paths:
  face_detector: "models/retinaface_int8.onnx"
  face_recognizer: "models/arcface_int8.onnx"
inference_params:
  face_detect_threshold: 0.7
  face_recognition_threshold: 0.6
  frame_rate: 10              # FPS for vision processing
  max_queue_size: 30          # Queue depth for frame buffer
```

**Key Design**: All inference engine logic (`InferenceEngine` class) reads this config to select model backends (ONNX vs TensorFlow Lite), optimize tensor operations, and manage memory. Changing platform requires only config modification, not code changes.

---

## Key Files Reference

| File | Purpose |
|------|---------|
| **docs/計畫書_2026-04-22.md** | Current project plan with phase progress and technical challenges |
| **docs/vibe_coding流程.md** | Complete development workflow with Claude Code (read this for dev process!) |
| **docs/專題計畫.md** | Complete project specification (8 phases, hardware specs, risk assessment) |
| **docs/進度統整報告_2026-04-22.md** | Comprehensive progress report (Phase 1-3 status, team effort tracking) |
| **config/inference_config.yaml** | Inference engine configuration (platform, model paths, thresholds, precision) |
| **src/database/schema.py** | SQLite table definitions (vip_members, blacklist, visit_logs, security_alerts, robot_status_log) |
| **src/ai_vision/inference_engine.py** | ONNX Runtime abstraction layer for multi-platform support (RPi4 vs Jetson Orin) |
| **src/ai_vision/face_recognizer.py** | VIP/blacklist matching with gender field support (512D embedding similarity) |
| **src/scripts/realtime_detection_insightface.py** | ✅ **WORKING** - Real-time face detection (InsightFace buffalo_sc, 12-13 FPS, GUI support) |
| **src/scripts/manage_vip_database.py** | Interactive CLI for VIP registration, blacklist management, database initialization |
| **src/scripts/benchmark.py** | Full performance testing (inference latency, CPU, memory, temperature, database) |
| **src/smartnav_ros2/smartnav_vision/** | ROS 2 vision node with InsightFace integration (孫瑋廷, Phase 3) |
| **requirements_minimal.txt** | Minimal pip dependencies (20 packages, ~1.5GB), optimized for RPi4 |
| **setup_rpi4_quick.sh** | One-command environment deployment (all 7 steps automated) |

---

## Face Detection Implementation - CURRENT STATUS

### ✅ InsightFace (ACTIVE - Recommended for Phase 2+)

**Status**: **WORKING and OPTIMIZED** - Achieved 12-13 FPS on RPi4 with buffalo_sc model

**Script**: `src/scripts/realtime_detection_insightface.py`

```bash
# Install InsightFace
pip install --no-cache-dir insightface

# Run real-time detection
export DISPLAY=:1
python3 scripts/realtime_detection_insightface.py
```

**Performance (RPi4)**:
- **FPS**: 12-13 (exceeds 10 FPS target)
- **Inference latency**: 100-150ms per frame
- **Memory usage**: ~350-400MB
- **CPU usage**: ~60-70%
- **Model size**: ~50MB (buffalo_sc, lightweight version)

**Advantages**:
- ✅ Excellent accuracy (RetinaFace ONNX detection)
- ✅ Side-face detection support
- ✅ Generates face embeddings (512D) for VIP recognition
- ✅ GUI support with OpenCV
- ✅ Automatic model download on first run
- ✅ Frame skipping optimization (every 3 frames)

**Usage**:
```python
from insightface.app import FaceAnalysis

face_app = FaceAnalysis(name='buffalo_sc', providers=['CPUExecutionProvider'])
faces = face_app.get(frame)  # Returns detected faces with embeddings
for face in faces:
    bbox = face.bbox  # [x1, y1, x2, y2]
    confidence = face.det_score
    embedding = face.embedding  # 512D vector for recognition
```

---

### Alternative Option A: MediaPipe (Fast but less accurate)

**Status**: Ready to use, no model download needed

```bash
pip install --no-cache-dir mediapipe -q
```

**Disadvantages for Phase 2+**:
- No embedding generation (can't do VIP recognition without additional model)
- Lower accuracy than InsightFace
- Limited side-face detection

---

### Alternative Option B: ONNX Models (Advanced, WIP)

**Status**: URLs outdated (404 errors), requires manual setup

Not recommended unless InsightFace performance becomes insufficient.



1. **Memory Management**: With 8GB RAM and 4 ARM cores, avoid large batch processing. Single-image inference is the norm (batch_size: 1).

2. **Disk Space**: Always run cleanup before major installations. Monitor with `df -h`. Leave >5GB free.

3. **Model Quantization**: INT8 precision is mandatory for RPi4 storage/inference speed. Jetson Orin can support FP16/FP32 via config change.

4. **Frame Rate Coupling**: Vision processing at 10 FPS, navigation at 20 Hz, HMI at 10 FPS. Carefully manage threads to avoid frame drops.

5. **Network Reliability**: Design for intermittent connectivity (LoRa/MQTT may fail). MQTT uses local SQLite queue for offline buffering; sync on reconnect.

6. **Privacy-First**: Never upload raw face images to cloud. Only upload face embedding vectors (128D), person ID, metadata, alerts.

7. **Error Handling**: Graceful degradation on sensor failures (e.g., if face detection fails, show "Please move closer" rather than crashing).

---

## Debugging Tips

**SSHing to Raspberry Pi for development**:
```bash
ssh xiange@192.168.1.125
source ~/ai_bank_robot_env/bin/activate
cd ~/ai_bank_robot
python3 -c "import sys; print(sys.path)"  # Verify venv is active
```

**Monitor resource usage**:
```bash
# Real-time memory + CPU
top -b -n 1 | head -15

# Disk usage
du -sh ~/* | sort -h

# Temperature
vcgencmd measure_temp

# GPU memory (if applicable)
free -h
```

**Test individual components**:
```python
# Test face detection alone
from src.ai_vision.face_detector import FaceDetector
detector = FaceDetector("models/retinaface_int8.onnx")
detections = detector.detect(frame)

# Test inference engine abstraction
from src.ai_vision.inference_engine import InferenceEngine
engine = InferenceEngine(config="config/inference_config.yaml")
```

---

## Related Resources

- **Project Plan**: `docs/專題計畫.md` - Full 8-phase roadmap, hardware specs, risk assessment
- **Development Workflow**: `docs/vibe_coding流程.md` - Windows-RPi4 development cycle
- **Progress Reports**: `docs/計畫書_2026-04-22.md` and `docs/進度統整報告_2026-04-22.md`
- **Official Docs**:
  - [ONNX Runtime](https://onnxruntime.ai/)
  - [OpenCV Python](https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html)
  - [OpenAI Whisper](https://github.com/openai/whisper)
  - [pyttsx3](https://pyttsx3.readthedocs.io/)
  - [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/)
- **Raspberry Pi**:
  - [Raspberry Pi OS Setup](https://www.raspberrypi.com/software/)
  - [ROS 2 on Raspberry Pi](https://docs.ros.org/en/foxy/Guides/Linux-Install-Debians.html)
