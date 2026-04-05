# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**Smart Bank AI Service Robot** (智慧銀行 VIP 迎賓與安全通報系統) - A comprehensive AI system integrating face recognition, speech processing, and HMI on a Raspberry Pi 4 + TurtleBot3 platform.

**Key Constraint**: Raspberry Pi 4 with 8GB RAM, 32GB SD card. All models are INT8 quantized, inference latency target is <500ms @ 10 FPS.

**Phase Status**:
- ✅ Phase 1 (environment setup) completed
- ✅ Phase 2a (real-time face detection) **COMPLETED** - 12-13 FPS with InsightFace buffalo_sc, 512D embeddings generated
- 🔄 Phase 2b (face recognition + liveness checking) **IN PROGRESS** - Next: VIP database matching + blink/headshake liveness detection
- ⏳ Phase 3+ (emotion analysis, speech, HMI, backend)

**Recent Achievement**: Successfully deployed InsightFace-based face detection at 12-13 FPS on RPi4 with GUI support. See `FEATURES.md` for complete feature list and `INSTALLATION_GUIDE.md` for deployment instructions.

---

## Architecture Overview

### 5 Core Modules

1. **Vision AI** (`vision_ai/`) - Face detection, liveness checking, face recognition, emotion analysis
2. **Speech & HMI** (`speech_hmi/`) - Whisper STT, pyttsx3 TTS, intent classification, 7" touchscreen UI
3. **Navigation** (`robot_navigation/`) - SLAM, Nav2, bottom layer control (teammate responsibility)
4. **Communication** (`communication/`) - LoRa alerts, MQTT sync, UART to STM32
5. **Core Workflow** (`core_workflow/`) - Main business logic, person type classification, alerts

### Design Patterns

- **Edge-first**: All face recognition inference runs locally on Raspberry Pi (privacy + latency)
- **Configuration-driven**: `config/inference_config.yaml` controls platform (RPi4 vs Jetson Orin), precision (INT8 vs FP16), model paths
- **Queue-based multi-threading**: Separate threads for face detection, recognition, decision logic, I/O (prevents bottlenecks)
- **Graceful degradation**: Core packages always install; optional Whisper may fail without blocking setup; Whisper can be installed later

### Model Specifications

- **Face Detection**: RetinaFace INT8 (<10MB, ~200ms latency)
- **Face Recognition**: ArcFace INT8 (128D embedding, <20MB, ~300ms latency)
- **Emotion Analysis**: EfficientNet-B0 (<15MB)
- **Speech Model**: Whisper (quantized, edge version)

### Database

- **Local**: SQLite (edge storage, fast queries)
  - `vip_members`: VIP list + face embeddings
  - `blacklist`: Blacklisted persons
  - `visit_logs`: Visitor history
  - `security_alerts`: Safety incidents
- **Cloud**: PostgreSQL (via MQTT sync)
- **Sync Strategy**: Hourly auto-sync; immediate sync for security alerts

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

### Files Reference for Deployment

- **requirements_minimal.txt** - Curated list of only essential packages (no types-* packages)
- **setup_rpi4_quick.sh** - Fully automated 7-step deployment script
- **INSTALLATION_GUIDE.md** - Detailed step-by-step guide with troubleshooting
- **FEATURES.md** - Complete feature list and development roadmap

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
python3 scripts/realtime_detection_insightface.py

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
python3 -c "from database.schema import DatabaseSchema; DatabaseSchema().initialize(); print('✓ Database ready')"

# Test database operations
python3 -c "
from database.schema import DatabaseSchema
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

- **vibe_coding流程.md**: Complete Chinese workflow guide
- **FEATURES.md**: Feature list and roadmap (English)
- **INSTALLATION_GUIDE.md**: Deployment guide (English)
- **CLAUDE.md** (this file): Architecture for Claude Code




1. Download quantized models: RetinaFace INT8, ArcFace INT8
2. Implement `vision_ai/face_detector.py` with ONNX Runtime inference
3. Implement `vision_ai/liveness_checker.py` using MediaPipe Face Landmarks
4. Implement `vision_ai/face_recognizer.py` with embedding vector comparison
5. Test on Raspberry Pi: benchmark latency, accuracy, memory usage
6. Initialize SQLite schema for VIP/blacklist database

### Phase 3 (Speech & HMI)
1. Test Whisper on edge (if installed in setup)
2. Implement `speech_hmi/speech_recognizer.py` for real-time STT
3. Implement `speech_hmi/intent_classifier.py` for semantic understanding
4. Implement `speech_hmi/speech_synthesizer.py` using pyttsx3 for TTS
5. Develop 7" touchscreen UI (Electron + React or Qt)

### Integration Points
- **ROS 2 / Navigation**: Handled by teammate. Coordinate on person detection → robot following logic via shared queue/topics
- **Backend Sync**: MQTT integration for cloud PostgreSQL sync (Phase 5)
- **LoRa Alerts**: Alert manager publishes VIP/security events to LoRa module (Phase 4)

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
| **FEATURES.md** | Complete feature list, development roadmap (Phase 1-6), performance metrics |
| **INSTALLATION_GUIDE.md** | Step-by-step deployment guide, troubleshooting, quick migration |
| **requirements_minimal.txt** | Minimal pip dependencies (20 packages, ~1.5GB), optimized for RPi4 |
| **setup_rpi4_quick.sh** | One-command environment deployment (all 7 steps automated) |
| `setup_rpi4_dev.sh` | Original comprehensive setup script (handles disk cleanup, optional Whisper) |
| `vibe_coding流程.md` | Complete development workflow with Claude Code (read this for dev process!) |
| `專題計畫.md` | Complete project specification (8 phases, team roles, risk assessment) |
| `config/inference_config.yaml` | Inference engine configuration (platform, model paths, thresholds, precision) |
| `database/schema.py` | SQLite table definitions (vip_members, blacklist, visit_logs, security_alerts, robot_status_log) |
| `vision_ai/inference_engine.py` | ONNX Runtime abstraction layer for multi-platform support (RPi4 vs Jetson Orin) |
| **scripts/realtime_detection_insightface.py** | ✅ **WORKING** - Real-time face detection (InsightFace buffalo_sc, 12-13 FPS, GUI support) |
| `scripts/benchmark.py` | Full performance testing (inference latency, CPU, memory, temperature, database) |
| `scripts/model_guide.py` | Guide for obtaining/installing face detection models (interactive)

---

## Face Detection Implementation - CURRENT STATUS

### ✅ InsightFace (ACTIVE - Recommended for Phase 2+)

**Status**: **WORKING and OPTIMIZED** - Achieved 12-13 FPS on RPi4 with buffalo_sc model

**Script**: `scripts/realtime_detection_insightface.py`

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
from vision_ai.face_detector import FaceDetector
detector = FaceDetector("models/retinaface_int8.onnx")
detections = detector.detect(frame)

# Test inference engine abstraction
from vision_ai.inference_engine import InferenceEngine
engine = InferenceEngine(config="config/inference_config.yaml")
```

---

## Related Resources

- **Project Plan**: `專題計畫.md` - Full 8-phase roadmap, hardware specs, risk assessment
- **Official Docs**:
  - [ONNX Runtime](https://onnxruntime.ai/)
  - [OpenCV Python](https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html)
  - [OpenAI Whisper](https://github.com/openai/whisper)
  - [pyttsx3](https://pyttsx3.readthedocs.io/)
- **Raspberry Pi**:
  - [Raspberry Pi OS Setup](https://www.raspberrypi.com/software/)
  - [ROS 2 on Raspberry Pi](https://docs.ros.org/en/foxy/Guides/Linux-Install-Debians.html)
