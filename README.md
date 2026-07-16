# W.A.R.D. — Wide-Area Reactive Defense

**W.A.R.D.** is a modular **airsoft base platform** designed to support interchangeable **M4-style replicas** using a standard **rail mounting system**.  
The project focuses on **mounting, stability, and modularity**, not weapon modification.

## Overview

- Base frame for airsoft applications
- Rail-based interface for swapping M4 platforms
- Designed for testing, training, and hobby use
- Emphasis on mechanical compatibility and safety

## Key Features

- Standard rail mounting (Picatinny-style)
- Quick platform swap capability
- Rigid, modular frame design
- Expandable for sensors, controls, or accessories

## Intended Use

- Airsoft training environments
- Controlled, private settings only
- Non-lethal airsoft replicas

## Safety Notes

- Eye protection required at all times
- Never use in public spaces
- Follow local airsoft laws and field rules
- Platform is a **mounting system**, not a weapon


## Detected RTSP stream

`ip_vision_rtsp.py` reads the H.265 camera stream, runs YOLO detection, draws
the detections, and hosts the annotated result as an H.264 RTSP stream.

Install the GStreamer RTSP server binding once:

```bash
sudo apt install gir1.2-gst-rtsp-server-1.0
```

Start the server:

```bash
python3 ip_vision_rtsp.py
```

The default output is `rtsp://<jetson-ip>:8555/detected`.

Input, output, detection, and encoding settings are configurable. For example:

```bash
python3 ip_vision_rtsp.py \
  --camera-url rtsp://192.168.144.26:8554/main.264 \
  --port 8555 --path detected --fps 15 --bitrate 4000
```

Use `--display` if a local preview window is also wanted.
## Repository Structure (example)

