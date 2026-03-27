#!/bin/bash

# Start a virtual display
echo "Starting Xvfb virtual display..."
Xvfb :1 -screen 0 1280x800x16 &
export DISPLAY=:1
sleep 1

# Start a lightweight window manager
echo "Starting Fluxbox window manager..."
fluxbox &
sleep 1

# Start VNC server
echo "Starting x11vnc server..."
x11vnc -display :1 -nopw -forever -shared &
sleep 1

# Start noVNC
echo "Starting noVNC on port 6080..."
websockify --web=/usr/share/novnc/ 6080 localhost:5900 &
sleep 1

echo ""
echo "=============================================="
echo " GUI environment is running!"
echo " Open this URL in your browser:"
echo "   https://<your-codespace>-6080.preview.app.github.dev/vnc.html"
echo "=============================================="
echo ""
echo "sick"
sleep 2
gh codespace ports visibility 6080:public
sleep 1
robotpy sim