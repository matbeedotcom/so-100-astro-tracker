#!/bin/bash

# Star Tracker Stop Script
# Cleanly stops all star tracker processes

echo "Stopping Star Tracker System..."

# Kill all tracked processes
if [ -f /tmp/star_tracker_pids.txt ]; then
    echo "Stopping tracked processes..."
    while read -r pid; do
        if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 $pid 2>/dev/null; then
            echo "Stopping process $pid..."
            kill $pid
        fi
    done < /tmp/star_tracker_pids.txt

    # Wait a moment then force kill if needed
    sleep 2
    while read -r pid; do
        if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 $pid 2>/dev/null; then
            echo "Force stopping process $pid..."
            kill -9 $pid
        fi
    done < /tmp/star_tracker_pids.txt

    rm -f /tmp/star_tracker_pids.txt
    echo "✅ All tracked processes stopped"
else
    echo "No PID file found, stopping by process name..."
fi

# Stop any remaining star tracker processes
echo "Cleaning up any remaining processes..."
pkill -f "star_tracker_node" 2>/dev/null
pkill -f "bno055_interface" 2>/dev/null
pkill -f "gps_interface" 2>/dev/null

echo "🛑 Star Tracker System stopped"