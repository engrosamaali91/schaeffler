#!/bin/bash
# run_all.sh — Record ROS 2 bag while TurtleBot launch is already running

set -e  # Exit immediately on error

# Find next iteration number
ITER=1
while [ -d "isaac_run_itr_${ITER}" ]; do
    ITER=$((ITER + 1))
done

# Set bag name
BAG_NAME="isaac_run_itr_${ITER}"

echo "------------------------------------------"
echo "📦 Starting ROS 2 bag recording..."
echo " Bag file: $BAG_NAME"
echo "------------------------------------------"

# Start recording only /odom topic in the background
ros2 bag record /odom -o "$BAG_NAME" &
BAG_PID=$!

# Handle Ctrl+C (SIGINT) cleanly
trap "echo -e '\n🛑 Stopping bag recording...'; kill $BAG_PID; wait $BAG_PID 2>/dev/null; echo '✅ Bag saved as $BAG_NAME'; exit 0" SIGINT

# Keep script alive until user stops recording
echo "🎥 Recording /odom... Press Ctrl+C to stop."
wait $BAG_PID
