#!/bin/bash
set -e  # Exit immediately on error

# Find next iteration number
ITER=1
while [ -d "isaac_run_itr_${ITER}" ]; do
    ITER=$((ITER + 1))
done

# Set names for files
BAG_NAME="isaac_run_itr_${ITER}"
CSV_RAW="sim_odom_raw_${ITER}.csv"


echo "------------------------------------------"
echo "📦 Starting ROS 2 bag recording..."
echo " Bag file: $BAG_NAME"
echo "------------------------------------------"

# Function to process data after recording
process_data() {
    echo -e "\n🛑 Stopping bag recording..."
    kill $BAG_PID
    wait $BAG_PID 2>/dev/null
    echo "✅ Bag saved as $BAG_NAME"

    # Convert bag to CSV using the Python script
    echo "📊 Converting bag to CSV..."
    python3 ./extract_rosbagdata.py --bag "$BAG_NAME" --output "$CSV_RAW"

    echo "✨ completed!"
    echo "Raw CSV file: $CSV_RAW"
    exit 0
}

# Set up trap for Ctrl+C
trap process_data SIGINT

# Start recording /odom topic   
ros2 bag record /odom -o "$BAG_NAME" &
BAG_PID=$!

echo "🎥 Recording /odom... Press Ctrl+C to stop."
wait $BAG_PID
