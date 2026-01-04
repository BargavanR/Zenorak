#!/usr/bin/env bash
# verify_launch.sh
# Quick check script: verifies that the main controller topics are present.

set -euo pipefail

# If workspace install/setup.bash exists in repository root, source it to ensure ros2 is available
if [ -f "install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "install/setup.bash"
fi

required_topics=(
  "/arm_trajectory_controller/joint_trajectory"
  "/diff_cont/cmd_vel_unstamped"
  "/joint_states"
)

# Get the topic list (plain names)
mapfile -t topics < <(ros2 topic list)

missing=()
for req in "${required_topics[@]}"; do
  found=0
  for t in "${topics[@]}"; do
    if [[ "$t" == "$req" ]]; then
      found=1
      break
    fi
  done
  if [[ $found -eq 0 ]]; then
    missing+=("$req")
  fi
done

if [ ${#missing[@]} -ne 0 ]; then
  echo "Missing required topics:" >&2
  for m in "${missing[@]}"; do
    echo "  - $m" >&2
  done
  echo "" >&2
  echo "Run the launch and check logs or spawn controllers manually." >&2
  exit 2
fi

# Optionally show types
echo "All required topics present. Types:" 
ros2 topic list -t | grep -F -e "/arm_trajectory_controller/joint_trajectory" -e "/diff_cont/cmd_vel_unstamped" -e "/joint_states" || true

echo "verify_launch: OK"
