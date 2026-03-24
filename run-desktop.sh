#!/usr/bin/env bash
# Run the desktop GUI with native JNI libraries from build/jni/release in LD_LIBRARY_PATH.
# Usage: ./run-desktop.sh
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"
JNI_DIR="$ROOT_DIR/build/jni/release"

if [ -d "$JNI_DIR" ]; then
  export LD_LIBRARY_PATH="$JNI_DIR:${LD_LIBRARY_PATH:-}"
  echo "Using JNI dir: $JNI_DIR"
else
  echo "Warning: JNI dir not found: $JNI_DIR (you may need to run a Gradle build to generate it)"
fi

# Build the fat jar
./gradlew jar --no-daemon --console=plain

# Run the fat jar
exec java -cp "$ROOT_DIR/build/libs/GoToBallLine.jar" frc.robot.GoToBallLine "$@"
