#!/bin/zsh

set -e

source /opt/ros/$ROS_DISTRO/setup.zsh

if [ -f "/workspace/install/setup.zsh" ]; then
    source /workspace/install/setup.zsh
    echo "✅ Workspace sourced successfully"
else
    echo "🔧 /workspace/install/setup.zsh not found, building workspace..."
    cd /workspace

    # Try to build and capture the exit code
    if colcon build; then
        echo "✅ Build successful! Sourcing workspace..."
        source /workspace/install/setup.zsh
    else
        # ANSI color codes for visibility
        echo "\033[1;31m" # Bold red
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        echo "❌ BUILD FAILED! ❌"
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        echo ""
        echo "🚨 The workspace failed to build with colcon build"
        echo "🔍 Check the build logs above for errors"
        echo "🛠️  Run 'colcon build' manually to see detailed error messages"
        echo "📁 You can still use the container, but ROS packages won't be available"
        echo ""
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        echo "\033[0m" # Reset color
    fi
fi


exec "$@"
