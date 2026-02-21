#!/bin/bash
# ==============================================================================
# Swarm Commander — One-Click Setup
# ==============================================================================

set -e

echo "================================================="
echo "  🚁 Swarm Commander Setup"
echo "================================================="

# Check Python
if ! command -v python3 &> /dev/null; then
    echo "ERROR: Python 3 not found. Install Python 3.10+"
    exit 1
fi

PYVER=$(python3 -c "import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')")
echo "✓ Python $PYVER detected"

# Check ArduPilot
ARDUCOPTER_BIN="$HOME/ardupilot/build/sitl/bin/arducopter"
if [ -f "$ARDUCOPTER_BIN" ]; then
    echo "✓ ArduCopter SITL binary found"
else
    echo "⚠ ArduCopter not found at $ARDUCOPTER_BIN"
    echo "  Install ArduPilot: https://ardupilot.org/dev/docs/building-setup-linux.html"
    echo "  Then build: cd ~/ardupilot && ./waf configure --board sitl && ./waf copter"
    echo ""
    echo "  Continuing anyway (pure simulations will still work)..."
fi

# Install dependencies
echo ""
echo "Installing Python dependencies..."
pip3 install -r requirements.txt --quiet 2>/dev/null || pip install -r requirements.txt --quiet

# Verify
echo ""
echo "Verifying imports..."
python3 -c "import pygame; print(f'  ✓ Pygame {pygame.ver}')"
python3 -c "import numpy; print(f'  ✓ NumPy {numpy.__version__}')"
python3 -c "
import collections
if not hasattr(collections, 'MutableMapping'):
    import collections.abc
    collections.MutableMapping = collections.abc.MutableMapping
import dronekit; print(f'  ✓ DroneKit {dronekit.__version__}')
" 2>/dev/null || echo "  ⚠ DroneKit not available (SITL features disabled)"

echo ""
echo "================================================="
echo "  ✓ Setup Complete!"
echo "================================================="
echo ""
echo "  Quick Start:"
echo "    python3 -u src/interactive_commander.py   # Full SITL demo"
echo "    python3 src/sim/visual_swarm.py            # Pure simulation"
echo ""
