#!/usr/bin/env bash
# Run WBC with a fixed plate spawn on the island and a randomized countertop apple.
set -euo pipefail

WBC_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ISAAC_PYTHON="${ISAAC_PYTHON:-$HOME/Erwin/isaac-sim-4.5.0/python.sh}"
KITCHEN_USD="$WBC_ROOT/FluxBisim/assets/environments/KitchenRoom/kitchen_room.usd"
if [[ ! -f "$KITCHEN_USD" ]]; then
    echo "Missing kitchen assets: $KITCHEN_USD (see README.md: 下载厨房资产)" >&2
    exit 1
fi
PLATE_USD="$WBC_ROOT/FluxBisim/assets/pick_place_fruit/plate/base.usd"
if [[ ! -f "$PLATE_USD" ]]; then
    echo "Missing plate asset: $PLATE_USD (see README.md: 下载厨房资产)" >&2
    exit 1
fi

APPLE_USD="$WBC_ROOT/FluxBisim/assets/pick_place_fruit/apple/apple.usd"
if [[ ! -f "$APPLE_USD" ]]; then
    echo "Missing apple asset: $APPLE_USD (see README.md: 下载厨房资产)" >&2
    exit 1
fi

cd "$WBC_ROOT"
export PYTHONPATH="$WBC_ROOT${PYTHONPATH:+:$PYTHONPATH}"
exec "$ISAAC_PYTHON" gear_sonic/scripts/run_sim_loop.py \
    --simulator isaac --enable-onscreen \
    --isaac-publish-camera --isaac-camera-source gui_perspective \
    --isaac-gui-perspective-zoom 4.0 \
    --camera-port 5555 --isaac-camera-fps 60 \
    --isaac-forward-k-to-deploy --isaac-robot-model sonic_g1_43dof \
    --isaac-scene-layer-path "$WBC_ROOT/gear_sonic/data/scenes/fluxbisim/kitchen.usda" \
    --isaac-initial-root-x -0.10 --isaac-initial-root-y -1.90 \
    --isaac-initial-root-yaw -9 \
    --isaac-initial-root-height 0.757 \
    --isaac-episode-directory "$WBC_ROOT/work_dirs/kitchen_episodes" \
    "$@"
