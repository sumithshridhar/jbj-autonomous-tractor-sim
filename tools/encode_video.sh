#!/usr/bin/env bash
# Turn rendered frames + HUD into an mp4 (run in WSL):  bash tools/encode_video.sh <run_name> [fps]
set -o pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
F="$ROOT/blender/out/frames_$1"
mkdir -p "$ROOT/media"
cd "$F" && ffmpeg -y -loglevel error -framerate "${2:-25}" -i %05d.png -vf "ass=hud.ass" -c:v libx264 -pix_fmt yuv420p -crf 22 -movflags +faststart "$ROOT/media/$1.mp4" \
  && echo "wrote media/$1.mp4 ($(du -h "$ROOT/media/$1.mp4" | cut -f1))"
