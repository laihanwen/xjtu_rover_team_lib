#!/usr/bin/env bash
set -euo pipefail

annotation_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export UV_CACHE_DIR="/home/hanwen/auv/.cache/uv"
export LABEL_STUDIO_BASE_DATA_DIR="${annotation_dir}/data"
export LABEL_STUDIO_LOCAL_FILES_SERVING_ENABLED=true
export LABEL_STUDIO_LOCAL_FILES_DOCUMENT_ROOT="/home/hanwen/auv/datasets/interim"

cd "${annotation_dir}"
exec uv run label-studio start --host 127.0.0.1 --port 8080
