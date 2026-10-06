#!/usr/bin/env bash

set -euo pipefail

INPUT_DIR="tests/data/replay/eval"
CONFIG_PATH="config/config.yaml"
BUILD_DIR="build"
HORIZONS="50,100,150"
STEP_MS=10
WARMUP=10
OUTPUT_PREFIX="docs/replay_eval_summary"

usage() {
    cat <<'EOF'
Usage: bash scripts/evaluate_replay.sh [options]

Options:
  --input-dir <dir>      Directory containing replay CSV files (default: tests/data/replay/eval)
  --config <path>        Predictor config YAML path (default: config/config.yaml)
  --build-dir <dir>      CMake build directory (default: build)
  --horizons <list>      Comma-separated horizon list in ms (default: 50,100,150)
  --step-ms <int>        Predictor step in ms (default: 10)
  --warmup <int>         Warmup update count (default: 10)
  --output-prefix <path> Output prefix without extension (default: docs/replay_eval_summary)
  -h, --help             Show this help message
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
    --input-dir)
        INPUT_DIR="$2"
        shift 2
        ;;
    --config)
        CONFIG_PATH="$2"
        shift 2
        ;;
    --build-dir)
        BUILD_DIR="$2"
        shift 2
        ;;
    --horizons)
        HORIZONS="$2"
        shift 2
        ;;
    --step-ms)
        STEP_MS="$2"
        shift 2
        ;;
    --warmup)
        WARMUP="$2"
        shift 2
        ;;
    --output-prefix)
        OUTPUT_PREFIX="$2"
        shift 2
        ;;
    -h | --help)
        usage
        exit 0
        ;;
    *)
        echo "Unknown argument: $1" >&2
        usage
        exit 2
        ;;
    esac
done

if [[ ! -d "$INPUT_DIR" ]]; then
    echo "Input directory does not exist: $INPUT_DIR" >&2
    exit 1
fi

if [[ ! -f "$CONFIG_PATH" ]]; then
    echo "Config file does not exist: $CONFIG_PATH" >&2
    exit 1
fi

if [[ ! "$STEP_MS" =~ ^[0-9]+$ ]] || [[ "$STEP_MS" -le 0 ]]; then
    echo "--step-ms must be a positive integer" >&2
    exit 1
fi

if [[ ! "$WARMUP" =~ ^[0-9]+$ ]]; then
    echo "--warmup must be a non-negative integer" >&2
    exit 1
fi

EXE_PATH="$BUILD_DIR/pingpong_tracker_predictor_replay"
if [[ ! -x "$EXE_PATH" ]]; then
    echo "Replay executable not found or not executable: $EXE_PATH" >&2
    echo "Build it with: cmake --build $BUILD_DIR --target pingpong_tracker_predictor_replay --parallel" >&2
    exit 1
fi

IFS=',' read -r -a HORIZON_LIST <<<"$HORIZONS"
if [[ ${#HORIZON_LIST[@]} -eq 0 ]]; then
    echo "--horizons cannot be empty" >&2
    exit 1
fi

for horizon in "${HORIZON_LIST[@]}"; do
    if [[ ! "$horizon" =~ ^[0-9]+$ ]] || [[ "$horizon" -le 0 ]]; then
        echo "Each horizon must be a positive integer, got: $horizon" >&2
        exit 1
    fi
done

CSV_FILES=()
while IFS= read -r -d '' file; do
    CSV_FILES+=("$file")
done < <(find "$INPUT_DIR" -maxdepth 1 -type f -name "*.csv" -print0 | sort -z)

if [[ ${#CSV_FILES[@]} -eq 0 ]]; then
    echo "No CSV files found in: $INPUT_DIR" >&2
    exit 1
fi

OUTPUT_DIR="$(dirname "$OUTPUT_PREFIX")"
mkdir -p "$OUTPUT_DIR"

CSV_SUMMARY="${OUTPUT_PREFIX}.csv"
MD_SUMMARY="${OUTPUT_PREFIX}.md"

printf "dataset,horizon_ms,filter_type,replay_samples,update_success,update_attempts,update_success_rate,filtered_rmse_m,horizon_rmse_m\n" >"$CSV_SUMMARY"

{
    printf "# Replay Evaluation Summary\n\n"
    printf "| dataset | horizon_ms | filter_type | replay_samples | update_success | update_attempts | success_rate | filtered_rmse_m | horizon_rmse_m |\n"
    printf "|---|---:|---|---:|---:|---:|---:|---:|---:|\n"
} >"$MD_SUMMARY"

for csv_file in "${CSV_FILES[@]}"; do
    dataset="$(basename "$csv_file")"

    for horizon in "${HORIZON_LIST[@]}"; do
        run_output="$("$EXE_PATH" "$csv_file" --config "$CONFIG_PATH" --horizon-ms "$horizon" --step-ms "$STEP_MS" --warmup "$WARMUP")"

        filter_type="$(awk -F': ' '/^Filter type:/ {print $2; exit}' <<<"$run_output")"
        replay_samples="$(awk -F': ' '/^Replay samples:/ {print $2; exit}' <<<"$run_output")"
        update_success="$(awk -F'[:/]' '/^Update success:/ {gsub(/ /, "", $2); print $2; exit}' <<<"$run_output")"
        update_attempts="$(awk -F'[:/]' '/^Update success:/ {gsub(/ /, "", $3); print $3; exit}' <<<"$run_output")"
        filtered_rmse="$(awk '/^Filtered RMSE \(m\):/ {print $4; exit}' <<<"$run_output")"
        horizon_rmse="$(awk -v h="$horizon" '$0 ~ ("^" h "ms Horizon RMSE \\(m\\):") {print $5; exit}' <<<"$run_output")"

        if [[ -z "$filter_type" || -z "$replay_samples" || -z "$update_success" || -z "$update_attempts" || -z "$filtered_rmse" || -z "$horizon_rmse" ]]; then
            echo "Failed to parse replay output for $dataset (horizon=${horizon}ms)" >&2
            echo "$run_output" >&2
            exit 1
        fi

        success_rate="$(awk -v succ="$update_success" -v att="$update_attempts" 'BEGIN { if (att == 0) { printf "0.0000" } else { printf "%.4f", succ / att } }')"

        printf "%s,%s,%s,%s,%s,%s,%s,%s,%s\n" \
            "$dataset" "$horizon" "$filter_type" "$replay_samples" "$update_success" "$update_attempts" "$success_rate" "$filtered_rmse" "$horizon_rmse" >>"$CSV_SUMMARY"

        printf "| %s | %s | %s | %s | %s | %s | %s | %s | %s |\n" \
            "$dataset" "$horizon" "$filter_type" "$replay_samples" "$update_success" "$update_attempts" "$success_rate" "$filtered_rmse" "$horizon_rmse" >>"$MD_SUMMARY"
    done
done

echo "Replay summary CSV written to: $CSV_SUMMARY"
echo "Replay summary Markdown written to: $MD_SUMMARY"
