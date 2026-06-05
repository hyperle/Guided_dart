#!/usr/bin/env zsh

export GUIDED_DART_PATH="/home/hyperlee/guided_dart"
export GUIDED_DART_SCRIPT_PATH="${GUIDED_DART_PATH}/.script"
export GUIDED_DART_GUIDANCE_CONFIG="${GUIDED_DART_PATH}/Host_tools/guidance/guidance_params.yaml"

case ":$PATH:" in
    *:"${GUIDED_DART_SCRIPT_PATH}":*) ;;
    *) export PATH="${GUIDED_DART_SCRIPT_PATH}:$PATH" ;;
esac
