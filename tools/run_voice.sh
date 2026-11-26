#!/usr/bin/env bash
# 一条命令起「语音 + 豆包音色 + 开发机声卡」的完整栈。
#
# 2026-11-26 定版：音色用 saturn_zh_female_keainvsheng_tob（语音合成 2.0 / seed-tts-2.0）。
# 换了机器/换了声卡，只改下面两个设备号，不要动别的。
#
#   ./tools/run_voice.sh                 # 正常跑（开麦）
#   ./tools/run_voice.sh --wav           # 不开麦，喂一段 wav 冒烟（不需要人说话）
#   ./tools/run_voice.sh --local         # 豆包不可用时先跑本地 187 音色
#
set -euo pipefail

cd "$(dirname "$0")/.."

MIC="${FWR_MIC:-plughw:1,0}"          # 开发机内置麦克风（card1）
SPK="${FWR_SPK:-plughw:2,0}"          # 开发机外接 USB 喇叭（card2）
VOICE="${FWR_VOICE:-saturn_zh_female_keainvsheng_tob}"

KEY_FILE="${FWR_VOLC_KEY_FILE:-$HOME/.volc_token}"
if [[ -z "${FWR_VOLC_APIKEY:-}" && -s "$KEY_FILE" ]]; then
  export FWR_VOLC_APIKEY="$(tr -d '[:space:]' < "$KEY_FILE")"
fi

for arg in "$@"; do
  case "$arg" in
    --local) PROVIDER=local ;;
    --wav)   MODE=wav ;;
    --help|-h)
      sed -n '2,10p' "$0" | sed 's/^# \{0,1\}//'
      exit 0
      ;;
  esac
done
PROVIDER="${PROVIDER:-doubao}"

if [[ "$PROVIDER" == "doubao" && -z "${FWR_VOLC_APIKEY:-}" ]]; then
  echo "⚠️  没有 FWR_VOLC_APIKEY（也没找到 $KEY_FILE），豆包音色不可用 → 自动退回本地音色" >&2
  PROVIDER=local
fi

# ROS 的 setup 脚本会引用未定义变量，和 `set -u` 冲突，必须临时关掉
set +u
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
set -u

args=(
  tts_provider:="$PROVIDER"
  speaker_device:="$SPK"
  tts_seed_voice:="$VOICE"
  simulation:=true
)

# 云端对话（方舟）：没配就保持离线问答
ARK_KEY_FILE="${FWR_ARK_KEY_FILE:-$HOME/.ark_key}"
CLOUD_MODEL="${FWR_CLOUD_MODEL:-doubao-seed-2-0-lite-260428}"
if [[ -z "${FWR_LLM_API_KEY:-}" && -s "$ARK_KEY_FILE" ]]; then
  export FWR_LLM_API_KEY="$(tr -d '[:space:]' < "$ARK_KEY_FILE")"
fi
if [[ -n "${FWR_LLM_API_KEY:-}" && -z "${FWR_CLOUD_OFF:-}" ]]; then
  args+=(
    cloud_base_url:=https://ark.cn-beijing.volces.com/api/v3
    cloud_model:="$CLOUD_MODEL"
  )
  # 方舟的思考型模型默认先吐 2~3 秒 reasoning；语音场景必须关掉。
  # 走环境变量而不是 launch 参数：CLI 上的 JSON 会被 launch 当 YAML 解析成 dict 报错。
  export FWR_CLOUD_EXTRA_JSON='{"thinking":{"type":"disabled"}}'
  echo "▶ 云端对话：方舟 $CLOUD_MODEL"
else
  echo "▶ 云端对话：关闭（只会回「当前离线」）"
fi

if [[ "${MODE:-}" == "wav" ]]; then
  args+=(microphone_enabled:=false input_wav:=models/voice/test_wavs/zh.wav)
  echo "▶ 冒烟模式：喂 models/voice/test_wavs/zh.wav，不需要说话"
else
  args+=(microphone_enabled:=true microphone_device:="$MIC")
  echo "▶ 开麦模式：说「小明同学」唤醒（mic=$MIC speaker=$SPK）"
fi

echo "▶ 音色：$PROVIDER / $VOICE"
exec ros2 launch fwr_voice voice.launch.py "${args[@]}"
