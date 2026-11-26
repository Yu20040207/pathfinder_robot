"""语音交互节点启动文件。

只起 `voice_node`；执行端（`voice_demo_executor`，模拟底盘）默认不起，
需要时带 `start_demo_executor:=true`。**它只发 `/demo/cmd_vel`，不会驱动实车。**

模型目录默认按下面顺序推断，可用 `FWR_VOICE_MODELS` 环境变量或同名参数覆盖：
  1. 环境变量 `FWR_VOICE_MODELS`
  2. 源码树的 `<repo>/models/voice`
  3. `~/Company_Project/Four-wheel-robot/models/voice`

音频设备默认留空（走 ALSA `default`）。开发机实测：card0 = NVidia（只有 HDMI，别用）、
card1 = 机身 ALC256、**card2 = 外接 USB 声卡**；主控麦克风在 card0。
**换机器/插拔声卡后编号会变**，先 `aplay -l | grep ^card` 确认再填。

注意：本文件会把节点 `prefix` 成仓库的 `./.venv/bin/python`，因为 `sherpa_onnx`
只装在项目 venv 里，而 `ros2 launch` 默认用 `/usr/bin/python3`（会 ModuleNotFoundError）。
副作用：launch 里进程显示成 `python-1`，但 ROS 节点名仍是 `/voice_node`。

示例：
    ros2 launch fwr_voice voice.launch.py simulation:=true
    ros2 launch fwr_voice voice.launch.py input_wav:=/tmp/wake.wav \
        speaker_device:=plughw:2,0
    ros2 launch fwr_voice voice.launch.py microphone_enabled:=true \
        microphone_device:=plughw:2,0 speaker_device:=plughw:2,0
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _models_root():
    explicit = os.environ.get("FWR_VOICE_MODELS", "").strip()
    if explicit:
        return explicit
    here = os.path.dirname(os.path.realpath(__file__))
    candidates = [
        os.path.normpath(os.path.join(here, "..", "..", "..", "..", "models", "voice")),
        os.path.expanduser("~/Company_Project/Four-wheel-robot/models/voice"),
    ]
    for path in candidates:
        if os.path.isdir(path):
            return path
    return ""


def _venv_python():
    """`sherpa_onnx` 只装在项目 `.venv/` 里；`ros2 launch` 默认用 /usr/bin/python3，
    会以 ModuleNotFoundError: sherpa_onnx 直接退出。所以默认把节点前缀成 venv 解释器，
    和 systemd 单元 `./.venv/bin/python ...` 的写法保持一致。"""
    explicit = os.environ.get("FWR_VOICE_PYTHON", "").strip()
    if explicit:
        return explicit
    here = os.path.dirname(os.path.realpath(__file__))
    for base in (os.path.normpath(os.path.join(here, "..", "..", "..", "..")),
                 os.path.expanduser("~/Company_Project/Four-wheel-robot")):
        candidate = os.path.join(base, ".venv", "bin", "python")
        if os.path.isfile(candidate):
            return candidate
    return ""


def _sub(root, name):
    return os.path.join(root, name) if root else ""


def generate_launch_description():
    root = _models_root()
    asr_default = _sub(root, "sensevoice")
    tts_default = _sub(root, "vits")
    prefix = _venv_python()
    return LaunchDescription([
        DeclareLaunchArgument("asr_model_dir", default_value=asr_default,
                              description="SenseVoice 模型目录（含 model.int8.onnx + tokens.txt）"),
        DeclareLaunchArgument("tts_model_dir", default_value=tts_default,
                              description="VITS 模型目录（含 model.onnx + tokens.txt）"),
        DeclareLaunchArgument("microphone_device", default_value="",
                              description="ALSA 采集设备，如 plughw:1,0；留空走 default"),
        DeclareLaunchArgument("speaker_device", default_value="",
                              description="ALSA 播放设备，如 plughw:1,0；留空走 default"),
        DeclareLaunchArgument("microphone_enabled", default_value="false",
                              description="打开麦克风采集（需要 asr_model_dir）"),
        DeclareLaunchArgument("input_wav", default_value="",
                              description="启动后播放并识别一次指定 WAV，便于不开麦冒烟"),
        DeclareLaunchArgument("simulation", default_value="true",
                              description="模拟模式才有 SIM_JOG；实车必须改 false"),
        DeclareLaunchArgument("cloud_base_url", default_value="",
                              description="OpenAI 兼容对话服务地址，留空则只有离线指令"),
        DeclareLaunchArgument("cloud_model", default_value="",
                              description="云端模型名"),
        DeclareLaunchArgument("tts_provider", default_value="local",
                              description="local=本地 VITS；aliyun=阿里云 CosyVoice；"
                                          "volcengine=火山引擎（豆包音色）"),
        DeclareLaunchArgument("tts_speaker_id", default_value="0",
                              description="本地 VITS 音色编号（0~186）"),
        DeclareLaunchArgument("tts_speed", default_value="1.0",
                              description="语速，本地 0.5~1.5，云端 0.5~2.0"),
        DeclareLaunchArgument("tts_cloud_voice", default_value="",
                              description="阿里云 CosyVoice 音色名，如 longanya_v3"),
        DeclareLaunchArgument("tts_cloud_model", default_value="cosyvoice-v3-flash",
                              description="阿里云语音模型"),
        DeclareLaunchArgument("tts_cloud_websocket_url", default_value="",
                              description="覆盖阿里云语音 WebSocket 地址（留空用官方默认）"),
        DeclareLaunchArgument("tts_volc_voice_type", default_value="",
                              description="火山引擎音色 ID（如 BV700_streaming / zh_female_xxx_bigtts）"),
        DeclareLaunchArgument("tts_volc_cluster", default_value="volcano_tts",
                              description="火山引擎业务集群，一般 volcano_tts"),
        DeclareLaunchArgument("tts_volc_endpoint", default_value="https://openspeech.bytedance.com/api/v1/tts",
                              description="火山引擎语音合成地址"),
        DeclareLaunchArgument("tts_volc_encoding", default_value="pcm",
                              description="火山引擎返回编码：pcm 或 wav"),
        DeclareLaunchArgument("tts_volc_appid_env", default_value="FWR_VOLC_APPID",
                              description="保存 appid 的环境变量名"),
        DeclareLaunchArgument("tts_volc_token_env", default_value="FWR_VOLC_TOKEN",
                              description="保存 access token 的环境变量名"),
        DeclareLaunchArgument("cloud_extra_json", default_value="",
                              description='兼容接口私有字段 JSON，方舟关思考传 {"thinking":{"type":"disabled"}}'),
        DeclareLaunchArgument("tts_seed_voice", default_value="saturn_zh_female_keainvsheng_tob",
                              description="豆包 Seed-TTS 音色（saturn_*_tob 这一族）"),
        DeclareLaunchArgument("tts_seed_resource_id", default_value="seed-tts-2.0",
                              description="豆包语音资源 ID（语音合成 2.0 = seed-tts-2.0）"),
        DeclareLaunchArgument("tts_seed_encoding", default_value="pcm",
                              description="豆包 Seed-TTS 返回编码"),
        DeclareLaunchArgument("tts_seed_apikey_env", default_value="FWR_VOLC_APIKEY",
                              description="保存豆包语音 API Key 的环境变量名"),
        DeclareLaunchArgument("start_demo_executor", default_value="false",
                              description="同时起模拟执行器（只发 /demo/cmd_vel，不动实车）"),
        Node(
            package="fwr_voice",
            executable="voice_node",
            name="voice_node",
            output="screen",
            prefix=prefix,
            parameters=[{
                "asr_model_dir": LaunchConfiguration("asr_model_dir"),
                "tts_model_dir": LaunchConfiguration("tts_model_dir"),
                "microphone_device": LaunchConfiguration("microphone_device"),
                "speaker_device": LaunchConfiguration("speaker_device"),
                "microphone_enabled": LaunchConfiguration("microphone_enabled"),
                "input_wav": LaunchConfiguration("input_wav"),
                "simulation": LaunchConfiguration("simulation"),
                "cloud_base_url": LaunchConfiguration("cloud_base_url"),
                "cloud_model": LaunchConfiguration("cloud_model"),
                "tts_provider": LaunchConfiguration("tts_provider"),
                "tts_speaker_id": LaunchConfiguration("tts_speaker_id"),
                "tts_speed": LaunchConfiguration("tts_speed"),
                "tts_cloud_voice": LaunchConfiguration("tts_cloud_voice"),
                "tts_cloud_model": LaunchConfiguration("tts_cloud_model"),
                "tts_cloud_websocket_url": LaunchConfiguration("tts_cloud_websocket_url"),
                "tts_volc_voice_type": LaunchConfiguration("tts_volc_voice_type"),
                "tts_volc_cluster": LaunchConfiguration("tts_volc_cluster"),
                "tts_volc_endpoint": LaunchConfiguration("tts_volc_endpoint"),
                "tts_volc_encoding": LaunchConfiguration("tts_volc_encoding"),
                "tts_volc_appid_env": LaunchConfiguration("tts_volc_appid_env"),
                "tts_volc_token_env": LaunchConfiguration("tts_volc_token_env"),
                "cloud_extra_json": LaunchConfiguration("cloud_extra_json"),
                "tts_seed_voice": LaunchConfiguration("tts_seed_voice"),
                "tts_seed_resource_id": LaunchConfiguration("tts_seed_resource_id"),
                "tts_seed_encoding": LaunchConfiguration("tts_seed_encoding"),
                "tts_seed_apikey_env": LaunchConfiguration("tts_seed_apikey_env"),
            }],
        ),
        Node(
            package="fwr_voice",
            executable="voice_demo_executor",
            name="voice_demo_executor",
            output="screen",
            prefix=prefix,
            condition=IfCondition(LaunchConfiguration("start_demo_executor")),
        ),
    ])
