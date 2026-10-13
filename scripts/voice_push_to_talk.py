"""Press Enter to record one utterance and submit its text to voice_node."""

import argparse
import time

import rclpy
from std_msgs.msg import String

from fwr_voice.audio import MicrophoneInput
from fwr_voice.asr import SherpaSenseVoice
from fwr_voice.ptt import run_once


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--model-dir", required=True)
    parser.add_argument("--device", default=None)
    args = parser.parse_args()
    asr = SherpaSenseVoice(args.model_dir)
    microphone = MicrophoneInput(device=args.device)
    rclpy.init()
    node = rclpy.create_node("voice_push_to_talk")
    publisher = node.create_publisher(String, "/robot/voice/text_input", 10)
    print("按回车开始说话；Ctrl+C 退出。")
    try:
        while True:
            result = run_once(microphone, asr, lambda text: publisher.publish(String(data=text)))
            print(f"识别：{result.text or '[无语音]'} ({result.latency_ms:.0f} ms)")
            time.sleep(0.1)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
