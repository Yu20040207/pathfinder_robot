"""One push-to-talk turn without coupling audio capture to ROS."""


def run_once(audio_input, asr, publish, wait_for_button=input):
    wait_for_button()
    clip = audio_input.record_once(15)
    transcript = asr.transcribe(clip)
    if transcript.text:
        publish(transcript.text)
    return transcript
