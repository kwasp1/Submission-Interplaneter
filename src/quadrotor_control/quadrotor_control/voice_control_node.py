#!/usr/bin/env python3
"""
Voice control node: listens to the system microphone, converts speech
to text using Google's free Web Speech API (via the SpeechRecognition
library), maps recognized keywords to geometry_msgs/Twist commands,
and publishes them to drive the simulated quadrotor.

Only acts when the shared /control_mode topic says "voice" - this is
how dual-mode switching (voice vs manual) is enforced: this node and
the GUI's manual controls both publish to the same Twist topic, but
each only acts when it's actually their turn.
"""
import threading
import tempfile

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import speech_recognition as sr
from faster_whisper import WhisperModel


# Keyword -> (linear.x, linear.y, linear.z, angular.z)
# Forward/backward move along X, left/right strafe along Y,
# up/down move along Z, stop zeros everything.
COMMAND_MAP = {
    'forward':  (0.4, 0.0, 0.0, 0.0),
    'backward': (-0.4, 0.0, 0.0, 0.0),
    'back':     (-0.4, 0.0, 0.0, 0.0),
    'left':     (0.0, 0.4, 0.0, 0.0),
    'right':    (0.0, -0.4, 0.0, 0.0),
    'up':       (0.0, 0.0, 0.4, 0.0),
    'down':     (0.0, 0.0, -0.4, 0.0),
    'stop':     (0.0, 0.0, 0.0, 0.0),
}

# How long (seconds) a movement command keeps driving the drone before
# auto-stopping, in case no new voice command arrives soon after -
# prevents "forward" from meaning "forward forever" if recognition
# happens to miss a follow-up "stop".
COMMAND_DURATION = 1.5


class VoiceControlNode(Node):
    def __init__(self):
        super().__init__('voice_control_node')

        self.current_mode = 'manual'  # default to manual until GUI says otherwise

        self.cmd_vel_pub = self.create_publisher(
            Twist, '/X3/gazebo/command/twist', 10)
        self.mode_sub = self.create_subscription(
            String, '/control_mode', self.mode_callback, 10)

        self.auto_stop_timer = None

        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Calibrate for ambient noise once at startup.
        with self.microphone as source:
            self.get_logger().info('Calibrating microphone for ambient noise...')
            self.recognizer.adjust_for_ambient_noise(source, duration=1.0)

        self.get_logger().info('Loading Whisper model (this may take a moment)...')
        # "small" model - good accuracy/speed tradeoff for short commands.
        # device="cpu" avoids any CUDA/cuDNN setup headaches; still fast
        # enough for short (~3s) command clips.
        self.whisper_model = WhisperModel('small', device='cpu', compute_type='int8')
        self.get_logger().info('Whisper model loaded.')

        self.listen_thread = threading.Thread(target=self.listen_loop, daemon=True)
        self.listen_thread.start()

        self.get_logger().info(
            'Voice control node started. Mode: manual (say a command '
            'once GUI switches to Voice Mode).')

    def mode_callback(self, msg: String):
        self.current_mode = msg.data
        self.get_logger().info(f'Control mode changed to: {self.current_mode}')

    def listen_loop(self):
        """Runs in a background thread - continuously listens for speech
        and recognizes it, regardless of current mode (so there's no lag
        switching into voice mode). Whether a recognized command actually
        moves the drone is gated separately, in publish_command()."""
        while rclpy.ok():
            try:
                with self.microphone as source:
                    audio = self.recognizer.listen(
                        source, timeout=3.0, phrase_time_limit=3.0)
            except sr.WaitTimeoutError:
                self.get_logger().info('(no speech detected in this window)')
                continue

            try:
                text = self.transcribe_with_whisper(audio)
                if text:
                    self.get_logger().info(f'Heard: "{text}"')
                    self.process_text(text)
                else:
                    self.get_logger().info('(audio captured, but could not be understood)')
            except Exception as e:
                self.get_logger().warn(f'Whisper transcription failed: {e}')

    def transcribe_with_whisper(self, audio) -> str:
        """Writes the captured audio to a temp WAV file and runs it
        through the local Whisper model. Returns lowercased text,
        or an empty string if nothing usable was transcribed."""
        with tempfile.NamedTemporaryFile(suffix='.wav', delete=True) as f:
            f.write(audio.get_wav_data())
            f.flush()
            segments, _info = self.whisper_model.transcribe(
                f.name, language='en', beam_size=5)
            text = ' '.join(segment.text for segment in segments).strip().lower()
        return text

    def process_text(self, text: str):
        for keyword, values in COMMAND_MAP.items():
            if keyword in text:
                self.publish_command(keyword, values)
                return
        self.get_logger().info(f'No matching command in: "{text}"')

    def publish_command(self, keyword: str, values):
        if self.current_mode != 'voice':
            self.get_logger().info(
                f'Heard "{keyword}" but currently in {self.current_mode} mode - ignoring.')
            return

        lx, ly, lz, az = values
        twist = Twist()
        twist.linear.x = lx
        twist.linear.y = ly
        twist.linear.z = lz
        twist.angular.z = az
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f'Published voice command: {keyword}')

        # Cancel any pending auto-stop from a previous command, then
        # schedule a new one - keeps the drone from drifting forever
        # if no further voice command follows.
        if self.auto_stop_timer is not None:
            self.auto_stop_timer.cancel()

        if keyword != 'stop':
            self.auto_stop_timer = self.create_timer(
                COMMAND_DURATION, self.auto_stop)

    def auto_stop(self):
        self.cmd_vel_pub.publish(Twist())  # all-zero Twist
        if self.auto_stop_timer is not None:
            self.auto_stop_timer.cancel()
            self.auto_stop_timer = None


def main(args=None):
    rclpy.init(args=args)
    node = VoiceControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
