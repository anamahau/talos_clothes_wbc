#!/usr/bin/env python
"""
Record a sensor_msgs/Image ROS topic to a video file (e.g. .mp4 or .avi),
with pause/resume controlled by a trigger topic.

Recording starts as soon as the node comes up. Every message received on
the trigger topic toggles the recording state: the first trigger pauses
recording (no frames are written, but the file stays open), the next
trigger resumes it, and so on - all segments are written into the SAME
output file, one after another, with no gaps or re-created files.

Usage examples:
    # Record RealSense #1 color stream to mp4 at 30 fps, toggled by /record_trigger
    ./record_image_topic.py --topic /camera1/color/image_raw --out experiment1.mp4 \\
        --trigger-topic /record_trigger

    # Record RealSense #2 at a custom fps
    ./record_image_topic.py --topic /camera2/color/image_raw --out experiment2.mp4 \\
        --fps 15 --trigger-topic /record_trigger

Trigger a pause/resume from the command line, e.g.:
    rostopic pub -1 /record_trigger std_msgs/Empty {}

Stop recording entirely with Ctrl+C - the video file is finalized (released) on shutdown.
"""

import argparse
import os

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import Empty


class ImageTopicRecorder:
    def __init__(self, topic, out_path, fps, fourcc_str, encoding, trigger_topic):
        self.topic = topic
        self.out_path = out_path
        self.fps = fps
        self.fourcc_str = fourcc_str
        self.encoding = encoding  # 'bgr8' for color, 'passthrough' to keep native encoding
        self.trigger_topic = trigger_topic

        self.bridge = CvBridge()
        self.writer = None
        self.frame_size = None  # (width, height)
        self.frame_count = 0

        # Recording starts active; each trigger message flips this flag.
        self.recording_active = True
        self.pause_count = 0  # how many times recording has been paused so far

        self.sub = rospy.Subscriber(
            self.topic, Image, self.callback, queue_size=1
        )

        self.trigger_sub = rospy.Subscriber(
            self.trigger_topic, Empty, self.trigger_callback, queue_size=1
        )

        rospy.on_shutdown(self.shutdown_hook)

        rospy.loginfo(
            "Recorder ready. Topic: %s | Output: %s | FPS: %s | Trigger topic: %s",
            self.topic, self.out_path, self.fps, self.trigger_topic
        )
        rospy.loginfo("Recording is ACTIVE. Send a trigger on %s to pause.", self.trigger_topic)

    def trigger_callback(self, msg):
        self.recording_active = not self.recording_active
        if self.recording_active:
            rospy.loginfo("Trigger received -> RESUMING recording into %s", self.out_path)
        else:
            self.pause_count += 1
            rospy.loginfo(
                "Trigger received -> PAUSING recording (pause #%d). "
                "Frames will be skipped until next trigger.",
                self.pause_count
            )

    def callback(self, msg):
        if not self.recording_active:
            # Paused: drop the frame entirely, but keep the writer open so
            # resuming continues appending to the same output file.
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding=self.encoding)
        except CvBridgeError as e:
            rospy.logerr("cv_bridge conversion failed: %s", e)
            return

        # If the incoming encoding is grayscale/depth-like, convert to BGR so
        # VideoWriter (which expects 3-channel frames) works consistently.
        if cv_img.ndim == 2:
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_GRAY2BGR)

        if self.writer is None:
            h, w = cv_img.shape[:2]
            self.frame_size = (w, h)
            fourcc = cv2.VideoWriter_fourcc(*self.fourcc_str)
            self.writer = cv2.VideoWriter(
                self.out_path, fourcc, self.fps, self.frame_size
            )
            if not self.writer.isOpened():
                rospy.logerr(
                    "Failed to open VideoWriter for %s with fourcc %s. "
                    "Try a different --fourcc (e.g. XVID with a .avi output).",
                    self.out_path, self.fourcc_str
                )
                rospy.signal_shutdown("VideoWriter failed to open")
                return
            rospy.loginfo("VideoWriter opened: %dx%d @ %s fps", w, h, self.fps)

        # Guard against a resolution change mid-recording (would corrupt the file)
        h, w = cv_img.shape[:2]
        if (w, h) != self.frame_size:
            rospy.logwarn_throttle(
                5.0,
                "Frame size changed (%dx%d -> expected %dx%d); resizing to match.",
                w, h, self.frame_size[0], self.frame_size[1]
            )
            cv_img = cv2.resize(cv_img, self.frame_size)

        self.writer.write(cv_img)
        self.frame_count += 1

        if self.frame_count % 100 == 0:
            rospy.loginfo("Recorded %d frames...", self.frame_count)

    def shutdown_hook(self):
        if self.writer is not None:
            self.writer.release()
            rospy.loginfo(
                "Saved %d frames to %s (recording was paused %d time(s) during the run)",
                self.frame_count, self.out_path, self.pause_count
            )
        else:
            rospy.logwarn("No frames were received; no video file was written.")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--topic", required=True,
        help="Image topic to subscribe to, e.g. /camera1/color/image_raw"
    )
    parser.add_argument(
        "--out", required=True,
        help="Output video file path, e.g. experiment.mp4 or experiment.avi"
    )
    parser.add_argument(
        "--fps", type=float, default=30.0,
        help="Output video FPS (default: 30). Should roughly match the camera's "
             "publish rate; mismatches will make playback speed look wrong."
    )
    parser.add_argument(
        "--fourcc", default="mp4v",
        help="FourCC codec for VideoWriter (default: mp4v for .mp4). "
             "Use XVID or MJPG with a .avi extension if mp4v is unavailable."
    )
    parser.add_argument(
        "--encoding", default="bgr8",
        help="Desired cv2 image encoding passed to cv_bridge (default: bgr8). "
             "Use 'passthrough' for depth/mono topics."
    )
    parser.add_argument(
        "--trigger-topic", default="/record_trigger",
        help="std_msgs/Empty topic used to toggle pause/resume (default: /record_trigger). "
             "Recording starts active; each message received on this topic flips "
             "the state (pause, then resume, then pause, ...)."
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()

    out_dir = os.path.dirname(os.path.abspath(args.out))
    if out_dir and not os.path.isdir(out_dir):
        os.makedirs(out_dir)

    rospy.init_node("image_topic_recorder", anonymous=True)

    recorder = ImageTopicRecorder(
        topic=args.topic,
        out_path=args.out,
        fps=args.fps,
        fourcc_str=args.fourcc,
        encoding=args.encoding,
        trigger_topic=args.trigger_topic,
    )

    rospy.spin()
