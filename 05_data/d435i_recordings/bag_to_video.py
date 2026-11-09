#!/usr/bin/env python3
"""Export a ROS1 color-image bag to MP4 without needing a running ROS master."""

import argparse
import itertools
import json
from pathlib import Path
import statistics

import cv2
import numpy as np
import rosbag


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument("--topic", default="/camera/color/image_raw")
    parser.add_argument("--fps", type=float, help="Override the measured frame rate")
    parser.add_argument("--rotation", type=int, choices=(0, 180),
                        help="Rotation in degrees; default from orientation.json")
    args = parser.parse_args()
    if not args.bag.is_file() or args.bag.suffix != ".bag":
        parser.error("Provide a completed .bag file")
    if args.fps is not None and (not np.isfinite(args.fps) or args.fps <= 0):
        parser.error("--fps must be positive and finite")

    root = Path(__file__).resolve().parent
    rotation = args.rotation
    if rotation is None:
        with (root / "orientation.json").open(encoding="utf-8") as stream:
            rotation = json.load(stream)["rotation_degrees"]
    if rotation not in (0, 180):
        parser.error("rotation_degrees must be 0 or 180")
    suffix = "_upright" if rotation == 180 else ""
    output = root / "videos" / (args.bag.stem + suffix + ".mp4")
    partial = output.with_name(output.stem + ".partial.mp4")
    report_path = root / "notes" / (args.bag.stem + suffix + "_conversion.json")
    for path in (output, partial, report_path):
        if path.exists():
            parser.error("Refusing to overwrite: " + str(path))
    output.parent.mkdir(parents=True, exist_ok=True)
    report_path.parent.mkdir(parents=True, exist_ok=True)

    with rosbag.Bag(str(args.bag), "r") as bag:
        count = bag.get_message_count(topic_filters=[args.topic])
        if not count:
            parser.error("No images on topic " + args.topic)
        messages = iter(bag.read_messages(topics=[args.topic]))
        sample = list(itertools.islice(messages, min(121, count)))
        first = sample[0][1]
        width, height = first.width, first.height
        times = [msg.header.stamp.to_sec() for _, msg, _ in sample]
        deltas = [b - a for a, b in zip(times, times[1:]) if b > a]
        if not deltas and args.fps is None:
            parser.error("Cannot estimate frame rate; specify --fps")
        fps = args.fps or (1.0 / statistics.median(deltas))
        if args.fps is None and abs(fps - round(fps)) < 0.05:
            fps = float(round(fps))
        writer = cv2.VideoWriter(str(partial), cv2.VideoWriter_fourcc(*"mp4v"),
                                 fps, (width, height))
        if not writer.isOpened():
            raise RuntimeError("Could not open MP4 encoder: " + str(partial))
        written = 0
        first_stamp = last_stamp = None
        try:
            for _, msg, _ in itertools.chain(sample, messages):
                if (msg.width, msg.height) != (width, height):
                    raise RuntimeError("Image dimensions changed during recording")
                channels = {"rgb8": 3, "bgr8": 3, "rgba8": 4,
                            "bgra8": 4, "mono8": 1}.get(msg.encoding)
                if channels is None:
                    raise RuntimeError("Unsupported encoding: " + msg.encoding)
                rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(height, msg.step)
                frame = rows[:, :width * channels].reshape(height, width, channels)
                conversions = {"rgb8": cv2.COLOR_RGB2BGR,
                               "rgba8": cv2.COLOR_RGBA2BGR,
                               "bgra8": cv2.COLOR_BGRA2BGR,
                               "mono8": cv2.COLOR_GRAY2BGR}
                if msg.encoding in conversions:
                    frame = cv2.cvtColor(frame, conversions[msg.encoding])
                if rotation == 180:
                    frame = cv2.rotate(frame, cv2.ROTATE_180)
                writer.write(np.ascontiguousarray(frame))
                stamp = msg.header.stamp.to_sec()
                if first_stamp is None:
                    first_stamp = stamp
                last_stamp = stamp
                written += 1
                if written % 300 == 0:
                    print(f"Converted {written}/{count} frames", flush=True)
        finally:
            writer.release()

    capture = cv2.VideoCapture(str(partial))
    decoded = 0
    try:
        if not capture.isOpened():
            raise RuntimeError("Cannot reopen generated MP4")
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            if frame.shape[:2] != (height, width):
                raise RuntimeError("Generated MP4 dimensions do not match the source")
            decoded += 1
    finally:
        capture.release()
    if decoded != written or written != count:
        raise RuntimeError(f"Frame count mismatch: source={count}, written={written}, decoded={decoded}")

    # A hard link publishes the validated video without overwriting an existing file.
    import os
    os.link(str(partial), str(output))
    partial.unlink()
    report = dict(source=str(args.bag.resolve()), topic=args.topic,
                  output=str(output), codec="mp4v", width=width, height=height,
                  source_encoding=first.encoding, fps=fps, source_frames=count,
                  rotation_degrees=rotation,
                  decoded_frames=decoded, video_duration_seconds=written / fps,
                  source_stamp_span_seconds=last_stamp - first_stamp,
                  size_bytes=output.stat().st_size,
                  timing="Constant frame rate; one output frame per source image, in bag order")
    with report_path.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)


if __name__ == "__main__":
    main()
