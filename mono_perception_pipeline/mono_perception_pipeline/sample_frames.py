"""Frame extraction for monocular vision pipelines (VO / COLMAP SfM).

Standalone preprocessing step: sample frames from an iPhone video at a
configurable interval, drop blurry frames using variance-of-Laplacian
sharpness scoring, and write the survivors as sequentially numbered
images so alphabetical order matches chronological order.

Run this before the ROS 2 node to produce a frame folder on disk.
"""

import argparse
import os

import cv2


def compute_blur_score(frame):
    """Return the variance of the Laplacian of a frame (sharpness proxy).

    Higher values indicate a sharper image; low values indicate blur
    (motion blur, defocus, etc).
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return cv2.Laplacian(gray, cv2.CV_64F).var()


def inspect_blur_scores(video_path, sample_interval=5):
    """Print the Laplacian-variance score for every sampled frame.

    Does not save anything. Use this to look at the distribution of
    sharpness scores in a video before choosing a blur_threshold for
    sample_frames().

    Args:
        video_path: Path to the input video file.
        sample_interval: Only score every Nth frame (matches the
            sampling density you intend to extract at).
    """
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise IOError(f"Could not open video: {video_path}")

    frame_index = 0
    sampled_index = 0
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break

            if frame_index % sample_interval == 0:
                score = compute_blur_score(frame)
                print(f"sampled_frame={sampled_index:04d} "
                      f"source_frame={frame_index:06d} "
                      f"laplacian_var={score:.2f}")
                sampled_index += 1

            frame_index += 1
    finally:
        cap.release()


def sample_frames(video_path, output_dir="frames", sample_interval=5,
                   blur_threshold=100.0, jpeg_quality=95):
    """Sample frames from a video, drop blurry ones, and save the rest.

    Single entry point for the whole extraction step: reads video_path with
    cv2.VideoCapture, keeps every sample_interval-th frame, scores each kept
    candidate with the variance of its Laplacian, and discards frames
    scoring below blur_threshold. Surviving frames are written to
    output_dir as zero-padded, sequentially numbered JPEGs (frame_0000.jpg,
    frame_0001.jpg, ...) so sorting filenames preserves chronological
    order.

    Call this with just a video_path before your publisher node starts
    spinning, e.g. sample_frames("input.mov"), to produce a frame folder
    ready for VO or COLMAP.

    Args:
        video_path: Path to the input video file.
        output_dir: Directory to write kept frames into (created if
            missing). Defaults to "./frames".
        sample_interval: Keep every Nth frame from the source video.
            Lower values (e.g. 1-3) suit VO; higher values suit SfM.
        blur_threshold: Minimum acceptable Laplacian variance. Frames
            scoring below this are dropped as too blurry.
        jpeg_quality: JPEG encoding quality (0-100) for saved frames.

    Returns:
        dict with keys 'frames_read', 'frames_sampled', 'frames_kept',
        and 'frames_dropped_blur'.
    """
    os.makedirs(output_dir, exist_ok=True)

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise IOError(f"Could not open video: {video_path}")

    frames_read = 0
    frames_sampled = 0
    frames_kept = 0
    frames_dropped_blur = 0

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            frames_read += 1
            frame_index = frames_read - 1

            if frame_index % sample_interval != 0:
                continue
            frames_sampled += 1

            score = compute_blur_score(frame)
            if score < blur_threshold:
                frames_dropped_blur += 1
                continue

            out_path = os.path.join(
                output_dir, f"frame_{frames_kept:04d}.jpg")
            cv2.imwrite(
                out_path, frame,
                [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
            frames_kept += 1
    finally:
        cap.release()

    summary = {
        "frames_read": frames_read,
        "frames_sampled": frames_sampled,
        "frames_kept": frames_kept,
        "frames_dropped_blur": frames_dropped_blur,
    }
    print(
        f"Read {summary['frames_read']} frames, "
        f"sampled {summary['frames_sampled']}, "
        f"kept {summary['frames_kept']}, "
        f"dropped {summary['frames_dropped_blur']} for blur."
    )
    return summary


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Extract sharp frames from a video for VO/SfM.")
    parser.add_argument("video_path", help="Path to the input video file.")
    parser.add_argument(
        "output_dir", nargs="?", default="frames",
        help="Directory to save kept frames into (default: ./frames).")
    parser.add_argument(
        "--interval", type=int, default=5,
        help="Keep every Nth frame (default: 5).")
    parser.add_argument(
        "--blur-threshold", type=float, default=100.0,
        help="Minimum Laplacian variance to keep a frame (default: 100.0).")
    parser.add_argument(
        "--jpeg-quality", type=int, default=95,
        help="JPEG quality for saved frames, 0-100 (default: 95).")
    parser.add_argument(
        "--inspect", action="store_true",
        help="Only print Laplacian-variance scores; save nothing. "
             "Use this first to pick a --blur-threshold.")
    args = parser.parse_args()

    if args.inspect:
        inspect_blur_scores(args.video_path, sample_interval=args.interval)
    else:
        sample_frames(
            args.video_path,
            args.output_dir,
            sample_interval=args.interval,
            blur_threshold=args.blur_threshold,
            jpeg_quality=args.jpeg_quality,
        )
