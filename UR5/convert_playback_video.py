#!/usr/bin/env python3
"""把 MATLAB 录出的 Motion JPEG AVI 转成 MP4 和 GIF。

背景（见 AGENT_HANDOFF.md 第 64.3 节）：
  本机 MATLAB R2025b 图形窗口无法显示（CEF/WebGL 渲染进程起不来），
  只能走 `-batch` 离屏录制；而 MATLAB 在本机只能写 Motion JPEG AVI，
  系统里又没有可用的 ffmpeg 可执行文件。所以用 OpenCV(cv2) 转 MP4、
  Pillow(PIL) 转 GIF。

编码器选择很重要：
  cv2 内置 FFMPEG，但本机 gstreamer 缺 H.264 和 MPEG-4 Part 2 解码器，
  所以 `totem` 播不了任何 MP4 —— 只有 Motion JPEG AVI 能播。
  MP4 优先用 H.264(avc1)：文件小、浏览器(Chrome)能直接播。
  fourcc 依次尝试 avc1 -> H264 -> mp4v。

用法:
    python3 convert_playback_video.py <input.avi> [--fps 30] [--scale 1.0]
                                      [--outdir DIR] [--no-gif]

播放建议:
    totem weld_corridor_motion.avi        # 本机唯一能播的格式
    weld_corridor_motion.mp4              # 用浏览器打开
    weld_corridor_motion.gif              # VSCode 里直接预览
"""

import argparse
import os
import sys

import cv2
from PIL import Image

# 按优先级排列；avc1 = H.264
MP4_FOURCCS = ("avc1", "H264", "mp4v")


def read_frames(path, scale):
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        sys.exit(f"打不开视频: {path}")

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    frames = []
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if scale != 1.0:
            frame = cv2.resize(
                frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA
            )
        frames.append(frame)
    cap.release()

    if not frames:
        sys.exit(f"没有读到任何帧: {path}")

    h, w = frames[0].shape[:2]
    print(f"读入 {len(frames)} 帧, {w}x{h}, fps={fps:.3f}")
    return frames, fps


def write_mp4(frames, fps, out_path):
    h, w = frames[0].shape[:2]

    for tag in MP4_FOURCCS:
        vw = cv2.VideoWriter(
            out_path, cv2.VideoWriter_fourcc(*tag), fps, (w, h)
        )
        if not vw.isOpened():
            continue
        for f in frames:
            vw.write(f)
        vw.release()

        if os.path.getsize(out_path) > 0:
            print(f"写出 MP4: {out_path}  "
                  f"({os.path.getsize(out_path)/1e6:.2f} MB, fourcc={tag})")
            return out_path

    print(f"  MP4 写出失败（试过 {MP4_FOURCCS}）: {out_path}")
    return None


def write_gif(frames, fps, out_path, max_width=800):
    imgs = []
    for f in frames:
        rgb = cv2.cvtColor(f, cv2.COLOR_BGR2RGB)
        im = Image.fromarray(rgb)

        if im.width > max_width:
            ratio = max_width / im.width
            im = im.resize((max_width, int(im.height * ratio)), Image.LANCZOS)

        # GIF 调色板量化
        imgs.append(im.convert("P", palette=Image.ADAPTIVE, colors=256))

    delay_ms = int(round(1000.0 / fps))
    imgs[0].save(
        out_path,
        save_all=True,
        append_images=imgs[1:],
        duration=delay_ms,
        loop=0,
        optimize=True,
    )
    print(f"写出 GIF: {out_path}  "
          f"({os.path.getsize(out_path)/1e6:.2f} MB, {imgs[0].size}, "
          f"{len(imgs)} 帧, delay={delay_ms}ms)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("input")
    ap.add_argument("--fps", type=float, default=None,
                    help="覆盖帧率（默认用文件里的）")
    ap.add_argument("--scale", type=float, default=1.0,
                    help="缩放系数，例如 0.5")
    ap.add_argument("--outdir", default=None)
    ap.add_argument("--no-gif", action="store_true")
    args = ap.parse_args()

    if not os.path.isfile(args.input):
        sys.exit(f"找不到文件: {args.input}")

    outdir = args.outdir or os.path.dirname(os.path.abspath(args.input))
    stem = os.path.splitext(os.path.basename(args.input))[0]

    frames, fps = read_frames(args.input, args.scale)
    if args.fps:
        fps = args.fps

    write_mp4(frames, fps, os.path.join(outdir, stem + ".mp4"))
    if not args.no_gif:
        write_gif(frames, fps, os.path.join(outdir, stem + ".gif"))


if __name__ == "__main__":
    main()
