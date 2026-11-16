#!/bin/bash
# Doi video (webm VP8 cua record_camera / man_hinh.webm, mp4v cu) sang MP4 H.264 + yuv420p:
# xem duoc tren dien thoai, Windows, Zalo/Messenger, PowerPoint. Ghi <ten>_chia_se.mp4 canh file goc.
# ffmpeg: ban dong goi trong imageio-ffmpeg (env conda graspnet), may khong co ffmpeg he thong.
# Dung: tools/chia_se_video.sh FILE [FILE...]
FF=$(~/miniconda3/envs/graspnet/bin/python -c "import imageio_ffmpeg as f; print(f.get_ffmpeg_exe())") || exit 1
for src in "$@"; do
    dst="${src%.*}_chia_se.mp4"
    # Most compatible: H.264 Constrained Baseline, 30 fps, plus a silent AAC track (some phone
    # apps refuse or mis-play video-only files; the 15 fps High-profile version did not play for a friend).
    "$FF" -hide_banner -loglevel error -y -i "$src" -f lavfi -i anullsrc=channel_layout=stereo:sample_rate=44100 \
        -shortest -vf "scale=trunc(iw/2)*2:trunc(ih/2)*2,fps=30" -c:v libx264 -preset medium -crf 21 \
        -profile:v baseline -level 4.0 -pix_fmt yuv420p -c:a aac -b:a 64k -movflags +faststart "$dst" \
        && echo "$dst ($(du -h "$dst" | cut -f1))"
done
