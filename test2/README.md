# MaixCAM visual tuner

## Network mode

1. Copy `maixcam_stream.py` to MaixCAM and run it instead of the normal vision app.
2. Connect MaixCAM and the PC to the same Wi-Fi or phone hotspot.
3. Double-click `start_tuner.bat` on the PC.

The tuner discovers MaixCAM automatically. It opens a detection preview, a binary mask,
and a scrollable slider window. Slider changes take effect immediately on the PC preview.

Use `保存参数` to keep the current values. Use `导出 Python` to write
`maixcam_tuned_parameters.py`, then replace the corresponding parameter block in the
normal MaixCAM `main.py`.

## Other video sources

Run with a USB camera:

```powershell
python pc_visual_tuner.py --source 0
```

Run with an HTTP/RTSP stream or video file:

```powershell
python pc_visual_tuner.py --source "http://camera-address/stream.mjpg"
python pc_visual_tuner.py --source "sample.mp4"
```

Run without a camera to check the interface:

```powershell
python pc_visual_tuner.py --source demo
```
