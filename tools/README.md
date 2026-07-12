# MaixCAM Shape Tuner

## One-click live MaixCAM tuning

1. Copy `maixcam_stream.py` to MaixCAM and run it there.
2. Connect MaixCAM and the PC to the same Wi-Fi or phone hotspot.
3. Double-click `C:\Users\28658\Desktop\888\start_visual_tuner.bat` on the PC.

The PC discovers the camera automatically and opens the detection preview,
binary mask, and slider window. If LAN broadcast discovery is blocked, enter
`http://MAIXCAM_IP:8080/stream.mjpg` in the Video source box and click Connect.

This desktop tool mirrors the current MaixCAM shape-classification pipeline while keeping the ROI fixed.

It opens three windows:

- `MaixCAM - Binary Mask`
- `MaixCAM - Detection Preview`
- `MaixCAM Shape Tuner`

Run with a USB/UVC camera:

```powershell
python tools/maixcam_tuner.py --source 0
```

Run with an RTSP/HTTP stream or a video file:

```powershell
python tools/maixcam_tuner.py --source "rtsp://camera-address/stream"
python tools/maixcam_tuner.py --source "sample.mp4"
```

Run without a camera:

```powershell
python tools/maixcam_tuner.py --source demo
```

`Save` writes `maixcam_tuner_settings.json`. `Export Python` writes a parameter block to
`maixcam_tuned_parameters.py` in this directory.

The existing 9600-baud UART link only carries shape text and cannot carry live video. The PC must be
able to open the MaixCAM as a UVC device or consume a network video stream from it.
