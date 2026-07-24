# pyright: reportMissingImports=false
"""MaixCAM camera streamer for the desktop shape tuning tool.

Run this file on MaixCAM, then launch start_tuner.bat on the PC.
Both devices must be connected to the same LAN or phone hotspot.
"""

from maix import app, camera, display, image, time as mtime
import _thread
import cv2
import socket


FRAME_W = 320
FRAME_H = 240
HTTP_PORT = 8080
DISCOVERY_PORT = 37020
DISCOVERY_MAGIC = b"MAIXCAM_TUNER"
JPEG_QUALITY = 80

latest_jpeg = None
latest_frame_id = 0
server_running = True


def send_all(client, data):
    view = memoryview(data)
    while len(view):
        sent = client.send(view)
        if sent <= 0:
            raise OSError("socket closed")
        view = view[sent:]


def stream_client(client):
    try:
        client.settimeout(3.0)
        request = client.recv(1024)
        if b"GET /snapshot.jpg" in request:
            frame = latest_jpeg
            if frame is None:
                send_all(client, b"HTTP/1.1 503 Service Unavailable\r\nConnection: close\r\n\r\n")
                return
            header = (
                b"HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\nContent-Length: "
                + str(len(frame)).encode()
                + b"\r\nConnection: close\r\n\r\n"
            )
            send_all(client, header + frame)
            return

        if b"GET /stream.mjpg" not in request and b"GET / " not in request:
            send_all(client, b"HTTP/1.1 404 Not Found\r\nConnection: close\r\n\r\n")
            return

        send_all(
            client,
            b"HTTP/1.1 200 OK\r\n"
            b"Cache-Control: no-store, no-cache, must-revalidate\r\n"
            b"Pragma: no-cache\r\n"
            b"Connection: close\r\n"
            b"Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n",
        )
        client.settimeout(5.0)
        sent_frame_id = -1
        while server_running:
            frame = latest_jpeg
            frame_id = latest_frame_id
            if frame is None or frame_id == sent_frame_id:
                mtime.sleep_ms(10)
                continue
            sent_frame_id = frame_id
            part = (
                b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
                + str(len(frame)).encode()
                + b"\r\n\r\n"
                + frame
                + b"\r\n"
            )
            send_all(client, part)
    except Exception:
        pass
    finally:
        try:
            client.close()
        except Exception:
            pass


def http_server():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", HTTP_PORT))
    server.listen(2)
    server.settimeout(1.0)
    while server_running:
        try:
            client, _ = server.accept()
            _thread.start_new_thread(stream_client, (client,))
        except Exception:
            pass
    server.close()


def discovery_beacon():
    beacon = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    beacon.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    payload = DISCOVERY_MAGIC + b" " + str(HTTP_PORT).encode()
    while server_running:
        try:
            beacon.sendto(payload, ("255.255.255.255", DISCOVERY_PORT))
        except Exception:
            pass
        mtime.sleep_ms(500)
    beacon.close()


def main():
    global latest_jpeg, latest_frame_id, server_running

    cam = camera.Camera(FRAME_W, FRAME_H, image.Format.FMT_RGB888)
    disp = display.Display()
    _thread.start_new_thread(http_server, ())
    _thread.start_new_thread(discovery_beacon, ())

    while not app.need_exit():
        frame_image = cam.read()
        rgb = image.image2cv(frame_image, ensure_bgr=False, copy=True)
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        ok, encoded = cv2.imencode(
            ".jpg", bgr, [int(cv2.IMWRITE_JPEG_QUALITY), JPEG_QUALITY]
        )
        if ok:
            latest_jpeg = encoded.tobytes()
            latest_frame_id += 1

        cv2.putText(
            rgb,
            "PC tuner server: port {}".format(HTTP_PORT),
            (5, 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 255, 0),
            1,
        )
        disp.show(image.cv2image(rgb, bgr=False, copy=False))

    server_running = False
    mtime.sleep_ms(100)


if __name__ == "__main__":
    main()
