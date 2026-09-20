"""
server.py — the actual "Follower Server" from the resume.

Two halves running concurrently:
  1. UDP listener — receives raw thermal frames from the ESP32
  2. WebSocket broadcaster — pushes computed boresight/target data out
     to the GCS dashboard, same pattern as the existing telemetry
     WebSocket in gcs-ground-control, kept consistent on purpose.

Run standalone: `python -m app.server`
"""
import asyncio
import json
import logging
import time

from .thermal_tracker import frame_from_bytes, track

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
log = logging.getLogger("follower_server")

UDP_LISTEN_PORT = 14600          # thermal frames arrive here from the ESP32
WS_PORT = 14601                  # GCS dashboard connects here for target data
FRAME_STALE_S = 2.0              # no new frame this long -> report "no target" even
                                  # if the last computed blob was confident

# Shared state between the two halves — last known target, and every
# connected dashboard websocket to broadcast to.
_latest_target: dict | None = None
_last_frame_time: float = 0.0
_ws_clients: set = set()


class ThermalUDPProtocol(asyncio.DatagramProtocol):
    def connection_made(self, transport):
        self.transport = transport
        log.info(f"UDP listener bound, awaiting thermal frames on :{UDP_LISTEN_PORT}")

    def datagram_received(self, data: bytes, addr):
        global _latest_target, _last_frame_time
        try:
            frame = frame_from_bytes(data)
        except ValueError as e:
            log.warning(f"Malformed thermal frame from {addr}: {e}")
            return

        result = track(frame)
        _last_frame_time = time.monotonic()

        if result is None:
            _latest_target = None
        else:
            _latest_target = {
                "pan_deg": round(result.pan_deg, 2),
                "tilt_deg": round(result.tilt_deg, 2),
                "confidence": round(result.confidence, 2),
                "ts": _last_frame_time,
            }


async def broadcast_loop():
    """Pushes current target state to every connected dashboard at 10Hz."""
    while True:
        await asyncio.sleep(0.1)
        stale = (time.monotonic() - _last_frame_time) > FRAME_STALE_S
        payload = json.dumps({
            "target": None if stale else _latest_target,
            "stale": stale,
        })
        dead = set()
        for ws in _ws_clients:
            try:
                await ws.send(payload)
            except Exception:
                dead.add(ws)
        _ws_clients.difference_update(dead)


async def ws_handler(websocket):
    _ws_clients.add(websocket)
    log.info(f"Dashboard connected ({len(_ws_clients)} total)")
    try:
        async for _ in websocket:
            pass  # this channel is output-only; ignore anything a client sends
    finally:
        _ws_clients.discard(websocket)
        log.info(f"Dashboard disconnected ({len(_ws_clients)} total)")


async def main():
    import websockets

    loop = asyncio.get_running_loop()
    transport, _ = await loop.create_datagram_endpoint(
        ThermalUDPProtocol, local_addr=("0.0.0.0", UDP_LISTEN_PORT)
    )

    ws_server = await websockets.serve(ws_handler, "0.0.0.0", WS_PORT)
    log.info(f"WebSocket broadcaster on :{WS_PORT}")

    try:
        await broadcast_loop()
    finally:
        transport.close()
        ws_server.close()


if __name__ == "__main__":
    asyncio.run(main())
