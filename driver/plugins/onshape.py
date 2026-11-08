import asyncio
import json
import math
import random
import string
import subprocess
import tempfile
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Any, AsyncIterator, Callable, ClassVar, NamedTuple, Optional, Type

import numpy as np
import customtkinter as ctk
import uvicorn
from fastapi import FastAPI, WebSocket
from fastapi.middleware.cors import CORSMiddleware
from scipy.spatial import transform


SERVER_HOST = "127.51.68.120"
SERVER_PORT = 8181
PROTOCOL_VERSION = "1.4.8.21486"
WAMP_SERVER_ID = "HexaMouse Onshape plugin"
MOUSE_ID = "mouse0"
CONTROLLER_ID = "controller0"
CONTROLLER_URI = f"wss://{SERVER_HOST}/3dconnexion3dcontroller/{CONTROLLER_ID}"
ONSHAPE_UPDATE_URI = f"wss://{SERVER_HOST}/3dconnexion#update"
ORBIT_SENSITIVITY = 0.08
TRANSLATION_SENSITIVITY = 0.0005
ZOOM_SENSITIVITY = 0.0002
CERT_DIR = Path(tempfile.gettempdir()) / "hexamouse-onshape"
CERT_FILE = CERT_DIR / "ip.crt"
KEY_FILE = CERT_DIR / "ip.key"

ORIGINS = [
    f"https://{SERVER_HOST}",
    f"https://{SERVER_HOST}:{SERVER_PORT}",
    "https://3dconnexion.com",
    "https://cad.onshape.com",
]

app = FastAPI()
app.add_middleware(
    CORSMiddleware,
    allow_origins=ORIGINS,
    allow_origin_regex=r"https://([a-z0-9-]+\.)+onshape\.com$",
    allow_methods=["*"],
    allow_headers=["*"],
)
input_subscribers: set[asyncio.Queue["MotionEvent"]] = set()
active_listener: dict[str, Any] | None = None
server_loop: asyncio.AbstractEventLoop | None = None


@dataclass
class MotionEvent:
    x: int
    y: int
    z: int
    pitch: int
    yaw: int
    roll: int
    period: int = 16
    type: str = "mtn"


class WampMessage(tuple[Any, ...]):
    registry: ClassVar[dict[int, Type["WampMessage"]]] = {}
    message_type: ClassVar[int]

    def __init_subclass__(cls):
        WampMessage.registry[cls.message_type] = cls

    def serialize(self):
        return list(self)

    def wire(self):
        return [self.message_type, *self.serialize()]


class Welcome(NamedTuple("WelcomeBase", [("session_id", str), ("version", int), ("server_ident", str)]), WampMessage):
    message_type: ClassVar[int] = 0


class Prefix(NamedTuple("PrefixBase", [("prefix", str), ("uri", str)]), WampMessage):
    message_type: ClassVar[int] = 1


class Call(NamedTuple("CallBase", [("call_id", str), ("proc_uri", str), ("args", list[Any])]), WampMessage):
    message_type: ClassVar[int] = 2

    def __new__(cls, call_id, proc_uri, *args):
        return super().__new__(cls, call_id, proc_uri, list(args))

    def serialize(self):
        return [self.call_id, self.proc_uri, *self.args]

    @classmethod
    def create(cls, proc_uri, *args):
        return cls("".join(random.choices(string.ascii_uppercase + string.digits, k=18)), proc_uri, *args)


class CallResult(NamedTuple("CallResultBase", [("call_id", str), ("result", Any)]), WampMessage):
    message_type: ClassVar[int] = 3


class CallError(NamedTuple("CallErrorBase", [("call_id", str), ("error_uri", str), ("desc", str), ("details", Optional[Any])]), WampMessage):
    message_type: ClassVar[int] = 4


class Subscribe(NamedTuple("SubscribeBase", [("topic", str)]), WampMessage):
    message_type: ClassVar[int] = 5


class Unsubscribe(NamedTuple("UnsubscribeBase", [("topic", str)]), WampMessage):
    message_type: ClassVar[int] = 6


class Event(NamedTuple("EventBase", [("topic", str), ("payload", Any)]), WampMessage):
    message_type: ClassVar[int] = 8


class WampProtocol:
    def __init__(self, websocket: WebSocket):
        self.websocket = websocket
        self.prefixes = {}
        self.call_handlers: dict[str, Callable] = {}
        self.subscribe_handlers: dict[str, Callable] = {}
        self.unsubscribe_handlers: dict[str, Callable] = {}

    async def begin(self):
        await self.websocket.accept(subprotocol="wamp")
        session_id = "".join(random.choices(string.ascii_uppercase + string.digits, k=16))
        await self.send(Welcome(session_id, 1, WAMP_SERVER_ID))

    async def send(self, message: WampMessage):
        await self.websocket.send_json(message.wire())

    async def next(self):
        data = await self.websocket.receive_json()
        return WampMessage.registry[data[0]](*data[1:])

    def resolve(self, uri):
        prefix, rest = uri.split(":", 1)
        return self.prefixes.get(prefix, "") + rest


class WampSession:
    def __init__(self, websocket: WebSocket):
        self.wamp = WampProtocol(websocket)
        self.pending = {}

    async def messages(self):
        while True:
            message = await self.wamp.next()
            if isinstance(message, Prefix):
                self.wamp.prefixes[message.prefix] = message.uri
            elif isinstance(message, Subscribe):
                handler = self.wamp.subscribe_handlers.get(self.wamp.resolve(message.topic))
                if handler:
                    await handler(message)
            elif isinstance(message, Unsubscribe):
                handler = self.wamp.unsubscribe_handlers.get(self.wamp.resolve(message.topic))
                if handler:
                    await handler(message)
            elif isinstance(message, Call):
                handler = self.wamp.call_handlers.get(self.wamp.resolve(message.proc_uri))
                if handler:
                    await self.wamp.send(CallResult(message.call_id, await handler(*message.args)))
            elif isinstance(message, CallResult):
                pending = self.pending.get(message.call_id)
                if pending:
                    pending["result"] = message.result
                    pending["done"].set()
            elif isinstance(message, CallError):
                pending = self.pending.get(message.call_id)
                if pending:
                    pending["error"] = message.desc
                    pending["done"].set()

    async def rpc(self, controller_uri, method, *args):
        call = Call.create(method, "", *args)
        pending = {"done": asyncio.Event(), "result": None, "error": None}
        self.pending[call.call_id] = pending
        await self.wamp.send(Event(controller_uri, call.wire()))
        await pending["done"].wait()
        del self.pending[call.call_id]
        if pending["error"]:
            raise ValueError(pending["error"])
        return pending["result"]


class Controller:
    def __init__(self, events: AsyncIterator[MotionEvent], session: WampSession, metadata: dict):
        self.events = events
        self.session = session
        self.metadata = metadata
        self.subscribed = False
        self.focus = False
        self.uri = CONTROLLER_URI
        session.wamp.subscribe_handlers[self.uri] = self.subscribe
        session.wamp.unsubscribe_handlers[self.uri] = self.unsubscribe
        session.wamp.call_handlers[ONSHAPE_UPDATE_URI] = self.update_state

    async def subscribe(self, _message):
        self.subscribed = True
        self.focus = True
        await publish_listener({"name": self.metadata.get("name"), "version": self.metadata.get("version"), "controller": CONTROLLER_ID, "focus": True})

    async def unsubscribe(self, _message):
        self.subscribed = False
        self.focus = False
        await publish_listener({"name": self.metadata.get("name"), "version": self.metadata.get("version"), "controller": CONTROLLER_ID, "focus": False, "registered": False})

    async def deregister(self):
        if self.subscribed:
            await self.unsubscribe(None)

    async def update_state(self, _controller_id, args):
        if args.get("focus") is not None:
            self.focus = args["focus"]

    async def rpc(self, method, *args):
        return await self.session.rpc(self.uri, method, *args)

    async def run(self):
        async for event in self.events:
            if self.focus and self.subscribed and self.metadata.get("name") in ("Onshape", "WebThreeJS Sample"):
                try:
                    await self.update_client(event)
                except Exception as error:
                    print(f"[OnshapePlugin] Camera update failed: {error}")

    async def update_client(self, event: MotionEvent):
        extents = await self.rpc("self:read", "model.extents")
        perspective = await self.rpc("self:read", "view.perspective")
        affine = np.asarray(await self.rpc("self:read", "view.affine"), dtype=np.float32).reshape(4, 4)
        camera_rotation = np.linalg.svd(affine[:3, :3].T)[0] @ np.linalg.svd(affine[:3, :3].T)[2]
        angles = np.array([event.pitch, event.yaw, -event.roll]) * ORBIT_SENSITIVITY
        rotation = camera_rotation @ transform.Rotation.from_euler("xyz", angles, degrees=True).as_matrix() @ camera_rotation.T
        rotation_delta = np.eye(4, dtype=np.float32)
        rotation_delta[:3, :3] = rotation
        translation = np.eye(4, dtype=np.float32)
        translation[3, :3] = np.array([-event.x, -event.z, event.y], dtype=np.float32) * TRANSLATION_SENSITIVITY
        pivot = (np.array(extents[:3]) + np.array(extents[3:])) * 0.5
        pivot_matrix = np.eye(4, dtype=np.float32)
        pivot_matrix[3, :3] = pivot
        inverse_pivot = np.eye(4, dtype=np.float32)
        inverse_pivot[3, :3] = -pivot
        new_affine = translation @ affine @ (inverse_pivot @ rotation_delta @ pivot_matrix)
        if not perspective:
            current_extents = await self.rpc("self:read", "view.extents")
            scale = 1.0 + event.y * ZOOM_SENSITIVITY
            await self.rpc("self:update", "view.extents", [value * scale for value in current_extents])
        await self.rpc("self:update", "motion", True)
        await self.rpc("self:update", "view.affine", new_affine.reshape(-1).tolist())


async def publish_listener(details: dict[str, Any]):
    global active_listener
    active_listener = details


async def create_controller(session: WampSession, events: AsyncIterator[MotionEvent]):
    await session.wamp.begin()
    message = await session.wamp.next()
    while isinstance(message, Prefix):
        session.wamp.prefixes[message.prefix] = message.uri
        message = await session.wamp.next()
    await session.wamp.send(CallResult(message.call_id, {"connexion": MOUSE_ID}))
    message = await session.wamp.next()
    controller = Controller(events, session, message.args[2])
    await session.wamp.send(CallResult(message.call_id, {"instance": CONTROLLER_ID}))
    return controller


@app.get("/3dconnexion/nlproxy")
async def get_info():
    return {"port": SERVER_PORT, "version": PROTOCOL_VERSION}


async def publish_event(event: MotionEvent):
    for queue in tuple(input_subscribers):
        while not queue.empty():
            try:
                queue.get_nowait()
            except asyncio.QueueEmpty:
                break
        queue.put_nowait(event)


@app.on_event("startup")
async def startup_server():
    global server_loop
    server_loop = asyncio.get_running_loop()


@app.on_event("shutdown")
async def shutdown_server():
    global server_loop
    server_loop = None


async def get_shared_input_events():
    queue: asyncio.Queue[MotionEvent] = asyncio.Queue()
    input_subscribers.add(queue)
    try:
        while True:
            yield await queue.get()
    finally:
        input_subscribers.discard(queue)


@app.websocket("/")
async def nlproxy(ws: WebSocket):
    session = WampSession(ws)
    controller = await create_controller(session, get_shared_input_events())
    try:
        async with asyncio.TaskGroup() as task_group:
            task_group.create_task(controller.run())
            task_group.create_task(session.messages())
    finally:
        await controller.deregister()


def ensure_certificate():
    if CERT_FILE.exists() and KEY_FILE.exists():
        return
    CERT_DIR.mkdir(parents=True, exist_ok=True)
    subprocess.run([
        "openssl", "req", "-x509", "-newkey", "rsa:2048", "-nodes", "-days", "3650",
        "-keyout", str(KEY_FILE), "-out", str(CERT_FILE), "-subj", f"/CN={SERVER_HOST}",
        "-addext", f"subjectAltName=IP:{SERVER_HOST}",
    ], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


class Plugin:
    def __init__(self):
        self.deadzone = 30.0
        self.rotation_deadzone = 45.0
        self.sensitivity = 0.2
        self.server = None
        self.server_thread = None
        self.started = False
        self.start()

    def start(self):
        if self.started:
            return
        ensure_certificate()
        config = uvicorn.Config(app, host=SERVER_HOST, port=SERVER_PORT, ws="auto", ssl_certfile=CERT_FILE, ssl_keyfile=KEY_FILE, log_level="warning")
        self.server = uvicorn.Server(config)
        self.server_thread = threading.Thread(target=self.server.run, name="onshape-server", daemon=True)
        self.server_thread.start()
        self.started = True
        print(f"[OnshapePlugin] Onshape server: https://{SERVER_HOST}:{SERVER_PORT}")

    def build_settings(self, parent):
        settings = ctk.CTkFrame(parent, fg_color="transparent")
        settings.pack(fill="x", padx=12, pady=(0, 6))

        deadzone_value = ctk.StringVar(value=f"{self.deadzone:.0f}")
        rotation_deadzone_value = ctk.StringVar(value=f"{self.rotation_deadzone:.0f}")
        sensitivity_value = ctk.StringVar(value=f"{self.sensitivity:.1f}x")

        ctk.CTkLabel(settings, text="Translation", width=72, anchor="w", font=("Helvetica", 10)).grid(row=0, column=0, padx=(0, 6), sticky="w")
        deadzone_slider = ctk.CTkSlider(settings, from_=0, to=50, number_of_steps=50, command=self._set_deadzone)
        deadzone_slider.set(self.deadzone)
        deadzone_slider.grid(row=0, column=1, sticky="ew")
        ctk.CTkLabel(settings, textvariable=deadzone_value, width=42, anchor="e", font=("Helvetica", 10)).grid(row=0, column=2, padx=(6, 0))

        ctk.CTkLabel(settings, text="Rotation", width=72, anchor="w", font=("Helvetica", 10)).grid(row=1, column=0, padx=(0, 6), sticky="w")
        rotation_deadzone_slider = ctk.CTkSlider(settings, from_=0, to=50, number_of_steps=50, command=self._set_rotation_deadzone)
        rotation_deadzone_slider.set(self.rotation_deadzone)
        rotation_deadzone_slider.grid(row=1, column=1, sticky="ew")
        ctk.CTkLabel(settings, textvariable=rotation_deadzone_value, width=42, anchor="e", font=("Helvetica", 10)).grid(row=1, column=2, padx=(6, 0))

        ctk.CTkLabel(settings, text="Sensitivity", width=72, anchor="w", font=("Helvetica", 10)).grid(row=2, column=0, padx=(0, 6), sticky="w")
        sensitivity_slider = ctk.CTkSlider(settings, from_=0.1, to=3.0, number_of_steps=29, command=self._set_sensitivity)
        sensitivity_slider.set(self.sensitivity)
        sensitivity_slider.grid(row=2, column=1, sticky="ew")
        ctk.CTkLabel(settings, textvariable=sensitivity_value, width=42, anchor="e", font=("Helvetica", 10)).grid(row=2, column=2, padx=(6, 0))

        settings.columnconfigure(1, weight=1)
        self._deadzone_display = deadzone_value
        self._rotation_deadzone_display = rotation_deadzone_value
        self._sensitivity_display = sensitivity_value

    def _set_deadzone(self, value):
        self.deadzone = float(value)
        self._deadzone_display.set(f"{self.deadzone:.0f}")

    def _set_sensitivity(self, value):
        self.sensitivity = float(value)
        self._sensitivity_display.set(f"{self.sensitivity:.1f}x")

    def _set_rotation_deadzone(self, value):
        self.rotation_deadzone = float(value)
        self._rotation_deadzone_display.set(f"{self.rotation_deadzone:.0f}")

    def stop(self):
        if self.server is not None:
            self.server.should_exit = True
        if self.server_thread is not None:
            self.server_thread.join(timeout=2)
        self.server = None
        self.server_thread = None
        self.started = False

    def on_packet(self, data):
        event = self._event_from_packet(data)
        if event is None:
            return
        self._publish_direct(event)

    def _publish_direct(self, event):
        loop = server_loop
        if loop is not None and loop.is_running():
            asyncio.run_coroutine_threadsafe(publish_event(event), loop)

    def _event_from_packet(self, data):
        names = ("X", "Y", "Z", "TiltX", "TiltY", "Yaw")
        try:
            values = [float(data.get(name, 0)) for name in names]
        except (TypeError, ValueError):
            return None
        if not all(math.isfinite(value) for value in values) or not any(values):
            return None
        event = dict(zip(("x", "z", "y", "pitch", "yaw", "roll"), values))
        for axis in ("x", "y", "z"):
            event[axis] = 0 if abs(event[axis]) <= self.deadzone else int(event[axis] * self.sensitivity)
        for axis in ("pitch", "yaw", "roll"):
            event[axis] = 0 if abs(event[axis]) <= self.rotation_deadzone else int(event[axis] * self.sensitivity)
        if not any(event.values()):
            return None
        if not any(event[axis] for axis in ("pitch", "yaw", "roll")):
            event["z"] = event["y"]
            event["y"] = 0
        return MotionEvent(**event)
