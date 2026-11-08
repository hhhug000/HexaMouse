class Plugin:
    def __init__(self):
        print("[LoggerPlugin] Initialized")

    def on_packet(self, data):
        # Simply print the raw telemetry dictionary to the console
        print(f"[LoggerPlugin] Received packet -> X: {data.get('X', 0):.1f}, Y: {data.get('Y', 0):.1f}, Z: {data.get('Z', 0):.1f}, Yaw: {data.get('Yaw', 0):.1f}")
