import asyncio
import logging

from dashboard.runner_manager import RunnerManager

# Configure logging
logging.basicConfig(level=logging.INFO)


class MockWebSocket:
    def __init__(self):
        self.accepted = False
        self.messages = []
        self.closed = False

    async def accept(self):
        self.accepted = True
        print("WS Accepted")

    async def send_text(self, text):
        self.messages.append(text)
        print(f"WS Received: {text}")

    async def close(self):
        self.closed = True
        print("WS Closed")


async def test_runner_mission_selection():
    print("Testing RunnerManager Mission Selection...")
    manager = RunnerManager()

    # Mock WebSocket
    ws = MockWebSocket()
    await manager.connect(ws)

    mission_name = "Starlink_Scan_M00"
    print(f"Starting simulation with mission: {mission_name}...")

    # This should fail if mission doesn't exist, or pass if it does.
    # We are testing the argument passing logic.
    try:
        await manager.start_simulation(mission_name)
    except Exception as e:
        print(f"Start failed (expected if path invalid, but checking logic): {e}")

    # Wait a bit
    await asyncio.sleep(2)

    # Verify messages contain the selected mission log
    found_mission_log = False
    for msg in ws.messages:
        if f"Selected mission: {mission_name}" in msg:
            found_mission_log = True
            break

    if found_mission_log:
        print("SUCCESS: Found mission selection log.")
    else:
        print("FAILURE: Did not find mission selection log.")
        print("Messages:", ws.messages)

    # Stop Simulation
    print("Stopping simulation...")
    await manager.stop_simulation()

    # Verify process is gone
    await asyncio.sleep(1)


if __name__ == "__main__":
    try:
        asyncio.run(test_runner_mission_selection())
    except Exception as e:
        print(f"Test failed with error: {e}")
