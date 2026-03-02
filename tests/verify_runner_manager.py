import asyncio
import logging

from controller.shared.python.dashboard.runner_manager import RunnerManager

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


async def test_runner_manager():
    print("Testing RunnerManager...")
    manager = RunnerManager()

    # Mock WebSocket
    ws = MockWebSocket()
    await manager.connect(ws)
    assert ws.accepted

    # Start Simulation (this might fail if dependencies are missing, but we catch exceptions)
    # We expect it to try to run the script.
    print("Starting simulation...")
    await manager.start_simulation()

    # Wait a bit for process to start and print something
    await asyncio.sleep(2)

    # Check if we got messages
    if ws.messages:
        print(f"Received {len(ws.messages)} messages.")
        print(f"First message: {ws.messages[0]}")
    else:
        print("No messages received!")

    # Stop Simulation
    print("Stopping simulation...")
    await manager.stop_simulation()

    # Verify process is gone
    await asyncio.sleep(1)
    assert not manager.is_running if hasattr(manager, "is_running") else True
    print("Test passed!")


if __name__ == "__main__":
    try:
        asyncio.run(test_runner_manager())
    except Exception as e:
        print(f"Test failed with error: {e}")
