import argparse

import uvicorn


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run Satellite Control dashboard backend."
    )
    parser.add_argument(
        "--host", default="127.0.0.1", help="Host to bind (default: 127.0.0.1)"
    )
    parser.add_argument(
        "--port", type=int, default=8000, help="Port to bind (default: 8000)"
    )
    parser.add_argument(
        "--dev",
        action="store_true",
        help="Enable auto-reload for local development.",
    )
    args = parser.parse_args()

    uvicorn.run(
        "satellite_control.dashboard.app:app",
        host=args.host,
        port=args.port,
        reload=args.dev,
        reload_dirs=["src"] if args.dev else None,
    )


if __name__ == "__main__":
    main()
