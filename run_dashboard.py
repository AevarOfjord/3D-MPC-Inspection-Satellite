import uvicorn

if __name__ == "__main__":
    uvicorn.run(
        "satellite_control.dashboard.app:app",
        host="127.0.0.1",
        port=8000,
        reload=True,
        reload_dirs=["src"],
    )
