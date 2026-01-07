import requests
import time
import subprocess
import signal
import os

def test_server():
    # Start the server in a subprocess
    process = subprocess.Popen([
        'uvicorn', 'src.api:app', '--host', '127.0.0.1', '--port', '8080', '--reload'
    ], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    # Give the server a moment to start
    time.sleep(3)

    try:
        # Test the health endpoint
        response = requests.get('http://127.0.0.1:8080/health')
        print(f"Health check status: {response.status_code}")
        print(f"Health check response: {response.json()}")

        # Test the root endpoint
        response = requests.get('http://127.0.0.1:8080/')
        print(f"Root endpoint status: {response.status_code}")
        print(f"Root endpoint response: {response.json()}")

        print("\nServer is running and responding to requests!")

    except requests.exceptions.ConnectionError:
        print("Could not connect to server. It may not have started properly.")
    except Exception as e:
        print(f"Error testing server: {e}")
    finally:
        # Terminate the server process
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()

if __name__ == "__main__":
    test_server()