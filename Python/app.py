from bridge import Bridge
import threading

def main():
    bridge: Bridge = Bridge()

    thread: threading.Thread | None = None

    command: str = input("Enter command:: ")

    if command == "help":
        print("help:: run -> start bridge, stop -> stop bridge")
    elif command == "run":
        thread = threading.Thread(target=Bridge.start(), daemon=True)
        thread.start()
    elif command == "stop":
        if thread:
            thread.join()
        Bridge.stop()
    else:
        print("Unrecognized command")

if __name__ == "__main__":
    main()