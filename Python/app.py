from bridge import Bridge
import threading

def main():
    bridge: Bridge = Bridge()

    thread: threading.Thread | None = None

    while True:
        command: str = input("Enter command:: ")

        if command == "help":
            print("help:: run -> start bridge, stop -> stop bridge")
        elif command == "run":
            if not thread:
                thread = threading.Thread(target=bridge.start(), daemon=True)
                thread.start()
            else:
                print("Already running.")
        elif command == "stop":
            if thread:
                thread.join()
            bridge.stop()
        elif command == "exit":
            break
        else:
            print("Unrecognized command")

if __name__ == "__main__":
    main()