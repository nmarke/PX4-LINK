from bridge import Bridge

def main():
    bridge: Bridge = Bridge()

    command: str = input("Enter command:: ")

    if command == "help":
        print("help:: run -> start bridge, stop -> stop bridge")
    elif command == "run":
        Bridge.start()
    elif command == "stop":
        Bridge.stop()
    else:
        print("Unrecognized command")

if __name__ == "__main__":
    main()