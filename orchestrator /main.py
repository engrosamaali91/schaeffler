# main.py
import argparse

def main():
    parser = argparse.ArgumentParser(description="Simple script that greets a user.")
    parser.add_argument("--name", required=True, help="Your name")
    args = parser.parse_args()

    print(f"👋 Hello, {args.name}! This message is from main.py")

if __name__ == "__main__":
    main()
