#!/usr/bin/env python3

import pathlib
import urllib.request


def main():
    base_url = "http://192.168.71.1/camera"
    output_dir = pathlib.Path("/home/max/downloads/kbs/images")

    # Ensure the directory exists
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"Fetching images to {output_dir}...")

    for i in range(1, 300):
        filename = f"img_{i}.pgm"
        filepath = output_dir / filename

        try:
            with urllib.request.urlopen(base_url) as response:
                data = response.read()

            with open(filepath, "wb") as f:
                f.write(data)

            print(f"Saved: {filename}")

        except Exception as e:
            print(f"Error fetching image {i}: {e}")
            # Optional: break on error or continue. Here we continue.


if __name__ == "__main__":
    main()
