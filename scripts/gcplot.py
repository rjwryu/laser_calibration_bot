#!/usr/bin/env python
# Author: Su Jing Long, Brian
desc = """\
Plot motor data collection results, a graph of feedback current vs angular
position, over a full 360°.
"""

import sys, argparse
from typing import TextIO
import pandas as pd
import matplotlib.pyplot as plt


def main(input_file: TextIO, output_file: TextIO):
    df = pd.read_csv(input_file)
    print(df)

    plt.figure(figsize=(10, 6))
    plt.plot(df["shaft_angle"], df["current"], 'b.')

    plt.xlabel("Motor Shaft Angle")
    plt.ylabel("Feedback Current")
    plt.title("Feedback Current vs. Motor Shaft Angle")
    plt.grid(True, linestyle="--", alpha=0.7)

    plt.savefig(output_file or "current_vs_angle.png")
    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description = desc)
    parser.add_argument(
        "-f", "--file",
        type=argparse.FileType("r"),
        default=sys.stdin,
        help="Path to the input file (defaults to stdin)"
    )
    parser.add_argument(
        "-o", "--output",
        type=argparse.FileType("w"),
        help="Path to the output file"
    )

    args = parser.parse_args()
    main(args.file, args.output)
