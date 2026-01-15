#!/usr/bin/env python3
# Author: Su Jing Long, Brian
desc = """\
Process stdin as experimental data, fitting the current vs. angle data to a
sinusoid. Save this sinusoid parameters to a file.
"""

import argparse, sys
from typing import TextIO
import numpy as np
import pandas as pd


def main(input_file: TextIO, output_file: TextIO) -> None:
    df = pd.read_csv(input_file)

    # group by shaft angle and aggregate using mean of feedback current
    out_df = df.groupby("shaft_angle")["current"].mean().reset_index()

    out_df.to_csv(output_file, index=False)



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
        default=sys.stdout,
        help="Path to the output file (defaults to stdout)"
    )

    args = parser.parse_args()
    main(args.file, args.output)
