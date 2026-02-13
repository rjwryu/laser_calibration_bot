#!/usr/bin/env python
# Author: Su Jing Long, Brian
desc = """\
Script to plot the data gathered and the inferred torque constant.
"""

import pandas as pd
import matplotlib.pyplot as plt
import argparse
from scipy import stats


def plot_graph(infile: str):
    df = pd.read_csv(infile)

    slope, intercept, r_value, p_value, std_err = stats.linregress(df["added_mass"], df["current"])
    df["pred_current"] = slope * df["added_mass"] + intercept

    print(df)
    print()
    print(f"Slope = {slope:+.4f}")
    print(f"Intercept = {intercept:+.4f}")
    print(f"R-squared = {r_value**2:+.4f}")
    print()
    print(f"Torque constant = {9.81 * 0.225 / slope:+.4f} Nm/A")

    plt.plot(df["added_mass"], df["current"], "ro")
    plt.xlabel("added mass (kg)")
    plt.ylabel("current (A)")
    plt.title("torque constant")
    plt.grid(True)

    plt.plot(df["added_mass"], df["pred_current"], "b-")
    plt.show()


def main():
    parser = argparse.ArgumentParser(description = desc)
    parser.add_argument(
        "-i", "--input",
        default="output.csv",
        help="Name of data input file to read. Defaults to \"output.csv\"."
    )

    args = parser.parse_args()

    plot_graph(args.input)

if __name__ == "__main__":
    main()
