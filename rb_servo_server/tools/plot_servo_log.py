#!/usr/bin/env python3
"""Plot basic rb_servo_server log timing and joint tracking."""

import argparse
import pandas as pd
import matplotlib.pyplot as plt


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", default="logs/servo_log.csv", nargs="?")
    args = parser.parse_args()

    df = pd.read_csv(args.log)

    if "period_ms" in df.columns:
        plt.figure()
        plt.plot(df["period_ms"])
        plt.xlabel("tick")
        plt.ylabel("period [ms]")
        plt.title("Servo loop period")
        plt.grid(True)
        plt.show()

    if "jitter_ms" in df.columns:
        plt.figure()
        plt.plot(df["jitter_ms"])
        plt.xlabel("tick")
        plt.ylabel("absolute jitter [ms]")
        plt.title("Servo loop absolute jitter")
        plt.grid(True)
        plt.show()

    for prefix, title in [("left", "Left"), ("right", "Right")]:
        actual_cols = [c for c in df.columns if c.startswith(f"{prefix}_q_actual_")]
        sent_cols = [c for c in df.columns if c.startswith(f"{prefix}_q_sent_")]
        if actual_cols:
            plt.figure()
            for c in actual_cols:
                plt.plot(df[c], label=c)
            plt.xlabel("tick")
            plt.ylabel("deg")
            plt.title(f"{title} q_actual")
            plt.legend()
            plt.grid(True)
            plt.show()
        if actual_cols and sent_cols:
            plt.figure()
            plt.plot(df[actual_cols[0]], label=f"{prefix} actual J0")
            plt.plot(df[sent_cols[0]], label=f"{prefix} sent J0")
            plt.xlabel("tick")
            plt.ylabel("deg")
            plt.title(f"{title} J0 actual vs sent")
            plt.legend()
            plt.grid(True)
            plt.show()


if __name__ == "__main__":
    main()
