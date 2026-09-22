import argparse
import csv
from pathlib import Path
import pandas as pd


def convert_txt_to_csv(input_file: Path, output_file: Path) -> None:
    routing_result = input_file.read_text(encoding="utf-8")
    if not routing_result.strip():
        raise ValueError("Input route file is empty")
    routing_result = (
        routing_result.replace("\r\n", "\n").replace("\r", "\n").replace("\n", "\\n")
    )

    output_file.parent.mkdir(parents=True, exist_ok=True)
    with output_file.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["id", "routing_result"])
        writer.writerow([0, routing_result])

    df = pd.read_csv(output_file, encoding="cp950")

    df.to_csv(output_file, index=False, encoding="utf-8")


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Convert route text into a CSV with id and routing_result columns."
    )
    parser.add_argument(
        "input", nargs="?", default="case1.txt", help="Input route file"
    )
    parser.add_argument(
        "output", nargs="?", default="case2.csv", help="Output CSV file"
    )
    return parser


def main() -> None:
    parser = build_arg_parser()
    args = parser.parse_args()
    convert_txt_to_csv(Path(args.input), Path(args.output))


if __name__ == "__main__":
    main()
