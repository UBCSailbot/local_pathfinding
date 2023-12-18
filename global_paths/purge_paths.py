"""This is a script to delete all generated path csv files in the global_paths directory.
It will devour any csv file that ends with a timestamp or contains a specified keyword.
"""
import argparse
import os
import re


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--key",
        nargs=1,
        type=str,
        default=None,
        help="keyword to search for in the names of files to be deleted",
    )
    args = parser.parse_args()
    delete_files(key=args.key)


def delete_files(key=None):
    dir_path = "/workspaces/sailbot_workspace/src/local_pathfinding/global_paths"
    files = os.listdir(dir_path)

    timestamp_pattern = re.compile(r"_\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}\.csv")

    # check if a keyword was entered
    if key is not None:
        key = re.compile(rf"{key}")

    for file_name in files:
        if re.search(timestamp_pattern, file_name) or (
            key is not None and re.search(key, file_name)
        ):
            file_path = os.path.join(dir_path, file_name)
            try:
                os.remove(file_path)
                print(f"Deleted: {os.path.basename(file_path)} from /global_paths")
            except OSError as e:
                print(f"Error deleting {file_path}: {e}")


if __name__ == "__main__":
    main()
