"""This is a script to delete all generated path csv files in the global_paths directory.
It will devour any csv file that ends with a timestamp.
"""
import os
import re


def main():
    delete_timestamped_files()


def delete_timestamped_files():
    dir_path = "/workspaces/sailbot_workspace/src/local_pathfinding/global_paths"
    # List all files in the directory
    files = os.listdir(dir_path)

    # Define a regular expression pattern to match filenames with timestamps at the end
    timestamp_pattern = re.compile(r"_\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2}\.csv")

    # Iterate through files and delete those with timestamps
    for file_name in files:
        if re.search(timestamp_pattern, file_name):
            file_path = os.path.join(dir_path, file_name)
            try:
                os.remove(file_path)
                print(f"Deleted: {os.path.basename(file_path)} from /global_paths")
            except OSError as e:
                print(f"Error deleting {file_path}: {e}")


if __name__ == "__main__":
    main()
