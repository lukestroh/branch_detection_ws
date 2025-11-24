#!/usr/bin/env python3
import os
import box_client.file_manager as fm
import pytest
import shutil


def test_move_file(file_manager: fm.FileManager, temp_dir: str):
    source_file = f"{temp_dir}/test_source_file.txt"
    dest_file = f"{temp_dir}/test_dest_file.txt"

    # Create source file for testing
    with open(source_file, "w") as f:
        f.write("This is a test file.")

    # Test moving file
    move_success = file_manager.move(source_file, dest_file)
    assert move_success, "Failed to move file."

    # Verify the file was moved
    assert not os.path.exists(source_file), "Source file still exists after move."
    assert os.path.exists(dest_file), "Destination file does not exist after move."

    # # Clean up
    # os.remove(dest_file)

    return


def test_move_folder(file_manager: fm.FileManager, temp_dir: str):
    source_folder = f"{temp_dir}/test_source_folder"
    dest_folder = f"{temp_dir}/test_dest_folder"

    # Create source folder for testing
    os.makedirs(source_folder, exist_ok=True)

    # Test moving folder
    move_success = file_manager.move(source_folder, dest_folder)
    assert move_success, "Failed to move folder."

    # Verify the folder was moved
    assert not os.path.exists(source_folder), "Source folder still exists after move."
    assert os.path.exists(dest_folder), "Destination folder does not exist after move."

    # Clean up
    shutil.rmtree(dest_folder, ignore_errors=True)

    return


def test_move_special_folder(file_manager: fm.FileManager, temp_dir: str):
    special_folder = file_manager.staging_dir
    print(special_folder)
    dest_folder = f"{temp_dir}/test_dest_folder"

    # Test moving special folder
    move_success = file_manager.move(special_folder, dest_folder)
    assert not move_success, "Special folder move should have failed."

    # Verify the special folder was not moved
    assert os.path.exists(special_folder), "Special folder should not have been moved."
    assert not os.path.exists(dest_folder), "Destination folder should not exist."

    return