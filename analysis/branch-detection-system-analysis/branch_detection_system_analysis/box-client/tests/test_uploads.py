#!/usr/bin/env python3
import os
import glob
from xmlrpc import client
import box_sdk_gen as box
from box_client.box_auth_client import BoxAuthClient

import pytest

__here__ = os.path.dirname(__file__)


def test_big_file_upload(auth_client: BoxAuthClient, testing_folder: box.Folder):
    local_file_path = f"{__here__}/tmp/large_test_file.bin"

    # Create a large test file (~52 MB)
    with open(local_file_path, "wb") as f:
        f.write(os.urandom(52 * 1024 * 1024))  # 52 MB of random data

    try:
        uploaded_file = auth_client.upload_file(
            file=local_file_path,
            parent_folder=testing_folder,
        )
        assert isinstance(uploaded_file, box.FileFull), "Large file upload failed."
        auth_client.delete_file(uploaded_file)
    finally:
        # Clean up local test file
        if os.path.exists(local_file_path):
            os.remove(local_file_path)
    return


def test_small_file_upload(auth_client: BoxAuthClient, testing_folder: box.Folder):
    local_file_path = f"{__here__}/tmp/small_test_file.bin"

    # Create a small test file
    with open(local_file_path, "wb") as f:
        f.write(b"This is a small test file.")

    try:
        upload_success = auth_client.upload_file(
            file=local_file_path,
            parent_folder=testing_folder,
        )
        assert isinstance(upload_success, box.File), "Small file upload failed."
        auth_client.delete_file(upload_success)
    finally:
        # Clean up local test file
        if os.path.exists(local_file_path):
            os.remove(local_file_path)
    return


def test_upload_existing_file(auth_client: BoxAuthClient, testing_folder: box.Folder):
    local_file_path = f"{__here__}/tmp/existing_test_file.bin"

    # Create a test file
    with open(local_file_path, "wb") as f:
        f.write(b"This is a test file for existing upload.")

    with pytest.raises(box.BoxAPIError):
        # First upload
        first_upload_success = auth_client.upload_file(
            file=local_file_path,
            parent_folder=testing_folder,
        )
        assert first_upload_success, "First upload of existing file failed."


    # Clean up local test file
    if os.path.exists(local_file_path):
        os.remove(local_file_path)
    return