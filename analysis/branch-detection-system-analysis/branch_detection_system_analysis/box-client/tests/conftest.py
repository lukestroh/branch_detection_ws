#!/usr/bin/env python3
import os
import pytest
import box_sdk_gen as box

from box_client.box_auth_client import BoxAuthClient
import box_client.file_manager as fm


@pytest.fixture(scope='session')
def temp_dir() -> str:
    __here__ = os.path.dirname(__file__)
    temp_dir = os.path.join(__here__, "tmp")
    os.makedirs(temp_dir, exist_ok=True)
    return temp_dir


@pytest.fixture(scope="session")
def auth_client() -> BoxAuthClient:
    return BoxAuthClient()


@pytest.fixture(scope="session")
def box_env_map() -> dict[str, str]:
    box_environments: dict = {
        "testing": "352392709920"
    }
    return box_environments

@pytest.fixture(scope="session")
def testing_folder(auth_client: BoxAuthClient, box_env_map: dict[str, str]) -> box.Folder:
    testing_folder = auth_client.get_folder(folder_id=box_env_map["testing"])
    return testing_folder


@pytest.fixture(scope='session')
def file_manager(temp_dir: str) -> fm.FileManager:
    staging_dir = f"{temp_dir}/staging"
    datalake_dir = f"{temp_dir}/datalake"
    logging_dir = f"{temp_dir}/logs"
    return fm.FileManager(staging_dir, datalake_dir, logging_dir)

