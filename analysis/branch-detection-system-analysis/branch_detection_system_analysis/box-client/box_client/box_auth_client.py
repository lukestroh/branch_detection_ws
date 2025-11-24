#!/usr/bin/env python3

"""
Box Authentication script
Author: Luke Strohbehn
"""

# TODO: Implement Lynx - Pipe output back to terminal

import base64
from typing import Any, Generator, Optional
from xmlrpc import server
import box_sdk_gen as box


# from boxsdk import BoxOAuthException
import dotenv
import webbrowser

# import contextlib
import json
import io
import os
import glob
import pprint as pp

import traceback
import time
import threading
from typing import Union

from box_client import file_manager as fm
from box_client.redirect_handler import RedirectHandler, AuthHTTPServer, run_server
from http.server import HTTPServer
from urllib.parse import urlparse, parse_qs

__here__ = os.path.dirname(__file__)
logging_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "logs")

import logging
logger = logging.getLogger(__name__)
logging.basicConfig(
    filename=os.path.join(logging_dir, "box_client.log"),
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s:%(lineno)d - %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S"
)

box_environments: dict = {
    "root": "0",
    "20240202_prosser_trials": "247341820732",
    "20240201_prosser_trials": "247314061474",
    "2025_ToFBranchDetection": "308822377240",
    "warehouse": "316868797576",
    "datalake": "351993973489",
}

class BoxClientTokenStorage(box.TokenStorage):

    def __init__(self, token_storage_dir: str) -> None:
        self.token_storage_dir = token_storage_dir
        self.get_storage_info()
        return
    
    def store(self, token: box.AccessToken) -> None:
        """Method to store new access_token and refresh_token
        :param access_token: access token provided by Box API
        :type access_token: box.AccessToken
        :returns: None
        :rtype: None
        """
        access_token = token.access_token
        refresh_token = token.refresh_token

        with open(self.box_refresh_token_path, "w") as f:
            f.write(base64.b64encode(refresh_token.encode()).decode())
        with open(self.box_access_token_path, "w") as f:
            f.write(base64.b64encode(access_token.encode()).decode())
        return

    def get(self) -> Optional[box.AccessToken]:
        if not (os.path.exists(self.box_refresh_token_path) and os.path.exists(self.box_access_token_path)):
            return None
        # use existing refresh token
        refresh_token = base64.b64decode(
            open(self.box_refresh_token_path, "r").read().strip().encode()
        ).decode()  # CHANGE THIS TO UTF-8
        access_token = base64.b64decode(open(self.box_access_token_path, "r").read().strip().encode()).decode()
        token = box.AccessToken(access_token=access_token, refresh_token=refresh_token)
        return token

    def clear(self) -> None:
        """Remove any stored token files so the next auth flow will re-authorize.
        :returns: None
        :rtype: None"""
        for p in (getattr(self, "box_access_token_path", None), getattr(self, "box_refresh_token_path", None)):
            try:
                if p and os.path.exists(p):
                    os.remove(p)
            except Exception as exc:
                logger.warning(f"Warning: failed to remove token file {p}: {exc}")
        return
    
    def get_storage_info(self) -> None:
        self.box_refresh_token_path = os.path.join(self.token_storage_dir, ".box_refresh_token")
        self.box_access_token_path = os.path.join(self.token_storage_dir, ".box_access_token")
        dotenv_path = os.path.join(self.token_storage_dir, ".env")
        dotenv.load_dotenv(dotenv_path=dotenv_path)

        self.app_client = os.environ["CLIENT_ID"]
        self.app_token = os.environ["BOX_APP_TOKEN"]
        self.app_secret = os.environ["BOX_APP_SECRET"]
        return

    

class BoxAuthClient:
    def __init__(self):
        self.token_storage = BoxClientTokenStorage(
            token_storage_dir=os.path.join(os.path.dirname(os.path.dirname(__file__)), "env")
        )
        self.client = self.box_connection()
        return

    def box_connection(self) -> box.BoxClient:
        """Returns an authenticated Box api connection

        :return: Authenticated Box Client object
        :rtype: box.BoxClient
        """
        try:
            # start OAuth process
            oauth = box.BoxOAuth(
                box.OAuthConfig(
                    client_id=self.token_storage.app_client,
                    client_secret=self.token_storage.app_secret,
                    token_storage=self.token_storage # uses store_tokens method above
                )
            )

        except Exception as e:  # catch everything, go to reauthorization
            # Clear any possibly corrupted/stale tokens so the interactive flow is clean
            try:
                self.token_storage.clear()
            except Exception:
                pass

            # start server
            PORT = 5000
            server = AuthHTTPServer(("localhost", PORT), RedirectHandler)
            server_thread = threading.Thread(target=run_server, args=(server, PORT), daemon=True)
            server_thread.start()

            # start OAuth process
            oauth = box.BoxOAuth(
                box.OAuthConfig(
                    client_id=self.token_storage.app_client,
                    client_secret=self.token_storage.app_secret,
                    token_storage=self.token_storage # uses store_tokens method above
                )
            )
            auth_url = oauth.get_authorize_url(
                options=box.GetAuthorizeUrlOptions(
                    redirect_uri=f"http://localhost:{PORT}"
                )
            )
            webbrowser.open(auth_url)

            # wait for handler to get auth code
            while server.auth_code is None:
                time.sleep(0.1)

            # Get and store token
            access_token = oauth.get_tokens_authorization_code_grant(authorization_code=server.auth_code)

            # clean up server
            server.shutdown()
            server_thread.join()

        client = box.BoxClient(auth=oauth)

        return client

    def get_folder(self, folder_id: str) -> box.Folder:
        """Get a folder object

        :param folder_name: The name of the folder to retrieve
        :type folder_name: str
        :return: The Box folder object
        :rtype: box.Folder
        """
        return self.client.folders.get_folder_by_id(folder_id=folder_id)
    
    def create_folder(self, parent_folder: box.Folder, folder_name: str) -> box.Folder:
        """Create a folder in Box

        :param parent_folder: The parent folder object
        :type parent_folder: box.Folder
        :param folder_name: The name of the folder to create
        :type folder_name: str
        :return: The created Box folder object
        :rtype: box.Folder
        """
        return self.client.folders.create_folder(
            parent=box.CreateFolderParent(id=parent_folder.id),
            name=folder_name
        )

    def upload_file(self, file: str, parent_folder: box.Folder) -> Union[box.File, box.FileFull, None]:
        """Upload a file to a Box folder using io.BytesIO. Checks file size and uses chunked upload for files > 50 MB.

        :param file: The path to the file to upload
        :type file: str
        :param parent_folder: The parent folder object
        :type parent_folder: box.Folder
        """
        filename = os.path.basename(file)
        file_size = os.path.getsize(file)
        with open(file, "rb") as f:
            file_stream = io.BytesIO(f.read())

            try:
                if file_size > 50 * 1024 * 1024:  # 50 MB
                    logger.info(f"Uploading large file {filename} using chunked upload...")
                    file = self.client.chunked_uploads.upload_big_file(
                        file=file_stream,
                        file_size=file_size,
                        parent_folder_id=parent_folder.id,
                        file_name=filename,
                    )
                else:
                    logger.info(f"Uploading file {filename}...")
                    files = self.client.uploads.upload_file(
                        attributes=box.UploadFileAttributes(
                            name=filename,
                            parent=parent_folder
                        ),
                        file=file_stream,
                    )
                    file = files.entries[0]  # type: box.File
            except box.BoxAPIError as e:
                logger.error(f"Failed to upload file {file} to Box folder {parent_folder.name}.")
                logger.error(traceback.format_exc())
                return None

        return file
    
    def delete_file_by_id(self, file_id: str) -> None:
        """Delete a file in Box by its ID.

        :param file_id: The ID of the file to delete
        :type file_id: str
        """
        self.client.files.delete_file_by_id(file_id=file_id)
        return
    
    def delete_file_by_name(self, folder: box.Folder, filename: str) -> None:
        """Delete a file in Box by its name.

        :param folder: The Box folder containing the file
        :type folder: box.Folder
        :param filename: The name of the file to delete
        :type filename: str
        """
        try:
            items = self.client.folders.get_folder_items(folder_id=folder.id)
            for item in items.entries:
                if getattr(item, "type", None) == "file" and item.name == filename:
                    self.delete_file_by_id(file_id=item.id)
                    return
        except Exception:
            # On any error conservatively return to avoid masking issues
            return
        return
    
    def delete_file(self, file: box.File) -> None:
        """Delete a Box file object.

        :param file: The Box file object to delete
        :type file: box.File
        """
        self.client.files.delete_file_by_id(file_id=file.id)
        return

    def download_file_by_id(self, file_id: str, local_path: str) -> None:
        """Download a file from Box by its ID.

        :param file_id: The ID of the file to download
        :type file_id: str
        :param local_path: The local path to save the downloaded file
        :type local_path: str
        """
        file_content = self.client.downloads.download_file(file_id=file_id)
        with open(local_path, "wb") as f:
            f.write(file_content)
        return
    
    def find_child_folder_by_name(self, parent_folder_id: str, name: str) -> Optional[str]:
        try:
            marker = 0
            while True:
                items: box.Items = self.client.folders.get_folder_items(
                    folder_id=parent_folder_id,
                    limit=1000,
                    marker=marker,
                    usemarker=True
                )
                for item in items.entries:
                    if getattr(item, "type", None) == "folder" and item.name == name:
                        return item.id
                    
                if not items.next_marker: # TODO: add test for this
                    break
                marker = items.next_marker


        except Exception:
            print(traceback.format_exc())
        return None
    
    def check_create_subfolder(self, subfolder_name: str, parent_folder: box.Folder) -> box.Folder:
        remote_subfolder_id = self.find_child_folder_by_name(
            parent_folder_id=parent_folder.id, name=subfolder_name
        )
        if not remote_subfolder_id:
            print(f"Creating folder {subfolder_name} in Box folder {parent_folder.name}")
            remote_subfolder = self.create_folder(
                parent_folder=parent_folder, folder_name=subfolder_name
            )
            return remote_subfolder
        else:
            print(f"Remote folder {subfolder_name} already exists in folder '{parent_folder.name}' -> id: {remote_subfolder_id}")
            return self.get_folder(folder_id=remote_subfolder_id)
        
    def file_exists_in_folder(self, folder: box.Folder, filename: str) -> bool:
        """Check whether a filename already exists in a Box folder (by name)."""
        try:
            
            items = self.client.folders.get_folder_items(folder_id=folder.id)
            for item in items.entries:
                if getattr(item, "type", None) == "file" and item.name == filename:
                    print(f"\tFile {filename} already exists in Box folder {folder.name}.")
                    return True
        except Exception:
            # On any error conservatively return False to attempt upload (retry will catch)
            return False
        return False
    


def upload_staging_to_box(bclient: BoxAuthClient,local_staging_dir: str, local_datalake_dir: str, logging_dir: str):
    file_manager = fm.FileManager(
        staging_dir=os.path.join(local_staging_dir),
        datalake_dir=os.path.join(local_datalake_dir),
        logging_dir=os.path.join(logging_dir),
    )

    # list immediate directories under staging
    if not os.path.isdir(local_staging_dir):
        logger.error(f"No staging directory found at {local_staging_dir}")
        return
    if len(os.listdir(local_staging_dir)) == 0:
        logger.info(f"No folders found in staging directory {local_staging_dir}")
        return

    # Box folders
    datalake_box_folder = bclient.get_folder(folder_id=box_environments['datalake'])

    # Upload staging folders
    failed_uploads = []
    for subfolder in sorted(os.listdir(local_staging_dir)):
        local_folder_path = os.path.join(local_staging_dir, subfolder)

        # Skip specific folders
        if subfolder in ("datalake", "warehouse"):
            logger.info(f"Skipping upload of folder {subfolder}")
            continue

        # check/create subfolder in Box
        # print(local_folder_path)
        remote_subfolder = bclient.check_create_subfolder(
            subfolder_name=subfolder, parent_folder=datalake_box_folder
        )
        # print(remote_subfolder)

        # walk local folder tree, ensure remote nested folders exist, upload files
        all_ok = True
        for dirpath, subdirs, local_files in os.walk(local_folder_path):
            # determine relative path from local_staging_dir
            # rel_path = os.path.relpath(dirpath, local_staging_dir)

            for local_file in local_files:
                if bclient.file_exists_in_folder(folder=remote_subfolder, filename=local_file):
                    logger.info(f"File {local_file} already exists in Box folder {remote_subfolder.name}; skipping upload.")
                    continue
                else:
                    uploaded_file = bclient.upload_file(
                        file=os.path.join(dirpath, local_file), parent_folder=remote_subfolder
                    )
                    if uploaded_file is None:
                        all_ok = False
                        failed_uploads.append(os.path.join(dirpath, local_file))

        if all_ok:
            logger.info(f"All files from local folder {subfolder} uploaded successfully to Box folder {remote_subfolder.name}.")
            # move local folder to datalake
            dest_folder = os.path.join(local_datalake_dir, subfolder)
            move_success = file_manager.move(local_folder_path, dest_folder)
            if move_success:
                logger.info(f"Local folder {subfolder} moved to datalake at {dest_folder}.")
            
        else:
            logger.error(f"Some files from local folder {subfolder} failed to upload to Box folder {remote_subfolder.name}.")
            # write failed uploads to a log file
            failed_log_path = os.path.join(logging_dir, f"failed_staging_uploads.log")
            with open(failed_log_path, "a") as f:
                for failed_file in failed_uploads:
                    f.write(f"{failed_file}\n")
            logger.info(f"Failed uploads logged to {failed_log_path}")

        failed_uploads.clear()
        logger.info(f"Finished processing folder {subfolder}.")
    logger.info("Staging upload process complete.")
    return


def upload_warehouse_to_box(bclient: BoxAuthClient, local_warehouse_dir: str, logging_dir: str):
    # Box folders
    warehouse_box_folder = bclient.get_folder(folder_id=box_environments['warehouse'])

    # Upload warehouse files
    failed_uploads = []
    for subfolder in sorted(os.listdir(local_warehouse_dir)):
        local_folder_path = os.path.join(local_warehouse_dir, subfolder)

        # Skip specific folders
        if subfolder in ("datalake", "warehouse"):
            logger.info(f"Skipping upload of folder {subfolder}")
            continue

        remote_subfolder = bclient.check_create_subfolder(
            subfolder_name=subfolder, parent_folder=warehouse_box_folder
        )

        all_ok = True
        for dirpath, subdirs, local_files in os.walk(local_folder_path):
            for local_file in local_files:
                if bclient.file_exists_in_folder(folder=remote_subfolder, filename=local_file):
                    logger.info(f"File {local_file} already exists in Box folder {remote_subfolder.name}; skipping upload.")
                    continue
                else:
                    uploaded_file = bclient.upload_file(
                        file=os.path.join(dirpath, local_file), parent_folder=remote_subfolder
                    )
                    if uploaded_file is None:
                        all_ok = False
                        failed_uploads.append(os.path.join(dirpath, local_file))
                    
        if all_ok:
            logger.info(f"All files from local folder {subfolder} uploaded successfully to Box folder {remote_subfolder.name}.")
        else:
            logger.error(f"Some files from local folder {subfolder} failed to upload to Box folder {remote_subfolder.name}.")
            # write failed uploads to a log file
            failed_log_path = os.path.join(logging_dir, f"failed_warehouse_uploads.log")
            with open(failed_log_path, "a") as f:
                for failed_file in failed_uploads:
                    f.write(f"{failed_file}\n")
            logger.info(f"Failed uploads logged to {failed_log_path}")

        failed_uploads.clear()
        logger.info(f"Finished processing folder {subfolder}.")
    logger.info("Warehouse upload process complete.")
    return


def main():
    bclient = BoxAuthClient()
    logger.info("Box client initialized")
    
    # Local directories
    local_root_dir = os.path.join(
        os.path.expanduser("~"), "branch_detection_ws", "bags", "2025_ToFBranchDetection",#  "warehouse"
    )
    local_datalake_dir = os.path.join(
        local_root_dir, "datalake"
    )
    os.makedirs(local_datalake_dir, exist_ok=True)
    local_warehouse_dir = os.path.join(
        os.path.expanduser("~"), "branch_detection_ws", "bags", "2025_ToFBranchDetection", "warehouse"
    )
    os.makedirs(local_warehouse_dir, exist_ok=True)
    local_staging_dir = os.path.join(
        os.path.expanduser("~"), "branch_detection_ws", "bags", "2025_ToFBranchDetection", "staging"
    )
    os.makedirs(local_staging_dir, exist_ok=True)

    
    upload_staging_to_box(
        bclient=bclient,
        local_staging_dir=local_staging_dir,
        local_datalake_dir=local_datalake_dir,
        logging_dir=logging_dir,
    )

    upload_warehouse_to_box(
        bclient=bclient,
        local_warehouse_dir=local_warehouse_dir,
        logging_dir=logging_dir,
    )

    

    return


if __name__ == "__main__":
    main()
