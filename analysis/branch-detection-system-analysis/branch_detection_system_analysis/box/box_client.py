#!/usr/bin/env python3

"""
Box Authentication script
Author: Luke Strohbehn
"""

# TODO: Implement Lynx - Pipe output back to terminal

import base64
from typing import Any, Generator
import boxsdk as box

# from boxsdk import BoxOAuthException
import dotenv
import webbrowser

# import contextlib
import requests
import json
import io
import os
import glob

from redirect_handler import RedirectHandler
from http.server import HTTPServer
from urllib.parse import urlparse, parse_qs


__here__ = os.path.dirname(__file__)

dotenv_path = os.path.join(__here__, ".env")
dot = dotenv.load_dotenv(dotenv_path=dotenv_path)

app_client = os.environ["CLIENT_ID"]
app_token = os.environ["BOX_APP_TOKEN"]
app_secret = os.environ["BOX_APP_SECRET"]

box_refresh_token_path = os.path.join(__here__, ".box_refresh_token")
box_access_token_path = os.path.join(__here__, ".box_access_token")

box_environments: dict = {
    "root": "0",
    "20240202_prosser_trials": "247341820732",
    "20240201_prosser_trials": "247314061474",
    "2025_ToFBranchDetection": "308822377240",
    "warehouse": "316868797576",
}


def run_server():
    PORT = 5000  # Ensure this matches your redirect URI port
    server = HTTPServer(("localhost", PORT), RedirectHandler)
    print(f"Starting server at http://localhost:{PORT}")
    server.serve_forever()
    return


class BoxClient:
    def __init__(self):
        self.client = self.box_connection()
        return

    def store_tokens(self, access_token: str, refresh_token: str):
        """Method to store new access_token and refresh_token
        Parameters
        ----------
            access_token (str): access token provided by Box API
            refresh_token (str): refresh token provided by Box API
        Returns
        -------
            None
        """
        try:
            with open(box_refresh_token_path, "w") as f:
                f.write(base64.b64encode(refresh_token.encode()).decode())
            with open(box_access_token_path, "w") as f:
                f.write(base64.b64encode(access_token.encode()).decode())
        except Exception as e:
            print("Error: %s" % (e,))
        return

    def box_connection(self) -> box.Client:
        """Returns an authenticated Box api connection
        Parameters
        ----------
            None
        Returns
        -------
            client (box.Client): Authenticated Box Client object
        """
        try:
            # use existing refresh token
            refresh_token = base64.b64decode(
                open(box_refresh_token_path, "r").read().strip().encode()
            ).decode()  # CHANGE THIS TO UTF-8
            access_token = base64.b64decode(open(box_access_token_path, "r").read().strip().encode()).decode()
            oauth = box.OAuth2(
                client_id=app_client,
                client_secret=app_secret,
                refresh_token=refresh_token,
                access_token=access_token,
                store_tokens=self.store_tokens,
            )

        except Exception as e:  # catch everything, go to reauthorization
            oauth = box.OAuth2(
                client_id=app_client,
                client_secret=app_secret,
                store_tokens=self.store_tokens,  # uses store_tokens method above
            )

            auth_url, csrf_token = oauth.get_authorization_url(f"http://localhost:5000")

            webbrowser.open(auth_url)
            user_input = input("Enter the url here: ").strip()

            query_components = parse_qs(urlparse(user_input).query)
            auth_code = query_components.get("code")[0]
            url_csrf_token = query_components.get("state")[0]

            assert url_csrf_token == csrf_token
            access_token, refresh_token = oauth.authenticate(auth_code)
            print("Authentication successful")

            pass

        client = box.Client(oauth)

        return client

    def get_folder(self, namespace: str = "staging", _id=None) -> box.object.folder.Folder:
        """Get a folder object
        Parameters
        ----------
            namespace (str): string of the folder name we want, using box_environments
        Returns
        -------
            folder (box.object.folder.Folder): box Folder object
        """
        if _id is not None:
            folder = self.client.folder(_id).get()
        else:
            folder = self.client.folder(folder_id=box_environments[namespace]).get()
        return folder

    def upload_bag_file(self, file_path: str, namespace: str = "staging", _id=None):
        """Upload a file to staging
        Parameters
        ----------
            file_path (str): path of file to be uploaded
            namespace (str): name of the folder to upload to, as defined in box_environments
                Default: "staging"

        """
        if _id is not None:
            folder = self.get_folder(_id=_id)
        else:
            folder = self.get_folder(namespace)
        a_file = folder.upload(file_path=file_path, upload_using_accelerator=True)  # , file_name=file_name)
        try:
            print(f'{a_file.get()["name"]} uploaded. ')
        except Exception as e:
            print(e)
        return

    def iterate_box_download_items(
        self, items: list
    ) -> Generator[
        io.BytesIO, None, None
    ]:  # A generator can be annotated by the generic type Generator[YieldType, SendType, ReturnType]
        """Iterate over the items returned by get_folder()
        Parameters
        ----------
            items (list): list of items
        Yields
        -------
            byte_data (io.BytesIO): a BytesIO object
        """
        for item in items:
            byte_data = self.client.file(item.id).content()
            yield io.BytesIO(byte_data)

    def get_items(self, folder: str):
        # Get the items in the folder you want
        folder = self.get_folder(folder)
        items = folder.get_items()
        # download the items for using
        print(items)

        # downloaded_item = iterate_box_download_items(client, items)
        # while True:
        #     try:
        #         print(next(downloaded_item).read())
        #     except StopIteration:
        #         break
        return items


def main():
    mybox = BoxClient()
    print(mybox.client.user().get())
    """
    creates one level of subfolders in box and uploads files
    """
    root_dir = os.path.join(
        os.path.expanduser("~"), "branch_detection_ws", "bags", "2025_ToFBranchDetection", "warehouse"
    )
    # root_dir = os.path.join("/media/luke/T7 Shield", "luke")
    folders = glob.glob("bds*", root_dir=root_dir, recursive=True)
    for folder in sorted(folders):
        # print(f"FOLDER: {folder}")
        subfolder_path = os.path.join(root_dir, folder)
        subfolder_id = None
        try:
            subfolder = mybox.client.folder(box_environments["warehouse"]).create_subfolder(folder)
            subfolder_id = subfolder.id
        except Exception as e:
            subfolder_id = e.context_info["conflicts"][0]["id"]
            subfolder = mybox.get_folder(_id=subfolder_id)
            print(f"Folder {subfolder.name} already exists")
        files = glob.glob("**/*.*", root_dir=subfolder_path, recursive=True)

        try:
            # TODO try preventing reupload requests: use get item, compare sha hash
            for file in files:
                mybox.upload_bag_file(os.path.join(subfolder_path, file), _id=subfolder_id)
        except Exception as e:  # TODO actually check for other exceptions to attempt retries
            print()
            filename = e.context_info["conflicts"]["name"]
            print(f"Item {filename} already exists")

    return


if __name__ == "__main__":
    import threading

    server_thread = threading.Thread(target=run_server, daemon=True)
    server_thread.start()

    # main_thread = threading.Thread(target=main, daemon=True)
    main()
    # main_thread.start()
