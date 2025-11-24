#! /usr/bin/env python3
import os
import shutil

# __here__ = os.path.dirname(__file__)
# logging_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), "logs")

import logging


class FileManagerError(Exception):
    """Custom exception class for FileManager errors."""
    pass


class FileManager:
    def __init__(self, staging_dir: str, datalake_dir: str, logging_dir: str) -> None:
        self.logger = logging.getLogger(__name__)
        logging.basicConfig(
            filename=os.path.join(logging_dir, "file_manager.log"),
            level=logging.INFO,
            format="%(asctime)s [%(levelname)s] %(name)s:%(lineno)d - %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S"
        )

        self.staging_dir = staging_dir
        self.datalake_dir = datalake_dir
        self.logging_dir = logging_dir 
        os.makedirs(self.datalake_dir, exist_ok=True)
        os.makedirs(self.logging_dir, exist_ok=True)
        return
    
    def _is_special_folder(self, folder_name: str) -> bool:
        """Check if the given folder name is a special folder.

        :param folder_name: The name of the folder to check.
        :type folder_name: str
        :return: True if the folder is special, False otherwise.
        :rtype: bool
        """
        return folder_name in [self.staging_dir, self.datalake_dir, self.logging_dir]
    
    def move(self, source: str, dest: str) -> bool:
        """Move a folder from source to destination.

        :param source_folder: The path of the source folder.
        :type source_folder: str
        :param dest_folder: The path of the destination folder.
        :type dest_folder: str
        :return: True if the move was successful, False otherwise.
        :rtype: bool
        """
        try:
            if self._is_special_folder(os.path.basename(source)) or self._is_special_folder(os.path.basename(dest)):
                self.logger.error(f"Attempted to move special folder: {source } or {dest}")
                return False

            shutil.move(source, dest)
            self.logger.info(f"Moved folder from {source} to {dest}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to move folder from {source} to {dest}: {e}")
            return False
        
    def move_preserve_tree(self, source: str, dest: str) -> bool:
        """Move contents of source folder to destination folder, preserving tree structure.

        :param source_folder: The path of the source folder.
        :type source_folder: str
        :param dest_folder: The path of the destination folder.
        :type dest_folder: str
        :return: True if the move was successful, False otherwise.
        :rtype: bool
        """
        try:
            if self._is_special_folder(os.path.basename(source)) or self._is_special_folder(os.path.basename(dest)):
                self.logger.error(f"Attempted to move special folder: {source } or {dest}")
                return False

            for root, dirs, files in os.walk(source):
                relative_path = os.path.relpath(root, source)
                dest_path = os.path.join(dest, relative_path)
                os.makedirs(dest_path, exist_ok=True)

                for file in files:
                    shutil.move(os.path.join(root, file), os.path.join(dest_path, file))

            self.logger.info(f"Moved contents from {source} to {dest} preserving tree structure")
            return True
        except Exception as e:
            self.logger.error(f"Failed to move contents from {source} to {dest}: {e}")
            return False
        

class WarehouseManager(FileManager):
    def __init__(self, warehouse_dir: str, logging_dir: str) -> None:
        super().__init__(warehouse_dir, logging_dir)
        self.warehouse_dir = warehouse_dir
        os.makedirs(self.warehouse_dir, exist_ok=True)
        return
    

class StagingManager(FileManager):
    def __init__(self, staging_dir: str, datalake_dir: str, logging_dir: str) -> None:
        super().__init__(staging_dir, datalake_dir, logging_dir)
        return