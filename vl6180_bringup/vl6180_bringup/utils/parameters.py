#!/usr/bin/env python3
"""
Author: Jostan Brown

Base class for saving parameters from a dataclass object
"""

from dataclasses import dataclass, fields, asdict

import logging
import os
import yaml


@dataclass
class Parameters:
    def load_from_yaml(self, file_path):
        logging.info(f"Loading parameters from {file_path}")

        if not os.path.exists(file_path):
            raise FileNotFoundError(f"Parameter file not found: {file_path}")

        with open(file_path, "r") as file:
            data = yaml.safe_load(file)

        for field in fields(self):
            if field.name in data:
                setattr(self, field.name, data[field.name])

        self.log_settings()

    def save_to_yaml(self, file_path):

        logging.info(f"Saving parameters to {file_path}")

        with open(file_path, "w") as file:
            yaml.dump(asdict(self), file)

    def log_settings(self):
        logging.info("Current settings:")
        for field in fields(self):
            logging.debug(f"{field.name}: {getattr(self, field.name)}")
