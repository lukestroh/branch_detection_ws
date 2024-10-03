#!/usr/bin/env python3

from io import TextIOWrapper
import xacro
import xml.dom.minidom

from zenlog import log

def load_urdf_from_xacro(xacro_path: str, mappings: dict | None = None) -> xml.dom.minidom.Document | TextIOWrapper:
    # TODO: if urdf_path not used, add step to load existing file.
    urdf_content = xacro.process_file(input_file_name=xacro_path, mappings=mappings)
    return urdf_content


def save_urdf(urdf_content: str, urdf_path: str) -> None:
    with open(urdf_path, "w") as f:
        f.write(urdf_content)
    log.info(f"Saved URDF to file '{urdf_path}'.")
    return
