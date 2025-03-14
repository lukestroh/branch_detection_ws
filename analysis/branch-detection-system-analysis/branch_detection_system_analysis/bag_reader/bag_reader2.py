#!usr/bin/env python3
from pathlib import Path
import sqlite3
from rclpy.serialization import deserialize_message
import rclpy.logging
from rosidl_runtime_py.utilities import get_message

import rosbag2_py

import zstandard

logger = rclpy.logging.get_logger("bag_reader")


class BagReader:
    def __init__(self, bag_file: str):
        if bag_file.endswith(".zstd"):
            input_file = Path(bag_file)
            logger.warn(f"{input_file}")
            output_dir = input_file.parent
            output_path = Path(output_dir) / input_file.stem

            if not Path(output_path).exists():
                with open(input_file, "rb") as compressed:
                    decomp = zstandard.ZstdDecompressor()
                    logger.info(f"{output_path}")
                    with open(output_path, "wb") as destination:
                        decomp.copy_stream(compressed, destination)
                    logger.info(f"Decompressed data at {output_path}")
            else:
                logger.info(f"Found decompressed bag at {output_path}")

            self.decompressed_bag_file = str(output_path)

        else:
            self.decompressed_bag_file = bag_file

        storage_options = rosbag2_py.StorageOptions(uri=self.decompressed_bag_file, storage_id="sqlite3")
        converter_options = rosbag2_py.ConverterOptions("", "")

        self.reader = rosbag2_py.SequentialReader()
        self.reader.open(storage_options, converter_options)

        topics: list[rosbag2_py.TopicMetadata] = self.reader.get_all_topics_and_types()

        self.topic_map = {topic.name: topic.type for topic in topics}

        # self.conn = sqlite3.connect(bag_file)
        # self.cursor = self.conn.cursor()
        # topics_data = self.conn.execute("SELECT id, name, type FROM topics").fetchall()

        # self.topics = [r[1] for r in topics_data]
        # self.topic_name_to_id = {r[1]: r[0] for r in topics_data}
        # self.topic_name_to_type = {r[1]: get_message(r[2]) for r in topics_data}
        return

    def get_next(self):
        # self.reader
        return

    def query(self, topic_name: str):
        topic_id = self.topic_name_to_id[topic_name]
        topic_msg_type = self.topic_name_to_type[topic_name]

        rows = self.cursor.execute("SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id))
        for ts, data in rows:
            yield ts, deserialize_message(data, topic_msg_type)

        return
