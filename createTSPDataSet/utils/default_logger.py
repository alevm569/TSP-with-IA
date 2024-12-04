import logging
import os
from logging.handlers import RotatingFileHandler

script_path = os.path.dirname(os.path.abspath(__file__))
utils_path = os.path.dirname(script_path)
project_path = os.path.dirname(utils_path)
log_path = os.path.join(project_path, "logs")

def create_default_logger(file_name: str):
    logger = logging.getLogger(f"{file_name}")

    # create folder for logs
    if not os.path.exists(log_path):
        os.makedirs(log_path)

    # rotate log at least 4 with 1MB each
    log_file = os.path.join(log_path, file_name)
    handler = RotatingFileHandler(log_file, maxBytes=1000000, backupCount=4)
    formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    handler.setFormatter(formatter)
    logger.addHandler(handler)
    logger.setLevel(logging.INFO)

    # add to see the log in the console
    console_handler = logging.StreamHandler()
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)
    return logger

