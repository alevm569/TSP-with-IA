import os
import sys
current_dir = os.path.dirname(os.path.realpath(__file__))
sys.path.append(current_dir)
from createTSPDataSet.utils.default_logger import create_default_logger
tsp_logger = create_default_logger("tsp.log")
tsp_logger.info("--> Starting TSP")
tsp_detail_logger = create_default_logger("tsp_detail.log")
