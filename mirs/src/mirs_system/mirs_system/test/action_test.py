import os
from mirs_system.ai.extractor import Extractor


ex = Extractor()

recordings = [
   os.path.join(os.path.dirname(__file__),"left_.avi"),
   os.path.join(os.path.dirname(__file__),"right_.avi"),
]
ex.extract(recordings)