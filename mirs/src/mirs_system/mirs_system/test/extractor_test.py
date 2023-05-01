import os
from mirs_system.ai.extractor import Extractor


ex = Extractor()

base_dir = os.path.dirname(__file__)
test_recordings = [
    os.path.join(base_dir,"left_.avi"),
    os.path.join(base_dir,"right_.avi"),

]

res = ex.extract(test_recordings)

print(res)