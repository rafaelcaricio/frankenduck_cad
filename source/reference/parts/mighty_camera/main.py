import cadquery as cq
from source_api import Model

model = Model()

model.import_step("import_1", "f4484a3777bee86a3f443ff41f07cd8f1de65fdcbb7d7d6b99c878c8dadae972", name="RV1103 Camera")

result = model.result()

