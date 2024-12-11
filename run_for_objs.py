import os
import os.path

import subprocess

OBJS_DIR = "./models/test"

def run(directory: str):
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith(".obj"):
                obj_path = os.path.join(root, file)
                output_path = obj_path.replace(".obj", "_output.obj")

                # TODO: ./build/YamadaMeshFixer {obj_path} -o {output_path}
                out_bytes = subprocess.check_output(['./build/YamadaMeshFixer', obj_path, '-o', output_path])



run(OBJS_DIR)