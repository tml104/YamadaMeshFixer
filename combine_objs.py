import os
import os.path

def combine_obj_models(input_files, output_file):
    """
    Combine multiple OBJ files into a single OBJ file.

    :param input_files: List of input OBJ file paths.
    :param output_file: Output OBJ file path.
    """
    vertices = []  # v (vertices)
    tex_coords = []  # vt (texture coordinates)
    normals = []  # vn (normals)
    faces = []  # f (faces)
    
    v_offset = 0  # Offset for vertices
    vt_offset = 0  # Offset for texture coordinates
    vn_offset = 0  # Offset for normals
    
    for obj_file in input_files:
        with open(obj_file, 'r') as f:
            for line in f:
                prefix = line.strip().split(' ', 1)[0]
                
                if prefix == 'v':  # Vertex
                    vertices.append(line.strip())
                elif prefix == 'vt':  # Texture Coordinate
                    tex_coords.append(line.strip())
                elif prefix == 'vn':  # Normal
                    normals.append(line.strip())
                elif prefix == 'f':  # Face
                    parts = line.strip().split()[1:]
                    updated_parts = []
                    for part in parts:
                        indices = part.split('/')
                        v_idx = int(indices[0]) + v_offset if indices[0] else ''  # Vertex index
                        vt_idx = int(indices[1]) + vt_offset if len(indices) > 1 and indices[1] else ''  # Texture index
                        vn_idx = int(indices[2]) + vn_offset if len(indices) > 2 and indices[2] else ''  # Normal index
                        updated_part = '/'.join(filter(None, [str(v_idx), str(vt_idx), str(vn_idx)]))
                        updated_parts.append(updated_part)
                    faces.append('f ' + ' '.join(updated_parts))
        
        # Update offsets
        v_offset += sum(1 for line in open(obj_file) if line.startswith('v '))
        vt_offset += sum(1 for line in open(obj_file) if line.startswith('vt '))
        vn_offset += sum(1 for line in open(obj_file) if line.startswith('vn '))
    
    # Write combined OBJ
    with open(output_file, 'w') as f_out:
        f_out.write('# Combined OBJ file\n')
        f_out.write('\n'.join(vertices) + '\n')
        f_out.write('\n'.join(tex_coords) + '\n')
        f_out.write('\n'.join(normals) + '\n')
        f_out.write('\n'.join(faces) + '\n')

def get_objs(directory: str):
    objs = []
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.endswith('.obj'):
                obj_path = os.path.join(root, file)
                objs.append(obj_path)
    return objs


# Example usage:

OBJS_DIR = "./models/test2"

input_objs = get_objs(OBJS_DIR)
input_objs.reverse()
output_obj = './models/test2/combined.obj'

print("input_objs: ", input_objs)
print("output_obj: ", output_obj)

combine_obj_models(input_objs, output_obj)