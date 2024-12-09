import os
import glob
import shutil

Import("env")


mylib_root = os.getcwd()
generated_src_dir = os.path.join(mylib_root, 'src')
generated_include_dir = os.path.join(mylib_root, 'include')
protoc_generator = os.path.join(mylib_root, '..', 'nanopb', 'generator-bin', 'protoc')

if not os.path.exists(generated_src_dir):
    os.makedirs(generated_src_dir)

if not os.path.exists(generated_include_dir):
    os.makedirs(generated_include_dir)

mylib_proto_dirs = [
    os.path.join(mylib_root, '..', '..', '..', '..', '..', 'proto')
]

nanopb_options = [
    f"--nanopb_out={generated_src_dir}",
]

# Clears the files in src and include
for filename in os.listdir(generated_src_dir):
    file_path = os.path.join(generated_src_dir, filename)
    os.remove(file_path)

for filename in os.listdir(generated_include_dir):
    file_path = os.path.join(generated_include_dir, filename)
    os.remove(file_path)

proto_files = []
for proto_dir in mylib_proto_dirs:
    protos = glob.glob(os.path.join(proto_dir, '*.proto'))
    proto_files += protos
    nanopb_options.append("--proto_path=" + proto_dir)
    nanopb_options.append("--nanopb_opt=-I" + proto_dir)
    # print(proto_dir)

# generates all the .pb.c and .pb.h files in src
for proto_file in proto_files:
    proto_file_basename = os.path.basename(proto_file)
    proto_file_without_ext = os.path.splitext(proto_file_basename)[0]
    generated_targets = [
        os.path.join(generated_src_dir, proto_file_without_ext + ".pb.c"),
        os.path.join(generated_src_dir, proto_file_without_ext + ".pb.h")
    ]
    # print(f"{proto_file_basename} -> {generated_targets}")
    cmd = protoc_generator + " " + " ".join(nanopb_options) + " " + proto_file_basename
    result = env.Execute(cmd)
    if (result != 0):
        print(f"Error({result}) processing cmd: '{cmd}'")
        exit(1)

#  Moves the .pb.h files into include
for filename in os.listdir(generated_src_dir):
    if filename.endswith('.h'):
        source = os.path.join(generated_src_dir, filename)
        destination = os.path.join(generated_include_dir, filename)

        shutil.move(source, destination)