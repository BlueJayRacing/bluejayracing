import os
import glob
import shutil
import filecmp

Import("env")

mylib_root = os.getcwd()

def copy_folder_contents(src_root, dest_root, folder_names):
    for folder in folder_names:
        src_folder = os.path.join(src_root, folder)
        dest_folder = os.path.join(dest_root, folder)
        
        if not os.path.exists(src_folder):
            print(f"Warning: Source folder '{src_folder}' does not exist.")
            continue
        
        os.makedirs(dest_folder, exist_ok=True)
        
        for item in os.listdir(src_folder):
            src_path = os.path.join(src_folder, item)
            dest_path = os.path.join(dest_folder, item)
            
            if os.path.isdir(src_path):
                # For directories, you might want to recursively copy if needed.
                shutil.copytree(src_path, dest_path, dirs_exist_ok=True)
                print(f"Copied directory '{src_path}' to '{dest_path}'")
            else:
                # If destination exists and is identical, skip copying.
                if os.path.exists(dest_path) and filecmp.cmp(src_path, dest_path, shallow=False):
                    print(f"Skipping '{src_path}' as no changes detected.")
                    continue
                shutil.copy2(src_path, dest_path)
                print(f"Copied '{src_path}' to '{dest_path}'")

source_directory = os.path.join(mylib_root, '..', '..', '..', '..', '..', 'common', 'nanopb')
destination_directory = mylib_root  # Change this to the actual destination root directory

folders_to_copy = ["src", "include"]

copy_folder_contents(source_directory, destination_directory, folders_to_copy)
