This folder, along with its subfolders, contains the firmware that runs on the ESP32 microcontrollers using ESP-IDF. In order to compile and flash our ESP-IDF projects, you need to do the following:
- Install the ESP-IDF extension on VS Code, and follow the ESP-IDF Setup to install the latest version of ESP-IDF (default parameters should be fine)
- Initialize all Git submodules with the following command: [git submodule update --init --recursive]
- Open up the root folder of the ESP-IDF project in VS Code
- Set the correct COM port, microcontroller, and ESP-IDF on the bottom left if necessary

Common Compilation Issues:
- Make sure that the .vscode directory in the root folder of your project has a c_cpp_properties.json, launch.json, and a settings.json. If it is missing any of those, delete the .vscode folder, and have ESP-IDF generate it using the [Add .vscode subdirectoy files] button found by clicking on the ESP-IDF Explorer Icon in the Activity Bar and looking under Advanced.
- Also see if deleting the build directory and rebuilding works as well.
- Make sure that you configured the SDK to have the size of flash be 4MD for the XIAO ESP32C6.
- Make sure that you configured the SDK to use a custom partition table in the project directory.