# tuw_checkerboard for ROS2


## VSCode
launch configuration
```yaml
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "composition_publisher",
            "type": "cppdbg",
            "request": "launch",
            "program": "${workspaceFolder}/tuw/install/tuw_checkerboard/lib/tuw_checkerboard/composition_publisher",
            "args": [],
            "stopAtEntry": true,
            "cwd": "${workspaceFolder}/tuw",
            "environment": [],
            "externalConsole": false,
            "MIMode": "gdb",
            "setupCommands": [
                {
                    "description": "Enable pretty-printing for gdb",
                    "text": "-enable-pretty-printing",
                    "ignoreFailures": true
                },
                {
                    "description":  "Set Disassembly Flavor to Intel",
                    "text": "-gdb-set disassembly-flavor intel",
                    "ignoreFailures": true
                }
            ]
        },
        {
            "name": "composition_subscriber",
            "type": "cppdbg",
            "request": "launch",
            "program": "${workspaceFolder}/tuw/install/tuw_checkerboard/lib/tuw_checkerboard/composition_subscriber",
            "args": [],
            "stopAtEntry": true,
            "cwd": "${workspaceFolder}/tuw",
            "environment": [],
            "externalConsole": false,
            "MIMode": "gdb",
            "setupCommands": [
                {
                    "description": "Enable pretty-printing for gdb",
                    "text": "-enable-pretty-printing",
                    "ignoreFailures": true
                },
                {
                    "description":  "Set Disassembly Flavor to Intel",
                    "text": "-gdb-set disassembly-flavor intel",
                    "ignoreFailures": true
                }
            ]
        },
        {
            "name": "composition_composed_pub_sub",
            "type": "cppdbg",
            "request": "launch",
            "program": "${workspaceFolder}/tuw/install/tuw_checkerboard/lib/tuw_checkerboard/composition_composed",
            "args": [],
            "stopAtEntry": true,
            "cwd": "${workspaceFolder}/tuw",
            "environment": [],
            "externalConsole": false,
            "MIMode": "gdb",
            "setupCommands": [
                {
                    "description": "Enable pretty-printing for gdb",
                    "text": "-enable-pretty-printing",
                    "ignoreFailures": true
                },
                {
                    "description":  "Set Disassembly Flavor to Intel",
                    "text": "-gdb-set disassembly-flavor intel",
                    "ignoreFailures": true
                }
            ]
        },
    ]
}
```