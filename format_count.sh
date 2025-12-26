#!/bin/bash

# 格式化（使用项目根目录的 .clang-format 配置）
clang-format -i -style=file $(find include/ -type f -name "*.hpp")
clang-format -i -style=file $(find src/ -type f -name "*.cpp")

# 统计代码行数
# cloc --git `git branch --show-current`
cloc src include --exclude-dir=build,dist,.vscode 