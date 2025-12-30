

BUILD_DIR="build"
PACKAGE_DIR="${BUILD_DIR}/package"

cmake -B ${BUILD_DIR} -DCMAKE_BUILD_TYPE=Release
cmake --build ${BUILD_DIR} -j$(nproc)

# 清理之前的打包文件
rm -rf ${PACKAGE_DIR}
rm -f ${BUILD_DIR}/*.deb ${BUILD_DIR}/*.tar.gz

# 进入构建目录
cd ${BUILD_DIR}

# 设置 DESTDIR
export DESTDIR=${PACKAGE_DIR}

# 打包 DEB（检查 dpkg-deb 是否存在）
if command -v dpkg-deb &> /dev/null; then
    echo "正在打包 DEB..."
    set +e
    cpack -G DEB
    DEB_RESULT=$?
    set -e
    if [ $DEB_RESULT -eq 0 ]; then
        echo "✓ DEB 打包成功"
    else
        echo "✗ DEB 打包失败"
    fi
else
    echo "跳过 DEB 打包（未安装 dpkg-deb）"
fi

# 打包 TGZ（总是可用）
echo "正在打包 TGZ..."
set +e
cpack -G TGZ
TGZ_RESULT=$?
set -e
if [ $TGZ_RESULT -eq 0 ]; then
    echo "✓ TGZ 打包成功"
else
    echo "✗ TGZ 打包失败"
fi

echo ""
echo "=========================================="
echo "打包完成！"
echo "=========================================="
echo "生成的文件："
find "$(pwd)" -maxdepth 1 -type f \( -name "*.deb" -o -name "*.tar.gz" \) -exec readlink -f {} \; || echo "未生成任何包文件"

