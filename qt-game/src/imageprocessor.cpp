#include "../include/imageprocessor.h"
#include <QDebug>
#include <QFile>
#include <QDir>

ImageProcessor::ImageProcessor()
    : m_isValid(false)
    , m_startPosition(-1, -1)
    , m_exitPosition(-1, -1)
{
}

ImageProcessor::~ImageProcessor()
{
}

bool ImageProcessor::loadImage(const QString& filePath)
{
    if (!QFile::exists(filePath)) {
        qDebug() << "图片文件不存在:" << filePath;
        return false;
    }

    m_originalImage = QImage(filePath);
    if (m_originalImage.isNull()) {
        qDebug() << "无法加载图片:" << filePath;
        return false;
    }

    qDebug() << "图片加载成功，大小:" << m_originalImage.size();
    return true;
}

bool ImageProcessor::processImage()
{
    if (m_originalImage.isNull()) {
        return false;
    }

    // 第一步：转换为二值图像
    if (!convertToBinary()) {
        return false;
    }

    // 第二步：调整大小到20x20网格
    if (!resizeToGrid()) {
        return false;
    }

    // 第三步：分析图片找出起点和终点
    if (!findSpecialPositions()) {
        return false;
    }

    m_isValid = true;
    return true;
}

bool ImageProcessor::generateCharMap(char charMap[][20])
{
    if (!m_isValid || m_processedImage.isNull()) {
        return false;
    }

    // 将处理后的图片转换为字符格式地图数据
    for (int y = 0; y < 20; ++y) {
        for (int x = 0; x < 20; ++x) {
            QPoint pixelPos(x, y);
            QRgb pixel = m_processedImage.pixel(pixelPos);

            if (pixelPos == m_startPosition) {
                charMap[x][y] = 'S'; // 起点
            } else if (pixelPos == m_exitPosition) {
                charMap[x][y] = 'E'; // 终点
            } else {
                charMap[x][y] = pixelToCellType(pixel);
            }
        }
    }

    return true;
}

bool ImageProcessor::generateMapFromImage(char mapData[][20])
{
    // 为了兼容性，保留这个函数但调用新的字符生成函数
    char charMap[20][20];
    if (!generateCharMap(charMap)) {
        return false;
    }

    // 复制到传入的数组
    for (int x = 0; x < 20; ++x) {
        for (int y = 0; y < 20; ++y) {
            mapData[x][y] = charMap[x][y];
        }
    }

    return true;
}

bool ImageProcessor::convertToBinary()
{
    if (m_originalImage.isNull()) {
        return false;
    }

    // 创建一个新的二值图像
    m_processedImage = QImage(m_originalImage.size(), QImage::Format_RGB32);

    for (int y = 0; y < m_originalImage.height(); ++y) {
        for (int x = 0; x < m_originalImage.width(); ++x) {
            QRgb pixel = m_originalImage.pixel(x, y);

            if (isBlackPixel(pixel)) {
                // 黑色像素 -> 黑色
                m_processedImage.setPixel(x, y, qRgb(0, 0, 0));
            } else {
                // 非黑色像素 -> 白色
                m_processedImage.setPixel(x, y, qRgb(255, 255, 255));
            }
        }
    }

    return true;
}

bool ImageProcessor::resizeToGrid()
{
    if (m_processedImage.isNull()) {
        return false;
    }

    // 计算缩放比例，使图片适应20x20网格
    int targetSize = 20;
    int sourceWidth = m_processedImage.width();
    int sourceHeight = m_processedImage.height();

    // 计算最佳缩放比例
    double scaleX = static_cast<double>(targetSize) / sourceWidth;
    double scaleY = static_cast<double>(targetSize) / sourceHeight;
    double scale = qMin(scaleX, scaleY); // 使用较小的缩放比例

    // 计算缩放后的尺寸
    int newWidth = static_cast<int>(sourceWidth * scale);
    int newHeight = static_cast<int>(sourceHeight * scale);

    // 创建缩放后的图像
    QImage scaledImage = m_processedImage.scaled(newWidth, newHeight, Qt::KeepAspectRatio, Qt::SmoothTransformation);

    // 创建最终的20x20图像，用白色填充
    m_processedImage = QImage(20, 20, QImage::Format_RGB32);
    m_processedImage.fill(qRgb(255, 255, 255));

    // 将缩放后的图像居中放置到20x20网格中
    int startX = (20 - newWidth) / 2;
    int startY = (20 - newHeight) / 2;

    for (int y = 0; y < newHeight; ++y) {
        for (int x = 0; x < newWidth; ++x) {
            // 检查目标坐标是否在20x20范围内
            int targetX = startX + x;
            int targetY = startY + y;

            if (targetX >= 0 && targetX < 20 && targetY >= 0 && targetY < 20) {
                // 检查源坐标是否在scaledImage范围内
                if (x >= 0 && x < scaledImage.width() && y >= 0 && y < scaledImage.height()) {
                    QRgb pixel = scaledImage.pixel(x, y);
                    m_processedImage.setPixel(targetX, targetY, pixel);
                }
            }
        }
    }

    return true;
}

bool ImageProcessor::findSpecialPositions()
{
    if (m_processedImage.isNull()) {
        return false;
    }

    // 分析图片，自动识别起点和终点
    return analyzeImageForPositions();
}

bool ImageProcessor::analyzeImageForPositions()
{
    // 简单的起点和终点识别算法
    // 假设起点在左上角，终点在右下角附近

    // 寻找最左上角的白色区域作为起点
    QPoint bestStart(-1, -1);
    QPoint bestExit(-1, -1);

    for (int y = 0; y < 20; ++y) {
        for (int x = 0; x < 20; ++x) {
            QPoint pos(x, y);
            QRgb pixel = m_processedImage.pixel(pos);

            if (isWhitePixel(pixel)) {
                // 检查是否是起点（左上角优先）
                if (bestStart == QPoint(-1, -1) ||
                    (x + y) < (bestStart.x() + bestStart.y())) {
                    bestStart = pos;
                }

                // 检查是否是终点（右下角优先）
                if (bestExit == QPoint(-1, -1) ||
                    (x + y) > (bestExit.x() + bestExit.y())) {
                    bestExit = pos;
                }
            }
        }
    }

    // 如果没有找到合适的位置，使用默认位置
    if (bestStart == QPoint(-1, -1)) {
        bestStart = QPoint(1, 1);
    }
    if (bestExit == QPoint(-1, -1)) {
        bestExit = QPoint(18, 18);
    }

    m_startPosition = bestStart;
    m_exitPosition = bestExit;

    return true;
}

char ImageProcessor::pixelToCellType(QRgb pixel) const
{
    if (isBlackPixel(pixel)) {
        return '#';  // 黑色像素 -> 墙
    } else {
        return ' ';  // 白色像素 -> 通路
    }
}

bool ImageProcessor::isBlackPixel(QRgb pixel) const
{
    // 判断像素是否为黑色（允许一定阈值）
    int gray = qGray(pixel);
    return gray < 128; // 灰度值小于128认为是黑色
}

bool ImageProcessor::isWhitePixel(QRgb pixel) const
{
    // 判断像素是否为白色（允许一定阈值）
    int gray = qGray(pixel);
    return gray >= 128; // 灰度值大于等于128认为是白色
}
