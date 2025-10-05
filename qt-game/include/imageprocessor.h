#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSOR_H

#include <QString>
#include <QImage>
#include <QPoint>
#include "map.h"

class ImageProcessor
{
public:
    ImageProcessor();
    ~ImageProcessor();

    // 图片处理
    bool loadImage(const QString& filePath);
    bool processImage();
    bool generateMapFromImage(char mapData[][20]);
    bool generateCharMap(char charMap[][20]);

    // 获取图片信息
    int getImageWidth() const { return m_originalImage.width(); }
    int getImageHeight() const { return m_originalImage.height(); }
    bool isValid() const { return m_isValid; }

    // 获取处理后的图片（用于预览）
    QImage getProcessedImage() const { return m_processedImage; }

    // 获取起点和终点位置
    QPoint getStartPosition() const { return m_startPosition; }
    QPoint getExitPosition() const { return m_exitPosition; }

private:
    QImage m_originalImage;
    QImage m_processedImage;
    bool m_isValid;
    QPoint m_startPosition;
    QPoint m_exitPosition;

    // 图片处理辅助函数
    bool convertToBinary();
    bool resizeToGrid();
    bool findSpecialPositions();
    bool analyzeImageForPositions();

    // 像素到地图单元的转换
    char pixelToCellType(QRgb pixel) const;
    bool isBlackPixel(QRgb pixel) const;
    bool isWhitePixel(QRgb pixel) const;
};

#endif // IMAGEPROCESSOR_H
