#include "../include/fog.h"
#include <QBrush>
#include <QPen>
#include <QDebug>

Fog::Fog(int width, int height)
    : m_width(width)
    , m_height(height)
{
    // 初始化所有区域为未知状态
    for (int x = 0; x < m_width; ++x) {
        for (int y = 0; y < m_height; ++y) {
            m_fogMap[x][y] = UNKNOWN;
        }
    }
}

Fog::~Fog()
{
    // 图形项将由场景清理
}

void Fog::initializeFog(QGraphicsScene* scene)
{
    if (!scene) return;
    
    clearFog(scene);
    
    const int cellSize = 30;
    
    // 创建迷雾层
    for (int y = 0; y < m_height; ++y) {
        for (int x = 0; x < m_width; ++x) {
            QGraphicsRectItem *fogItem = new QGraphicsRectItem(x * cellSize, y * cellSize, cellSize, cellSize);
<<<<<<< HEAD
            fogItem->setBrush(QBrush(getFogColor(VISIBLE)));
            fogItem->setPen(QPen(Qt::NoPen));
            fogItem->setZValue(3); // 迷雾层在地图层之上，玩家层之下，确保黑色标记可见
=======
            fogItem->setBrush(QBrush(getFogColor(UNKNOWN)));
            fogItem->setPen(QPen(Qt::NoPen));
            fogItem->setZValue(10); // 迷雾层在玩家层之上，地图层之下
>>>>>>> e3a6255ae8c87c7b077a5583bf70026efc77e1bc
            
            scene->addItem(fogItem);
            m_fogItems.append(fogItem);
        }
    }
}

void Fog::clearFog(QGraphicsScene* scene)
{
    if (!scene) return;
    
    for (QGraphicsRectItem* item : m_fogItems) {
        scene->removeItem(item);
        delete item;
    }
    m_fogItems.clear();
}

void Fog::updateVisibility(int playerX, int playerY, int visionRadius)
{
    // 计算方形视野范围：visionRadius表示方形视野的半尺寸
    // 例如visionRadius=2时，方形视野为5x5（从-2到+2，共5个单位）
    int halfSize = visionRadius;
    int startX = qMax(0, playerX - halfSize);
    int endX = qMin(m_width - 1, playerX + halfSize);
    int startY = qMax(0, playerY - halfSize);
    int endY = qMin(m_height - 1, playerY + halfSize);

    // 更新可见区域（方形视野）
    for (int x = 0; x < m_width; ++x) {
        for (int y = 0; y < m_height; ++y) {
            // 检查是否在方形视野范围内
            bool inVision = (x >= startX && x <= endX && y >= startY && y <= endY);

            if (inVision) {
                // 在视野范围内
                if (m_fogMap[x][y] == UNKNOWN) {
                    m_fogMap[x][y] = VISIBLE;
                } else if (m_fogMap[x][y] == EXPLORED) {
                    m_fogMap[x][y] = VISIBLE;
                }
<<<<<<< HEAD
                // 注意：PERMANENT_MARK状态不会被视野更新覆盖
=======
>>>>>>> e3a6255ae8c87c7b077a5583bf70026efc77e1bc
            } else {
                // 不在视野范围内
                if (m_fogMap[x][y] == VISIBLE) {
                    m_fogMap[x][y] = EXPLORED;
                }
            }
        }
    }
    
    // 更新图形显示
    drawFog(nullptr);
}

void Fog::drawFog(QGraphicsScene* scene)
{
    Q_UNUSED(scene)
    
    // 更新所有迷雾项的显示
    for (int y = 0; y < m_height; ++y) {
        for (int x = 0; x < m_width; ++x) {
            int index = y * m_width + x;
            if (index < m_fogItems.size()) {
                QGraphicsRectItem* item = m_fogItems[index];
                item->setBrush(QBrush(getFogColor(m_fogMap[x][y])));
            }
        }
    }
}

Fog::FogState Fog::getFogState(int x, int y) const
{
    if (isValidPosition(x, y)) {
        return m_fogMap[x][y];
    }
    return UNKNOWN;
}

bool Fog::isVisible(int x, int y) const
{
    return getFogState(x, y) == VISIBLE;
}

bool Fog::isExplored(int x, int y) const
{
    return getFogState(x, y) == EXPLORED || getFogState(x, y) == VISIBLE;
}

void Fog::setFogState(int x, int y, FogState state)
{
    if (isValidPosition(x, y)) {
        m_fogMap[x][y] = state;
        updateFogItem(x, y, state);
    }
}

void Fog::markAsExplored(int x, int y)
{
    if (isValidPosition(x, y) && m_fogMap[x][y] == UNKNOWN) {
        m_fogMap[x][y] = EXPLORED;
        updateFogItem(x, y, EXPLORED);
    }
}

void Fog::resetFog()
{
    for (int x = 0; x < m_width; ++x) {
        for (int y = 0; y < m_height; ++y) {
<<<<<<< HEAD
            m_fogMap[x][y] = VISIBLE;
=======
            m_fogMap[x][y] = UNKNOWN;
>>>>>>> e3a6255ae8c87c7b077a5583bf70026efc77e1bc
        }
    }
    drawFog(nullptr);
}

bool Fog::isValidPosition(int x, int y) const
{
    return x >= 0 && x < m_width && y >= 0 && y < m_height;
}

void Fog::updateFogItem(int x, int y, FogState state)
{
    int index = y * m_width + x;
    if (index < m_fogItems.size()) {
        QGraphicsRectItem* item = m_fogItems[index];
        item->setBrush(QBrush(getFogColor(state)));
    }
}

QColor Fog::getFogColor(FogState state) const
{
    switch (state) {
    case UNKNOWN:
        return QColor(0, 0, 0, 255);      // 黑色，完全不透明
    case EXPLORED:
        return QColor(0, 0, 0, 100);      // 半透明黑色
    case VISIBLE:
        return QColor(0, 0, 0, 0);        // 完全透明
<<<<<<< HEAD
    case PERMANENT_MARK:
        return QColor(0, 0, 0, 255);      // 永久黑色痕迹，完全不透明
=======
>>>>>>> e3a6255ae8c87c7b077a5583bf70026efc77e1bc
    default:
        return QColor(0, 0, 0, 255);
    }
}

