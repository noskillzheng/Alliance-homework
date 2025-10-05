#ifndef FOG_H
#define FOG_H

#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include <QPoint>
#include <QSet>

class Fog
{
public:
    enum FogState {
        UNKNOWN = 0,    // 未知区域（黑色）
        EXPLORED = 1,   // 已探索区域（半透明）
        VISIBLE = 2     // 当前可见区域（完全透明）
    };
    
    Fog(int width = 20, int height = 20);
    ~Fog();
    
    // 迷雾管理
    void initializeFog(QGraphicsScene* scene);
    void clearFog(QGraphicsScene* scene);
    void updateVisibility(int playerX, int playerY, int visionRadius = 2); // 方形视野半尺寸
    void drawFog(QGraphicsScene* scene);
    
    // 获取迷雾状态
    FogState getFogState(int x, int y) const;
    bool isVisible(int x, int y) const;
    bool isExplored(int x, int y) const;
    
    // 设置迷雾状态
    void setFogState(int x, int y, FogState state);
    void markAsExplored(int x, int y);
    
    // 重置迷雾
    void resetFog();
    
    // 获取尺寸
    int getWidth() const { return m_width; }
    int getHeight() const { return m_height; }

private:
    FogState m_fogMap[20][20];  // 迷雾状态地图
    int m_width, m_height;
    QList<QGraphicsRectItem*> m_fogItems;  // 迷雾图形项
    
    // 辅助函数
    bool isValidPosition(int x, int y) const;
    void updateFogItem(int x, int y, FogState state);
    QColor getFogColor(FogState state) const;
};

#endif // FOG_H

