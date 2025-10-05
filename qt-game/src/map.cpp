#include "../include/map.h"
#include "../include/imageprocessor.h"
#include <QBrush>
#include <QPen>
#include <QRandomGenerator>
#include <QDebug>

Map::Map(int width, int height)
    : m_width(width)
    , m_height(height)
{
    // Initialize map with walls
    for (int x = 0; x < m_width; ++x) {
        for (int y = 0; y < m_height; ++y) {
            m_map[x][y] = WALL;
        }
    }
}

Map::~Map()
{
    // Graphics items will be cleaned up by the scene
}

void Map::loadMap(const char mapData[][20], int width, int height)
{
    m_width = width;
    m_height = height;
    
    for (int x = 0; x < m_width; ++x) {
        for (int y = 0; y < m_height; ++y) {
            m_map[x][y] = mapData[x][y];
        }
    }
    
    findSpecialPositions();
}

void Map::drawMap(QGraphicsScene* scene)
{
    if (!scene) return;
    
    const int cellSize = 30;
    
    for (int y = 0; y < m_height; ++y) {
        for (int x = 0; x < m_width; ++x) {
            QGraphicsRectItem *item = new QGraphicsRectItem(x * cellSize, y * cellSize, cellSize, cellSize);
            
            switch (m_map[x][y]) {
            case WALL:
                item->setBrush(QBrush(Qt::gray));
                break;
            case PATH:
                item->setBrush(QBrush(Qt::white));
                break;
            case TRAP:
                item->setBrush(QBrush(Qt::white));
                break;
            case EXIT:
                item->setBrush(QBrush(Qt::green));
                break;
            case START:
                item->setBrush(QBrush(Qt::blue));
                break;
            }
            
            item->setPen(QPen(Qt::black, 1));
            scene->addItem(item);
            m_graphicsItems.append(item);
        }
    }
}

void Map::clearMap(QGraphicsScene* scene)
{
    if (!scene) return;
    
    for (QGraphicsRectItem* item : m_graphicsItems) {
        scene->removeItem(item);
        delete item;
    }
    m_graphicsItems.clear();
}

char Map::getCell(int x, int y) const
{
    if (x >= 0 && x < m_width && y >= 0 && y < m_height) {
        return m_map[x][y];
    }
    return WALL; // Return wall for invalid positions
}

void Map::setCell(int x, int y, char value)
{
    if (x >= 0 && x < m_width && y >= 0 && y < m_height) {
        m_map[x][y] = value;
    }
}

bool Map::isValidPosition(int x, int y) const
{
    return x >= 0 && x < m_width && y >= 0 && y < m_height;
}

bool Map::isWalkable(int x, int y) const
{
    return isValidPosition(x, y) && m_map[x][y] != WALL;
}

const char* Map::getMapData() const
{
    // 返回按行优先顺序排列的地图数据
    static char rowMajorMap[400]; // 20*20 = 400
    for (int y = 0; y < m_height; ++y) {
        for (int x = 0; x < m_width; ++x) {
            rowMajorMap[y * m_width + x] = m_map[x][y];
        }
    }
    return rowMajorMap;
}

void Map::findSpecialPositions()
{
    m_trapPositions.clear();
    m_startPosition = QPoint(-1, -1);
    m_exitPosition = QPoint(-1, -1);
    
    for (int x = 0; x < m_width; ++x) {
        for (int y = 0; y < m_height; ++y) {
            switch (m_map[x][y]) {
            case START:
                m_startPosition = QPoint(x, y);
                break;
            case EXIT:
                m_exitPosition = QPoint(x, y);
                break;
            case TRAP:
                m_trapPositions.append(QPoint(x, y));
                break;
            }
        }
    }
}

void Map::generateMap1()
{
    // 地图1：经典递归回溯迷宫 - 参考DFS算法设计的复杂迷宫
    // 1表示墙，0表示路径
    const char mapData[20][20] = {
        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
        {1,4,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
        {1,0,1,1,0,1,0,1,1,1,1,1,1,1,1,1,1,1,0,1},
        {1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1},
        {1,0,1,0,1,1,1,1,1,1,1,1,1,1,1,1,0,1,0,1},
        {1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,1},
        {1,0,1,1,1,1,1,1,1,1,1,1,1,1,0,1,0,1,0,1},
        {1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,1,0,1},
        {1,0,1,1,1,1,1,1,1,1,1,1,0,1,0,1,0,1,0,1},
        {1,0,0,0,0,0,0,0,0,0,0,1,0,1,0,1,0,1,0,1},
        {1,0,1,1,1,1,1,1,1,1,0,1,0,1,0,1,0,1,0,1},
        {1,0,1,0,0,0,0,0,0,1,0,1,0,1,0,1,0,1,0,1},
        {1,0,1,0,1,1,1,1,0,1,0,1,0,1,0,1,0,1,0,1},
        {1,1,1,0,1,0,0,1,0,1,0,1,0,1,0,1,0,1,0,1},
        {1,0,1,0,1,0,1,1,0,1,0,1,0,1,0,1,0,1,0,1},
        {1,0,1,0,1,0,0,0,0,1,0,1,0,1,0,1,0,1,0,1},
        {1,0,1,0,1,1,1,1,1,1,0,1,0,1,0,1,0,1,0,1},
        {1,0,0,0,0,0,0,0,0,0,0,1,0,1,0,1,0,1,0,1},
        {1,0,1,1,1,1,1,1,1,1,1,1,0,1,0,0,0,1,0,1},
        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,3,1}
    };
    
    // 加载地图数据
    for (int x = 0; x < 20; ++x) {
        for (int y = 0; y < 20; ++y) {
            m_map[x][y] = mapData[y][x];  // 修正坐标索引：mapData[y][x] 对应 m_map[x][y]
        }
    }
    
    // 不再预定义陷阱，由游戏动态生成
    
    m_width = 20;
    m_height = 20;
    findSpecialPositions();
}

void Map::generateMap2()
{
    // 地图2：经典螺旋迷宫 - 参考传统螺旋迷宫设计
    // 1表示墙，0表示路径
    const char mapData[20][20] = {
        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
        {1,4,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1},
        {1,0,1,1,1,0,1,1,1,1,1,1,1,1,0,1,1,1,0,1},
        {1,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,0,1},
        {1,0,1,0,1,1,1,1,1,1,1,1,1,1,1,1,0,1,0,1},
        {1,0,1,0,1,0,0,0,0,0,0,0,0,0,0,1,0,1,0,1},
        {1,0,1,0,1,0,1,0,1,1,1,1,1,1,0,1,0,1,0,1},
        {1,0,1,0,1,0,1,0,0,0,0,0,0,1,0,1,0,1,0,1},
        {1,0,1,0,1,0,1,0,1,1,1,1,0,1,0,1,0,1,0,1},
        {1,0,1,0,1,0,1,0,1,0,0,1,0,1,0,1,0,1,0,1},
        {1,0,1,0,0,0,1,0,1,0,0,1,0,1,0,1,0,1,0,1},
        {1,0,1,0,1,0,1,0,1,1,1,1,0,1,0,1,0,1,0,1},
        {1,0,0,0,1,0,1,0,0,0,0,0,0,1,0,1,0,1,0,1},
        {1,0,1,0,1,0,1,1,1,1,1,1,1,1,0,1,0,1,0,1},
        {1,0,1,0,1,0,0,0,0,0,0,0,0,0,0,1,0,1,0,1},
        {1,0,1,0,1,1,1,1,1,1,1,1,1,1,1,1,0,1,0,1},
        {1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,1},
        {1,0,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,1},
        {1,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,1},
        {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,3,1}
    };
    
    // 加载地图数据
    for (int x = 0; x < 20; ++x) {
        for (int y = 0; y < 20; ++y) {
            m_map[x][y] = mapData[y][x];  // 修正坐标索引：mapData[y][x] 对应 m_map[x][y]
        }
    }
    
    // 不再预定义陷阱，由游戏动态生成
    
    m_width = 20;
    m_height = 20;
    findSpecialPositions();
}

void Map::generateRandomMap()
{
    // 简化的随机迷宫生成算法
    // 初始化所有格子为墙
    for (int x = 0; x < 20; ++x) {
        for (int y = 0; y < 20; ++y) {
            m_map[x][y] = WALL;
        }
    }
    
    // 创建基础路径结构
    // 水平路径
    for (int y = 1; y < 19; y += 2) {
        for (int x = 1; x < 19; ++x) {
            m_map[x][y] = PATH;
        }
    }
    
    // 垂直连接路径
    for (int x = 2; x < 18; x += 2) {
        for (int y = 2; y < 18; y += 2) {
            m_map[x][y] = PATH;
        }
    }
    
    // 添加随机连接
    for (int i = 0; i < 15; ++i) {
        int x = QRandomGenerator::global()->bounded(1, 19);
        int y = QRandomGenerator::global()->bounded(1, 19);
        m_map[x][y] = PATH;
    }
    
    // 确保起点和终点可达
    m_map[1][1] = START;
    m_map[18][18] = EXIT;
    
    // 在路径上随机放置陷阱
    int trapCount = 0;
    int attempts = 0;
    while (trapCount < 6 && attempts < 100) {
        int x = QRandomGenerator::global()->bounded(1, 19);
        int y = QRandomGenerator::global()->bounded(1, 19);
        
        if (m_map[x][y] == PATH && x != 1 && y != 1 && x != 18 && y != 18) {
            m_map[x][y] = TRAP;
            trapCount++;
        }
        attempts++;
    }

    m_width = 20;
    m_height = 20;
    findSpecialPositions();
}

bool Map::generateMapFromImage(const QString& imagePath)
{
    ImageProcessor processor;
    if (!processor.loadImage(imagePath)) {
        qDebug() << "无法加载图片:" << imagePath;
        return false;
    }

    if (!processor.processImage()) {
        qDebug() << "图片处理失败";
        return false;
    }

    // 生成字符格式的地图数据
    char charMap[20][20];
    if (!processor.generateCharMap(charMap)) {
        qDebug() << "地图生成失败";
        return false;
    }

    // 将字符格式转换为数字格式
    for (int x = 0; x < 20; ++x) {
        for (int y = 0; y < 20; ++y) {
            switch (charMap[x][y]) {
            case '#':
                m_map[x][y] = WALL;
                break;
            case ' ':
                m_map[x][y] = PATH;
                break;
            case 'S':
                m_map[x][y] = START;
                break;
            case 'E':
                m_map[x][y] = EXIT;
                break;
            default:
                m_map[x][y] = WALL; // 默认设为墙
                break;
            }
        }
    }

    // 设置起点和终点
    m_startPosition = processor.getStartPosition();
    m_exitPosition = processor.getExitPosition();

    // 清空陷阱位置（从图片生成的地图不预设陷阱）
    m_trapPositions.clear();

    m_width = 20;
    m_height = 20;
    findSpecialPositions();

    qDebug() << "从图片生成地图成功，起点: (" << m_startPosition.x() << ", " << m_startPosition.y() << ")";
    qDebug() << "终点: (" << m_exitPosition.x() << ", " << m_exitPosition.y() << ")";

    return true;
}

