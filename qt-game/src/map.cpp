#include "../include/map.h"
#include "../include/imageprocessor.h"
#include <QBrush>
#include <QPen>
#include <QRandomGenerator>
#include <QDebug>
#include <vector>
#include <queue>
#include <algorithm>
#include <random>

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
            case PURPLE_MARK:
                item->setBrush(QBrush(Qt::magenta));
                break;
            case BLACK_MARK:
                item->setBrush(QBrush(Qt::black));
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
    m_purpleMarkPositions.clear();
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
            case PURPLE_MARK:
                m_purpleMarkPositions.append(QPoint(x, y));
                break;
            case BLACK_MARK:
                m_blackMarkPositions.append(QPoint(x, y));
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
    placeRandomTraps(); // 生成陷阱

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
    placeRandomTraps(); // 生成陷阱

    m_width = 20;
    m_height = 20;
    findSpecialPositions();
}

void Map::generateRandomMap()
{
    // 使用DFS算法生成完美的随机迷宫
    // 初始化所有格子为墙
    for (int x = 0; x < 20; ++x) {
        for (int y = 0; y < 20; ++y) {
            m_map[x][y] = WALL;
        }
    }

    // DFS迷宫生成
    dfsMazeGeneration(1, 1);

    // 确保终点位置可达 - 如果不可达，创建最小路径
    if (!isReachable(1, 1, 18, 18)) {
        // 创建最小路径连接
        createMinimalPath();
    }

    // 设置起点和终点
    m_map[1][1] = START;
    m_map[18][18] = EXIT;

    // 随机地图不在地图数据中放置陷阱，由游戏动态生成

    m_width = 20;
    m_height = 20;
    findSpecialPositions();
}

void Map::dfsMazeGeneration(int startX, int startY)
{
    // 方向数组：上、右、下、左
    const int dx[4] = {-2, 0, 2, 0};
    const int dy[4] = {0, 2, 0, -2};

    // 随机打乱方向顺序
    std::vector<int> directions = {0, 1, 2, 3};
    std::shuffle(directions.begin(), directions.end(), std::mt19937(std::random_device()()));

    // 将起点设为路径
    m_map[startX][startY] = PATH;

    // DFS遍历
    for (int dir : directions) {
        int nx = startX + dx[dir];
        int ny = startY + dy[dir];

        // 检查新位置是否有效且未访问
        if (isValidForMaze(nx, ny)) {
            // 在两个位置之间打通墙壁
            int wallX = startX + dx[dir] / 2;
            int wallY = startY + dy[dir] / 2;

            m_map[wallX][wallY] = PATH;
            m_map[nx][ny] = PATH;

            // 递归生成
            dfsMazeGeneration(nx, ny);
        }
    }
}

bool Map::isValidForMaze(int x, int y)
{
    // 检查边界和是否已访问
    return x >= 1 && x < 19 && y >= 1 && y < 19 && m_map[x][y] == WALL;
}

bool Map::isReachable(int startX, int startY, int endX, int endY)
{
    // 使用BFS检查起点到终点是否可达
    if (!isValidPosition(startX, startY) || !isValidPosition(endX, endY)) {
        return false;
    }

    // 创建访问标记
    std::vector<std::vector<bool>> visited(20, std::vector<bool>(20, false));

    // 方向数组：上、右、下、左
    const int dx[4] = {-1, 0, 1, 0};
    const int dy[4] = {0, 1, 0, -1};

    // BFS队列
    std::queue<QPoint> q;
    q.push(QPoint(startX, startY));
    visited[startX][startY] = true;

    while (!q.empty()) {
        QPoint current = q.front();
        q.pop();

        int x = current.x();
        int y = current.y();

        // 检查是否到达终点
        if (x == endX && y == endY) {
            return true;
        }

        // 探索四个方向
        for (int i = 0; i < 4; ++i) {
            int nx = x + dx[i];
            int ny = y + dy[i];

            if (isValidPosition(nx, ny) && !visited[nx][ny] && m_map[nx][ny] != WALL) {
                visited[nx][ny] = true;
                q.push(QPoint(nx, ny));
            }
        }
    }

    return false;
}

void Map::placeRandomTraps()
{
    // 清空之前的陷阱
    for (int x = 0; x < 20; ++x) {
        for (int y = 0; y < 20; ++y) {
            if (m_map[x][y] == TRAP) {
                m_map[x][y] = PATH;
            }
        }
    }

    // 在路径上随机放置陷阱
    int trapCount = 0;
    int attempts = 0;
    const int maxTraps = 8; // 最多8个陷阱

    while (trapCount < maxTraps && attempts < 200) {
        int x = QRandomGenerator::global()->bounded(1, 19);
        int y = QRandomGenerator::global()->bounded(1, 19);

        // 只在路径上放置陷阱，且不放在起点和终点附近，且必须是玩家可达的位置
        if (m_map[x][y] == PATH &&
            !(x == 1 && y == 1) && !(x == 18 && y == 18) &&
            !(x == 1 && y == 2) && !(x == 2 && y == 1) &&  // 起点附近
            !(x == 18 && y == 17) && !(x == 17 && y == 18) && // 终点附近
            isReachableFromStart(x, y)) { // 确保玩家可以到达
            m_map[x][y] = TRAP;
            trapCount++;
        }
        attempts++;
    }

    qDebug() << "随机地图生成完成，放置了" << trapCount << "个陷阱";
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

    // 清空陷阱位置，然后重新生成陷阱
    m_trapPositions.clear();
    placeRandomTraps(); // 为图片生成的地图也生成陷阱

    m_width = 20;
    m_height = 20;
    findSpecialPositions();

    qDebug() << "从图片生成地图成功，起点: (" << m_startPosition.x() << ", " << m_startPosition.y() << ")";
    qDebug() << "终点: (" << m_exitPosition.x() << ", " << m_exitPosition.y() << ")";

    return true;
}

void Map::createMinimalPath()
{
    // 创建从起点(1,1)到终点(18,18)的最小路径
    // 使用BFS找到最短路径，避免强制整列/整行
    
    // 使用BFS找到最短路径
    std::vector<std::vector<QPoint>> parent(20, std::vector<QPoint>(20, QPoint(-1, -1)));
    std::vector<std::vector<bool>> visited(20, std::vector<bool>(20, false));
    
    // 方向数组：上、右、下、左
    const int dx[4] = {-1, 0, 1, 0};
    const int dy[4] = {0, 1, 0, -1};
    
    // BFS队列
    std::queue<QPoint> q;
    q.push(QPoint(1, 1));
    visited[1][1] = true;
    
    bool found = false;
    while (!q.empty() && !found) {
        QPoint current = q.front();
        q.pop();
        
        int x = current.x();
        int y = current.y();
        
        // 检查是否到达终点
        if (x == 18 && y == 18) {
            found = true;
            break;
        }
        
        // 探索四个方向
        for (int i = 0; i < 4; ++i) {
            int nx = x + dx[i];
            int ny = y + dy[i];
            
            if (isValidPosition(nx, ny) && !visited[nx][ny] && m_map[nx][ny] != WALL) {
                visited[nx][ny] = true;
                parent[nx][ny] = current;
                q.push(QPoint(nx, ny));
            }
        }
    }
    
    // 如果找到了路径，回溯并打通路径
    if (found) {
        QPoint current(18, 18);
        while (current != QPoint(1, 1)) {
            m_map[current.x()][current.y()] = PATH;
            current = parent[current.x()][current.y()];
        }
        m_map[1][1] = PATH;
    } else {
        // 如果BFS失败，使用简单的L形路径
        createSimplePath();
    }
}

void Map::createSimplePath()
{
    // 创建从起点(1,1)到终点(18,18)的简单路径
    // 使用更智能的路径连接，避免强制整列/整行
    
    // 从起点向右到中间位置
    for (int x = 1; x <= 9; ++x) {
        m_map[x][1] = PATH;
    }
    
    // 从中间位置向下到中间行
    for (int y = 1; y <= 9; ++y) {
        m_map[9][y] = PATH;
    }
    
    // 从中间位置向右到终点附近
    for (int x = 9; x <= 17; ++x) {
        m_map[x][9] = PATH;
    }
    
    // 从终点附近向下到终点
    for (int y = 9; y <= 17; ++y) {
        m_map[17][y] = PATH;
    }
    
    // 最后连接到终点
    m_map[17][18] = PATH;
    m_map[18][18] = EXIT;
    
    // 确保起点正确设置
    m_map[1][1] = START;
}

void Map::addPurpleMark(int x, int y)
{
    if (isValidPosition(x, y)) {
        m_map[x][y] = PURPLE_MARK;
        QPoint pos(x, y);
        if (!m_purpleMarkPositions.contains(pos)) {
            m_purpleMarkPositions.append(pos);
        }
    }
}

void Map::clearPurpleMarks()
{
    // 清除地图上的紫色印记
    for (int x = 0; x < m_width; ++x) {
        for (int y = 0; y < m_height; ++y) {
            if (m_map[x][y] == PURPLE_MARK) {
                m_map[x][y] = PATH;
            }
        }
    }
    m_purpleMarkPositions.clear();
}

void Map::addBlackMark(int x, int y)
{
    if (isValidPosition(x, y)) {
        m_map[x][y] = BLACK_MARK;
        QPoint pos(x, y);
        if (!m_blackMarkPositions.contains(pos)) {
            m_blackMarkPositions.append(pos);
        }
    }
}

void Map::clearBlackMarks()
{
    // 清除地图上的黑色痕迹
    for (int x = 0; x < m_width; ++x) {
        for (int y = 0; y < m_height; ++y) {
            if (m_map[x][y] == BLACK_MARK) {
                m_map[x][y] = PATH;
            }
        }
    }
    m_blackMarkPositions.clear();
}

bool Map::isReachableFromStart(int x, int y)
{
    // 检查从起点是否能到达指定位置
    QPoint startPos = getStartPosition();
    if (startPos.x() == -1 || startPos.y() == -1) {
        return false; // 起点未设置
    }

    return isReachable(startPos.x(), startPos.y(), x, y);
}
