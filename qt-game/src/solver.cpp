#include "../include/solver.h"
#include "../include/map.h"
#include <QHash>
#include <QQueue>
#include <QDebug>

Solver::Solver()
{
}

Solver::~Solver()
{
}

QList<QPoint> Solver::findPath(const Map* map, QPoint start, QPoint end)
{
    if (!map) return QList<QPoint>();

    // 如果起点或终点不可行走，返回空路径
    if (!map->isWalkable(start.x(), start.y()) || !map->isWalkable(end.x(), end.y())) {
        return QList<QPoint>();
    }

    // 如果起点和终点相同，返回只包含起点的路径
    if (start == end) {
        return QList<QPoint>() << start;
    }

    // A*算法实现
    QList<Node> openList;
    QHash<QPoint, QPoint> cameFrom;
    QHash<QPoint, int> gScore;
    QHash<QPoint, bool> closedSet;

    // 初始化起点
    Node startNode(start, 0, heuristic(start, end));
    openList.append(startNode);
    gScore[start] = 0;

    while (!openList.isEmpty()) {
        // 找到开放列表中f值最小的节点
        Node current = getLowestFNode(openList);

        if (current.pos == end) {
            // 找到路径，返回重建的路径
            return reconstructPath(cameFrom, current.pos);
        }

        // 从开放列表中移除当前节点
        openList.removeOne(current);
        closedSet[current.pos] = true;

        // 检查邻居节点
        QList<QPoint> neighbors = getNeighbors(current.pos, map);

        for (const QPoint& neighbor : neighbors) {
            if (closedSet.contains(neighbor)) {
                continue;
            }

            // 计算到邻居的代价
            int tentativeG = gScore[current.pos] + 1;

            if (!gScore.contains(neighbor) || tentativeG < gScore[neighbor]) {
                // 记录路径
                cameFrom[neighbor] = current.pos;
                gScore[neighbor] = tentativeG;

                // 计算启发式值并添加到开放列表
                Node neighborNode(neighbor, tentativeG, heuristic(neighbor, end));
                if (!isInOpenList(openList, neighbor)) {
                    openList.append(neighborNode);
                }
            }
        }
    }

    // 没有找到路径
    return QList<QPoint>();
}

QList<QPoint> Solver::reconstructPath(const QHash<QPoint, QPoint>& cameFrom, QPoint current)
{
    QList<QPoint> path;
    path.prepend(current);

    while (cameFrom.contains(current)) {
        current = cameFrom[current];
        path.prepend(current);
    }

    return path;
}

int Solver::heuristic(QPoint a, QPoint b)
{
    // 曼哈顿距离作为启发式函数
    return abs(a.x() - b.x()) + abs(a.y() - b.y());
}

QList<QPoint> Solver::getNeighbors(QPoint pos, const Map* map)
{
    QList<QPoint> neighbors;
    int x = pos.x();
    int y = pos.y();

    // 四个方向：上、下、左、右
    QPoint directions[4] = {
        QPoint(x, y - 1), // 上
        QPoint(x, y + 1), // 下
        QPoint(x - 1, y), // 左
        QPoint(x + 1, y)  // 右
    };

    for (const QPoint& dir : directions) {
        if (map->isValidPosition(dir.x(), dir.y()) && map->isWalkable(dir.x(), dir.y())) {
            neighbors.append(dir);
        }
    }

    return neighbors;
}

bool Solver::isInOpenList(const QList<Solver::Node>& openList, QPoint pos)
{
    for (const Solver::Node& node : openList) {
        if (node.pos == pos) {
            return true;
        }
    }
    return false;
}

Solver::Node Solver::getLowestFNode(const QList<Solver::Node>& openList)
{
    Solver::Node lowest = openList.first();
    for (const Solver::Node& node : openList) {
        if (node.f < lowest.f) {
            lowest = node;
        }
    }
    return lowest;
}

bool Solver::isValidPath(const QList<QPoint>& path, const Map* map)
{
    if (path.isEmpty()) return false;

    for (int i = 0; i < path.size() - 1; ++i) {
        QPoint current = path[i];
        QPoint next = path[i + 1];

        // 检查连续性（只能相邻移动）
        int dx = abs(current.x() - next.x());
        int dy = abs(current.y() - next.y());
        if ((dx + dy) != 1) return false; // 不是相邻移动

        // 检查下一个位置是否可行走
        if (!map->isWalkable(next.x(), next.y())) return false;
    }

    return true;
}

int Solver::getPathLength(const QList<QPoint>& path)
{
    return path.size();
}
