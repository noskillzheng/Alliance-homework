#ifndef SOLVER_H
#define SOLVER_H

#include <QList>
#include <QPoint>
#include <QVector>
#include <QPair>

class Map;

class Solver
{
public:
    struct Node {
        QPoint pos;
        int g; // 代价从起点到当前节点
        int h; // 启发式代价从当前节点到终点
        int f; // 总代价 g + h
        QPoint parent;

        Node(QPoint p = QPoint(), int g = 0, int h = 0, QPoint parent = QPoint(-1, -1))
            : pos(p), g(g), h(h), f(g + h), parent(parent) {}

        bool operator>(const Node& other) const { return f > other.f; }
        bool operator==(const Node& other) const { return pos == other.pos; }
        bool operator==(const QPoint& other) const { return pos == other; }
    };

    Solver();
    ~Solver();

    // 寻路算法
    QList<QPoint> findPath(const Map* map, QPoint start, QPoint end);

    // 检查路径是否有效
    bool isValidPath(const QList<QPoint>& path, const Map* map);

    // 获取路径长度
    int getPathLength(const QList<QPoint>& path);

private:
    // A*算法辅助函数
    QList<QPoint> reconstructPath(const QHash<QPoint, QPoint>& cameFrom, QPoint current);
    int heuristic(QPoint a, QPoint b);
    QList<QPoint> getNeighbors(QPoint pos, const Map* map);

    // 检查节点是否已在开放列表中
    bool isInOpenList(const QList<Solver::Node>& openList, QPoint pos);

    // 获取开放列表中最低f值的节点
    Solver::Node getLowestFNode(const QList<Solver::Node>& openList);
};

#endif // SOLVER_H
