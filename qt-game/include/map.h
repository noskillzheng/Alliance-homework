#ifndef MAP_H
#define MAP_H

#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include <QPoint>
#include <QList>

class Map
{
public:
    enum CellType {
        WALL = 1,
        PATH = 0,
        TRAP = 2,
        EXIT = 3,
        START = 4
    };
    
    Map(int width = 20, int height = 20);
    ~Map();
    
    // Map management
    void loadMap(const char mapData[][20], int width, int height);
    void drawMap(QGraphicsScene* scene);
    void clearMap(QGraphicsScene* scene);
    
    // Getters
    int getWidth() const { return m_width; }
    int getHeight() const { return m_height; }
    char getCell(int x, int y) const;
    const char* getMapData() const;
    QPoint getStartPosition() const { return m_startPosition; }
    QPoint getExitPosition() const { return m_exitPosition; }
    QList<QPoint> getTrapPositions() const { return m_trapPositions; }
    
    // Cell operations
    void setCell(int x, int y, char value);
    bool isValidPosition(int x, int y) const;
    bool isWalkable(int x, int y) const;
    
    // Map generation
    void generateMap1();
    void generateMap2();
    void generateRandomMap();
    bool generateMapFromImage(const QString& imagePath);

private:
    char m_map[20][20];
    int m_width, m_height;
    QPoint m_startPosition;
    QPoint m_exitPosition;
    QList<QPoint> m_trapPositions;
    QList<QGraphicsRectItem*> m_graphicsItems;
    
    void findSpecialPositions();
};

#endif // MAP_H
