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
<<<<<<< HEAD
        START = 4,
        PURPLE_MARK = 5,
        BLACK_MARK = 6
    };

    Map(int width = 20, int height = 20);
    ~Map();

=======
        START = 4
    };
    
    Map(int width = 20, int height = 20);
    ~Map();
    
>>>>>>> e3a6255ae8c87c7b077a5583bf70026efc77e1bc
    // Map management
    void loadMap(const char mapData[][20], int width, int height);
    void drawMap(QGraphicsScene* scene);
    void clearMap(QGraphicsScene* scene);
<<<<<<< HEAD

=======
    
>>>>>>> e3a6255ae8c87c7b077a5583bf70026efc77e1bc
    // Getters
    int getWidth() const { return m_width; }
    int getHeight() const { return m_height; }
    char getCell(int x, int y) const;
    const char* getMapData() const;
    QPoint getStartPosition() const { return m_startPosition; }
    QPoint getExitPosition() const { return m_exitPosition; }
    QList<QPoint> getTrapPositions() const { return m_trapPositions; }
<<<<<<< HEAD
    QList<QPoint> getPurpleMarkPositions() const { return m_purpleMarkPositions; }
    QList<QPoint> getBlackMarkPositions() const { return m_blackMarkPositions; }

=======
    
>>>>>>> e3a6255ae8c87c7b077a5583bf70026efc77e1bc
    // Cell operations
    void setCell(int x, int y, char value);
    bool isValidPosition(int x, int y) const;
    bool isWalkable(int x, int y) const;
<<<<<<< HEAD
    void addPurpleMark(int x, int y);
    void clearPurpleMarks();
    void addBlackMark(int x, int y);
    void clearBlackMarks();

=======
    
>>>>>>> e3a6255ae8c87c7b077a5583bf70026efc77e1bc
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
<<<<<<< HEAD
    QList<QPoint> m_purpleMarkPositions;
    QList<QPoint> m_blackMarkPositions;
    QList<QGraphicsRectItem*> m_graphicsItems;

    void findSpecialPositions();
    void dfsMazeGeneration(int startX, int startY);
    bool isValidForMaze(int x, int y);
    bool isReachable(int startX, int startY, int endX, int endY);
    void placeRandomTraps();
    void createSimplePath();
    void createMinimalPath();
public:
    bool isReachableFromStart(int x, int y);
=======
    QList<QGraphicsRectItem*> m_graphicsItems;
    
    void findSpecialPositions();
>>>>>>> e3a6255ae8c87c7b077a5583bf70026efc77e1bc
};

#endif // MAP_H
