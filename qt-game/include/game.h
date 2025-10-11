#ifndef GAME_H
#define GAME_H

#include <QObject>
#include <QGraphicsScene>
#include <QTimer>
#include <QList>
#include <QRandomGenerator>
#include "player.h"
#include "map.h"
#include "trap.h"
#include "fog.h"
#include "solver.h"

class Game : public QObject
{
    Q_OBJECT

public:
    explicit Game(QGraphicsScene* scene, QObject* parent = nullptr);
    ~Game();
<<<<<<< HEAD

=======
    
>>>>>>> e3a6255ae8c87c7b077a5583bf70026efc77e1bc
    // Game control
    void startGame();
    void resetGame();
    void pauseGame();
    void resumeGame();
<<<<<<< HEAD

=======
    
>>>>>>> e3a6255ae8c87c7b077a5583bf70026efc77e1bc
    // Map management
    void loadMap1();
    void loadMap2();
    void loadRandomMap();
    bool loadMapFromImage(const QString& imagePath);
<<<<<<< HEAD

=======
    
>>>>>>> e3a6255ae8c87c7b077a5583bf70026efc77e1bc
    // Fog mode
    void setFogMode(bool enabled);
    bool isFogMode() const { return m_fogMode; }

    // Auto solver
    void startAutoSolve();
    void stopAutoSolve();
    bool isAutoSolving() const { return m_autoSolving; }
<<<<<<< HEAD

=======
    
>>>>>>> e3a6255ae8c87c7b077a5583bf70026efc77e1bc
    // Player control
    void movePlayer(int dx, int dy);
    bool isGameOver() const;
    bool isVictory() const;
<<<<<<< HEAD

=======
    
>>>>>>> e3a6255ae8c87c7b077a5583bf70026efc77e1bc
    // Getters
    Player* getPlayer() const { return m_player; }
    Map* getMap() const { return m_map; }
    int getScore() const { return m_score; }
    int getHealth() const;
<<<<<<< HEAD
    int getTrapCount() const;

=======
    
>>>>>>> e3a6255ae8c87c7b077a5583bf70026efc77e1bc
    // Game state
    enum GameState {
        MENU,
        PLAYING,
        PAUSED,
        GAME_OVER,
        VICTORY
    };
<<<<<<< HEAD

    GameState getState() const { return m_state; }

    // Map types
    enum MapType {
        MAP1,
        MAP2,
        RANDOM_MAP,
        IMAGE_MAP
    };

    MapType getCurrentMapType() const { return m_currentMapType; }

=======
    
    GameState getState() const { return m_state; }

>>>>>>> e3a6255ae8c87c7b077a5583bf70026efc77e1bc
signals:
    void gameOver();
    void victory();
    void healthChanged(int health);
    void scoreChanged(int score);
<<<<<<< HEAD
    void trapCountChanged(int count);
=======
>>>>>>> e3a6255ae8c87c7b077a5583bf70026efc77e1bc

private slots:
    void updateGame();
    void checkCollisions();
<<<<<<< HEAD
=======
    void cleanupTriggeredTraps();
>>>>>>> e3a6255ae8c87c7b077a5583bf70026efc77e1bc
    void autoMoveStep();

private:
    QGraphicsScene* m_scene;
    Player* m_player;
    Map* m_map;
    Fog* m_fog;
<<<<<<< HEAD
    QTimer* m_gameTimer;
    QTimer* m_collisionTimer;
=======
    QList<Trap*> m_traps;
    QTimer* m_gameTimer;
    QTimer* m_collisionTimer;
    QTimer* m_trapSpawnTimer;
>>>>>>> e3a6255ae8c87c7b077a5583bf70026efc77e1bc
    QTimer* m_autoMoveTimer;

    int m_score;
    GameState m_state;
    bool m_fogMode;
    bool m_autoSolving;
    QList<QPoint> m_solutionPath;
    int m_currentPathIndex;
<<<<<<< HEAD
    MapType m_currentMapType;
    Solver* m_solver;
    
    // 陷阱系统
    enum TrapState {
        INACTIVE,    // 未激活槽位
        ACTIVE,      // 陷阱激活显示中
        DISAPPEARED, // 陷阱已消失，等待重新生成
        TRIGGERED    // 陷阱被触发，永久消失
    };

    struct GameTrap {
        QPoint position;
        TrapState state;
        QTimer* timer;
        QGraphicsRectItem* visualItem;
    };

    QList<GameTrap> m_traps;
    QList<QPoint> m_blackMarkPositions;
    QList<QGraphicsRectItem*> m_blackMarkItems;
    QTimer* m_trapGenerationTimer;
    int m_maxTraps;
    bool m_trapsVisible;

    void setupGame();
    void initializeTraps();
    void generateTrap();
    void onTrapTimeout();
    void updateTrapVisibilityForPlayer(const QPoint& playerPos);
    void checkTrapCollisions();
    void checkExitCollision();
    void updateScore(int points);
    bool isBlackMark(int x, int y) const;
    QPoint findValidTrapPosition();
    void clearAllTraps();
    void resetTraps();
=======
    Solver* m_solver;
    
    void setupGame();
    void createTraps();
    void clearTraps();
    void checkTrapCollisions();
    void checkExitCollision();
    void updateScore(int points);
    void spawnRandomTrap();
    void removeExpiredTraps();
>>>>>>> e3a6255ae8c87c7b077a5583bf70026efc77e1bc
};

#endif // GAME_H
