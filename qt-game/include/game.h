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
    
    // Game control
    void startGame();
    void resetGame();
    void pauseGame();
    void resumeGame();
    
    // Map management
    void loadMap1();
    void loadMap2();
    void loadRandomMap();
    bool loadMapFromImage(const QString& imagePath);
    
    // Fog mode
    void setFogMode(bool enabled);
    bool isFogMode() const { return m_fogMode; }

    // Auto solver
    void startAutoSolve();
    void stopAutoSolve();
    bool isAutoSolving() const { return m_autoSolving; }
    
    // Player control
    void movePlayer(int dx, int dy);
    bool isGameOver() const;
    bool isVictory() const;
    
    // Getters
    Player* getPlayer() const { return m_player; }
    Map* getMap() const { return m_map; }
    int getScore() const { return m_score; }
    int getHealth() const;
    
    // Game state
    enum GameState {
        MENU,
        PLAYING,
        PAUSED,
        GAME_OVER,
        VICTORY
    };
    
    GameState getState() const { return m_state; }

signals:
    void gameOver();
    void victory();
    void healthChanged(int health);
    void scoreChanged(int score);

private slots:
    void updateGame();
    void checkCollisions();
    void cleanupTriggeredTraps();
    void autoMoveStep();

private:
    QGraphicsScene* m_scene;
    Player* m_player;
    Map* m_map;
    Fog* m_fog;
    QList<Trap*> m_traps;
    QTimer* m_gameTimer;
    QTimer* m_collisionTimer;
    QTimer* m_trapSpawnTimer;
    QTimer* m_autoMoveTimer;

    int m_score;
    GameState m_state;
    bool m_fogMode;
    bool m_autoSolving;
    QList<QPoint> m_solutionPath;
    int m_currentPathIndex;
    Solver* m_solver;
    
    void setupGame();
    void createTraps();
    void clearTraps();
    void checkTrapCollisions();
    void checkExitCollision();
    void updateScore(int points);
    void spawnRandomTrap();
    void removeExpiredTraps();
};

#endif // GAME_H
