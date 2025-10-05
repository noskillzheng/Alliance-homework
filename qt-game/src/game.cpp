#include "../include/game.h"
#include <QDebug>

Game::Game(QGraphicsScene* scene, QObject* parent)
    : QObject(parent)
    , m_scene(scene)
    , m_player(nullptr)
    , m_map(nullptr)
    , m_fog(nullptr)
    , m_solver(nullptr)
    , m_score(0)
    , m_state(MENU)
    , m_fogMode(false)
    , m_autoSolving(false)
    , m_currentPathIndex(0)
{
    m_gameTimer = new QTimer(this);
    m_collisionTimer = new QTimer(this);
    m_trapSpawnTimer = new QTimer(this);
    m_autoMoveTimer = new QTimer(this);

    connect(m_gameTimer, &QTimer::timeout, this, &Game::updateGame);
    connect(m_collisionTimer, &QTimer::timeout, this, &Game::checkCollisions);
    connect(m_trapSpawnTimer, &QTimer::timeout, this, &Game::spawnRandomTrap);
    connect(m_autoMoveTimer, &QTimer::timeout, this, &Game::autoMoveStep);

    m_collisionTimer->start(100); // Check collisions every 100ms
    m_trapSpawnTimer->start(3000); // Spawn trap every 3 seconds

    m_solver = new Solver();
}

Game::~Game()
{
    clearTraps();
    delete m_player;
    delete m_map;
    delete m_fog;
    delete m_solver;
}

void Game::startGame()
{
    if (m_state == MENU) {
        setupGame();
        m_state = PLAYING;
        m_gameTimer->start(16); // ~60 FPS
        m_trapSpawnTimer->start(3000); // 重新启动陷阱生成
        if (m_autoSolving) {
            m_autoMoveTimer->start(200); // 启动自动移动
        }
    }
}

void Game::resetGame()
{
    m_state = PLAYING; // 重置后应该是游戏进行中的状态，玩家在起点
    m_score = 0;
    // m_fogMode保持原状，不在重置时改变迷雾模式状态
    m_autoSolving = false;
    m_solutionPath.clear();
    m_currentPathIndex = 0;

    // 清理玩家和陷阱，但保持地图
    if (m_player) {
        m_scene->removeItem(m_player);
        delete m_player;
        m_player = nullptr;
    }

    clearTraps();

    if (m_fog) {
        m_fog->clearFog(m_scene);
        delete m_fog;
        m_fog = nullptr;
    }

    m_gameTimer->stop();
    m_trapSpawnTimer->stop();
    m_autoMoveTimer->stop();

    // 重新绘制地图（如果有的话）
    if (m_map) {
        m_map->clearMap(m_scene);
        m_map->drawMap(m_scene);

        // 创建玩家在起点位置
        QPoint startPos = m_map->getStartPosition();
        m_player = new Player(startPos.x(), startPos.y(), 100);
        m_scene->addItem(m_player);
        m_player->setZValue(5);

        // 如果迷雾模式开启，重新初始化迷雾
        if (m_fogMode) {
            if (m_fog) {
                m_fog->clearFog(m_scene);
                delete m_fog;
            }
            m_fog = new Fog(m_map->getWidth(), m_map->getHeight());
            m_fog->initializeFog(m_scene);
            m_fog->updateVisibility(startPos.x(), startPos.y(), 2); // 5x5方形视野
        }
    }

    emit scoreChanged(m_score);
    if (m_player) {
        emit healthChanged(m_player->getHealth());
    }
}

void Game::pauseGame()
{
    if (m_state == PLAYING) {
        m_state = PAUSED;
        m_gameTimer->stop();
        m_trapSpawnTimer->stop();
        m_autoMoveTimer->stop();
    }
}

void Game::resumeGame()
{
    if (m_state == PAUSED) {
        m_state = PLAYING;
        m_gameTimer->start(16);
        m_trapSpawnTimer->start(3000); // 重新启动陷阱生成
        if (m_autoSolving) {
            m_autoMoveTimer->start(200); // 重新启动自动移动
        }
    }
}

void Game::loadMap1()
{
    if (m_map) {
        m_map->clearMap(m_scene);
        delete m_map;
    }
    
    m_map = new Map(20, 20);
    m_map->generateMap1();
    m_map->drawMap(m_scene);
    
    if (m_state == MENU) {
        setupGame();
    }
}

void Game::loadMap2()
{
    if (m_map) {
        m_map->clearMap(m_scene);
        delete m_map;
    }
    
    m_map = new Map(20, 20);
    m_map->generateMap2();
    m_map->drawMap(m_scene);
    
    if (m_state == MENU) {
        setupGame();
    }
}

bool Game::loadMapFromImage(const QString& imagePath)
{
    if (m_map) {
        m_map->clearMap(m_scene);
        delete m_map;
    }

    m_map = new Map(20, 20);
    if (m_map->generateMapFromImage(imagePath)) {
        m_map->drawMap(m_scene);

        if (m_state == MENU) {
            setupGame();
        }
        return true;
    } else {
        // 如果图片加载失败，回退到默认地图
        qDebug() << "图片地图加载失败，使用默认地图1";
        m_map->generateMap1();
        m_map->drawMap(m_scene);

        if (m_state == MENU) {
            setupGame();
        }
        return false;
    }
}

void Game::loadRandomMap()
{
    if (m_map) {
        m_map->clearMap(m_scene);
        delete m_map;
    }
    
    m_map = new Map(20, 20);
    m_map->generateRandomMap();
    m_map->drawMap(m_scene);
    
    if (m_state == MENU) {
        setupGame();
    }
}


void Game::movePlayer(int dx, int dy)
{
    if (m_state != PLAYING || !m_player || !m_map) return;
    
    if (m_player->move(dx, dy, m_map->getMapData(), m_map->getWidth(), m_map->getHeight())) {
        // 更新迷雾视野
        if (m_fogMode && m_fog) {
            m_fog->updateVisibility(m_player->getX(), m_player->getY(), 2); // 5x5方形视野
        }
        
        checkTrapCollisions();
        checkExitCollision();
    }
}

bool Game::isGameOver() const
{
    return m_state == GAME_OVER;
}

bool Game::isVictory() const
{
    return m_state == VICTORY;
}

int Game::getHealth() const
{
    return m_player ? m_player->getHealth() : 0;
}

void Game::setupGame()
{
    if (!m_map) return;
    
    // Create player at start position
    QPoint startPos = m_map->getStartPosition();
    if (m_player) {
        m_scene->removeItem(m_player);
        delete m_player;
    }
    
    m_player = new Player(startPos.x(), startPos.y(), 100);
    m_scene->addItem(m_player);
    
    // 设置玩家层级
    m_player->setZValue(5);
    
    // 初始化迷雾系统
    if (m_fogMode) {
        if (m_fog) {
            m_fog->clearFog(m_scene);
            delete m_fog;
        }
        m_fog = new Fog(m_map->getWidth(), m_map->getHeight());
        m_fog->initializeFog(m_scene);
        m_fog->updateVisibility(startPos.x(), startPos.y(), 2); // 5x5方形视野
    }
    
    // Create traps
    createTraps();
    
    // Reset score
    m_score = 0;
    emit scoreChanged(m_score);
    emit healthChanged(m_player->getHealth());
}

void Game::createTraps()
{
    clearTraps();

    if (!m_map) return;

    // 游戏开始时不立即生成陷阱，由定时器控制生成
    // 保持原有陷阱位置信息，但不立即创建陷阱对象
}

void Game::clearTraps()
{
    for (Trap* trap : m_traps) {
        m_scene->removeItem(trap);
        delete trap;
    }
    m_traps.clear();
}

void Game::checkTrapCollisions()
{
    if (!m_player || m_state != PLAYING) return;
    
    for (int i = m_traps.size() - 1; i >= 0; --i) {
        Trap* trap = m_traps[i];
        if (trap->isActive() && 
            m_player->getX() == trap->getX() && 
            m_player->getY() == trap->getY()) {
            
            // Trigger trap and show visual effect
            trap->trigger(true); // 标记为玩家触发
            
            // Damage player
            m_player->takeDamage(trap->getDamage());
            emit healthChanged(m_player->getHealth());
            
            // Deactivate trap but don't remove immediately
            // The trap will be removed by its own timer after 0.5 seconds
            trap->deactivate();
            
            // Update map to remove trap
            if (m_map) {
                m_map->setCell(m_player->getX(), m_player->getY(), 0); // Set to path
            }
            
            if (!m_player->isAlive()) {
                m_state = GAME_OVER;
                m_gameTimer->stop();
                emit gameOver();
                return;
            }
            
            // Only trigger one trap per collision check
            break;
        }
    }
}

void Game::checkExitCollision()
{
    if (!m_player || !m_map || m_state != PLAYING) return;
    
    QPoint exitPos = m_map->getExitPosition();
    if (m_player->getX() == exitPos.x() && m_player->getY() == exitPos.y()) {
        m_state = VICTORY;
        m_gameTimer->stop();
        updateScore(1000); // Bonus for completing
        emit victory();
    }
}

void Game::updateScore(int points)
{
    m_score += points;
    emit scoreChanged(m_score);
}

void Game::updateGame()
{
    // 定期清理过期陷阱
    removeExpiredTraps();
}

void Game::checkCollisions()
{
    if (m_state == PLAYING) {
        checkTrapCollisions();
        checkExitCollision();
        cleanupTriggeredTraps();
    }
}

void Game::cleanupTriggeredTraps()
{
    // 清理已触发且隐藏的陷阱
    for (int i = m_traps.size() - 1; i >= 0; --i) {
        Trap* trap = m_traps[i];
        if (!trap->isActive() && !trap->isVisible()) {
            m_scene->removeItem(trap);
            m_traps.removeAt(i);
            delete trap;
        }
    }
}

void Game::spawnRandomTrap()
{
    if (!m_map || !m_player || m_state != PLAYING) return;

    // 获取玩家位置
    int playerX = m_player->getX();
    int playerY = m_player->getY();

    // 随机选择陷阱位置，避免玩家脚下
    int attempts = 0;
    while (attempts < 50) {
        int x = QRandomGenerator::global()->bounded(1, 19);
        int y = QRandomGenerator::global()->bounded(1, 19);

        // 检查是否是玩家脚下
        if (x == playerX && y == playerY) {
            attempts++;
            continue;
        }

        // 检查是否是路径位置
        if (m_map->isWalkable(x, y)) {
            // 检查该位置是否已有陷阱
            bool hasTrap = false;
            for (Trap* trap : m_traps) {
                if (trap->getX() == x && trap->getY() == y) {
                    hasTrap = true;
                    break;
                }
            }

            if (!hasTrap) {
                Trap* trap = new Trap(x, y, 20);
                m_scene->addItem(trap);
                m_traps.append(trap);
                return;
            }
        }

        attempts++;
    }
}

void Game::removeExpiredTraps()
{
    for (int i = m_traps.size() - 1; i >= 0; --i) {
        Trap* trap = m_traps[i];
        if (trap->isExpired()) {
            m_scene->removeItem(trap);
            m_traps.removeAt(i);
            delete trap;
        }
    }
}

void Game::setFogMode(bool enabled)
{
    m_fogMode = enabled;

    if (m_fogMode && m_map) {
        // 如果启用迷雾模式且已有地图，立即初始化迷雾
        if (m_fog) {
            m_fog->clearFog(m_scene);
            delete m_fog;
        }
        m_fog = new Fog(m_map->getWidth(), m_map->getHeight());
        m_fog->initializeFog(m_scene);

        // 如果有玩家，更新视野
        if (m_player) {
            m_fog->updateVisibility(m_player->getX(), m_player->getY(), 2); // 5x5方形视野
        }
    } else if (!m_fogMode && m_fog) {
        // 如果禁用迷雾模式，清理迷雾
        m_fog->clearFog(m_scene);
        delete m_fog;
        m_fog = nullptr;
    }
}

void Game::startAutoSolve()
{
    if (!m_map || !m_player || m_state != PLAYING) return;

    QPoint start = QPoint(m_player->getX(), m_player->getY());
    QPoint end = m_map->getExitPosition();

    // 使用A*算法寻找路径
    m_solutionPath = m_solver->findPath(m_map, start, end);

    if (!m_solutionPath.isEmpty()) {
        m_autoSolving = true;
        m_currentPathIndex = 0;
        m_autoMoveTimer->start(200); // 每200ms移动一步

        qDebug() << "自动寻路开始，路径长度：" << m_solutionPath.size();
    } else {
        qDebug() << "无法找到路径";
    }
}

void Game::stopAutoSolve()
{
    m_autoSolving = false;
    m_autoMoveTimer->stop();
    m_solutionPath.clear();
    m_currentPathIndex = 0;
}

void Game::autoMoveStep()
{
    if (!m_autoSolving || m_solutionPath.isEmpty() || !m_player) return;

    // 检查是否到达终点
    QPoint currentPos = QPoint(m_player->getX(), m_player->getY());
    QPoint endPos = m_map->getExitPosition();

    if (currentPos == endPos) {
        stopAutoSolve();
        return;
    }

    // 检查是否到达路径终点
    if (m_currentPathIndex >= m_solutionPath.size() - 1) {
        stopAutoSolve();
        return;
    }

    // 移动到下一个路径点
    QPoint nextPos = m_solutionPath[m_currentPathIndex + 1];
    int dx = nextPos.x() - currentPos.x();
    int dy = nextPos.y() - currentPos.y();

    // 尝试移动
    if (m_player->move(dx, dy, m_map->getMapData(), m_map->getWidth(), m_map->getHeight())) {
        m_currentPathIndex++;

        // 更新迷雾视野
        if (m_fogMode && m_fog) {
            m_fog->updateVisibility(m_player->getX(), m_player->getY(), 2); // 5x5方形视野
        }

        // 检查碰撞
        checkTrapCollisions();
        checkExitCollision();
    } else {
        // 如果移动失败，停止自动寻路
        stopAutoSolve();
    }
}

