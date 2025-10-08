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
    , m_currentMapType(MAP1)
{
    m_gameTimer = new QTimer(this);
    m_collisionTimer = new QTimer(this);
    m_autoMoveTimer = new QTimer(this);

    connect(m_gameTimer, &QTimer::timeout, this, &Game::updateGame);
    connect(m_collisionTimer, &QTimer::timeout, this, &Game::checkCollisions);
    connect(m_autoMoveTimer, &QTimer::timeout, this, &Game::autoMoveStep);

    m_collisionTimer->start(100); // Check collisions every 100ms

    // 初始化陷阱系统
    m_maxTraps = 10; // 最大陷阱数量
    m_trapsVisible = false;
    m_trapGenerationTimer = new QTimer(this);
    connect(m_trapGenerationTimer, &QTimer::timeout, this, &Game::generateTrap);

    m_solver = new Solver();
}

Game::~Game()
{
    delete m_player;
    delete m_map;
    delete m_fog;
    delete m_solver;
    clearAllTraps();
}

void Game::startGame()
{
    if (m_state == MENU) {
        // 确保地图已加载（地图已在选择时绘制）
        if (m_map) {
            // 地图已经绘制，只需要初始化游戏逻辑
            setupGame();
        }

        m_state = PLAYING;
        m_gameTimer->start(16); // ~60 FPS
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
    
    // 重置陷阱管理
    m_blackMarkPositions.clear();
    m_maxTraps = 10; // 重置陷阱数量

    // 发送陷阱数量变化信号
    emit trapCountChanged(m_maxTraps);
    
    // 清除地图上的紫色印记和黑色痕迹
    if (m_map) {
        m_map->clearPurpleMarks();
        m_map->clearBlackMarks();
    }

    // 清理游戏中的黑色标记记录
    for (QGraphicsRectItem* blackMarkItem : m_blackMarkItems) {
        m_scene->removeItem(blackMarkItem);
        delete blackMarkItem;
    }
    m_blackMarkItems.clear();
    m_blackMarkPositions.clear();

    // 重置迷雾系统，清除所有陷阱痕迹
    if (m_fog) {
        m_fog->resetFog();
        // 如果有玩家，更新视野
        if (m_player) {
            QPoint startPos = m_map->getStartPosition();
            m_fog->updateVisibility(startPos.x(), startPos.y(), 2);
        }
        m_fog->drawFog(m_scene);
    }

    // 重置陷阱系统
    resetTraps();

    // 清理玩家和陷阱，但保持地图
    if (m_player) {
        m_scene->removeItem(m_player);
        delete m_player;
        m_player = nullptr;
    }

    if (m_fog) {
        m_fog->clearFog(m_scene);
        delete m_fog;
        m_fog = nullptr;
    }

    m_gameTimer->stop();
    m_trapGenerationTimer->stop();
    m_autoMoveTimer->stop();

    // 重新绘制地图（如果有的话）
    if (m_map) {
        // 如果是随机地图，重置时重新生成新的随机地图
        if (m_currentMapType == RANDOM_MAP) {
            m_map->clearMap(m_scene);
            delete m_map;
            m_map = new Map(20, 20);
            m_map->generateRandomMap();
            m_map->drawMap(m_scene);
        } else {
            m_map->clearMap(m_scene);
            m_map->drawMap(m_scene);
        }

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

        // 重新初始化陷阱系统
        initializeTraps();

        // 更新玩家当前位置的陷阱可见性
        updateTrapVisibilityForPlayer(startPos);
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
        m_trapGenerationTimer->stop();
        m_autoMoveTimer->stop();
    }
}

void Game::resumeGame()
{
    if (m_state == PAUSED) {
        m_state = PLAYING;
        m_gameTimer->start(16);
        m_trapGenerationTimer->start(2000); // 重新启动陷阱生成
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
    m_currentMapType = MAP1;

    // 立即绘制地图预览
    m_map->drawMap(m_scene);
}

void Game::loadMap2()
{
    if (m_map) {
        m_map->clearMap(m_scene);
        delete m_map;
    }

    m_map = new Map(20, 20);
    m_map->generateMap2();
    m_currentMapType = MAP2;

    // 立即绘制地图预览
    m_map->drawMap(m_scene);
}

bool Game::loadMapFromImage(const QString& imagePath)
{
    if (m_map) {
        m_map->clearMap(m_scene);
        delete m_map;
    }

    m_map = new Map(20, 20);
    if (m_map->generateMapFromImage(imagePath)) {
        m_currentMapType = IMAGE_MAP;
        // 立即绘制地图预览
        m_map->drawMap(m_scene);
        return true;
    } else {
        // 如果图片加载失败，回退到默认地图
        qDebug() << "图片地图加载失败，使用默认地图1";
        m_map->generateMap1();
        m_currentMapType = MAP1;
        // 立即绘制地图预览
        m_map->drawMap(m_scene);
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
    m_currentMapType = RANDOM_MAP;

    // 立即绘制地图预览
    m_map->drawMap(m_scene);
}


void Game::movePlayer(int dx, int dy)
{
    if (m_state != PLAYING || !m_player || !m_map) return;

    if (m_player->move(dx, dy, m_map->getMapData(), m_map->getWidth(), m_map->getHeight())) {
        QPoint newPlayerPos(m_player->getX(), m_player->getY());

        // 更新迷雾视野
        if (m_fogMode && m_fog) {
            m_fog->updateVisibility(m_player->getX(), m_player->getY(), 2); // 5x5方形视野
        }

        // 检查玩家当前位置是否是被触发的陷阱位置
        updateTrapVisibilityForPlayer(newPlayerPos);

        checkTrapCollisions();
        checkExitCollision();
    }
}

void Game::updateTrapVisibilityForPlayer(const QPoint& playerPos)
{
    if (!m_fog) return;

    // 检查所有被触发的陷阱位置
    for (int i = 0; i < m_traps.size(); ++i) {
        const GameTrap& trap = m_traps[i];
        if (trap.state == TRIGGERED) {
            QPoint trapPos = trap.position;

            // 如果玩家站在被触发的陷阱位置上
            if (playerPos == trapPos) {
                // 设置为透明，让玩家颜色显示
                m_fog->setFogState(trapPos.x(), trapPos.y(), Fog::VISIBLE);
            } else {
                // 玩家离开后，恢复黑色痕迹显示
                m_fog->setFogState(trapPos.x(), trapPos.y(), Fog::PERMANENT_MARK);
            }
        }
    }

    // 更新迷雾显示
    if (m_scene) {
        m_fog->drawFog(m_scene);
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

int Game::getTrapCount() const
{
    int count = 0;
    for (const GameTrap& trap : m_traps) {
        if (trap.state == ACTIVE || trap.state == DISAPPEARED) {
            count++;
        }
    }
    return count;
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

    // 初始化陷阱系统
    initializeTraps();

    // 更新玩家当前位置的陷阱可见性
    QPoint currentStartPos = m_map->getStartPosition();
    updateTrapVisibilityForPlayer(currentStartPos);

    // Reset score
    m_score = 0;
    emit scoreChanged(m_score);
    emit healthChanged(m_player->getHealth());
}

void Game::initializeTraps()
{
    if (!m_map) return;

    // 清理旧的陷阱
    clearAllTraps();

    // 初始化陷阱槽位
    m_traps.clear();
    for (int i = 0; i < m_maxTraps; ++i) {
        GameTrap trap;
        trap.position = QPoint(-1, -1);
        trap.state = INACTIVE;
        trap.timer = nullptr;
        trap.visualItem = nullptr;
        m_traps.append(trap);
    }

    qDebug() << "初始化了" << m_maxTraps << "个陷阱槽位";

    // 发送陷阱数量变化信号
    emit trapCountChanged(m_maxTraps);

    // 启动陷阱生成定时器，每2秒尝试生成一个陷阱
    m_trapGenerationTimer->start(2000);
}

QPoint Game::findValidTrapPosition()
{
    if (!m_map || !m_player) return QPoint(-1, -1);

    QPoint playerPos(m_player->getX(), m_player->getY());

    // 尝试100次找到有效位置
    for (int attempts = 0; attempts < 100; ++attempts) {
        int x = QRandomGenerator::global()->bounded(1, 19);
        int y = QRandomGenerator::global()->bounded(1, 19);

        // 检查是否是有效位置
        QPoint startPos = m_map->getStartPosition();
        QPoint exitPos = m_map->getExitPosition();

        if (m_map->getCell(x, y) == Map::PATH && // 只在白色路径上放置陷阱
            !isBlackMark(x, y) &&
            QPoint(x, y) != playerPos &&
            QPoint(x, y) != startPos &&
            QPoint(x, y) != exitPos &&
            abs(x - playerPos.x()) > 1 && abs(y - playerPos.y()) > 1 && // 不直接在玩家脚下
            m_map->isReachableFromStart(x, y)) { // 确保玩家可以到达

            // 检查是否与现有陷阱位置冲突
            bool positionConflict = false;
            for (const GameTrap& existingTrap : m_traps) {
                if (existingTrap.state != INACTIVE && existingTrap.state != TRIGGERED &&
                    existingTrap.position == QPoint(x, y)) {
                    positionConflict = true;
                    break;
                }
            }

            if (!positionConflict) {
                return QPoint(x, y);
            }
        }
    }

    return QPoint(-1, -1); // 未能找到有效位置
}

void Game::generateTrap()
{
    // 查找未激活的陷阱槽位
    for (int i = 0; i < m_traps.size(); ++i) {
        if (m_traps[i].state == INACTIVE) {
            QPoint pos = findValidTrapPosition();
            if (pos.x() != -1) {
                // 创建陷阱
                GameTrap& trap = m_traps[i];
                trap.position = pos;
                trap.state = ACTIVE;

                // 创建可视化图形项（初始为隐藏状态）
                trap.visualItem = new QGraphicsRectItem(pos.x() * 30, pos.y() * 30, 30, 30);
                trap.visualItem->setBrush(QBrush(Qt::white)); // 初始为白色（隐藏）
                trap.visualItem->setPen(QPen(Qt::black, 1));
                trap.visualItem->setVisible(true);
                trap.visualItem->setZValue(2);
                m_scene->addItem(trap.visualItem);

                // 创建定时器，3秒后消失
                trap.timer = new QTimer(this);
                trap.timer->setSingleShot(true);
                connect(trap.timer, &QTimer::timeout, this, &Game::onTrapTimeout);
                trap.timer->start(3000); // 3秒后消失

                qDebug() << "生成了陷阱在位置 (" << pos.x() << ", " << pos.y() << ")";
                break;
            }
        }
    }
}

void Game::onTrapTimeout()
{
    // 找到超时的陷阱
    for (int i = 0; i < m_traps.size(); ++i) {
        GameTrap& trap = m_traps[i];
        if (trap.timer && trap.timer->isActive() == false) { // 定时器超时
            if (trap.state == ACTIVE) {
                // 陷阱消失，等待重新生成
                trap.state = DISAPPEARED;

                // 隐藏陷阱显示（设为白色）
                if (trap.visualItem) {
                    trap.visualItem->setBrush(QBrush(Qt::white));
                }

                // 5秒后重新生成
                if (trap.timer) {
                    trap.timer->start(5000);
                }

                qDebug() << "陷阱在位置 (" << trap.position.x() << ", " << trap.position.y() << ") 消失";
            } else if (trap.state == DISAPPEARED) {
                // 重新生成陷阱
                QPoint newPos = findValidTrapPosition();
                if (newPos.x() != -1) {
                    // 移除旧的图形项
                    if (trap.visualItem) {
                        m_scene->removeItem(trap.visualItem);
                        delete trap.visualItem;
                    }

                    // 更新位置和状态
                    trap.position = newPos;
                    trap.state = ACTIVE;

                    // 创建新的图形项（显示为黑色）
                    trap.visualItem = new QGraphicsRectItem(newPos.x() * 30, newPos.y() * 30, 30, 30);
                    trap.visualItem->setBrush(QBrush(Qt::black)); // 显示为黑色印记
                    trap.visualItem->setPen(QPen(Qt::darkGray, 2));
                    trap.visualItem->setVisible(true);
                    trap.visualItem->setZValue(2);
                    m_scene->addItem(trap.visualItem);

                    // 3秒后消失
                    trap.timer->start(3000);

                    qDebug() << "陷阱重新生成在位置 (" << newPos.x() << ", " << newPos.y() << ")";
                } else {
                    // 找不到合适位置，设为非活跃
                    trap.state = INACTIVE;
                    if (trap.visualItem) {
                        m_scene->removeItem(trap.visualItem);
                        delete trap.visualItem;
                        trap.visualItem = nullptr;
                    }
                    if (trap.timer) {
                        delete trap.timer;
                        trap.timer = nullptr;
                    }
                }
            }
        }
    }
}

bool Game::isBlackMark(int x, int y) const
{
    return m_blackMarkPositions.contains(QPoint(x, y));
}


void Game::checkTrapCollisions()
{
    if (!m_player || m_state != PLAYING) return;

    QPoint playerPos(m_player->getX(), m_player->getY());

    // 检查玩家是否站在陷阱位置上
    for (int i = 0; i < m_traps.size(); ++i) {
        GameTrap& trap = m_traps[i];
        if (trap.state == ACTIVE && trap.position == playerPos) {
            // 播放音效或其他效果
            qDebug() << "Player hit a trap at (" << trap.position.x() << ", " << trap.position.y() << ")!";

            // 在地图上永久标记黑色痕迹
            if (m_map) {
                m_map->addBlackMark(playerPos.x(), playerPos.y());
            }
            m_blackMarkPositions.append(playerPos);

            // 在迷雾系统中留下永久黑色印记（这是实际显示的地方）
            if (m_fog) {
                m_fog->setFogState(playerPos.x(), playerPos.y(), Fog::PERMANENT_MARK); // 永久黑色痕迹
            }

            // 陷阱被触发，永久消失
            trap.state = TRIGGERED;

            // 移除图形项和定时器
            if (trap.visualItem) {
                m_scene->removeItem(trap.visualItem);
                delete trap.visualItem;
                trap.visualItem = nullptr;
            }
            if (trap.timer) {
                trap.timer->stop();
                delete trap.timer;
                trap.timer = nullptr;
            }

            // 伤害玩家
            m_player->takeDamage(20);
            emit healthChanged(m_player->getHealth());

            // 发送陷阱数量变化信号
            emit trapCountChanged(m_maxTraps);

            // 如果玩家死亡，游戏结束
            if (!m_player->isAlive()) {
                m_state = GAME_OVER;
                m_gameTimer->stop();
                m_trapGenerationTimer->stop();
                emit gameOver();
                return;
            }

            break; // 只处理一个陷阱碰撞
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
    // 游戏主循环
    // 更新迷雾显示，确保黑色痕迹可见
    if (m_fog && m_scene) {
        m_fog->drawFog(m_scene);
    }
}

void Game::checkCollisions()
{
    if (m_state == PLAYING) {
        checkTrapCollisions();
        checkExitCollision();
        // 陷阱碰撞检查已在checkTrapCollisions中处理
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







void Game::clearAllTraps()
{
    // 清理所有陷阱
    for (GameTrap& trap : m_traps) {
        if (trap.visualItem) {
            m_scene->removeItem(trap.visualItem);
            delete trap.visualItem;
        }
        if (trap.timer) {
            trap.timer->stop();
            delete trap.timer;
        }
    }
    m_traps.clear();

    // 清理所有黑色印记图形项
    for (QGraphicsRectItem* blackMarkItem : m_blackMarkItems) {
        m_scene->removeItem(blackMarkItem);
        delete blackMarkItem;
    }
    m_blackMarkItems.clear();
    m_blackMarkPositions.clear();
}

void Game::resetTraps()
{
    // 重置所有陷阱状态
    for (GameTrap& trap : m_traps) {
        if (trap.visualItem) {
            m_scene->removeItem(trap.visualItem);
            delete trap.visualItem;
            trap.visualItem = nullptr;
        }
        if (trap.timer) {
            trap.timer->stop();
            delete trap.timer;
            trap.timer = nullptr;
        }
        trap.position = QPoint(-1, -1);
        trap.state = INACTIVE;
    }
}

