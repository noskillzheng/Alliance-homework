#include "mainwindow.h"
#include "./ui_mainwindow.h"
#include <QPushButton>
#include <QVBoxLayout>
#include <QWidget>
#include <QGraphicsView>
#include <QDebug>
#include <QMessageBox>

const char MAP1[15][10] = {
    {1,1,1,1,1,1,1,1,1,1},
    {1,0,0,0,0,0,2,0,0,1},
    {1,1,1,0,1,1,1,0,1,1},
    {1,0,0,0,0,0,0,0,0,1},
    {1,0,1,1,1,0,1,1,0,1},
    {1,0,0,1,0,0,0,1,0,1},
    {1,1,0,1,0,1,0,1,0,3},
    {1,2,0,0,0,1,0,0,2,1},
    {1,0,1,1,1,1,1,1,0,1},
    {1,0,0,0,0,2,0,0,0,1},
    {1,1,1,1,1,1,0,1,0,1},
    {1,0,0,0,0,0,0,1,0,1},
    {1,0,1,1,1,1,1,1,0,1},
    {1,0,0,0,2,0,0,0,0,1},
    {1,1,1,1,1,1,1,1,1,1}
};

const char MAP2[15][10] = {
    {1,1,1,1,1,1,1,1,1,1},
    {1,0,1,0,0,0,1,2,0,1},
    {1,0,1,0,1,0,1,0,1,1},
    {1,0,0,0,1,0,0,0,0,1},
    {1,1,1,1,1,1,1,0,1,1},
    {1,0,0,0,2,0,0,0,2,1},
    {1,0,1,1,1,1,1,1,0,1},
    {1,0,0,0,0,0,0,1,0,1},
    {1,1,1,1,1,0,1,1,0,1},
    {1,0,0,2,0,0,0,1,0,1},
    {1,0,1,1,1,1,0,1,0,1},
    {1,0,0,0,0,1,0,0,0,1},
    {1,1,1,1,0,1,1,1,1,1},
    {1,0,2,0,0,2,0,0,0,1},
    {1,1,1,1,1,1,1,1,3,1}
};

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    ,ui(new Ui::MainWindow),health(100)
{
    ui->setupUi(this);
    resize(800, 600);
    setupMenu();
    setupGameScene();
    loadMap1(); // 默认加载地图1
    
    // 连接陷阱计时器
    connect(&trapTimer, &QTimer::timeout, this, &MainWindow::hideTrap);
}

MainWindow::~MainWindow()
{
    delete ui;
}
void MainWindow::setupMenu()
{
    QWidget *menuWidget = new QWidget(this);
    QVBoxLayout *layout = new QVBoxLayout(menuWidget);
    
    QPushButton *map1Button = new QPushButton("加载地图 1", this);
    QPushButton *map2Button = new QPushButton("加载地图 2", this);
    
    connect(map1Button, &QPushButton::clicked, this, &MainWindow::loadMap1);
    connect(map2Button, &QPushButton::clicked, this, &MainWindow::loadMap2);
    
    layout->addWidget(map1Button);
    layout->addWidget(map2Button);
    
    // 健康条
    healthBar = new QProgressBar(this);
    healthBar->setRange(0, 100);
    healthBar->setValue(health);
    healthBar->setTextVisible(true);
    healthBar->setFormat("HP: %v/100");
    layout->addWidget(healthBar);
    
    setCentralWidget(menuWidget);
}

void MainWindow::setupGameScene()
{
    scene = new QGraphicsScene(this);
    QGraphicsView *view = new QGraphicsView(scene, this);
    view->setFixedSize(600, 500);
    view->setSceneRect(0, 0, 400, 500);
    centralWidget()->layout()->addWidget(view);
}

void MainWindow::loadMap1()
{
    memcpy(currentMap, MAP1, sizeof(MAP1));
    resetGame();
}

void MainWindow::loadMap2()
{
    memcpy(currentMap, MAP2, sizeof(MAP2));
    resetGame();
}

void MainWindow::resetGame()
{
    scene->clear();
    traps.clear();
    trapPositions.clear();
    health = 100;
    updateHealthBar();
    
    // 初始化玩家位置 (1,1)
    playerX = 1;
    playerY = 1;
    // 查找出口位置
    exitX = -1;
    exitY = -1;
    for (int x = 0; x < 15; ++x) {
        for (int y = 0; y < 10; ++y) {
            if (currentMap[x][y] == 3) {
                exitX = x;
                exitY = y;
                break;
            }
        }
        if (exitX != -1) break;
    }
    
    // 绘制地图
    drawMap();
    
    // 创建玩家
    player = new QGraphicsRectItem(0, 0, 30, 30);
    player->setPos(playerX * 30, playerY * 30);
    player->setBrush(Qt::red);
    scene->addItem(player);
}

void MainWindow::drawMap()
{
    const int cellSize = 30;
    
    for (int y = 0; y < 10; ++y) {
        for (int x = 0; x < 15; ++x) {
            QGraphicsRectItem *item = new QGraphicsRectItem(x * cellSize, y * cellSize, cellSize, cellSize);
            
            if (currentMap[x][y] == 1) { // 墙
                item->setBrush(Qt::darkGray);
            } else { // 路径
                item->setBrush(Qt::lightGray);
                if (currentMap[x][y] == 2) { // 陷阱
                    trapPositions.append(QPoint(x, y));
                } else if (currentMap[x][y] == 3) { // 出口
                    item->setBrush(Qt::green); // 出口用绿色表示
                    exitItem = item; // 保存出口引用
                }
            }
            
            scene->addItem(item);
        }
    }
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    switch (event->key()) {
    case Qt::Key_W: movePlayer(0, -1); break;
    case Qt::Key_S: movePlayer(0, 1); break;
    case Qt::Key_A: movePlayer(-1, 0); break;
    case Qt::Key_D: movePlayer(1, 0); break;
    default: QMainWindow::keyPressEvent(event);
    }
}

void MainWindow::movePlayer(int dx, int dy)
{
    int newX = playerX + dx;
    int newY = playerY + dy;
    
    // 检查边界和墙壁
    if (newX >= 0 && newX < 15 && newY >= 0 && newY < 10 && 
        currentMap[newX][newY] != 1) {
        playerX = newX;
        playerY = newY;
        player->setPos(playerX * 30, playerY * 30);
        checkTrapCollision();
        checkExitReached();
    }
}

void MainWindow::checkTrapCollision()
{
    for (int i = 0; i < trapPositions.size(); ++i) {
        QPoint pos = trapPositions[i];
        if (playerX == pos.x() && playerY == pos.y()) {
            // 显示陷阱
            QGraphicsRectItem *trap = new QGraphicsRectItem(pos.x() * 30, pos.y() * 30, 30, 30);
            trap->setBrush(Qt::magenta);
            scene->addItem(trap);
            traps.append(trap);
            
            // 移除陷阱标记
            currentMap[pos.x()][pos.y()] = 0;
            trapPositions.removeAt(i);
            
            // 扣减生命值
            health -= 20;
            updateHealthBar();
            // 检查是否死亡
            if (health <= 0) {
                QMessageBox::information(this, "游戏结束", "你的血量耗尽！");
                resetGame();
            }
            
            // 启动隐藏计时器
            trapTimer.start(300);
            break;
        }
    }
}


void MainWindow::updateHealthBar()
{
    healthBar->setValue(health);
}

void MainWindow::checkExitReached()
{
    if (playerX == exitX && playerY == exitY) {
        showVictoryMessage();
    }
}

void MainWindow::showVictoryMessage()
{
    // 创建胜利消息框
    QMessageBox msgBox(this);
    msgBox.setWindowTitle("胜利!");
    msgBox.setText("恭喜你成功通关迷宫!");
    msgBox.setStandardButtons(QMessageBox::Ok);
    msgBox.setDefaultButton(QMessageBox::Ok);
    
    // 添加重新开始按钮
    QPushButton *restartButton = msgBox.addButton("重新开始", QMessageBox::ActionRole);
    
    // 添加返回菜单按钮
    QPushButton *menuButton = msgBox.addButton("返回菜单", QMessageBox::ActionRole);
    
    msgBox.exec();
    
    if (msgBox.clickedButton() == restartButton) {
        resetGame(); // 重新开始当前地图
    } else if (msgBox.clickedButton() == menuButton) {
        // 清除场景，返回菜单状态
        scene->clear();
        traps.clear();
        trapPositions.clear();
        player = nullptr;
        exitItem = nullptr;
    }
}

void MainWindow::hideTrap()
{
    for (QGraphicsRectItem *trap : traps) {
        scene->removeItem(trap);
        delete trap;
    }
    traps.clear();
    trapTimer.stop();
}