#include "../include/mainwindow.h"
#include "../include/ui_mainwindow.h"
#include <QMessageBox>
#include <QApplication>
#include <QDebug>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , game(nullptr)
    , gameStarted(false)
{
    ui->setupUi(this);
    setWindowTitle(QString::fromUtf8("迷宫探险 - Maze Adventure"));
    resize(1000, 700);
    
    setupUI();
    setupMenu();
    
    // Initialize game scene first
    scene = new QGraphicsScene(this);
    
    setupGameArea();
    setupControlPanel();
    
    // Initialize game
    game = new Game(scene, this);
    
    // Connect game signals
    connect(game, &Game::gameOver, this, &MainWindow::onGameOver);
    connect(game, &Game::victory, this, &MainWindow::onVictory);
    connect(game, &Game::healthChanged, this, &MainWindow::onHealthChanged);
    connect(game, &Game::scoreChanged, this, &MainWindow::onScoreChanged);
    connect(game, &Game::trapCountChanged, this, &MainWindow::onTrapCountChanged);

    // Show menu initially (don't load any map yet)
    stackedWidget->setCurrentWidget(menuWidget);
}

MainWindow::~MainWindow()
{
    delete ui;
    delete game;
}

void MainWindow::setupUI()
{
    // Create stacked widget for menu/game switching
    stackedWidget = new QStackedWidget(this);
    setCentralWidget(stackedWidget);
    
    // Create menu and game widgets
    menuWidget = new QWidget();
    gameWidget = new QWidget();
    
    stackedWidget->addWidget(menuWidget);
    stackedWidget->addWidget(gameWidget);
}

void MainWindow::setupMenu()
{
    QVBoxLayout *menuLayout = new QVBoxLayout(menuWidget);
    menuLayout->setAlignment(Qt::AlignCenter);
    
    // Title
    QLabel *titleLabel = new QLabel(QString::fromUtf8("迷宫探险"), this);
    titleLabel->setAlignment(Qt::AlignCenter);
    titleLabel->setStyleSheet("font-size: 32px; font-weight: bold; color: #2c3e50; margin: 20px;");
    menuLayout->addWidget(titleLabel);
    
    // Subtitle
    QLabel *subtitleLabel = new QLabel("Maze Adventure Game", this);
    subtitleLabel->setAlignment(Qt::AlignCenter);
    subtitleLabel->setStyleSheet("font-size: 16px; color: #7f8c8d; margin-bottom: 30px;");
    menuLayout->addWidget(subtitleLabel);
    
    // Map selection group
    QGroupBox *mapGroup = new QGroupBox(QString::fromUtf8("选择地图 / Select Map"), this);
    mapGroup->setStyleSheet("QGroupBox { font-weight: bold; }");
    QVBoxLayout *mapLayout = new QVBoxLayout(mapGroup);
    
    map1Button = new QPushButton(QString::fromUtf8("地图 1 - 经典递归迷宫"), this);
    map2Button = new QPushButton(QString::fromUtf8("地图 2 - 螺旋迷宫"), this);
    randomMapButton = new QPushButton(QString::fromUtf8("随机地图 - Prim算法"), this);
    imageMapButton = new QPushButton(QString::fromUtf8("从图片生成地图"), this);

    map1Button->setStyleSheet("QPushButton { padding: 10px; font-size: 14px; }");
    map2Button->setStyleSheet("QPushButton { padding: 10px; font-size: 14px; }");
    randomMapButton->setStyleSheet("QPushButton { padding: 10px; font-size: 14px; }");
    imageMapButton->setStyleSheet("QPushButton { padding: 10px; font-size: 14px; }");

    connect(map1Button, &QPushButton::clicked, this, &MainWindow::loadMap1);
    connect(map2Button, &QPushButton::clicked, this, &MainWindow::loadMap2);
    connect(randomMapButton, &QPushButton::clicked, this, &MainWindow::loadRandomMap);
    connect(imageMapButton, &QPushButton::clicked, this, &MainWindow::loadImageMap);

    mapLayout->addWidget(map1Button);
    mapLayout->addWidget(map2Button);
    mapLayout->addWidget(randomMapButton);
    mapLayout->addWidget(imageMapButton);
    
    menuLayout->addWidget(mapGroup);
    
    // Fog mode group
    QGroupBox *fogGroup = new QGroupBox(QString::fromUtf8("游戏模式 / Game Mode"), this);
    fogGroup->setStyleSheet("QGroupBox { font-weight: bold; }");
    QVBoxLayout *fogLayout = new QVBoxLayout(fogGroup);
    
    fogModeCheckBox = new QCheckBox(QString::fromUtf8("迷雾模式 / Fog Mode"), this);
    fogModeCheckBox->setStyleSheet("QCheckBox { font-size: 14px; padding: 5px; }");
    fogModeCheckBox->setToolTip(QString::fromUtf8("启用迷雾模式：地图初始为未知状态，只能看到周围5x5区域"));
    connect(fogModeCheckBox, &QCheckBox::toggled, this, &MainWindow::toggleFogMode);
    
    fogLayout->addWidget(fogModeCheckBox);
    menuLayout->addWidget(fogGroup);
    
    // Start button
    startButton = new QPushButton(QString::fromUtf8("开始游戏 / Start Game"), this);
    startButton->setStyleSheet("QPushButton { padding: 15px; font-size: 16px; font-weight: bold; background-color: #27ae60; color: white; border: none; border-radius: 5px; }");
    startButton->setEnabled(false); // Initially disabled, enabled after map selection
    connect(startButton, &QPushButton::clicked, this, &MainWindow::startGame);
    menuLayout->addWidget(startButton);
    
    // Instructions
    QLabel *instructionsLabel = new QLabel(
        QString::fromUtf8(
        "游戏说明 / Game Instructions:\n"
        "• 使用 WASD 键控制角色移动\n"
        "• 避开陷阱，保持血量\n"
        "• 找到绿色出口完成关卡\n"
        "• 红色方块是陷阱，触碰会扣血\n"
        "• 蓝色方块是起点，绿色方块是终点\n\n"
        "Controls: WASD keys to move\n"
        "Avoid traps (red squares) and reach the green exit!"
        )
    );
    instructionsLabel->setStyleSheet("font-size: 12px; color: #34495e; margin: 20px;");
    instructionsLabel->setWordWrap(true);
    menuLayout->addWidget(instructionsLabel);
    
    menuLayout->addStretch();
}

void MainWindow::setupGameArea()
{
    QVBoxLayout *gameLayout = new QVBoxLayout(gameWidget);
    
    // Game view
    gameView = new QGraphicsView(scene, this);
    gameView->setFixedSize(600, 600);
    gameView->setSceneRect(0, 0, 600, 600);
    gameView->setStyleSheet("border: 2px solid #34495e;");
    gameLayout->addWidget(gameView);
}

void MainWindow::setupControlPanel()
{
    QHBoxLayout *controlLayout = new QHBoxLayout();
    
    // Health bar
    healthLabel = new QLabel(QString::fromUtf8("血量:"), this);
    healthBar = new QProgressBar(this);
    healthBar->setRange(0, 100);
    healthBar->setValue(100);
    healthBar->setTextVisible(true);
    healthBar->setFormat("HP: %v/100");
    healthBar->setStyleSheet("QProgressBar { height: 20px; }");
    
    // Score
    scoreLabel = new QLabel(QString::fromUtf8("分数: 0"), this);
    scoreLabel->setStyleSheet("font-weight: bold; color: #2c3e50;");
    
    // Trap count
    trapCountLabel = new QLabel(QString::fromUtf8("剩余陷阱: 0"), this);
    trapCountLabel->setStyleSheet("font-weight: bold; color: #e74c3c;");
    
    // Control buttons
    pauseButton = new QPushButton(QString::fromUtf8("暂停"), this);
    resetButton = new QPushButton(QString::fromUtf8("重置"), this);
    menuButton = new QPushButton(QString::fromUtf8("返回菜单"), this);
    autoSolveButton = new QPushButton(QString::fromUtf8("自动寻路"), this);

    pauseButton->setStyleSheet("QPushButton { padding: 8px; }");
    resetButton->setStyleSheet("QPushButton { padding: 8px; }");
    menuButton->setStyleSheet("QPushButton { padding: 8px; }");
    autoSolveButton->setStyleSheet("QPushButton { padding: 8px; }");

    connect(pauseButton, &QPushButton::clicked, this, &MainWindow::pauseGame);
    connect(resetButton, &QPushButton::clicked, this, &MainWindow::resetGame);
    connect(menuButton, &QPushButton::clicked, this, &MainWindow::showMenu);
    connect(autoSolveButton, &QPushButton::clicked, this, &MainWindow::startAutoSolve);
    
    controlLayout->addWidget(healthLabel);
    controlLayout->addWidget(healthBar);
    controlLayout->addWidget(scoreLabel);
    controlLayout->addWidget(trapCountLabel);
    controlLayout->addStretch();
    controlLayout->addWidget(pauseButton);
    controlLayout->addWidget(resetButton);
    controlLayout->addWidget(autoSolveButton);
    controlLayout->addWidget(menuButton);
    
    // Add control panel to game widget
    QVBoxLayout *gameLayout = qobject_cast<QVBoxLayout*>(gameWidget->layout());
    if (gameLayout) {
        gameLayout->addLayout(controlLayout);
    }
}

void MainWindow::keyPressEvent(QKeyEvent *event)
{
    if (stackedWidget->currentWidget() == gameWidget && gameStarted) {
        switch (event->key()) {
        case Qt::Key_W:
            game->movePlayer(0, -1);
            break;
        case Qt::Key_S:
            game->movePlayer(0, 1);
            break;
        case Qt::Key_A:
            game->movePlayer(-1, 0);
            break;
        case Qt::Key_D:
            game->movePlayer(1, 0);
            break;
        case Qt::Key_Space:
            pauseGame();
            break;
        default:
            QMainWindow::keyPressEvent(event);
        }
    } else {
        QMainWindow::keyPressEvent(event);
    }
}

void MainWindow::loadMap1()
{
    if (game) {
        game->loadMap1();

        // Update button styles to show selected map
        map1Button->setStyleSheet("QPushButton { padding: 10px; font-size: 14px; background-color: #3498db; color: white; }");
        map2Button->setStyleSheet("QPushButton { padding: 10px; font-size: 14px; }");
        randomMapButton->setStyleSheet("QPushButton { padding: 10px; font-size: 14px; }");
        imageMapButton->setStyleSheet("QPushButton { padding: 10px; font-size: 14px; }");

        // Enable start button after map selection
        startButton->setEnabled(true);
    }
}

void MainWindow::loadMap2()
{
    if (game) {
        game->loadMap2();

        // Update button styles to show selected map
        map1Button->setStyleSheet("QPushButton { padding: 10px; font-size: 14px; }");
        map2Button->setStyleSheet("QPushButton { padding: 10px; font-size: 14px; background-color: #3498db; color: white; }");
        randomMapButton->setStyleSheet("QPushButton { padding: 10px; font-size: 14px; }");
        imageMapButton->setStyleSheet("QPushButton { padding: 10px; font-size: 14px; }");

        // Enable start button after map selection
        startButton->setEnabled(true);
    }
}

void MainWindow::loadRandomMap()
{
    if (game) {
        game->loadRandomMap();

        // Update button styles to show selected map
        map1Button->setStyleSheet("QPushButton { padding: 10px; font-size: 14px; }");
        map2Button->setStyleSheet("QPushButton { padding: 10px; font-size: 14px; }");
        randomMapButton->setStyleSheet("QPushButton { padding: 10px; font-size: 14px; background-color: #3498db; color: white; }");
        imageMapButton->setStyleSheet("QPushButton { padding: 10px; font-size: 14px; }");

        // Enable start button after map selection
        startButton->setEnabled(true);
    }
}

void MainWindow::startGame()
{
    if (game && !gameStarted) {
        // 确保地图已加载
        if (game->getMap() == nullptr) {
            QMessageBox::warning(this, QString::fromUtf8("警告"), QString::fromUtf8("请先选择一张地图！"));
            return;
        }

        game->startGame();
        stackedWidget->setCurrentWidget(gameWidget);
        gameStarted = true;
        updateUI();
    }
}

void MainWindow::toggleFogMode(bool enabled)
{
    if (game) {
        game->setFogMode(enabled);
    }
}

void MainWindow::loadImageMap()
{
    QString fileName = QFileDialog::getOpenFileName(this,
        tr("选择迷宫图片"), // 对话框标题
        "", // 默认路径
        tr("图片文件 (*.png *.jpg *.jpeg *.bmp *.gif *.tiff);;所有文件 (*)")); // 文件过滤器

    if (!fileName.isEmpty()) {
        if (game) {
            bool success = game->loadMapFromImage(fileName);

            if (success) {
                // 更新按钮样式表示当前选择的地图
                map1Button->setStyleSheet("QPushButton { padding: 10px; font-size: 14px; }");
                map2Button->setStyleSheet("QPushButton { padding: 10px; font-size: 14px; }");
                randomMapButton->setStyleSheet("QPushButton { padding: 10px; font-size: 14px; }");
                imageMapButton->setStyleSheet("QPushButton { padding: 10px; font-size: 14px; background-color: #3498db; color: white; }");

                // Enable start button after map selection
                startButton->setEnabled(true);

                QMessageBox::information(this, QString::fromUtf8("成功"),
                    QString(QString::fromUtf8("成功从图片生成迷宫地图！\n图片尺寸: %1x%2\n起点: (%3,%4)\n终点: (%5,%6)")).arg(
                        game->getMap()->getWidth()).arg(game->getMap()->getHeight())
                        .arg(game->getMap()->getStartPosition().x()).arg(game->getMap()->getStartPosition().y())
                        .arg(game->getMap()->getExitPosition().x()).arg(game->getMap()->getExitPosition().y()));
            } else {
                QMessageBox::warning(this, QString::fromUtf8("失败"),
                    QString::fromUtf8("无法从图片生成迷宫地图，将使用默认地图。"));
            }
        }
    }
}

void MainWindow::startAutoSolve()
{
    if (game && gameStarted) {
        if (game->isAutoSolving()) {
            game->stopAutoSolve();
            autoSolveButton->setText(QString::fromUtf8("自动寻路"));
        } else {
            game->startAutoSolve();
            autoSolveButton->setText(QString::fromUtf8("停止寻路"));
        }
    }
}

void MainWindow::pauseGame()
{
    if (game && gameStarted) {
        if (game->getState() == Game::PLAYING) {
            game->pauseGame();
            pauseButton->setText(QString::fromUtf8("继续"));
        } else if (game->getState() == Game::PAUSED) {
            game->resumeGame();
            pauseButton->setText(QString::fromUtf8("暂停"));
        }
    }
}

void MainWindow::resetGame()
{
    if (game) {
        game->resetGame();

        // 重置后游戏应该处于可玩状态，但不改变地图选择状态
        gameStarted = true;
        healthBar->setValue(100);
        scoreLabel->setText(QString::fromUtf8("分数: 0"));
        trapCountLabel->setText(QString::fromUtf8("剩余陷阱: 0"));
        pauseButton->setText(QString::fromUtf8("暂停"));
        autoSolveButton->setText(QString::fromUtf8("自动寻路"));

        // 如果是随机地图，重置时会重新生成地图，所以需要重新启用开始按钮
        if (game->getCurrentMapType() == Game::RANDOM_MAP) {
            startButton->setEnabled(true);
        }

        updateUI();
    }
}

void MainWindow::showMenu()
{
    if (game) {
        game->resetGame();
        gameStarted = false;
    }
    stackedWidget->setCurrentWidget(menuWidget);
    startButton->setEnabled(false); // Disable start button when returning to menu
    updateUI();
}

void MainWindow::onGameOver()
{
    QMessageBox::information(this, QString::fromUtf8("游戏结束"), QString::fromUtf8("你的血量耗尽了！\nGame Over - You ran out of health!"));
    showMenu();
}

void MainWindow::onVictory()
{
    QMessageBox::information(this, QString::fromUtf8("胜利！"),
        QString(QString::fromUtf8("恭喜你成功通关迷宫！\nCongratulations! You completed the maze!\n\n最终分数: %1")).arg(game->getScore()));
    autoSolveButton->setText(QString::fromUtf8("自动寻路"));
    showMenu();
}

void MainWindow::onHealthChanged(int health)
{
    healthBar->setValue(health);
}

void MainWindow::onScoreChanged(int score)
{
    scoreLabel->setText(QString(QString::fromUtf8("分数: %1")).arg(score));
}

void MainWindow::onTrapCountChanged(int count)
{
    trapCountLabel->setText(QString(QString::fromUtf8("剩余陷阱: %1")).arg(count));
}

void MainWindow::updateUI()
{
    if (gameStarted && game) {
        pauseButton->setEnabled(true);
        resetButton->setEnabled(true);
    } else {
        pauseButton->setEnabled(false);
        resetButton->setEnabled(false);
    }
}