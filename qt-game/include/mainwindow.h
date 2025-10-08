#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QKeyEvent>
#include <QProgressBar>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QGroupBox>
#include <QStackedWidget>
#include <QCheckBox>
#include <QFileDialog>
#include "game.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

protected:
    void keyPressEvent(QKeyEvent *event) override;

private slots:
    void loadMap1();
    void loadMap2();
    void loadRandomMap();
    void loadImageMap();
    void startGame();
    void pauseGame();
    void resetGame();
    void showMenu();
    void toggleFogMode(bool enabled);
    void startAutoSolve();
    void onGameOver();
    void onVictory();
    void onHealthChanged(int health);
    void onScoreChanged(int score);
    void onTrapCountChanged(int count);

private:
    Ui::MainWindow *ui;
    void setupUI();
    void setupMenu();
    void setupGameArea();
    void setupControlPanel();
    void updateUI();

    // UI Components
    QStackedWidget *stackedWidget;
    QWidget *menuWidget;
    QWidget *gameWidget;
    QGraphicsScene *scene;
    QGraphicsView *gameView;
    
    // Menu components
    QPushButton *map1Button;
    QPushButton *map2Button;
    QPushButton *randomMapButton;
    QPushButton *imageMapButton;
    QPushButton *startButton;
    QCheckBox *fogModeCheckBox;
    
    // Game components
    QPushButton *pauseButton;
    QPushButton *resetButton;
    QPushButton *menuButton;
    QPushButton *autoSolveButton;
    QProgressBar *healthBar;
    QLabel *scoreLabel;
    QLabel *healthLabel;
    QLabel *trapCountLabel;
    QLabel *instructionsLabel;
    
    // Game logic
    Game *game;
    bool gameStarted;

};
#endif // MAINWINDOW_H
