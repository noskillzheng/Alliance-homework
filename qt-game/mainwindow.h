#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include <QTimer>
#include <QKeyEvent>
#include <QProgressBar>

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
    void hideTrap();

private:
    Ui::MainWindow *ui;
    void setupMenu();
    void setupGameScene();
    void resetGame();
    void drawMap();
    void movePlayer(int dx, int dy);
    void checkTrapCollision();
    void updateHealthBar();
    void checkExitReached(); // 检查是否到达出口
    void showVictoryMessage(); // 显示胜利消息

    QGraphicsScene *scene;
    QGraphicsRectItem *player;
    QGraphicsRectItem *exitItem; // 新增：出口图形项
    QList<QGraphicsRectItem*> traps;
    QList<QPoint> trapPositions;
    QProgressBar *healthBar;
    char currentMap[15][10]; // 15x10地图
    int playerX, playerY;
    int exitX, exitY; // 出口坐标
    int health;
    QTimer trapTimer;

};
#endif // MAINWINDOW_H
