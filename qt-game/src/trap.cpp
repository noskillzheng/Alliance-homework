#include "../include/trap.h"
#include <QBrush>
#include <QPen>

Trap::Trap(int x, int y, int damage)
    : QGraphicsRectItem(x * 30, y * 30, 30, 30)
    , m_x(x)
    , m_y(y)
    , m_damage(damage)
    , m_active(true)
    , m_triggered(false)
    , m_triggeredByPlayer(false)
    , m_expired(false)
<<<<<<< HEAD
    , m_hasBlackMark(true)  // 初始显示
=======
>>>>>>> e3a6255ae8c87c7b077a5583bf70026efc77e1bc
{
    m_triggerTimer = new QTimer(this);
    m_triggerTimer->setSingleShot(true);
    connect(m_triggerTimer, &QTimer::timeout, this, &Trap::onTriggerTimeout);

<<<<<<< HEAD
    // 确保陷阱初始时可见
    setVisible(true);
    setZValue(2); // 设置陷阱的Z值，在玩家之下
=======
    m_respawnTimer = new QTimer(this);
    m_respawnTimer->setSingleShot(true);
    connect(m_respawnTimer, &QTimer::timeout, this, &Trap::onRespawnTimeout);
>>>>>>> e3a6255ae8c87c7b077a5583bf70026efc77e1bc

    updateVisual();
}

Trap::~Trap()
{
}

void Trap::activate()
{
    m_active = true;
    updateVisual();
}

void Trap::deactivate()
{
    m_active = false;
    updateVisual();
}

void Trap::trigger(bool byPlayer)
{
    if (!m_active || m_triggered) return;

    m_triggered = true;
    m_triggeredByPlayer = byPlayer;
    showTrap();

    // 启动0.5秒定时器
    m_triggerTimer->start(500);
}

void Trap::showTrap()
{
    // 触发后显示为红色
    setBrush(QBrush(Qt::red));
    setPen(QPen(Qt::darkRed, 2));
    setVisible(true);
}

void Trap::hideTrap()
{
<<<<<<< HEAD
    if (m_hasBlackMark) {
        // 如果有黑色标记，保持可见并显示黑色标记
        showBlackMark();
    } else {
        // 没有黑色标记时隐藏陷阱
        setVisible(false);
    }
    deactivate();
}

void Trap::onTriggerTimeout()
{
    // 0.5秒后隐藏陷阱
    hideTrap();
}


void Trap::setBlackMark(bool hasMark)
{
    m_hasBlackMark = hasMark;
    if (hasMark) {
        showBlackMark();
    }
}

void Trap::showBlackMark()
{
    // 显示黑色痕迹
    setBrush(QBrush(Qt::black));
    setPen(QPen(Qt::darkGray, 2));
    setVisible(true);
}

=======
    setVisible(false);
    deactivate();
}

void Trap::startRespawnTimer()
{
    if (!m_triggeredByPlayer) {
        // 如果不是玩家触发的，启动重新生成定时器（5秒后重新生成）
        m_respawnTimer->start(5000);
    } else {
        // 如果是玩家触发的，标记为过期，不再重新生成
        m_expired = true;
    }
}

void Trap::markAsExpired()
{
    m_expired = true;
}

void Trap::onTriggerTimeout()
{
    // 0.5秒后隐藏陷阱并启动重新生成定时器
    hideTrap();
    startRespawnTimer();
}

void Trap::onRespawnTimeout()
{
    if (!m_expired) {
        // 重新激活陷阱
        m_triggered = false;
        m_triggeredByPlayer = false;
        activate();
        setVisible(true);
    }
}

>>>>>>> e3a6255ae8c87c7b077a5583bf70026efc77e1bc
void Trap::updateVisual()
{
    if (m_triggered) {
        // 已触发状态显示为红色
        setBrush(QBrush(Qt::red));
        setPen(QPen(Qt::darkRed, 2));
<<<<<<< HEAD
        setVisible(true);
    } else if (m_hasBlackMark) {
        // 鼹鼠现身状态显示为黑色
        setBrush(QBrush(Qt::black));
        setPen(QPen(Qt::darkGray, 2));
        setVisible(true);
    } else {
        // 鼹鼠隐藏状态
        setVisible(false);
    }
}

void Trap::showMole()
{
    m_hasBlackMark = true;
    updateVisual();
}

void Trap::hideMole()
{
    m_hasBlackMark = false;
    updateVisual();
}

void Trap::updatePosition(int x, int y)
{
    m_x = x;
    m_y = y;
    setPos(x * 30, y * 30);
=======
    } else if (m_active) {
        // 未触发状态与路径颜色相同（白色）
        setBrush(QBrush(Qt::white));
        setPen(QPen(Qt::black, 1));
    } else {
        // 非激活状态
        setBrush(QBrush(Qt::white));
        setPen(QPen(Qt::black, 1));
    }
>>>>>>> e3a6255ae8c87c7b077a5583bf70026efc77e1bc
}