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
    , m_hasBlackMark(true)  // 初始显示
{
    m_triggerTimer = new QTimer(this);
    m_triggerTimer->setSingleShot(true);
    connect(m_triggerTimer, &QTimer::timeout, this, &Trap::onTriggerTimeout);

    // 确保陷阱初始时可见
    setVisible(true);
    setZValue(2); // 设置陷阱的Z值，在玩家之下

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

void Trap::updateVisual()
{
    if (m_triggered) {
        // 已触发状态显示为红色
        setBrush(QBrush(Qt::red));
        setPen(QPen(Qt::darkRed, 2));
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
}