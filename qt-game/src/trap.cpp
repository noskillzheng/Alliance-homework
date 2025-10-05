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
{
    m_triggerTimer = new QTimer(this);
    m_triggerTimer->setSingleShot(true);
    connect(m_triggerTimer, &QTimer::timeout, this, &Trap::onTriggerTimeout);

    m_respawnTimer = new QTimer(this);
    m_respawnTimer->setSingleShot(true);
    connect(m_respawnTimer, &QTimer::timeout, this, &Trap::onRespawnTimeout);

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

void Trap::updateVisual()
{
    if (m_triggered) {
        // 已触发状态显示为红色
        setBrush(QBrush(Qt::red));
        setPen(QPen(Qt::darkRed, 2));
    } else if (m_active) {
        // 未触发状态与路径颜色相同（白色）
        setBrush(QBrush(Qt::white));
        setPen(QPen(Qt::black, 1));
    } else {
        // 非激活状态
        setBrush(QBrush(Qt::white));
        setPen(QPen(Qt::black, 1));
    }
}