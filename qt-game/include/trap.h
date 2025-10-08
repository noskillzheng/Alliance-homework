#ifndef TRAP_H
#define TRAP_H

#include <QGraphicsRectItem>
#include <QPoint>
#include <QTimer>
#include <QObject>

class Trap : public QObject, public QGraphicsRectItem
{
    Q_OBJECT


public:
    Trap(int x, int y, int damage = 20);
    ~Trap();

    // Getters
    int getX() const { return m_x; }
    int getY() const { return m_y; }
    int getDamage() const { return m_damage; }
    bool isActive() const { return m_active; }
    bool isTriggeredByPlayer() const { return m_triggeredByPlayer; }
    bool isExpired() const { return m_expired; }
    bool hasBlackMark() const { return m_hasBlackMark; }

    // Trap behavior
    void activate();
    void deactivate();
    void trigger(bool byPlayer = false);
    void setBlackMark(bool hasMark);
    
    // Mole trap behavior
    void showMole();
    void hideMole();
    void updatePosition(int x, int y);

    // Visual effects
    void showTrap();
    void hideTrap();
    void showBlackMark();

private slots:
    void onTriggerTimeout();

private:
    int m_x, m_y;
    int m_damage;
    bool m_active;
    bool m_triggered;
    bool m_triggeredByPlayer;
    bool m_expired;
    bool m_hasBlackMark;
    QTimer* m_triggerTimer;

    void updateVisual();
};

#endif // TRAP_H