#ifndef TRAP_H
#define TRAP_H

#include <QGraphicsRectItem>
#include <QPoint>
#include <QTimer>
#include <QObject>

class Trap : public QObject, public QGraphicsRectItem
{
    Q_OBJECT

<<<<<<< HEAD

=======
>>>>>>> e3a6255ae8c87c7b077a5583bf70026efc77e1bc
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
<<<<<<< HEAD
    bool hasBlackMark() const { return m_hasBlackMark; }
=======
>>>>>>> e3a6255ae8c87c7b077a5583bf70026efc77e1bc

    // Trap behavior
    void activate();
    void deactivate();
    void trigger(bool byPlayer = false);
<<<<<<< HEAD
    void setBlackMark(bool hasMark);
    
    // Mole trap behavior
    void showMole();
    void hideMole();
    void updatePosition(int x, int y);
=======
    void startRespawnTimer();
    void markAsExpired();
>>>>>>> e3a6255ae8c87c7b077a5583bf70026efc77e1bc

    // Visual effects
    void showTrap();
    void hideTrap();
<<<<<<< HEAD
    void showBlackMark();

private slots:
    void onTriggerTimeout();
=======

private slots:
    void onTriggerTimeout();
    void onRespawnTimeout();
>>>>>>> e3a6255ae8c87c7b077a5583bf70026efc77e1bc

private:
    int m_x, m_y;
    int m_damage;
    bool m_active;
    bool m_triggered;
    bool m_triggeredByPlayer;
    bool m_expired;
<<<<<<< HEAD
    bool m_hasBlackMark;
    QTimer* m_triggerTimer;
=======
    QTimer* m_triggerTimer;
    QTimer* m_respawnTimer;
>>>>>>> e3a6255ae8c87c7b077a5583bf70026efc77e1bc

    void updateVisual();
};

#endif // TRAP_H