#ifndef PLAYER_H
#define PLAYER_H

#include <QGraphicsRectItem>
#include <QPoint>

class Player : public QGraphicsRectItem
{
public:
    Player(int x = 0, int y = 0, int health = 100);
    
    // Getters
    int getX() const { return m_x; }
    int getY() const { return m_y; }
    int getHealth() const { return m_health; }
    bool isAlive() const { return m_health > 0; }
    
    // Movement
    bool move(int dx, int dy, const char* mapData, int mapWidth, int mapHeight);
    void setPosition(int x, int y);
    
    // Health management
    void takeDamage(int damage);
    void heal(int amount);
    void resetHealth();
    
    // Visual updates
    void updateVisual();

private:
    int m_x, m_y;
    int m_health;
    int m_maxHealth;
    QPoint m_startPosition;
};

#endif // PLAYER_H
