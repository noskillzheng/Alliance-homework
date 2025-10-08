#include "../include/player.h"
#include <QBrush>
#include <QPen>

Player::Player(int x, int y, int health)
    : QGraphicsRectItem(0, 0, 30, 30)
    , m_x(x)
    , m_y(y)
    , m_health(health)
    , m_maxHealth(health)
    , m_startPosition(x, y)
{
    setBrush(QBrush(Qt::red));
    setPen(QPen(Qt::black, 2));
    setPos(x * 30, y * 30);
}

bool Player::move(int dx, int dy, const char* mapData, int mapWidth, int mapHeight)
{
    int newX = m_x + dx;
    int newY = m_y + dy;
    
    // Check boundaries
    if (newX < 0 || newX >= mapWidth || newY < 0 || newY >= mapHeight) {
        return false;
    }
    
    // Get the cell type at the new position
    char cellType = mapData[newY * mapWidth + newX];
    
    // Only allow movement to paths (0), traps (2), exit (3), or start (4)
    // Block movement to walls (1)
    if (cellType == 1) { // WALL
        return false;
    }
    
    // Allow movement to other cell types
    m_x = newX;
    m_y = newY;
    setPos(m_x * 30, m_y * 30);
    return true;
}

void Player::setPosition(int x, int y)
{
    m_x = x;
    m_y = y;
    setPos(m_x * 30, m_y * 30);
}

void Player::takeDamage(int damage)
{
    m_health -= damage;
    if (m_health < 0) m_health = 0;
    updateVisual();
}

void Player::heal(int amount)
{
    m_health += amount;
    if (m_health > m_maxHealth) m_health = m_maxHealth;
    updateVisual();
}

void Player::resetHealth()
{
    m_health = m_maxHealth;
    updateVisual();
}

void Player::updateVisual()
{
    // Change color based on health
    if (m_health > 60) {
        setBrush(QBrush(Qt::red));
    } else if (m_health > 30) {
        setBrush(QBrush(Qt::yellow));
    } else {
        setBrush(QBrush(Qt::darkRed));
    }
}
