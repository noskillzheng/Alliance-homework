#include "Player.h"
#include "Map.h"

Player::Player(int startX, int startY, int hp) : x(startX), y(startY), hp(hp) {}

bool Player::move(char direction, const Map& map) {
    int newX = x;
    int newY = y;

    switch (direction) {
        case 'w': newY--; break;
        case 's': newY++; break;
        case 'a': newX--; break;
        case 'd': newX++; break;
        default: return false; // 无效输入
    }

    // 检查新位置是否可走（非墙）
    if (map.isValidPosition(newX, newY)) {
        char cell = map.getCell(newX, newY);
        if (cell != '#') { // 假设'#'是墙
            x = newX;
            y = newY;
            return true;
        }
    }
    return false;
}

void Player::getPosition(int& x, int& y) const {
    x = this->x;
    y = this->y;
}

int Player::getHP() const {
    return hp;
}

void Player::takeDamage(int damage) {
    hp -= damage;
    if (hp < 0) hp = 0;
}

