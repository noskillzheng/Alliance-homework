#ifndef PLAYER_H
#define PLAYER_H

class Map;  // 前向声明

class Player {
public:
    Player(int startX, int startY, int hp);

    // 移动函数，返回是否移动成功（如果撞墙则失败）
    bool move(char direction, const Map& map);

    // 获取位置
    void getPosition(int& x, int& y) const;

    // 获取生命值
    int getHP() const;

    // 受伤
    void takeDamage(int damage);

private:
    int x;
    int y;
    int hp;
};

#endif // PLAYER_H
