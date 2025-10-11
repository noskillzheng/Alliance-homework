#ifndef GAME_ENGINE_H
#define GAME_ENGINE_H

#include "Map.h"
#include "Player.h"
#include <vector>
#include <string>

class GameEngine {
public:
    GameEngine();

    // 运行游戏
    void run();

private:
    // 加载地图列表
    void loadMaps();

    // 显示主菜单
    int showMainMenu();

    // 游戏主循环
    void gameLoop(Map& map, Player& player);

    // 处理陷阱
    void handleTraps(Player& player, const Map& map);

    std::vector<std::string> mapFiles;
    bool gameRunning;
};

#endif // GAME_ENGINE_H

