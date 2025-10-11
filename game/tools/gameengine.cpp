#include "GameEngine.h"
#include <iostream>
#include <cctype> // tolower

GameEngine::GameEngine() : gameRunning(true) {
    loadMaps();
}

void GameEngine::loadMaps() {
    // 假设有两个地图文件
    mapFiles.push_back("map1.txt");
    mapFiles.push_back("map2.txt");
}

int GameEngine::showMainMenu() {
    int choice;
    std::cout << "===== Maze Game =====" << std::endl;
    for (size_t i = 0; i < mapFiles.size(); i++) {
        std::cout << i + 1 << ". Play " << mapFiles[i] << std::endl;
    }
    std::cout << mapFiles.size() + 1 << ". Exit" << std::endl;
    std::cout << "Select: ";
    std::cin >> choice;
    return choice;
}

void GameEngine::gameLoop(Map& map, Player& player) {
    // 初始位置设为地图上的'P'（玩家初始位置）？但这里我们为了简单，在main中初始化玩家位置
    // 我们假设地图上有一个起始点'S'，玩家将放在那里
    // 陷阱用'T'表示，当玩家走到陷阱上时，受到伤害，陷阱消失（变为空格）

    char input;
    bool inGame = true;

    while (inGame) {
        // 显示地图
        system("clear"); // Linux下清屏
        map.render();

        // 显示玩家状态
        int x, y;
        player.getPosition(x, y);
        std::cout << "Position: (" << x << ", " << y << ") HP: " << player.getHP() << std::endl;
        std::cout << "Use WASD to move, Q to quit: ";
        std::cin >> input;
        input = std::tolower(input);

        if (input == 'q') {
            inGame = false;
            break;
        }

        // 移动玩家
        if (player.move(input, map)) {
            // 检查陷阱
            char cell = map.getCell(x, y);
            if (cell == 'T') {
                player.takeDamage(10);
                map.setCell(x, y, ' '); // 陷阱消失
            }

            // 检查终点？这里没有设定终点，可以自行扩展
        }

        // 检查玩家死亡
        if (player.getHP() <= 0) {
            std::cout << "You died!" << std::endl;
            inGame = false;
        }
    }
}

void GameEngine::run() {
    while (gameRunning) {
        int choice = showMainMenu();
        if (choice == static_cast<int>(mapFiles.size() + 1)) {
            gameRunning = false;
            break;
        } else if (choice >= 1 && choice <= static_cast<int>(mapFiles.size())) {
            // 加载选择的地图
            Map map(mapFiles[choice - 1]);

            // 初始化玩家：需要在地图上找到起始点'S'
            int startX = -1, startY = -1;
            for (int y = 0; y < map.getHeight(); y++) {
                for (int x = 0; x < map.getWidth(); x++) {
                    if (map.getCell(x, y) == 'S') {
                        startX = x;
                        startY = y;
                        break;
                    }
                }
                if (startX != -1) break;
            }

            if (startX == -1) {
                std::cerr << "No start position found on the map!" << std::endl;
                continue;
            }

            Player player(startX, startY, 100); // 初始100点生命

            // 进入游戏循环
            gameLoop(map, player);
        } else {
            std::cout << "Invalid choice!" << std::endl;
        }
    }
}
