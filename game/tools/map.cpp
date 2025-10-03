#include "Map.h"
#include <fstream>
#include <iostream>

Map::Map(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open map file: " << filename << std::endl;
        exit(1);
    }

    std::string line;
    while (std::getline(file, line)) {
        grid.push_back(line);
    }

    if (grid.empty()) {
        std::cerr << "Map is empty!" << std::endl;
        exit(1);
    }

    height = grid.size();
    width = grid[0].size();

    // 确保每行长度相同
    for (int i = 0; i < height; i++) {
        if (static_cast<int>(grid[i].size()) != width) {
            std::cerr << "Map row " << i << " has inconsistent width!" << std::endl;
            exit(1);
        }
    }
}

void Map::render() const {
    for (const auto& row : grid) {
        std::cout << row << std::endl;
    }
}

char Map::getCell(int x, int y) const {
    if (isValidPosition(x, y)) {
        return grid[y][x];
    }
    return '?'; // 表示无效位置
}

void Map::setCell(int x, int y, char c) {
    if (isValidPosition(x, y)) {
        grid[y][x] = c;
    }
}

int Map::getWidth() const {
    return width;
}

int Map::getHeight() const {
    return height;
}

bool Map::isValidPosition(int x, int y) const {
    return x >= 0 && x < width && y >= 0 && y < height;
}

