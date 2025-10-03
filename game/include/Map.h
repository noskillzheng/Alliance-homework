#ifndef MAP_H
#define MAP_H

#include <vector>
#include <string>

class Map {
public:
    // 构造函数，加载地图文件
    Map(const std::string& filename);

    // 显示地图
    void render() const;

    // 获取地图上的字符
    char getCell(int x, int y) const;

    // 设置地图上的字符
    void setCell(int x, int y, char c);

    // 获取地图尺寸
    int getWidth() const;
    int getHeight() const;

    // 检查坐标是否在地图范围内
    bool isValidPosition(int x, int y) const;

private:
    std::vector<std::string> grid;
    int width;
    int height;
};

#endif // MAP_H
