#include "utils/config_reader.hpp"
#include <iostream>
#include <fstream>
#include <algorithm>

namespace armor_detector {

ConfigReader::ConfigReader(const std::string& config_dir)
    : config_dir_(config_dir) {
    std::cout << "[ConfigReader] 配置读取器初始化，目录: " << config_dir_ << std::endl;
}

bool ConfigReader::loadConfig(const std::string& filename) {
    std::string filepath = config_dir_ + "/" + filename;
    
    // 根据扩展名选择解析方式
    if (filepath.find(".yaml") != std::string::npos || filepath.find(".yml") != std::string::npos) {
        return loadYAML(filepath);
    }
    return loadConf(filepath);
}

bool ConfigReader::loadYAML(const std::string& filepath) {
    std::ifstream file(filepath);
    
    if (!file.is_open()) {
        std::cerr << "[ConfigReader] 无法打开配置文件: " << filepath << std::endl;
        return false;
    }
    
    std::string line;
    std::vector<std::string> prefix_stack;  // 前缀栈，用于处理嵌套
    int last_indent = -1;
    
    while (std::getline(file, line)) {
        // 跳过空行和注释
        std::string trimmed = trim(line);
        if (trimmed.empty() || trimmed[0] == '#') {
            continue;
        }
        
        // 计算缩进（空格数）
        size_t indent = 0;
        while (indent < line.length() && line[indent] == ' ') {
            indent++;
        }
        
        // 根据缩进调整前缀栈（只在缩进减少时弹栈）
        if (static_cast<int>(indent) < last_indent) {
            // 缩进减少，弹出栈
            int levels = ((last_indent - static_cast<int>(indent)) / 2) + 1;
            for (int i = 0; i < levels && !prefix_stack.empty(); i++) {
                prefix_stack.pop_back();
            }
        }
        
        // 查找冒号
        size_t colon_pos = trimmed.find(':');
        if (colon_pos != std::string::npos) {
            std::string key = trim(trimmed.substr(0, colon_pos));
            std::string value = trim(trimmed.substr(colon_pos + 1));
            
            // 先移除行尾注释
            size_t comment_pos = value.find('#');
            if (comment_pos != std::string::npos) {
                value = trim(value.substr(0, comment_pos));
            }
            
            // 再移除引号（处理干净的value）
            if (!value.empty() && value.length() >= 2) {
                if ((value.front() == '"' && value.back() == '"') ||
                    (value.front() == '\'' && value.back() == '\'')) {
                    value = value.substr(1, value.length() - 2);
                }
            }
            
            if (value.empty()) {
                // 这是一个父节点，加入前缀栈
                prefix_stack.push_back(key);
            } else {
                // 这是一个叶子节点，保存配置
                std::string full_key = key;
                if (!prefix_stack.empty()) {
                    full_key = "";
                    for (const auto& prefix : prefix_stack) {
                        full_key += prefix + ".";
                    }
                    full_key += key;
                }
                config_[full_key] = value;
            }
        }
        
        last_indent = static_cast<int>(indent);
    }
    
    file.close();
    std::cout << "[ConfigReader] 成功加载YAML配置文件: " << filepath 
              << " (" << config_.size() << " 项配置)" << std::endl;
    return true;
}

bool ConfigReader::loadConf(const std::string& filepath) {
    std::ifstream file(filepath);
    
    if (!file.is_open()) {
        std::cerr << "[ConfigReader] 无法打开配置文件: " << filepath << std::endl;
        return false;
    }
    
    std::string line;
    while (std::getline(file, line)) {
        line = trim(line);
        
        // 跳过空行和注释
        if (line.empty() || line[0] == '#') {
            continue;
        }
        
        // 解析键值对（格式：key = value 或 key: value）
        size_t sep_pos = line.find('=');
        if (sep_pos == std::string::npos) {
            sep_pos = line.find(':');
        }
        
        if (sep_pos != std::string::npos) {
            std::string key = trim(line.substr(0, sep_pos));
            std::string value = trim(line.substr(sep_pos + 1));
            
            // 移除行尾注释
            size_t comment_pos = value.find('#');
            if (comment_pos != std::string::npos) {
                value = trim(value.substr(0, comment_pos));
            }
            
            // 移除值两端的引号
            if (!value.empty() && (value.front() == '"' || value.front() == '\'')) {
                value = value.substr(1, value.length() - 2);
            }
            
            config_[key] = value;
        }
    }
    
    file.close();
    std::cout << "[ConfigReader] 成功加载配置文件: " << filepath 
              << " (" << config_.size() << " 项配置)" << std::endl;
    return true;
}

std::string ConfigReader::trim(const std::string& str) {
    size_t first = str.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return "";
    }
    size_t last = str.find_last_not_of(" \t\r\n");
    return str.substr(first, last - first + 1);
}

int ConfigReader::getInt(const std::string& key, int default_value) {
    auto it = config_.find(key);
    if (it == config_.end()) {
        return default_value;
    }
    
    try {
        return std::stoi(it->second);
    } catch (...) {
        std::cerr << "[ConfigReader] 警告：无法将 '" << key << "' 转换为整数" << std::endl;
        return default_value;
    }
}

float ConfigReader::getFloat(const std::string& key, float default_value) {
    auto it = config_.find(key);
    if (it == config_.end()) {
        return default_value;
    }
    
    try {
        return std::stof(it->second);
    } catch (...) {
        std::cerr << "[ConfigReader] 警告：无法将 '" << key << "' 转换为浮点数" << std::endl;
        return default_value;
    }
}

double ConfigReader::getDouble(const std::string& key, double default_value) {
    auto it = config_.find(key);
    if (it == config_.end()) {
        return default_value;
    }
    
    try {
        return std::stod(it->second);
    } catch (...) {
        std::cerr << "[ConfigReader] 警告：无法将 '" << key << "' 转换为双精度浮点数" << std::endl;
        return default_value;
    }
}

std::string ConfigReader::getString(const std::string& key, const std::string& default_value) {
    auto it = config_.find(key);
    if (it == config_.end()) {
        return default_value;
    }
    return it->second;
}

bool ConfigReader::getBool(const std::string& key, bool default_value) {
    auto it = config_.find(key);
    if (it == config_.end()) {
        return default_value;
    }
    
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    
    if (value == "true" || value == "1" || value == "yes" || value == "on") {
        return true;
    }
    if (value == "false" || value == "0" || value == "no" || value == "off") {
        return false;
    }
    
    return default_value;
}

void ConfigReader::printAll() {
    std::cout << "[ConfigReader] 所有配置项：" << std::endl;
    for (const auto& pair : config_) {
        std::cout << "  " << pair.first << " = " << pair.second << std::endl;
    }
}

} // namespace armor_detector
