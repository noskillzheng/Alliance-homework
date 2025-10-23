#include "utils/config_loader.hpp"
#include <iostream>
#include <algorithm>

namespace trajectory {

std::map<std::string, std::string> ConfigLoader::config_;

bool ConfigLoader::loadFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "无法打开配置文件: " << filename << std::endl;
        return false;
    }
    
    config_.clear();
    std::string line;
    std::string current_section;
    
    while (std::getline(file, line)) {
        line = trim(line);
        
        // 跳过空行和注释
        if (line.empty() || line[0] == '#') {
            continue;
        }
        
        // 处理section（带冒号但没有值）
        if (line.back() == ':' && line.find('[') == std::string::npos) {
            current_section = line.substr(0, line.length() - 1);
            continue;
        }
        
        // 处理键值对
        size_t colon_pos = line.find(':');
        if (colon_pos != std::string::npos) {
            std::string key = trim(line.substr(0, colon_pos));
            std::string value = trim(line.substr(colon_pos + 1));
            
            // 移除注释部分
            size_t comment_pos = value.find('#');
            if (comment_pos != std::string::npos) {
                value = trim(value.substr(0, comment_pos));
            }
            
            // 构建完整的key
            std::string full_key = current_section.empty() ? key : current_section + "." + key;
            config_[full_key] = value;
        }
    }
    
    file.close();
    return true;
}

double ConfigLoader::getDouble(const std::string& key, double default_value) {
    auto it = config_.find(key);
    if (it != config_.end()) {
        try {
            double value = std::stod(it->second);
            
            // 参数范围验证
            if (key == "constraints.max_velocity" && (value <= 0 || value > 10.0)) {
                std::cerr << "警告: " << key << "=" << value << " 超出合理范围[0.1, 10.0]，使用默认值\n";
                return default_value;
            }
            if (key == "constraints.max_acceleration" && (value <= 0 || value > 20.0)) {
                std::cerr << "警告: " << key << "=" << value << " 超出合理范围[0.1, 20.0]，使用默认值\n";
                return default_value;
            }
            if (key == "performance.time_safety_factor" && (value < 1.0 || value > 2.0)) {
                std::cerr << "警告: " << key << "=" << value << " 超出合理范围[1.0, 2.0]，使用默认值\n";
                return default_value;
            }
            if (key == "performance.velocity_utilization" && (value < 0.3 || value > 1.0)) {
                std::cerr << "警告: " << key << "=" << value << " 超出合理范围[0.3, 1.0]，使用默认值\n";
                return default_value;
            }
            if (key == "performance.waypoint_velocity_ratio" && (value < 0.3 || value > 0.98)) {
                std::cerr << "警告: " << key << "=" << value << " 超出合理范围[0.3, 0.98]，使用默认值\n";
                return default_value;
            }
            if (key == "performance.min_segment_time" && (value < 0.1 || value > 5.0)) {
                std::cerr << "警告: " << key << "=" << value << " 超出合理范围[0.1, 5.0]，使用默认值\n";
                return default_value;
            }
            
            return value;
        } catch (...) {
            std::cerr << "警告: 无法解析 " << key << " 为double，使用默认值" << std::endl;
        }
    }
    return default_value;
}

int ConfigLoader::getInt(const std::string& key, int default_value) {
    auto it = config_.find(key);
    if (it != config_.end()) {
        try {
            return std::stoi(it->second);
        } catch (...) {
            std::cerr << "警告: 无法解析 " << key << " 为int，使用默认值" << std::endl;
        }
    }
    return default_value;
}

std::string ConfigLoader::getString(const std::string& key, const std::string& default_value) {
    auto it = config_.find(key);
    if (it != config_.end()) {
        // 移除引号
        std::string value = it->second;
        if (value.front() == '"' && value.back() == '"') {
            return value.substr(1, value.length() - 2);
        }
        return value;
    }
    return default_value;
}

std::vector<double> ConfigLoader::getDoubleArray(const std::string& key) {
    std::vector<double> result;
    auto it = config_.find(key);
    if (it != config_.end()) {
        std::string value = it->second;
        
        // 移除方括号
        if (value.front() == '[' && value.back() == ']') {
            value = value.substr(1, value.length() - 2);
        }
        
        // 分割并转换
        auto parts = split(value, ',');
        for (const auto& part : parts) {
            try {
                result.push_back(std::stod(trim(part)));
            } catch (...) {
                std::cerr << "警告: 无法解析数组元素: " << part << std::endl;
            }
        }
    }
    return result;
}

void ConfigLoader::printConfig() {
    std::cout << "\n========== 已加载的配置 ==========\n";
    for (const auto& pair : config_) {
        std::cout << pair.first << ": " << pair.second << "\n";
    }
    std::cout << "==================================\n\n";
}

std::string ConfigLoader::trim(const std::string& str) {
    size_t first = str.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return "";
    }
    size_t last = str.find_last_not_of(" \t\r\n");
    return str.substr(first, last - first + 1);
}

std::vector<std::string> ConfigLoader::split(const std::string& str, char delimiter) {
    std::vector<std::string> result;
    std::stringstream ss(str);
    std::string item;
    while (std::getline(ss, item, delimiter)) {
        result.push_back(item);
    }
    return result;
}

} // namespace trajectory

