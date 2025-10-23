#ifndef CONFIG_READER_HPP
#define CONFIG_READER_HPP

#include <string>
#include <map>
#include <vector>

namespace armor_detector {

/**
 * @brief 配置文件读取器（支持YAML格式）
 * 简化版YAML解析器，支持基本嵌套结构
 */
class ConfigReader {
public:
    /**
     * @brief 构造函数
     * @param config_dir 配置文件目录
     */
    explicit ConfigReader(const std::string& config_dir = "config/");
    
    /**
     * @brief 加载配置文件（自动识别.yaml/.conf格式）
     * @param filename 文件名
     * @return 加载是否成功
     */
    bool loadConfig(const std::string& filename);
    
    /**
     * @brief 获取整数配置值（支持嵌套路径，如 "camera.width"）
     * @param key 配置键
     * @param default_value 默认值
     * @return 配置值
     */
    int getInt(const std::string& key, int default_value = 0);
    
    /**
     * @brief 获取浮点数配置值
     */
    float getFloat(const std::string& key, float default_value = 0.0f);
    
    /**
     * @brief 获取双精度浮点数配置值
     */
    double getDouble(const std::string& key, double default_value = 0.0);
    
    /**
     * @brief 获取字符串配置值
     */
    std::string getString(const std::string& key, const std::string& default_value = "");
    
    /**
     * @brief 获取布尔配置值
     */
    bool getBool(const std::string& key, bool default_value = false);
    
    /**
     * @brief 打印所有配置
     */
    void printAll();

private:
    std::string config_dir_;                    // 配置文件目录
    std::map<std::string, std::string> config_; // 扁平化的配置键值对
    
    std::string trim(const std::string& str);
    bool loadYAML(const std::string& filepath);
    bool loadConf(const std::string& filepath);
    void parseYAMLLine(const std::string& line, std::vector<std::string>& prefix_stack, int& last_indent);
};

} // namespace armor_detector

#endif // CONFIG_READER_HPP

