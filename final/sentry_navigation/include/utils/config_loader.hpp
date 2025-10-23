#ifndef CONFIG_LOADER_HPP
#define CONFIG_LOADER_HPP

#include <string>
#include <map>
#include <vector>
#include <fstream>
#include <sstream>

namespace trajectory {

/**
 * @brief 简单的YAML配置文件加载器
 */
class ConfigLoader {
public:
    /**
     * @brief 从YAML文件加载配置
     * @param filename YAML文件路径
     * @return 是否加载成功
     */
    static bool loadFromFile(const std::string& filename);
    
    /**
     * @brief 获取double类型参数
     */
    static double getDouble(const std::string& key, double default_value = 0.0);
    
    /**
     * @brief 获取int类型参数
     */
    static int getInt(const std::string& key, int default_value = 0);
    
    /**
     * @brief 获取string类型参数
     */
    static std::string getString(const std::string& key, const std::string& default_value = "");
    
    /**
     * @brief 获取double数组参数
     */
    static std::vector<double> getDoubleArray(const std::string& key);
    
    /**
     * @brief 打印所有加载的配置
     */
    static void printConfig();

private:
    static std::map<std::string, std::string> config_;
    
    static std::string trim(const std::string& str);
    static std::vector<std::string> split(const std::string& str, char delimiter);
};

} // namespace trajectory

#endif // CONFIG_LOADER_HPP

