#ifndef READ_ROS_PARAMS_H
#define READ_ROS_PARAMS_H

#include <iostream>
#include <vector>

namespace loam {

    template<typename T>
    inline void printKey(const std::string &key, T value) {
        std::cout << "CompslamLio: " << key << "  set to: " << value << std::endl;
    }

    template<>
    inline void printKey(const std::string &key, std::vector<double> vector) {
        std::cout << "CompslamLio " << key << " set to: ";
        for (const auto &element: vector) {
            std::cout << element << ",";
        }
        std::cout << std::endl;
    }

// Implementation of Templating
    template<typename T>
    T tryGetParam(const std::string &key, const ros::NodeHandle &privateNode) {
        T value;
        if (privateNode.getParam(key, value)) {
            printKey(key, value);
            return value;
        } else if (privateNode.getParam("/" + key, value)) {
            printKey("/" + key, value);
            return value;
        } else {
            std::cout << "\033[31m-----------------------------------" << std::endl;
            std::cout << "CompslamLio - " << key << " not specified." << std::endl;
            std::cout << "Throwing runtime error." << std::endl;
            std::cout << "-----------------------------------\033[0m" << std::endl;
            throw std::runtime_error("CompslamLio - " + key + " not specified.");
        }
    }

}  // namespace loam

#endif  // READ_ROS_PARAMS_H
