#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <ctime>

class Log
{
public:
    static void Init(const std::string& path)
    {
        file().open(path, std::ios::out | std::ios::trunc);
        if (!file().is_open())
            std::cerr << "[LOG] Failed to open log file: " << path << std::endl;
    }

    static void Info(const std::string& msg) { write("INFO", msg); }
    static void Warn(const std::string& msg) { write("WARN", msg); }
    static void Error(const std::string& msg) { write("ERROR", msg); }

private:
    static std::ofstream& file()
    {
        static std::ofstream f;
        return f;
    }

    static void write(const std::string& level, const std::string& msg)
    {
        std::string line = "[" + level + "] " + msg;
        std::cout << line << std::endl;
        if (file().is_open())
        {
            file() << line << std::endl;
            file().flush();
        }
    }
};