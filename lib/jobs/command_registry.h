// command_registry.h
#ifndef COMMAND_REGISTRY_H
#define COMMAND_REGISTRY_H

#include <functional>
#include <unordered_map>
#include <vector>

using CommandFunction = std::function<bool(const std::vector<double> &params)>;

class CommandRegistry
{
public:
    CommandRegistry() = default;
    void register_command(const std::string &name, CommandFunction func)
    {
        registry[name] = func;
    }

    void register_command(const std::string &name, std::function<bool()> func)
    {
        registry[name] = [func](const std::vector<double> &)
        { return func(); };
    }

    CommandFunction get_command(const std::string &name) const
    {
        auto it = registry.find(name);
        if (it != registry.end())
        {
            return it->second;
        }
        return nullptr;
    }

private:
    std::unordered_map<std::string, CommandFunction> registry;
};

#endif
