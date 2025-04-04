#include "td3learn/types.hpp"
#include <unordered_map>

namespace td3learn
{

    std::string resultToString(Result result)
    {
        static const std::unordered_map<Result, std::string> result_strings = {
            {Result::SUCCESS, "Success"},
            {Result::ERROR_INVALID_ARGUMENT, "Invalid argument"},
            {Result::ERROR_INITIALIZATION, "Initialization failed"},
            {Result::ERROR_EXECUTION, "Execution failed"},
            {Result::ERROR_MEMORY, "Memory error"},
            {Result::ERROR_NOT_IMPLEMENTED, "Not implemented"},
            {Result::ERROR_HARDWARE, "Hardware error"},
            {Result::ERROR_TIMEOUT, "Operation timed out"}};

        auto it = result_strings.find(result);
        if (it != result_strings.end())
        {
            return it->second;
        }
        return "Unknown error";
    }

} // namespace td3learn
