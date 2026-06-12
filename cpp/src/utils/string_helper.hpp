#include <string>

inline std::string replace_substring_first(const std::string &original,
                                           const std::string &to_replace,
                                           const std::string &replacement) {
    std::string result = original;
    size_t pos = result.find(to_replace);
    if (pos != std::string::npos) {
        result.replace(pos, to_replace.length(), replacement);
    }
    return result;
}
