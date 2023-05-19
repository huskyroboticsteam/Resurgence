#include <unordered_set>

#include <nlohmann/json.hpp>

namespace util {

using json = nlohmann::json;
using val_t = json::value_t;

/**
   Check if the given json object has the given key.
 */
bool hasKey(const json& j, const std::string& key);

/**
   Check if the given json object has the given key with the given type.
 */
bool validateKey(const json& j, const std::string& key, const val_t& type);

/**
   Check if the given json object has the given key, with a type in the given set of types.
 */
bool validateKey(const json& j, const std::string& key,
				 const std::unordered_set<val_t>& types);
/**
   Check if the value in the given json object at the given key is a string in the given set of
   allowed values.
 */
bool validateOneOf(const json& j, const std::string& key,
				   const std::unordered_set<std::string>& vals);

/**
   Check if the value in the given json object at the given key is a floating-point number
   between min and max, inclusive.
 */
bool validateRange(const json& j, const std::string& key, double min, double max);

} // namespace util
