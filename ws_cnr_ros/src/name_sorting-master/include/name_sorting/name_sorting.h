#ifndef NAME_SORTING__NAME_SORTING_H
#define NAME_SORTING__NAME_SORTING_H

// ----
#include <string>
#include <vector>
#include <sstream>

namespace name_sorting
{
/**
 * \brief
 * permutationName, reorder vectors.
 * *order_names* names in the desired order
 * *names* current order of names,
 * *position, *velocity*, *effort* vectors to be reordered
 */
[[deprecated("Use permutationName with std::stringstream* report")]]
bool permutationName( const std::vector<std::string>& order_names,
                      std::vector<std::string>& names,
                      std::vector<double>& position,
                      std::vector<double>& velocity,
                      std::vector<double>& effort,
                      const std::string& whoami);


/**
 * @brief permutationName reorder vectors.
 * @param order_names: names in the desired order
 * @param names: current order of names,
 * @param position: vector to be reordered
 * @param velocity: vector to be reordered
 * @param effort: vector to be reordered
 * @param report: stream to log error (if desired). nullptr to disable.
 * @return true if ok
 */
bool permutationName( const std::vector<std::string>& order_names,
                      std::vector<std::string>& names,
                      std::vector<double>& position,
                      std::vector<double>& velocity,
                      std::vector<double>& effort,
                      std::stringstream* report=nullptr);

/**
 * \brief
 * permutationName, reorder vectors.
 * *order_names* names in the desired order
 * *names* current order of names,
 * *position, *velocity* vectors to be reordered
 */
[[deprecated("Use permutationName with std::stringstream* report")]]
bool permutationName( const std::vector<std::string>& order_names,
                      std::vector<std::string>& names,
                      std::vector<double>& position,
                      std::vector<double>& velocity,
                      const std::string& whoami
                    );

/**
 * @brief permutationName reorder vectors.
 * @param order_names: names in the desired order
 * @param names: current order of names,
 * @param position: vector to be reordered
 * @param velocity: vector to be reordered
 * @param report: stream to log error (if desired). nullptr to disable.
 * @return true if ok
 */
bool permutationName( const std::vector<std::string>& order_names,
                      std::vector<std::string>& names,
                      std::vector<double>& position,
                      std::vector<double>& velocity,
                      std::stringstream* report=nullptr);
/**
 * \brief
 * permutationName, reorder vectors.
 * *order_names* names in the desired order
 * *names* current order of names,
 * *position vector to be reordered
 */
[[deprecated("Use permutationName with std::stringstream* report")]]
bool permutationName( const std::vector<std::string>& order_names,
                      std::vector<std::string>& names,
                      std::vector<double>& position,
                      const std::string& whoami
                    );

/**
 * @brief permutationName reorder vectors.
 * @param order_names: names in the desired order
 * @param names: current order of names,
 * @param position: vector to be reordered
 * @param report: stream to log error (if desired). nullptr to disable.
 * @return true if ok
 */
bool permutationName( const std::vector<std::string>& order_names,
                      std::vector<std::string>& names,
                      std::vector<double>& position,
                      std::stringstream* report=nullptr);

}  // namespace name_sorting


#endif  // NAME_SORTING__NAME_SORTING_H
