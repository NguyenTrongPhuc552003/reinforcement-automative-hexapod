#ifndef TD3LEARN_TYPES_HPP
#define TD3LEARN_TYPES_HPP

#include <vector>
#include <memory>
#include <cstdint>
#include <string>
#include <array>

namespace td3learn
{

    /**
     * @brief Type for scalar values throughout the system
     */
    using Scalar = float;

    /**
     * @brief Vector class for storing state and action data
     */
    template <typename T>
    class Vector
    {
    public:
        /**
         * @brief Create an empty vector
         */
        Vector() = default;

        /**
         * @brief Create a vector with given size
         * @param size Number of elements
         */
        explicit Vector(size_t size) : data_(size) {}

        /**
         * @brief Create a vector from initializer list
         * @param list Initializer list of values
         */
        Vector(std::initializer_list<T> list) : data_(list) {}

        /**
         * @brief Access element at given index
         * @param index Element index
         * @return Reference to element
         */
        T &operator[](size_t index) { return data_[index]; }

        /**
         * @brief Access element at given index (const)
         * @param index Element index
         * @return Const reference to element
         */
        const T &operator[](size_t index) const { return data_[index]; }

        /**
         * @brief Get vector size
         * @return Number of elements
         */
        size_t size() const { return data_.size(); }

        /**
         * @brief Resize the vector
         * @param size New size
         */
        void resize(size_t size) { data_.resize(size); }

        /**
         * @brief Get pointer to underlying data
         * @return Pointer to first element
         */
        T *data() { return data_.data(); }

        /**
         * @brief Get pointer to underlying data (const)
         * @return Const pointer to first element
         */
        const T *data() const { return data_.data(); }

        // STL-compatible iterators
        auto begin() { return data_.begin(); }
        auto end() { return data_.end(); }
        auto begin() const { return data_.begin(); }
        auto end() const { return data_.end(); }

    private:
        std::vector<T> data_;
    };

    /**
     * @brief State vector type
     */
    using State = Vector<Scalar>;

    /**
     * @brief Action vector type
     */
    using Action = Vector<Scalar>;

    /**
     * @brief Experience tuple for reinforcement learning
     */
    struct Experience
    {
        State state;
        Action action;
        Scalar reward;
        State next_state;
        bool done;
    };

    /**
     * @brief Result codes for API functions
     */
    enum class Result
    {
        SUCCESS,
        ERROR_INVALID_ARGUMENT,
        ERROR_INITIALIZATION,
        ERROR_EXECUTION,
        ERROR_MEMORY,
        ERROR_NOT_IMPLEMENTED,
        ERROR_HARDWARE,
        ERROR_TIMEOUT
    };

    /**
     * @brief Convert Result to string
     * @param result Result code
     * @return String representation
     */
    std::string resultToString(Result result);

} // namespace td3learn

#endif // TD3LEARN_TYPES_HPP
