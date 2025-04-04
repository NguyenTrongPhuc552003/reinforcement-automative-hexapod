#ifndef TD3LEARN_REPLAY_HPP
#define TD3LEARN_REPLAY_HPP

#include <vector>
#include <memory>
#include <random>
#include "td3learn/types.hpp"

namespace td3learn
{

    // Forward declaration
    class ReplayBufferImpl;

    /**
     * @brief Experience replay buffer for TD3
     */
    class ReplayBuffer
    {
    public:
        /**
         * @brief Create a replay buffer
         * @param capacity Maximum buffer capacity
         * @param state_dim State dimension
         * @param action_dim Action dimension
         */
        ReplayBuffer(size_t capacity, size_t state_dim, size_t action_dim);

        /**
         * @brief Destroy the replay buffer
         */
        ~ReplayBuffer();

        /**
         * @brief Add experience to buffer
         * @param state Current state
         * @param action Action taken
         * @param reward Reward received
         * @param next_state Next state
         * @param done Whether episode ended
         */
        void add(
            const State &state,
            const Action &action,
            Scalar reward,
            const State &next_state,
            bool done);

        /**
         * @brief Sample batch of experiences
         * @param batch_size Number of experiences to sample
         * @return Vector of sampled experiences
         */
        std::vector<Experience> sample(size_t batch_size);

        /**
         * @brief Get current buffer size
         * @return Number of experiences in buffer
         */
        size_t size() const;

        /**
         * @brief Check if buffer is empty
         * @return True if buffer is empty
         */
        bool empty() const;

        /**
         * @brief Clear the buffer
         */
        void clear();

    private:
        std::unique_ptr<ReplayBufferImpl> pImpl;
    };

} // namespace td3learn

#endif // TD3LEARN_REPLAY_HPP
