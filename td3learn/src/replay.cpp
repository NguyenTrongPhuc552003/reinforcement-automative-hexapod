#include <vector>
#include <random>
#include <stdexcept>
#include "td3learn/replay.hpp"

namespace td3learn
{

    class ReplayBufferImpl
    {
    public:
        ReplayBufferImpl(size_t capacity, size_t state_dim, size_t action_dim)
            : capacity(capacity), state_dim(state_dim), action_dim(action_dim),
              current_size(0), position(0)
        {
            states.resize(capacity, std::vector<Scalar>(state_dim));
            actions.resize(capacity, std::vector<Scalar>(action_dim));
            rewards.resize(capacity);
            next_states.resize(capacity, std::vector<Scalar>(state_dim));
            dones.resize(capacity);

            // Initialize random generator
            random_engine = std::mt19937(std::random_device{}());
        }

        void add(
            const State &state,
            const Action &action,
            Scalar reward,
            const State &next_state,
            bool done)
        {
            // Check dimensions
            if (state.size() != state_dim || action.size() != action_dim || next_state.size() != state_dim)
            {
                throw std::invalid_argument("Invalid dimensions in add experience");
            }

            // Store experience
            for (size_t i = 0; i < state_dim; ++i)
            {
                states[position][i] = state[i];
                next_states[position][i] = next_state[i];
            }

            for (size_t i = 0; i < action_dim; ++i)
            {
                actions[position][i] = action[i];
            }

            rewards[position] = reward;
            dones[position] = done;

            // Update position and size
            position = (position + 1) % capacity;
            current_size = std::min(current_size + 1, capacity);
        }

        std::vector<Experience> sample(size_t batch_size)
        {
            if (batch_size > current_size)
            {
                throw std::runtime_error("Cannot sample more experiences than buffer size");
            }

            std::vector<Experience> batch;
            batch.reserve(batch_size);

            // Generate random indices
            std::uniform_int_distribution<size_t> dist(0, current_size - 1);

            for (size_t i = 0; i < batch_size; ++i)
            {
                size_t idx = dist(random_engine);

                Experience exp;
                exp.state.resize(state_dim);
                exp.action.resize(action_dim);
                exp.next_state.resize(state_dim);

                for (size_t j = 0; j < state_dim; ++j)
                {
                    exp.state[j] = states[idx][j];
                    exp.next_state[j] = next_states[idx][j];
                }

                for (size_t j = 0; j < action_dim; ++j)
                {
                    exp.action[j] = actions[idx][j];
                }

                exp.reward = rewards[idx];
                exp.done = dones[idx];

                batch.push_back(exp);
            }

            return batch;
        }

        size_t size() const
        {
            return current_size;
        }

        bool empty() const
        {
            return current_size == 0;
        }

        void clear()
        {
            current_size = 0;
            position = 0;
        }

    private:
        size_t capacity;
        size_t state_dim;
        size_t action_dim;
        size_t current_size;
        size_t position;

        // Store experiences efficiently using separate vectors
        std::vector<std::vector<Scalar>> states;
        std::vector<std::vector<Scalar>> actions;
        std::vector<Scalar> rewards;
        std::vector<std::vector<Scalar>> next_states;
        std::vector<bool> dones;

        // Random generator
        std::mt19937 random_engine;
    };

    // ReplayBuffer implementation
    ReplayBuffer::ReplayBuffer(size_t capacity, size_t state_dim, size_t action_dim)
        : pImpl(std::make_unique<ReplayBufferImpl>(capacity, state_dim, action_dim)) {}

    ReplayBuffer::~ReplayBuffer() = default;

    void ReplayBuffer::add(
        const State &state,
        const Action &action,
        Scalar reward,
        const State &next_state,
        bool done)
    {
        pImpl->add(state, action, reward, next_state, done);
    }

    std::vector<Experience> ReplayBuffer::sample(size_t batch_size)
    {
        return pImpl->sample(batch_size);
    }

    size_t ReplayBuffer::size() const
    {
        return pImpl->size();
    }

    bool ReplayBuffer::empty() const
    {
        return pImpl->empty();
    }

    void ReplayBuffer::clear()
    {
        pImpl->clear();
    }

} // namespace td3learn
