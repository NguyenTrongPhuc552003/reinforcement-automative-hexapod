#ifndef TD3LEARN_ENVIRONMENT_HPP
#define TD3LEARN_ENVIRONMENT_HPP

#include <memory>
#include <string>
#include "td3learn/types.hpp"
#include "td3learn/config.hpp"

namespace td3learn
{

    // Forward declaration
    class EnvironmentImpl;

    /**
     * @brief Environment interface for reinforcement learning
     */
    class Environment
    {
    public:
        /**
         * @brief Create an environment
         * @param config Environment configuration
         */
        explicit Environment(const EnvironmentConfig &config);

        /**
         * @brief Destroy the environment
         * Virtual destructor is needed for polymorphism
         */
        virtual ~Environment();

        /**
         * @brief Initialize the environment
         * @return Result code
         */
        virtual Result init();

        /**
         * @brief Reset the environment to initial state
         * @return Initial state
         */
        virtual State reset();

        /**
         * @brief Take a step in the environment
         * @param action Action to take
         * @param[out] next_state Next state
         * @param[out] reward Reward received
         * @param[out] done Whether episode ended
         * @param[out] info Additional information
         * @return Result code
         */
        virtual Result step(
            const Action &action,
            State &next_state,
            Scalar &reward,
            bool &done,
            std::string &info);

        /**
         * @brief Get state dimension
         * @return State dimension
         */
        virtual int getStateDimension() const;

        /**
         * @brief Get action dimension
         * @return Action dimension
         */
        virtual int getActionDimension() const;

        /**
         * @brief Get action space limits
         * @param[out] low Lower bounds
         * @param[out] high Upper bounds
         */
        virtual void getActionLimits(State &low, State &high) const;

        /**
         * @brief Create an environment of the specified type
         * @param type Environment type
         * @param config Environment configuration
         * @return Shared pointer to the created environment
         */
        static std::shared_ptr<Environment> create(
            const std::string &type,
            const EnvironmentConfig &config);

    protected:
        std::unique_ptr<EnvironmentImpl> pImpl;
    };

} // namespace td3learn

#endif // TD3LEARN_ENVIRONMENT_HPP
