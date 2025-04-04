#ifndef TD3LEARN_AGENT_HPP
#define TD3LEARN_AGENT_HPP

#include <memory>
#include "td3learn/types.hpp"
#include "td3learn/config.hpp"
#include "td3learn/models.hpp"
#include "td3learn/replay.hpp"

namespace td3learn
{

    // Forward declaration
    class TD3AgentImpl;

    /**
     * @brief TD3 agent implementation
     */
    class TD3Agent
    {
    public:
        /**
         * @brief Create a TD3 agent
         * @param config TD3 configuration
         */
        explicit TD3Agent(const TD3Config &config);

        /**
         * @brief Destroy the TD3 agent
         */
        ~TD3Agent();

        /**
         * @brief Initialize the agent
         * @return Result code
         */
        Result init();

        /**
         * @brief Select action for given state
         * @param state Current state
         * @param add_noise Whether to add exploration noise
         * @return Selected action
         */
        Action selectAction(const State &state, bool add_noise = false);

        /**
         * @brief Store experience in replay buffer
         * @param state Current state
         * @param action Action taken
         * @param reward Reward received
         * @param next_state Next state
         * @param done Whether episode ended
         */
        void storeExperience(
            const State &state,
            const Action &action,
            Scalar reward,
            const State &next_state,
            bool done);

        /**
         * @brief Update networks with one batch
         * @return Loss values for critics and actor
         */
        std::vector<Scalar> update();

        /**
         * @brief Save agent models
         * @param path Directory to save models
         * @return Result code
         */
        Result saveModels(const std::string &path);

        /**
         * @brief Load agent models
         * @param path Directory to load models from
         * @return Result code
         */
        Result loadModels(const std::string &path);

        /**
         * @brief Get current configuration
         * @return TD3 configuration
         */
        const TD3Config &getConfig() const;

        /**
         * @brief Export model for inference with TIDL
         * @param path Output path for TIDL model
         * @param tidl_config TIDL configuration
         * @return Result code
         */
        Result exportTIDLModel(const std::string &path, const TIDLConfig &tidl_config);

    private:
        std::unique_ptr<TD3AgentImpl> pImpl;
    };

} // namespace td3learn

#endif // TD3LEARN_AGENT_HPP
