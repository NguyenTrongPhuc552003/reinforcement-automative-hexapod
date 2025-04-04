#ifndef TD3LEARN_TRAINER_HPP
#define TD3LEARN_TRAINER_HPP

#include <memory>
#include <functional>
#include "td3learn/types.hpp"
#include "td3learn/config.hpp"
#include "td3learn/agent.hpp"
#include "td3learn/environment.hpp"

namespace td3learn
{

    // Forward declaration
    class TrainerImpl;

    /**
     * @brief Callback function for training events
     */
    using TrainingCallback = std::function<void(int epoch, int step, const std::vector<Scalar> &metrics)>;

    /**
     * @brief Trainer class for managing TD3 training
     */
    class Trainer
    {
    public:
        /**
         * @brief Create a trainer
         * @param agent TD3 agent
         * @param env Training environment
         * @param config Training configuration
         */
        Trainer(
            std::shared_ptr<TD3Agent> agent,
            std::shared_ptr<Environment> env,
            const TrainingConfig &config);

        /**
         * @brief Destroy the trainer
         */
        ~Trainer();

        /**
         * @brief Initialize the trainer
         * @return Result code
         */
        Result init();

        /**
         * @brief Run training for specified number of epochs
         * @param num_epochs Number of epochs
         * @return Result code
         */
        Result train(int num_epochs);

        /**
         * @brief Evaluate current policy
         * @param num_episodes Number of evaluation episodes
         * @param render Whether to render environment
         * @return Average reward
         */
        Scalar evaluate(int num_episodes, bool render = false);

        /**
         * @brief Set callback for training events
         * @param callback Callback function
         */
        void setCallback(TrainingCallback callback);

        /**
         * @brief Export trained model to filesystem
         * @param path Directory to save model
         * @return Result code
         */
        Result exportModel(const std::string &path);

        /**
         * @brief Export model for TIDL acceleration
         * @param path Output path for TIDL model
         * @param config TIDL configuration
         * @return Result code
         */
        Result exportTIDLModel(const std::string &path, const TIDLConfig &config);

    private:
        std::unique_ptr<TrainerImpl> pImpl;
    };

} // namespace td3learn

#endif // TD3LEARN_TRAINER_HPP
