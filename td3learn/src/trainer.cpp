#include <filesystem>
#include <fstream>
#include <algorithm>
#include "td3learn/trainer.hpp"
#include "td3learn/utils.hpp"

namespace td3learn
{

    class TrainerImpl
    {
    public:
        TrainerImpl(
            std::shared_ptr<TD3Agent> agent,
            std::shared_ptr<Environment> env,
            const TrainingConfig &config)
            : agent(agent), env(env), config(config),
              callback(nullptr), total_steps(0) {}

        ~TrainerImpl() = default;

        Result init()
        {
            try
            {
                // Initialize agent if not already done
                if (agent->init() != Result::SUCCESS)
                {
                    utils::Logger::error("Failed to initialize agent");
                    return Result::ERROR_INITIALIZATION;
                }

                // Initialize environment if not already done
                if (env->init() != Result::SUCCESS)
                {
                    utils::Logger::error("Failed to initialize environment");
                    return Result::ERROR_INITIALIZATION;
                }

                // Create directories for saving models and logs
                if (!std::filesystem::exists(config.save_path))
                {
                    std::filesystem::create_directories(config.save_path);
                }
                if (!std::filesystem::exists(config.log_path))
                {
                    std::filesystem::create_directories(config.log_path);
                }

                utils::Logger::info("Trainer initialized successfully");
                return Result::SUCCESS;
            }
            catch (const std::exception &e)
            {
                utils::Logger::error("Failed to initialize trainer: " + std::string(e.what()));
                return Result::ERROR_INITIALIZATION;
            }
        }

        Result train(int num_epochs)
        {
            try
            {
                // Training loop
                for (int epoch = 0; epoch < num_epochs; ++epoch)
                {
                    utils::Logger::info("Starting epoch " + std::to_string(epoch + 1) +
                                        " of " + std::to_string(num_epochs));

                    Scalar epoch_reward = 0.0f;
                    int steps_this_epoch = 0;

                    // Reset environment
                    State state = env->reset();
                    bool done = false;

                    for (int step = 0; step < config.steps_per_epoch && !done; ++step)
                    {
                        // Select action with exploration noise
                        Action action = agent->selectAction(state, true);

                        // Take step in environment
                        State next_state;
                        Scalar reward;
                        std::string info;
                        Result result = env->step(action, next_state, reward, done, info);

                        if (result != Result::SUCCESS)
                        {
                            utils::Logger::error("Environment step failed: " + resultToString(result));
                            return result;
                        }

                        // Store experience in replay buffer
                        agent->storeExperience(state, action, reward, next_state, done);

                        // Update networks
                        std::vector<Scalar> metrics = agent->update();

                        // Update state and statistics
                        state = next_state;
                        epoch_reward += reward;
                        steps_this_epoch++;
                        total_steps++;

                        // Call callback if provided
                        if (callback && step % 100 == 0)
                        {
                            std::vector<Scalar> callback_metrics = metrics;
                            callback_metrics.push_back(epoch_reward / (step + 1)); // Add average reward
                            callback(epoch, step, callback_metrics);
                        }

                        if (done)
                        {
                            break;
                        }
                    }

                    // Calculate average reward for the epoch
                    Scalar avg_reward = steps_this_epoch > 0 ? epoch_reward / steps_this_epoch : 0.0f;
                    utils::Logger::info("Epoch " + std::to_string(epoch + 1) +
                                        " completed. Average reward: " +
                                        std::to_string(avg_reward));

                    // Evaluate agent periodically
                    if ((epoch + 1) % config.evaluate_every == 0)
                    {
                        Scalar eval_reward = evaluate(config.evaluation_episodes, false);
                        utils::Logger::info("Evaluation after epoch " +
                                            std::to_string(epoch + 1) +
                                            ": Average reward = " +
                                            std::to_string(eval_reward));

                        // Save model if configured
                        if (!config.save_path.empty())
                        {
                            std::string epoch_path = config.save_path + "/epoch_" +
                                                     std::to_string(epoch + 1);
                            std::filesystem::create_directory(epoch_path);
                            agent->saveModels(epoch_path);
                        }
                    }
                }

                utils::Logger::info("Training completed successfully");
                return Result::SUCCESS;
            }
            catch (const std::exception &e)
            {
                utils::Logger::error("Training failed: " + std::string(e.what()));
                return Result::ERROR_EXECUTION;
            }
        }

        Scalar evaluate(int num_episodes, bool render)
        {
            Scalar total_reward = 0.0f;

            for (int episode = 0; episode < num_episodes; ++episode)
            {
                Scalar episode_reward = 0.0f;
                State state = env->reset();
                bool done = false;

                while (!done)
                {
                    // Select action without exploration noise
                    Action action = agent->selectAction(state, false);

                    // Take step in environment
                    State next_state;
                    Scalar reward;
                    std::string info;
                    Result result = env->step(action, next_state, reward, done, info);

                    if (result != Result::SUCCESS)
                    {
                        utils::Logger::error("Evaluation step failed: " + resultToString(result));
                        return 0.0f;
                    }

                    // Update state and reward
                    state = next_state;
                    episode_reward += reward;
                }

                total_reward += episode_reward;
            }

            return total_reward / num_episodes;
        }

        void setCallback(TrainingCallback callback)
        {
            this->callback = callback;
        }

        Result exportModel(const std::string &path)
        {
            try
            {
                // Create directory if it doesn't exist
                if (!std::filesystem::exists(path))
                {
                    std::filesystem::create_directories(path);
                }

                // Save TD3 model
                return agent->saveModels(path);
            }
            catch (const std::exception &e)
            {
                utils::Logger::error("Failed to export model: " + std::string(e.what()));
                return Result::ERROR_EXECUTION;
            }
        }

        Result exportTIDLModel(const std::string &path, const TIDLConfig &config)
        {
            try
            {
                // Create directory if it doesn't exist
                if (!std::filesystem::exists(path))
                {
                    std::filesystem::create_directories(path);
                }

                // Export to TIDL format
                return agent->exportTIDLModel(path, config);
            }
            catch (const std::exception &e)
            {
                utils::Logger::error("Failed to export TIDL model: " + std::string(e.what()));
                return Result::ERROR_EXECUTION;
            }
        }

    private:
        std::shared_ptr<TD3Agent> agent;
        std::shared_ptr<Environment> env;
        TrainingConfig config;
        TrainingCallback callback;
        int total_steps;
    };

    // Trainer implementation
    Trainer::Trainer(
        std::shared_ptr<TD3Agent> agent,
        std::shared_ptr<Environment> env,
        const TrainingConfig &config)
        : pImpl(std::make_unique<TrainerImpl>(agent, env, config)) {}

    Trainer::~Trainer() = default;

    Result Trainer::init()
    {
        return pImpl->init();
    }

    Result Trainer::train(int num_epochs)
    {
        return pImpl->train(num_epochs);
    }

    Scalar Trainer::evaluate(int num_episodes, bool render)
    {
        return pImpl->evaluate(num_episodes, render);
    }

    void Trainer::setCallback(TrainingCallback callback)
    {
        pImpl->setCallback(callback);
    }

    Result Trainer::exportModel(const std::string &path)
    {
        return pImpl->exportModel(path);
    }

    Result Trainer::exportTIDLModel(const std::string &path, const TIDLConfig &config)
    {
        return pImpl->exportTIDLModel(path, config);
    }

} // namespace td3learn
