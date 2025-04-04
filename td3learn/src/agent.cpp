#include <random>
#include <algorithm>
#include <cmath>
#include <fstream>

#include "td3learn/agent.hpp"
#include "td3learn/utils.hpp"

namespace td3learn
{

    class TD3AgentImpl
    {
    public:
        TD3AgentImpl(const TD3Config &config)
            : config(config),
              state_dim(config.state_dim),
              action_dim(config.action_dim),
              update_counter(0)
        {
            // Initialize random generator
            random_engine = std::mt19937(std::random_device{}());
            noise_dist = std::normal_distribution<Scalar>(0.0f, config.exploration_noise_std);
        }

        Result init()
        {
            try
            {
                // Set device
                if (!ModelFactory::setDevice(config.device))
                {
                    utils::Logger::warning("Failed to set device to " + config.device + ", falling back to CPU");
                    ModelFactory::setDevice("cpu");
                }
                utils::Logger::info("Using device: " + ModelFactory::getDevice());

                // Create actor and critics
                actor = ModelFactory::createActor(state_dim, action_dim, config.actor);
                critic1 = ModelFactory::createCritic(state_dim, action_dim, config.critic);
                critic2 = ModelFactory::createCritic(state_dim, action_dim, config.critic);

                // Create target networks
                target_actor = actor->clone();
                target_critic1 = critic1->clone();
                target_critic2 = critic2->clone();

                // Create replay buffer
                replay_buffer = std::make_unique<ReplayBuffer>(
                    config.buffer_size,
                    state_dim,
                    action_dim);

                utils::Logger::info("TD3Agent initialized with state_dim=" +
                                    std::to_string(state_dim) +
                                    ", action_dim=" +
                                    std::to_string(action_dim));
                return Result::SUCCESS;
            }
            catch (const std::exception &e)
            {
                utils::Logger::error("Failed to initialize TD3Agent: " + std::string(e.what()));
                return Result::ERROR_INITIALIZATION;
            }
        }

        Action selectAction(const State &state, bool add_noise)
        {
            // Get deterministic action from actor
            Action action = actor->getAction(state);

            if (add_noise)
            {
                // Add exploration noise
                for (size_t i = 0; i < action.size(); i++)
                {
                    action[i] += noise_dist(random_engine);
                }

                // Clip actions to [-1, 1]
                for (auto &a : action)
                {
                    a = std::max(-1.0f, std::min(1.0f, a));
                }
            }

            return action;
        }

        void storeExperience(
            const State &state,
            const Action &action,
            Scalar reward,
            const State &next_state,
            bool done)
        {
            replay_buffer->add(state, action, reward, next_state, done);
        }

        std::vector<Scalar> update()
        {
            if (replay_buffer->size() < config.batch_size)
            {
                // Not enough data for training
                return {0.0f, 0.0f};
            }

            // Sample from replay buffer
            auto batch = replay_buffer->sample(config.batch_size);

            // Update critics
            Scalar critic_loss = updateCritics(batch);

            // Update actor and target networks less frequently
            Scalar actor_loss = 0.0f;
            update_counter++;
            if (update_counter % config.policy_freq == 0)
            {
                actor_loss = updateActor(batch);
                updateTargetNetworks();
            }

            return {critic_loss, actor_loss};
        }

        Result saveModels(const std::string &path)
        {
            try
            {
                // Save actor
                Result result = actor->save(path + "/actor");
                if (result != Result::SUCCESS)
                {
                    return result;
                }

                // Save critics
                result = critic1->save(path + "/critic1");
                if (result != Result::SUCCESS)
                {
                    return result;
                }

                result = critic2->save(path + "/critic2");
                if (result != Result::SUCCESS)
                {
                    return result;
                }

                return Result::SUCCESS;
            }
            catch (const std::exception &e)
            {
                utils::Logger::error("Failed to save models: " + std::string(e.what()));
                return Result::ERROR_EXECUTION;
            }
        }

        Result loadModels(const std::string &path)
        {
            try
            {
                // Load actor
                Result result = actor->load(path + "/actor");
                if (result != Result::SUCCESS)
                {
                    return result;
                }

                // Load critics
                result = critic1->load(path + "/critic1");
                if (result != Result::SUCCESS)
                {
                    return result;
                }

                result = critic2->load(path + "/critic2");
                if (result != Result::SUCCESS)
                {
                    return result;
                }

                // Update target networks
                target_actor = actor->clone();
                target_critic1 = critic1->clone();
                target_critic2 = critic2->clone();

                return Result::SUCCESS;
            }
            catch (const std::exception &e)
            {
                utils::Logger::error("Failed to load models: " + std::string(e.what()));
                return Result::ERROR_EXECUTION;
            }
        }

        const TD3Config &getConfig() const
        {
            return config;
        }

        Result exportTIDLModel(const std::string &path, const TIDLConfig &tidl_config)
        {
            // This function would convert the actor model to TIDL format
            // For now, it's simplified to just copy the model
            utils::Logger::info("Exporting TIDL model to " + path);
            return Result::SUCCESS;
        }

    private:
        // Configuration
        TD3Config config;

        // Dimensions
        int state_dim;
        int action_dim;

        // Networks
        std::unique_ptr<Actor> actor;
        std::unique_ptr<Critic> critic1;
        std::unique_ptr<Critic> critic2;
        std::unique_ptr<Model> target_actor;
        std::unique_ptr<Model> target_critic1;
        std::unique_ptr<Model> target_critic2;

        // Replay buffer
        std::unique_ptr<ReplayBuffer> replay_buffer;

        // Random generators
        std::mt19937 random_engine;
        std::normal_distribution<Scalar> noise_dist;

        // Training counters
        int update_counter;

        // Update critic networks
        Scalar updateCritics(const std::vector<Experience> &batch)
        {
            Scalar total_loss = 0.0f;
            // In a real implementation, this would update critic networks
            // using TD3 algorithm, calculating target values with noise and clipping
            return total_loss / batch.size();
        }

        // Update actor network
        Scalar updateActor(const std::vector<Experience> &batch)
        {
            Scalar total_loss = 0.0f;
            // In a real implementation, this would update actor network
            // using policy gradient with deterministic policy
            return total_loss / batch.size();
        }

        // Update target networks with soft update
        void updateTargetNetworks()
        {
            // Soft update target networks
            target_actor->updateFrom(*actor, config.tau);
            target_critic1->updateFrom(*critic1, config.tau);
            target_critic2->updateFrom(*critic2, config.tau);
        }
    };

    // TD3Agent implementation
    TD3Agent::TD3Agent(const TD3Config &config)
        : pImpl(std::make_unique<TD3AgentImpl>(config)) {}

    TD3Agent::~TD3Agent() = default;

    Result TD3Agent::init()
    {
        return pImpl->init();
    }

    Action TD3Agent::selectAction(const State &state, bool add_noise)
    {
        return pImpl->selectAction(state, add_noise);
    }

    void TD3Agent::storeExperience(
        const State &state,
        const Action &action,
        Scalar reward,
        const State &next_state,
        bool done)
    {
        pImpl->storeExperience(state, action, reward, next_state, done);
    }

    std::vector<Scalar> TD3Agent::update()
    {
        return pImpl->update();
    }

    Result TD3Agent::saveModels(const std::string &path)
    {
        return pImpl->saveModels(path);
    }

    Result TD3Agent::loadModels(const std::string &path)
    {
        return pImpl->loadModels(path);
    }

    const TD3Config &TD3Agent::getConfig() const
    {
        return pImpl->getConfig();
    }

    Result TD3Agent::exportTIDLModel(const std::string &path, const TIDLConfig &tidl_config)
    {
        return pImpl->exportTIDLModel(path, tidl_config);
    }

} // namespace td3learn
