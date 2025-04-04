#ifndef TD3LEARN_MODELS_HPP
#define TD3LEARN_MODELS_HPP

#include <memory>
#include <string>
#include <vector>
#include "td3learn/types.hpp"
#include "td3learn/config.hpp"

namespace td3learn
{

    /**
     * @brief Base model interface
     */
    class Model
    {
    public:
        virtual ~Model() = default;

        /**
         * @brief Forward pass through the model
         * @param input Input vector
         * @return Output vector
         */
        virtual std::vector<Scalar> forward(const std::vector<Scalar> &input) = 0;

        /**
         * @brief Save model to file
         * @param path Save path
         * @return Result code
         */
        virtual Result save(const std::string &path) = 0;

        /**
         * @brief Load model from file
         * @param path Load path
         * @return Result code
         */
        virtual Result load(const std::string &path) = 0;

        /**
         * @brief Create a clone of this model
         * @return Cloned model
         */
        virtual std::unique_ptr<Model> clone() const = 0;

        /**
         * @brief Update this model with another model's parameters
         * @param other Source model
         * @param tau Interpolation factor (0 to 1)
         */
        virtual void updateFrom(const Model &other, Scalar tau) = 0;
    };

    /**
     * @brief Actor network (policy)
     */
    class Actor : public virtual Model
    {
    public:
        /**
         * @brief Get action for a given state
         * @param state Input state
         * @return Action vector
         */
        virtual Action getAction(const State &state) = 0;

        /**
         * @brief Get action with added exploration noise
         * @param state Input state
         * @param noise_std Standard deviation of noise
         * @return Action vector with noise
         */
        virtual Action getNoisyAction(const State &state, Scalar noise_std) = 0;

        /**
         * @brief Create an actor network
         * @param state_dim State dimension
         * @param action_dim Action dimension
         * @param config Network configuration
         * @return Unique pointer to actor
         */
        static std::unique_ptr<Actor> create(
            int state_dim, int action_dim, const NetworkConfig &config);
    };

    /**
     * @brief Critic network (value function)
     */
    class Critic : public virtual Model
    {
    public:
        /**
         * @brief Get Q-value for state-action pair
         * @param state State vector
         * @param action Action vector
         * @return Q-value scalar
         */
        virtual Scalar getValue(const State &state, const Action &action) = 0;

        /**
         * @brief Create a critic network
         * @param state_dim State dimension
         * @param action_dim Action dimension
         * @param config Network configuration
         * @return Unique pointer to critic
         */
        static std::unique_ptr<Critic> create(
            int state_dim, int action_dim, const NetworkConfig &config);
    };

    /**
     * @brief Factory for creating neural network models
     */
    class ModelFactory
    {
    public:
        /**
         * @brief Set device for model execution
         * @param device Device name ("cpu", "tidl", "opencl")
         * @return Whether device was successfully set
         */
        static bool setDevice(const std::string &device);

        /**
         * @brief Get current device for model execution
         * @return Device name
         */
        static std::string getDevice();

        /**
         * @brief Create an actor model
         * @param state_dim State dimension
         * @param action_dim Action dimension
         * @param config Network configuration
         * @return Actor model
         */
        static std::unique_ptr<Actor> createActor(
            int state_dim, int action_dim, const NetworkConfig &config);

        /**
         * @brief Create a critic model
         * @param state_dim State dimension
         * @param action_dim Action dimension
         * @param config Network configuration
         * @return Critic model
         */
        static std::unique_ptr<Critic> createCritic(
            int state_dim, int action_dim, const NetworkConfig &config);
    };

} // namespace td3learn

#endif // TD3LEARN_MODELS_HPP
