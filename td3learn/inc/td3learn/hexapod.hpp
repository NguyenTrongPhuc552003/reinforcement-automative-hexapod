#ifndef TD3LEARN_HEXAPOD_HPP
#define TD3LEARN_HEXAPOD_HPP

#include <memory>
#include "td3learn/environment.hpp"
#include "td3learn/config.hpp"

// Forward declaration for user space hexapod interface
namespace hexapod
{
    class Hexapod;
}

namespace td3learn
{

    // Forward declaration
    class HexapodEnvironmentImpl;

    /**
     * @brief Hexapod-specific environment implementation
     */
    class HexapodEnvironment : public Environment
    {
    public:
        /**
         * @brief Create a hexapod environment
         * @param config Hexapod-specific environment configuration
         */
        explicit HexapodEnvironment(const HexapodEnvironmentConfig &config);

        /**
         * @brief Destroy the hexapod environment
         */
        ~HexapodEnvironment();

        /**
         * @brief Initialize the hexapod environment
         * @return Result code
         */
        Result init() override;

        /**
         * @brief Reset the hexapod environment
         * @return Initial state
         */
        State reset() override;

        /**
         * @brief Take a step with the hexapod
         * @param action Action to take
         * @param[out] next_state Next state
         * @param[out] reward Reward received
         * @param[out] done Whether episode ended
         * @param[out] info Additional information
         * @return Result code
         */
        Result step(
            const Action &action,
            State &next_state,
            Scalar &reward,
            bool &done,
            std::string &info) override;

        /**
         * @brief Get state dimension
         * @return State dimension
         */
        int getStateDimension() const override;

        /**
         * @brief Get action dimension
         * @return Action dimension
         */
        int getActionDimension() const override;

        /**
         * @brief Get action limits
         * @param[out] low Lower bounds
         * @param[out] high Upper bounds
         */
        void getActionLimits(State &low, State &high) const override;

        /**
         * @brief Set simulation mode
         * @param simulation True for simulation, false for real hardware
         * @return Result code
         */
        Result setSimulationMode(bool simulation);

        /**
         * @brief Check if in simulation mode
         * @return True if in simulation mode
         */
        bool isSimulation() const;

    private:
        // Implementation using PIMPL idiom
        std::unique_ptr<HexapodEnvironmentImpl> impl;
    };

} // namespace td3learn

#endif // TD3LEARN_HEXAPOD_HPP
