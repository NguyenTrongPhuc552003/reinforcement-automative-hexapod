#include <stdexcept>

#include "td3learn/environment.hpp"
#include "td3learn/hexapod.hpp"
#include "td3learn/utils.hpp"

namespace td3learn
{

    class EnvironmentImpl
    {
    public:
        EnvironmentImpl(const EnvironmentConfig &config)
            : config(config), initialized(false) {}

        virtual ~EnvironmentImpl() = default;

        Result init()
        {
            initialized = true;
            return Result::SUCCESS;
        }

        virtual State reset()
        {
            return State();
        }

        virtual Result step(
            const Action &action,
            State &next_state,
            Scalar &reward,
            bool &done,
            std::string &info)
        {
            return Result::ERROR_NOT_IMPLEMENTED;
        }

        virtual int getStateDimension() const
        {
            return 0;
        }

        virtual int getActionDimension() const
        {
            return 0;
        }

        virtual void getActionLimits(State &low, State &high) const
        {
            low.resize(0);
            high.resize(0);
        }

    protected:
        EnvironmentConfig config;
        bool initialized;
    };

    // Environment implementation
    Environment::Environment(const EnvironmentConfig &config)
        : pImpl(std::make_unique<EnvironmentImpl>(config)) {}

    Environment::~Environment() = default;

    Result Environment::init()
    {
        return pImpl->init();
    }

    State Environment::reset()
    {
        return pImpl->reset();
    }

    Result Environment::step(
        const Action &action,
        State &next_state,
        Scalar &reward,
        bool &done,
        std::string &info)
    {
        return pImpl->step(action, next_state, reward, done, info);
    }

    int Environment::getStateDimension() const
    {
        return pImpl->getStateDimension();
    }

    int Environment::getActionDimension() const
    {
        return pImpl->getActionDimension();
    }

    void Environment::getActionLimits(State &low, State &high) const
    {
        pImpl->getActionLimits(low, high);
    }

    // Factory method implementation
    std::shared_ptr<Environment> Environment::create(
        const std::string &type,
        const EnvironmentConfig &config)
    {
        if (type == "simulation" || type == "real")
        {
            // Check if we need to cast to a HexapodEnvironmentConfig
            const auto *hexapod_config = dynamic_cast<const HexapodEnvironmentConfig *>(&config);
            if (hexapod_config)
            {
                return std::make_shared<HexapodEnvironment>(*hexapod_config);
            }
            else
            {
                // Default config for hexapod if not provided
                HexapodEnvironmentConfig default_config;
                default_config.type = type;
                return std::make_shared<HexapodEnvironment>(default_config);
            }
        }

        utils::Logger::error("Unknown environment type: " + type);
        return nullptr;
    }

} // namespace td3learn
