#ifndef TD3LEARN_TIDLSTUB_HPP
#define TD3LEARN_TIDLSTUB_HPP

#include <string>

/**
 * @brief TIDL (TI Deep Learning) stub interface for build compatibility
 *
 * This file provides minimal stubs for TIDL functions to enable the code
 * to build without the actual TIDL library. These functions are only used
 * when ENABLE_TIDL is defined but the actual TIDL headers are not available.
 */

namespace td3learn
{
    /**
     * @brief Stub implementation of TidlGetPreferredBatchSize
     * @param numNetworks The number of networks to process in parallel
     * @return Always returns 0 (no cores available)
     */
    inline int TidlGetPreferredBatchSize(int numNetworks)
    {
        // Stub implementation always returns 0 (no cores available)
        return 0;
    }

    /**
     * @brief Stub TIDL export function
     * @param modelPath Path to save the exported model
     * @return Always returns false (export not supported)
     */
    inline bool ExportModelToTIDL(const std::string &modelPath)
    {
        return false;
    }
}

#endif // TD3LEARN_TIDLSTUB_HPP
