#ifndef FLVC_CONFIG_HPP
#define FLVC_CONFIG_HPP

namespace flvc {

enum class OptimizationLevel : unsigned char {
    NO_DELTA_CODING= 0,
    NO_ATTRIBUTE_DILEAVING = 1,
    NO_BIT_INTERLEAVING = 2,
    NO_COMPRESSION = 3,
    DEFAULT = 4,
};


// CONFIGURABLE PART ===================================================================================================

/// The level of optimization. ALWAYS push this with DEFAULT value.
constexpr OptimizationLevel OPTIMIZATION_LEVEL = OptimizationLevel::DEFAULT;

/// This flag can help with manual debugging of the output data in a hex editor.
/// NEVER push this with a true value.
constexpr bool ANNOTATE_BINARY = false;


// OPERATORS (DO NOT TOUCH) ============================================================================================

constexpr bool operator<(OptimizationLevel a, OptimizationLevel b) {
    return static_cast<unsigned char>(a) < static_cast<unsigned char>(b);
}

constexpr bool operator<=(OptimizationLevel a, OptimizationLevel b) {
    return static_cast<unsigned char>(a) <= static_cast<unsigned char>(b);
}

constexpr bool operator>(OptimizationLevel a, OptimizationLevel b) {
    return static_cast<unsigned char>(a) > static_cast<unsigned char>(b);
}

constexpr bool operator>=(OptimizationLevel a, OptimizationLevel b) {
    return static_cast<unsigned char>(a) >= static_cast<unsigned char>(b);
}

}  // namespace flvc

#endif  // FLVCCONFIG_HPP
