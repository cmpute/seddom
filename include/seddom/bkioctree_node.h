#pragma once

#include <vector>
#include <array>
#include <cstdint>
#include <chrono>
#include <Eigen/Dense>
#include <msgpack.hpp>

namespace seddom
{
    /// Occupancy states
    enum class State : char
    {
        // states
        UNKNOWN    = 0b0000,
        FREE       = 0b0010,
        OCCUPIED   = 0b0011,

        // state source, by default (0b0000) is from lidar hit
        OCCLUDED   = 0b0100,
        PRUNED     = 0b1000, // not used
    };

    constexpr State operator |( const State lhs, const State rhs)
    {
        return (State)(char(lhs) | char(rhs));
    }

    enum class SaveFormat : char
    {
        FULL,
        LABEL_WITH_DUAL_VAR,
        LABEL_WITH_VAR,
        LABEL

        // TODO: add option for skip free and unknown blocks
    };

    /*
     * @brief Inference ouputs and occupancy state.
     *
     * Occupancy has member variables: m_A and m_B (kernel densities of positive
     * and negative class, respectively) and State.
     * Before using this class, set the static member variables first.
     */
    template <size_t NumClass>
    class Semantics
    {
    public:
        using ClassVector = Eigen::Matrix<float, NumClass, 1>;

        static float prior; // prior on each class
        static SaveFormat save_format;

        /*
         * @brief Constructors and destructor.
         */
        Semantics() : ms(), _state(State::UNKNOWN)
        {
            ms.setConstant(prior);
        }

        Semantics(const Semantics &other) : ms(other.ms), _state(other._state) {}

        Semantics &operator=(const Semantics &other)
        {
            ms = other.ms;
            _state = other._state;
            return *this;
        }

        ~Semantics() {}

        /*
         * @brief Exact updates for nonparametric Bayesian kernel inference
         * @param ybar kernel density estimate of positive class (occupied)
         */
        bool update(const ClassVector &ybars, std::chrono::system_clock::time_point timestamp);
        bool update(const ClassVector &ybars)
        {
            return update(ybars, std::chrono::system_clock::now());
        }
        bool update_free(float ybar, std::chrono::system_clock::time_point timestamp);
        bool update_free(float ybar)
        {
            return update_free(ybar, std::chrono::system_clock::now());
        }

        /// Get probability of occupancy.
        ClassVector get_probs() const;

        /// Get variance of occupancy (uncertainty)
        ClassVector get_vars() const;

        /*
         * @brief Get occupancy state of the node.
         * @return occupancy state (see State).
         */
        inline State get_state() const { return _state; }

        inline void mark_occluded() { _state = _state | State::OCCLUDED; }

        inline bool is_classified() const { return _state != State::UNKNOWN; }

        size_t get_semantics() const;

        template <typename Packer>
        void msgpack_pack(Packer &pk) const;
        void msgpack_unpack(msgpack::object const& o);

    private:
        ClassVector ms;
        State _state;
        std::chrono::system_clock::time_point _latest_time;  // time of last state update
    };

    template <size_t NumClass>
    using SemanticOctreeNode = Semantics<NumClass>;
}

#include "impl/bkioctree_node.hpp"
