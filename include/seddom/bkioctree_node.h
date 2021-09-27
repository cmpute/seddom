#pragma once

#include <vector>
#include <array>
#include <cstdint>
#include <chrono>
#include <Eigen/Dense>
#include <msgpack.hpp>

namespace seddom
{
    /// Occupancy grid states
    enum class State : char
    {
        // states
        UNKNOWN    = 0b0000,
        FREE       = 0b0010, // classified
        OCCUPIED   = 0b0011, // classified
        OCCLUDED   = 0b0100, // occlusion flag
        PRUNED     = 0b1000, // not used
    };

    constexpr State operator| (const State lhs, const State rhs) { return (State)(char(lhs) | char(rhs)); }
    constexpr State operator& (const State lhs, const State rhs) { return (State)(char(lhs) & char(rhs)); }

    enum class SaveFormat : char
    {
        FULL,
        LABEL_WITH_DUAL_VAR, // semantic value with confidence score of free and highest category
        LABEL_WITH_VAR, // semantic value with confidence of highest category
        LABEL // semantic value only
    };

    enum class SaveOptions : char
    {
        ALL_BLOCKS, // by default all blocks will be saved
        SKIP_FREE, // don't save free blocks
        SKIP_FREE_UNKNOWN // don't save free and unknown blocks (no distinct class?)
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
        static SaveOptions save_options;

        /*
         * @brief Constructors and destructor.
         */
        Semantics() : logits(), _state(State::UNKNOWN)
        {
            logits.setConstant(prior);
        }

        Semantics(const Semantics &other) : logits(other.logits), _state(other._state) {}

        Semantics &operator=(const Semantics &other)
        {
            logits = other.logits;
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

        inline std::chrono::system_clock::time_point get_stamp() const { return _latest_time; }

        inline void mark_occluded(std::chrono::system_clock::time_point timestamp)
        { 
            _state = _state | State::OCCLUDED;
            _latest_time = timestamp;
        }

        inline bool is_occluded() const { return (_state & State::OCCLUDED) != State::UNKNOWN; }
        inline bool is_classified() const { return (_state & (State)0b0010) != State::UNKNOWN; }
        inline bool is_occupied() const { return (_state & (State)0b0011) == State::OCCUPIED; }

        size_t get_semantics() const;

        template <typename Packer>
        void msgpack_pack(Packer &pk) const;
        void msgpack_unpack(msgpack::object const& o);

    private:
        ClassVector logits;
        State _state; // TODO: separate occlusion from norm state, occlusion can be still a problem if we see something through a hole?
        std::chrono::system_clock::time_point _latest_time;  // time of last state update
    };

    template <size_t NumClass>
    using SemanticOctreeNode = Semantics<NumClass>;
}

#include "impl/bkioctree_node.hpp"
