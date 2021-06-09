#pragma once

#include <vector>
#include <array>
#include <cstdint>
#include <Eigen/Dense>
#include <msgpack.hpp>

namespace seddom
{
    /// Occupancy states
    enum class State : char
    {
        UNKNOWN,
        FREE,
        OCCUPIED,
        PREDICTED,
        PRUNED,
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
         * @param hit whether the data comes from an actual hit
         */
        void update(const ClassVector &ybars, bool hit);
        void update_free(float ybar);

        /// Get probability of occupancy.
        ClassVector get_probs() const;

        /// Get variance of occupancy (uncertainty)
        ClassVector get_vars() const;

        /*
         * @brief Get occupancy state of the node.
         * @return occupancy state (see State).
         */
        inline State get_state() const { return _state; }

        inline bool is_classified() const { return _state != State::UNKNOWN; }

        size_t get_semantics() const;

        template <typename Packer>
        void msgpack_pack(Packer &pk) const;
        void msgpack_unpack(msgpack::object const& o);

    private:
        ClassVector ms;
        State _state;
    };

    template <size_t NumClass>
    using SemanticOctreeNode = Semantics<NumClass>;
}

#include "impl/bkioctree_node.hpp"
