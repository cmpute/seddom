#pragma once

#include <algorithm>
#include <assert.h>

#include "bkioctree_node.h"

namespace seddom
{
    template <size_t NumClass>
    float Semantics<NumClass>::prior = 1.f;

    template <size_t NumClass>
    typename Semantics<NumClass>::ClassVector
    Semantics<NumClass>::get_probs() const
    {
        return ms.normalized();
    }

    template <size_t NumClass>
    typename Semantics<NumClass>::ClassVector
    Semantics<NumClass>::get_vars() const
    {
        float sum = ms.sum();
        auto probs = ms.array() / sum;
        return (probs.array() - probs.array().square()) / (sum + 1);
    }

    template <size_t NumClass>
    void Semantics<NumClass>::update(const ClassVector &ybars, bool hit)
    {
        ms += ybars;

        if (get_semantics() == 0)
            _state = State::FREE;
        else if (hit || _state == State::OCCUPIED)
            _state = State::OCCUPIED; // hit or previously hit
        else
            _state = State::PREDICTED;
    }

    template <size_t NumClass>
    void Semantics<NumClass>::update_free(float ybar)
    {
        assert(ybar >= 0);
        if (ybar <= 0)
            return;
        ms[0] += ybar;
        if (get_semantics() == 0)
            _state = State::FREE;
    }

    template <size_t NumClass>
    size_t Semantics<NumClass>::get_semantics() const
    {
        typename ClassVector::Index imax;
        ms.maxCoeff(&imax);
        return imax;
    }

    template <size_t NumClass>
    template <typename Packer>
    void Semantics<NumClass>::msgpack_pack(Packer &pk) const
    {
        const size_t l = NumClass * sizeof(float);
        pk.pack_ext(l, SEMANTIC_OCTREE_NODE_MSGPACK_EXT_TYPE);

        union
        {
            float f;
            uint32_t i;
        } mem;
        char buf[l];
        for (int i = 0; i < NumClass; i++)
        {
            mem.f = ms(i);
            _msgpack_store32(&buf[i * sizeof(float)], mem.i);
        }
        pk.pack_ext_body(buf, l);
    }
}
