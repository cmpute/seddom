#pragma once

#include <algorithm>
#include <assert.h>

#include "bkioctree_node.h"

namespace seddom
{
    template <size_t NumClass>
    float Semantics<NumClass>::prior = 1.f;

    template <size_t NumClass>
    SaveFormat save_format = SaveFormat::LABEL_WITH_DUAL_VAR;

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
    bool Semantics<NumClass>::update(const ClassVector &ybars, bool hit)
    {
        if (ybars.sum() < 1e-5)
            return false; // skip if the inferenced value is too small

        ms += ybars;

        if (get_semantics() == 0)
            _state = State::FREE;
        else if (hit || _state == State::OCCUPIED)
            _state = State::OCCUPIED; // hit or previously hit
        else
            _state = State::PREDICTED;
        return true;
    }

    template <size_t NumClass>
    void Semantics<NumClass>::update_free(float ybar)
    {
        assert(ybar >= 0);
        if (ybar <= 1e-5)
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

        union { float f; uint32_t i; } mem;
        char buf[l];
        for (int i = 0; i < NumClass; i++) // TODO: pack state and skip unknown
        {
            mem.f = ms(i) - prior;
            _msgpack_store32(&buf[i * sizeof(float)], mem.i);
        }
        pk.pack_ext_body(buf, l);
    }

    template <size_t NumClass>
    void Semantics<NumClass>::msgpack_unpack(msgpack::object const& o)
    {
        assert(o.type == msgpack::type::EXT);

        union { float f; uint32_t i; } mem;

        switch(o.via.ext.type())
        {
            case SEMANTIC_OCTREE_NODE_MSGPACK_EXT_TYPE:
                for (int i = 0; i < NumClass; i++)
                {
                    _msgpack_load32(uint32_t, o.via.ext.data() + i * sizeof(float), &mem.i);
                    ms[i] += mem.f;
                }
                if (get_semantics() == 0)
                    _state = State::FREE;
                else
                    _state = State::OCCUPIED;
                break;
            default:
                throw msgpack::type_error();
        }
    }
}
