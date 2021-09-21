#pragma once

#include <algorithm>
#include <assert.h>

#include "bkioctree_node.h"

namespace seddom
{
    template <size_t NumClass>
    float Semantics<NumClass>::prior = .1f;

    template <size_t NumClass>
    SaveFormat Semantics<NumClass>::save_format = SaveFormat::LABEL_WITH_DUAL_VAR;

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
    bool Semantics<NumClass>::update(const ClassVector &ybars, std::chrono::system_clock::time_point timestamp)
    {
        if (ybars.sum() < 1e-5)
            return false; // skip if the impact is too small

        ms += ybars;
        _state = get_semantics() == 0 ? State::FREE : State::OCCUPIED; // removed state source flags
        _latest_time = timestamp;
        return true;
    }

    template <size_t NumClass>
    bool Semantics<NumClass>::update_free(float ybar, std::chrono::system_clock::time_point timestamp)
    {
        if (ybar < 1e-5)
            return false; // skip if the impact is too small

        ms[0] += ybar;
        _state = get_semantics() == 0 ? State::FREE : State::OCCUPIED; // removed state source flags
        _latest_time = timestamp;
        return true;
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
        if (_state == State::UNKNOWN)
        {
            pk.pack_nil();
            return;
        }
        union { float f; uint32_t i; } mem;
        switch(save_format)
        {
            case SaveFormat::FULL:
            {
                const size_t l = NumClass * sizeof(float) + 1;
                pk.pack_ext(l, SEMANTIC_OCTREE_NODE_MSGPACK_EXT_TYPE + (char)SaveFormat::FULL);

                char buf[l];
                buf[0] = (char)_state;
                char *ptr = buf + 1;
                for (int i = 0; i < NumClass; i++)
                {
                    mem.f = ms(i) - prior;
                    _msgpack_store32(ptr, mem.i);
                    ptr += sizeof(float);
                }
                pk.pack_ext_body(buf, l);
                break;
            }
            case SaveFormat::LABEL_WITH_DUAL_VAR:
            {
                if (_state == State::FREE)
                    goto CASE_LABEL_WITH_VAR;
                const size_t l = 3 * sizeof(float) + 2;
                pk.pack_ext(l, SEMANTIC_OCTREE_NODE_MSGPACK_EXT_TYPE + (char)SaveFormat::LABEL_WITH_DUAL_VAR);

                char buf[l];
                buf[0] = (char)_state;
                buf[1] = get_semantics();
                mem.f = ms.sum() - prior * NumClass;
                _msgpack_store32(buf+2, mem.i);
                mem.f = ms[0] - prior;
                _msgpack_store32(buf+2+sizeof(float), mem.i);
                mem.f = ms[buf[1]] - prior;
                _msgpack_store32(buf+2+2*sizeof(float), mem.i);
                pk.pack_ext_body(buf, l);
                break;
            }
            CASE_LABEL_WITH_VAR:
            case SaveFormat::LABEL_WITH_VAR:
            {
                const size_t l = 2 * sizeof(float) + 2;
                pk.pack_ext(l, SEMANTIC_OCTREE_NODE_MSGPACK_EXT_TYPE + (char)SaveFormat::LABEL_WITH_VAR);

                char buf[l];
                buf[0] = (char)_state;
                buf[1] = get_semantics();
                mem.f = ms.sum() - prior * NumClass;
                _msgpack_store32(buf+2, mem.i);
                mem.f = ms[buf[1]] - prior;
                _msgpack_store32(buf+2+sizeof(float), mem.i);
                pk.pack_ext_body(buf, l);
                break;
            }
            case SaveFormat::LABEL:
            {
                const size_t l = 2;
                pk.pack_ext(l, SEMANTIC_OCTREE_NODE_MSGPACK_EXT_TYPE + (char)SaveFormat::LABEL);

                char buf[l];
                buf[0] = (char)_state;
                buf[1] = get_semantics();
                pk.pack_ext_body(buf, l);
                break;
            }
        }
    }

    template <size_t NumClass>
    void Semantics<NumClass>::msgpack_unpack(msgpack::object const& o)
    {
        if(o.type == msgpack::type::NIL)
            return;
        assert(o.type == msgpack::type::EXT);

        union { float f; uint32_t i; } mem;
        const char *ptr = o.via.ext.data();
        switch(o.via.ext.type())
        {
            case SEMANTIC_OCTREE_NODE_MSGPACK_EXT_TYPE + (char)SaveFormat::FULL:
            {
                _state = (State)*ptr++;
                for (int i = 0; i < NumClass; i++)
                {
                    _msgpack_load32(uint32_t, ptr, &mem.i);
                    ms[i] += mem.f;
                    ptr += sizeof(float);
                }
                break;
            }
            case SEMANTIC_OCTREE_NODE_MSGPACK_EXT_TYPE + (char)SaveFormat::LABEL_WITH_DUAL_VAR:
            {
                _state = (State)*ptr++;
                char semantics = *ptr++;

                _msgpack_load32(uint32_t, ptr, &mem.i);
                float sum = mem.f;
                ptr += sizeof(float);

                _msgpack_load32(uint32_t, ptr, &mem.i);
                float prob_free = mem.f;
                ptr += sizeof(float);

                _msgpack_load32(uint32_t, ptr, &mem.i);
                float prob_sem = mem.f;

                ms[0] += prob_free;
                ms[semantics] += prob_sem;
                float prob_others = (sum - prob_free - prob_sem) / (NumClass - 2);
                for (int i = 1; i < NumClass; i++)
                    if (i != semantics)
                        ms[i] += prob_others;
                break;
            }
            case SEMANTIC_OCTREE_NODE_MSGPACK_EXT_TYPE + (char)SaveFormat::LABEL_WITH_VAR:
            {
                _state = (State)*ptr++;
                char semantics = *ptr++;

                _msgpack_load32(uint32_t, ptr, &mem.i);
                float sum = mem.f;
                ptr += sizeof(float);

                _msgpack_load32(uint32_t, ptr, &mem.i);
                float prob_sem = mem.f;

                ms[semantics] += prob_sem;
                float prob_others = (sum - prob_sem) / (NumClass - 1);
                for (int i = 0; i < NumClass; i++)
                    if (i != semantics)
                        ms[i] += prob_others;
                break;
            }
            case SEMANTIC_OCTREE_NODE_MSGPACK_EXT_TYPE + (char)SaveFormat::LABEL:
            {
                _state = (State)*ptr++;
                char semantics = *ptr++;
                ms[semantics] += prior * 10;
                break;
            }
            default:
                throw msgpack::type_error();
        }
    }
}
