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
    SaveOptions Semantics<NumClass>::save_options = SaveOptions::ALL_BLOCKS;

    template <size_t NumClass>
    typename Semantics<NumClass>::ClassVector
    Semantics<NumClass>::get_probs() const
    {
        return logits.normalized();
    }

    template <size_t NumClass>
    typename Semantics<NumClass>::ClassVector
    Semantics<NumClass>::get_vars() const
    {
        float sum = logits.sum();
        auto probs = logits.array() / sum;
        return (probs.array() - probs.array().square()) / (sum + 1);
    }

    template <size_t NumClass>
    bool Semantics<NumClass>::update(const ClassVector &ybars, std::chrono::system_clock::time_point timestamp)
    {
        if (ybars.sum() < 1e-5)
            return false; // skip if the impact is too small

        logits += ybars;
        _state = (get_semantics() == 0) ? State::FREE : State::OCCUPIED; // removed state source flags
        _latest_time = timestamp;
        return true;
    }

    template <size_t NumClass>
    bool Semantics<NumClass>::update_free(float ybar, std::chrono::system_clock::time_point timestamp)
    {
        if (ybar < 1e-5)
            return false; // skip if the impact is too small

        logits[0] += ybar;
        _state = (get_semantics() == 0) ? State::FREE : State::OCCUPIED; // removed state source flags
        _latest_time = timestamp;
        return true;
    }

    template <size_t NumClass>
    size_t Semantics<NumClass>::get_semantics() const
    {
        typename ClassVector::Index imax;
        logits.maxCoeff(&imax);
        return imax;
    }

    /********************
     *   Serialization  *
     ********************
     * Note that cross-platform endianness compatibility is not guaranteed.
     * If this is required, boost.endian library can be used to make the following code compatible.
     */

    struct DLabelWithDualVar
    {
        char state;
        uint32_t timestamp;
        char semantics;
        float logit_total, logit_free, logit_semantics;
    };

    struct DLabelWithVar
    {
        char state;
        uint32_t timestamp;
        char semantics;
        float logit_total, logit_semantics;
    };

    struct DLabel
    {
        char state;
        uint32_t timestamp;
        char semantics;
    };

    template <size_t NumClass>
    template <typename Packer>
    void Semantics<NumClass>::msgpack_pack(Packer &pk) const
    {
        if ((_state & (State)0b0001) == State::UNKNOWN)
        {
            pk.pack_nil();
            return;
        }

        char state = (char)_state & 0b0011; // only store occupying status
        uint32_t timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(_latest_time.time_since_epoch()).count();

        switch(save_format)
        {
            case SaveFormat::FULL:
            {
                const size_t l = 1 // state
                               + 4 // timestamp
                               + NumClass * sizeof(float); // logits
                pk.pack_ext(l, (char)SaveFormat::FULL);

                char buf[l]; char *ptr = buf;
                *ptr++ = state;
                memcpy(ptr, &timestamp, sizeof(uint32_t));
                ptr += sizeof(uint32_t);

                union { float f; uint32_t i; } mem;
                for (int i = 0; i < NumClass; i++)
                {
                    float v = logits(i) - prior;
                    memcpy(ptr, &v, sizeof(float));
                    ptr += sizeof(float);
                }
                pk.pack_ext_body(buf, l);
                break;
            }
            case SaveFormat::LABEL_WITH_DUAL_VAR:
            {
                if (_state == State::FREE)
                    goto CASE_LABEL_WITH_VAR;

                DLabelWithDualVar data;
                data.state = state;
                data.timestamp = timestamp;
                data.semantics = get_semantics();
                data.logit_total = logits.sum() - prior * NumClass;
                data.logit_free = logits[0] - prior;
                data.logit_semantics = logits[data.semantics] - prior;

                pk.pack_ext(sizeof(DLabelWithDualVar), (char)SaveFormat::LABEL_WITH_DUAL_VAR);
                pk.pack_ext_body(reinterpret_cast<char*>(&data), sizeof(DLabelWithDualVar));
                break;
            }
            CASE_LABEL_WITH_VAR:
            case SaveFormat::LABEL_WITH_VAR:
            {
                DLabelWithVar data;
                data.state = state;
                data.timestamp = timestamp;
                data.semantics = get_semantics();
                data.logit_total = logits.sum() - prior * NumClass;
                data.logit_semantics = logits[data.semantics] - prior;

                pk.pack_ext(sizeof(DLabelWithVar), (char)SaveFormat::LABEL_WITH_VAR);
                pk.pack_ext_body(reinterpret_cast<char*>(&data), sizeof(DLabelWithVar));
                break;
            }
            case SaveFormat::LABEL:
            {
                DLabel data;
                data.state = state;
                data.timestamp = timestamp;
                data.semantics = get_semantics();

                pk.pack_ext(sizeof(DLabel), (char)SaveFormat::LABEL);
                pk.pack_ext_body(reinterpret_cast<char*>(&data), sizeof(DLabel));
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

        uint32_t timestamp;
        const char *ptr = o.via.ext.data();
        switch(o.via.ext.type())
        {
            case (char)SaveFormat::FULL:
            {
                union { float f; uint32_t i; } mem;
                _state = (State)*ptr++;
                timestamp = *reinterpret_cast<const float*>(ptr);
                ptr += sizeof(float);

                for (int i = 0; i < NumClass; i++)
                {
                    logits[i] += *reinterpret_cast<const float*>(ptr);
                    ptr += sizeof(float);
                }
                break;
            }
            case (char)SaveFormat::LABEL_WITH_DUAL_VAR:
            {
                const DLabelWithDualVar *data = reinterpret_cast<const DLabelWithDualVar*>(ptr);
                _state = (State)data->state;
                timestamp = data->timestamp;
                logits[0] += data->logit_free;
                logits[data->semantics] += data->logit_semantics;
                float logit_others = (data->logit_total - data->logit_free - data->logit_semantics) / (NumClass - 2);
                for (int i = 1; i < NumClass; i++)
                    if (i != data->semantics)
                        logits[i] += logit_others;
                break;
            }
            case (char)SaveFormat::LABEL_WITH_VAR:
            {
                const DLabelWithVar *data = reinterpret_cast<const DLabelWithVar*>(ptr);
                _state = (State)data->state;
                timestamp = data->timestamp;
                logits[data->semantics] += data->logit_semantics;
                float logit_others = (data->logit_total - data->logit_semantics) / (NumClass - 1);
                for (int i = 0; i < NumClass; i++)
                    if (i != data->semantics)
                        logits[i] += logit_others;
                break;
            }
            case (char)SaveFormat::LABEL:
            {
                const DLabel *data = reinterpret_cast<const DLabel*>(ptr);
                _state = (State)data->state;
                timestamp = data->timestamp;
                logits[data->semantics] += prior * 10;
                break;
            }
            default:
                throw msgpack::type_error();
        }

        _latest_time = std::chrono::system_clock::time_point(std::chrono::milliseconds(timestamp));
    }
}
