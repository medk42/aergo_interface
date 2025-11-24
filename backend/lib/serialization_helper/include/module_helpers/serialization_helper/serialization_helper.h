#pragma once

#include <vector>
#include <cstdint>
#include <cstring>
#include <cstddef>
#include <type_traits>

#if (defined(_MSVC_LANG) ? _MSVC_LANG : __cplusplus) >= 202002L
    #include <bit>
    static_assert(std::endian::native == std::endian::little, "This code assumes a little-endian architecture");
#endif

namespace aergo::module::helpers::serialization_helper
{
    namespace serialization
    {
        template<class T, typename ByteT>
        inline void push(std::vector<ByteT>& buf, const T& v)
        {
            static_assert(sizeof(ByteT) == 1, "ByteT must be 1 byte");

            if constexpr (std::is_same_v<T, bool>)
            {
                buf.push_back(static_cast<ByteT>(v ? 1 : 0));
            }
            else
            {
                static_assert(std::is_trivially_copyable_v<T>, "serialization::push requires trivially copyable type");

                const void* p = &v;
                const auto old = buf.size();
                buf.resize(old + sizeof(T));
                std::memcpy(buf.data() + old, p, sizeof(T));
            }
        }

        template<class ByteT>
        inline void pushBytes(std::vector<ByteT>& buf, const void* data, size_t size)
        {
            static_assert(sizeof(ByteT) == 1, "ByteT must be 1 byte");
            const auto old = buf.size();
            buf.resize(old + size);
            std::memcpy(buf.data() + old, data, size);
        }
    }

    namespace deserialization
    {
        class BufferReader
        {
        public:
            BufferReader(const void* data, size_t size) 
            : data_(static_cast<const std::byte*>(data)), size_(size) { }

            /// @brief Read a T type from buffer.
            template<typename T>
            bool read(T& v)
            {
                static_assert(std::is_trivially_copyable_v<T>, "BufferReader::read requires trivially copyable type");

                if (pos_ + sizeof(T) > size_)
                {
                    return false; // out of bounds
                }
                std::memcpy(&v, data_ + pos_, sizeof(T));  // safe for alignment & aliasing
                pos_ += sizeof(T);
                return true;
            }

            /// @brief Read raw bytes from buffer.
            bool readBytes(void* out_data, size_t out_size)
            {
                if (pos_ + out_size > size_)
                {
                    return false; // out of bounds
                }
                std::memcpy(out_data, data_ + pos_, out_size); // safe for alignment & aliasing
                pos_ += out_size;
                return true;
            }

        private:
            const std::byte* data_{ nullptr };
            size_t size_{ 0 };
            size_t pos_{ 0 };
        };

        template<>
        inline bool BufferReader::read(bool& v)
        {
            uint8_t b;
            if (!read(b)) return false;
            v = (b != 0);
            return true;
        }
    }
}