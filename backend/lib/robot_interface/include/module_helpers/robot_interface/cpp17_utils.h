#pragma once

#if (defined(_MSVC_LANG) ? _MSVC_LANG : __cplusplus) >= 202002L

    #include <span>

    template<typename T>
    using Span = std::span<T>;

#else

    #include <vector>

    namespace aergo::util
    {
        template<typename T>
        class Span
        {
        public:
            using value_type = T;
            using pointer = T*;
            using const_pointer = const T*;
            using reference = T&;
            using const_reference = const T&;
            using size_type = std::size_t;

            Span() : data_(nullptr), size_(0) {}
            Span(T* data, size_type size) : data_(data), size_(size) {}

            template<typename Alloc>
            Span(std::vector<T, Alloc>& v) : data_(v.data()), size_(v.size()) {}

            template<typename Alloc>
            Span(const std::vector<T, Alloc>& v) : data_(v.data()), size_(v.size()) {}

            pointer data() { return data_; }
            const_pointer data() const { return data_; }
            size_type size() const { return size_; }
            bool empty() const { return size_ == 0; }

            reference operator[](size_type i) { return data_[i]; }
            const_reference operator[](size_type i) const { return data_[i]; }

            // begin/end for range-for
            pointer begin() { return data_; }
            const_pointer begin() const { return data_; }
            const_pointer cbegin() const { return data_; }
            pointer end() { return data_ + size_; }
            const_pointer end() const { return data_ + size_; }
            const_pointer cend() const { return data_ + size_; }

        private:
            T* data_;
            size_type size_;
        };
    }

    template<typename T>
    using Span = aergo::util::Span<T>;

#endif
