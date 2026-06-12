#pragma once

#include <cstddef>
#include <utility>
#include <vector>

template <typename Tag, typename UnderlyingType> class StrongType {
  private:
    UnderlyingType value;

  public:
    explicit StrongType(UnderlyingType v = 0) : value(v) {}
    UnderlyingType get() const { return value; }
    operator UnderlyingType() const { return value; }

    // // Comparison operators
    // bool operator<(const StrongType &other) const {
    //     return value < other.value;
    // }
    // bool operator<=(const StrongType &other) const {
    //     return value <= other.value;
    // }
    // bool operator>(const StrongType &other) const {
    //     return value > other.value;
    // }
    // bool operator>=(const StrongType &other) const {
    //     return value >= other.value;
    // }
    // bool operator==(const StrongType &other) const {
    //     return value == other.value;
    // }
    // bool operator!=(const StrongType &other) const {
    //     return value != other.value;
    // }

    // Arithmetic operators
    StrongType &operator++() {
        ++value;
        return *this;
    }
    StrongType operator++(int) {
        StrongType tmp(*this);
        ++value;
        return tmp;
    }
    StrongType &operator--() {
        --value;
        return *this;
    }
    StrongType operator--(int) {
        StrongType tmp(*this);
        --value;
        return tmp;
    }
};

template <typename StrongIndex, typename T> class StrongTypedVector {
  private:
    using Storage = std::vector<T>;
    Storage values;

    static typename Storage::size_type to_index(StrongIndex id) {
        return static_cast<typename Storage::size_type>(id);
    }

  public:
    using value_type = T;
    using size_type = typename Storage::size_type;
        using reference = typename Storage::reference;
        using const_reference = typename Storage::const_reference;

    StrongTypedVector() = default;
    explicit StrongTypedVector(size_type count) : values(count) {}
    StrongTypedVector(size_type count, const T &value) : values(count, value) {}

    bool empty() const { return values.empty(); }
    size_type size() const { return values.size(); }
    StrongIndex size_as_strong_index() const {
        return StrongIndex(values.size());
    }
    void clear() { values.clear(); }
    void reserve(size_type count) { values.reserve(count); }
    void resize(size_type count) { values.resize(count); }

    bool contains(StrongIndex id) const { return to_index(id) < values.size(); }

    reference operator[](StrongIndex id) { return values[to_index(id)]; }
    const_reference operator[](StrongIndex id) const {
        return values[to_index(id)];
    }

    reference at(StrongIndex id) { return values.at(to_index(id)); }
    const_reference at(StrongIndex id) const {
        return values.at(to_index(id));
    }

    StrongIndex push_back(const T &value) {
        values.push_back(value);
        return StrongIndex(values.size() - 1);
    }

    StrongIndex push_back(T &&value) {
        values.push_back(std::move(value));
        return StrongIndex(values.size() - 1);
    }

    template <typename... Args> StrongIndex emplace_back(Args &&...args) {
        values.emplace_back(std::forward<Args>(args)...);
        return StrongIndex(values.size() - 1);
    }

    auto begin() { return values.begin(); }
    auto end() { return values.end(); }
    auto begin() const { return values.begin(); }
    auto end() const { return values.end(); }
    auto cbegin() const { return values.cbegin(); }
    auto cend() const { return values.cend(); }
};
