#pragma once

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
