#pragma once

#include <iostream>
#include <tuple>
#include <utility>  // std::index_sequence, std::make_index_sequence

#include "geometries.h"

template <std::size_t I, typename Tuple>
void print_one(std::ostream& os, const Tuple& tup) {
    if constexpr (I > 0) {
        os << ", ";
    }

    os << std::get<I>(tup);
}

template <typename Tuple, std::size_t... Is>
void print_tuple_impl(std::ostream& os, const Tuple& tup,
                      std::index_sequence<Is...>) {
    (print_one<Is>(os, tup), ...);
}

template <typename... Ts>
std::ostream& operator<<(std::ostream& os, const std::tuple<Ts...>& tup) {
    os << '(';
    print_tuple_impl(os, tup, std::make_index_sequence<sizeof...(Ts)>{});
    os << ')';
    return os;
}

// std::ostream &operator<<(std::ostream &os, const Point_3 &point)
// {
//     os << "(" << point.x() << ", " << point.y() << ", " << point.z() << ")";
//     return os;
// }

// std::ostream &operator<<(std::ostream &os, const Triangle_3 &triangle)
// {
//     os << "[" << triangle.vertex(0) << ", " << triangle.vertex(1) << ", " <<
//     triangle.vertex(2) << "]"; return os;
// }
