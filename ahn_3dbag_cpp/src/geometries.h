#pragma once

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>

using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
// using Kernel = CGAL::Exact_predicates_exact_constructions_kernel;

using Vector_3 = Kernel::Vector_3;
using Point_3 = Kernel::Point_3;
using Triangle_3 = Kernel::Triangle_3;
using Plane_3 = Kernel::Plane_3;

using Vector_2 = Kernel::Vector_2;
using Point_2 = Kernel::Point_2;
using Triangle_2 = Kernel::Triangle_2;
