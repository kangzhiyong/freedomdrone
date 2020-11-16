//
//  my_point.hpp
//  FCND-CPPSim
//
//  Created by kangzhiyong on 2020/3/14.
//  Copyright © 2020 Fotokite. All rights reserved.
//

#ifndef my_point_hpp
#define my_point_hpp

#include <algorithm>
#include <array>
#include <vector>
#include <cmath>
#include <iostream>
#include <random>
using namespace std;

/**
 * Class for representing a point. coordinate_type must be a numeric type.
 */
template<typename coordinate_type, size_t dimensions>
class point
{
public:
    point(){}
    point(std::array<coordinate_type, dimensions> c) : coords_(c)
    {
    }
    point(std::initializer_list<coordinate_type> list)
    {
        size_t n = min(dimensions, list.size());
        std::copy_n(list.begin(), n, coords_.begin());
    }
    /**
     * Returns the coordinate in the given dimension.
     *
     * @param index dimension index (zero based)
     * @return coordinate in the given dimension
     */
    coordinate_type get(size_t index) const
    {
        return coords_[index];
    }
    /**
     * Returns the distance squared from this point to another
     * point.
     *
     * @param pt another point
     * @return distance squared from this point to the other point
     */
    float distance(const point<coordinate_type, dimensions>& pt) const
    {
        float dist = 0;
        for (size_t i = 0; i < dimensions; ++i)
        {
            float d = get(i) - pt.get(i);
            dist += d * d;
        }
        return dist;
    }
    
    bool operator ==(const point<coordinate_type, dimensions>& p)
    {
        return std::equal(coords_.begin(), coords_.end(), p.coords_.begin());
    }
    
    coordinate_type& operator[](size_t i)
    {
        return coords_[i];
    }
    
    point<coordinate_type, dimensions> operator+(point<coordinate_type, dimensions> &p)
    {
        size_t n = min(dimensions, p.coords_.size());
        std::array<coordinate_type, dimensions> a;
        for (size_t i = 0; i < n; ++i)
        {
            a[i] = get(i) + p[i];
        }
        return a;
    }

    point<coordinate_type, dimensions> operator-(point<coordinate_type, dimensions>& p)
    {
        size_t n = min(dimensions, p.coords_.size());
        std::array<coordinate_type, dimensions> a;
        for (size_t i = 0; i < n; ++i)
        {
            a[i] = get(i) - p[i];
        }
        return a;
    }

//    void operator=(point<coordinate_type, dimensions> &p)
//    {
//        size_t n = std::min(dimensions, p.coords_.size());
//        std::copy_n(p.coords_.begin(), n, coords_.begin());
//    }
    
    void print(std::string prefix="")
    {
        std::cout << prefix << "( ";
        for (size_t i = 0; i < dimensions; ++i)
        {
            std::cout << get(i) << " ";
        }
        std::cout<< ")" << std::endl;
    }
    
    bool operator <(const point<coordinate_type, dimensions>& d) const
    {
        return distance({0, 0, 0}) < d.distance({0, 0, 0});
    }
private:
    std::array<coordinate_type, dimensions> coords_;
};
 
template<typename coordinate_type, size_t dimensions>
std::ostream& operator<<(std::ostream& out, const point<coordinate_type, dimensions>& pt)
{
    out << '(';
    for (size_t i = 0; i < dimensions; ++i)
    {
        if (i > 0)
            out << ", ";
        out << pt.get(i);
    }
    out << ')';
    return out;
}

typedef point<float, 2> point2D;
typedef point<float, 3> point3D;
typedef point<float, 4> point4D;
typedef point<double, 3> point3DD;
typedef point<int, 2> point2DI;

#endif /* my_point_hpp */
