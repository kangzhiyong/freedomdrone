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
    
    void get(coordinate_type &x, coordinate_type &y, coordinate_type &z)
    {
        x = coords_[0];
        y = coords_[1];
        z = coords_[2];
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
        return sqrt(dist);
    }

    point<coordinate_type, dimensions> operator-()
    {
        std::array<coordinate_type, dimensions> a;
        for (size_t i = 0; i < dimensions; ++i)
        {
            a[i] = -get(i);
        }
        return a;
    }

    bool operator ==(const point<coordinate_type, dimensions>& p)
    {
        return std::equal(coords_.begin(), coords_.end(), p.coords_.begin());
    }
    
    coordinate_type& operator[](int i)
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

    point<coordinate_type, dimensions> operator+(coordinate_type distance)
    {
        std::array<coordinate_type, dimensions> a;
        for (size_t i = 0; i < dimensions; ++i)
        {
            a[i] = get(i) + distance;
        }
        return a;
    }

    point<coordinate_type, dimensions> operator*(coordinate_type m)
    {
        std::array<coordinate_type, dimensions> a;
        for (size_t i = 0; i < dimensions; ++i)
        {
            a[i] = get(i) * m;
        }
        return a;
    }

    point<coordinate_type, dimensions> operator*(point<coordinate_type, dimensions> m)
    {
        std::array<coordinate_type, dimensions> a;
        for (size_t i = 0; i < dimensions; ++i)
        {
            a[i] = get(i) * m.get(i);
        }
        return a;
    }
    
    point<coordinate_type, dimensions> operator/(coordinate_type m)
    {
        std::array<coordinate_type, dimensions> a;
        for (size_t i = 0; i < dimensions; ++i)
        {
            a[i] = get(i) / m;
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
    
    inline float mag()
    {
        return distance({0, 0, 0});
    }

    inline point<coordinate_type, dimensions> norm()
    {
        point<coordinate_type, dimensions> res;
        float l = mag();
        if (l == 0.0f)
            return {0, 0, 0};
        return operator/(l);
    }

    inline point<coordinate_type, dimensions> cross(point<coordinate_type, dimensions> v)
    {
        point<coordinate_type, dimensions> resVector;
        resVector[0] = coords_[1] * v[2] - coords_[2] * v[1];
        resVector[1] = coords_[2] * v[0] - coords_[0] * v[2];
        resVector[2] = coords_[0] * v[1] - coords_[1] * v[0];
        return resVector;
    }
    
    size_t count()
    {
        return coords_.size();
    }
    typename array<coordinate_type, dimensions>::iterator begin()
    {
        return coords_.begin();
    }
    
    typename array<coordinate_type, dimensions>::iterator end()
    {
        return coords_.end();
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

template<typename coordinate_type, size_t dimensions>
inline point<coordinate_type, dimensions> operator*(float a, point<coordinate_type, dimensions> b)
{
    std::array<coordinate_type, dimensions> xa;
    for (size_t i = 0; i < dimensions; ++i)
    {
        xa[i] = b.get(i) * a;
    }
    return xa;
}

template<typename coordinate_type, size_t dimensions>
inline point<coordinate_type, dimensions> operator+(point<coordinate_type, dimensions> q, point<coordinate_type, dimensions> p)
{
    size_t n = min(q.count(), p.count());
    std::array<coordinate_type, dimensions> a;
    for (size_t i = 0; i < n; ++i)
    {
        a[i] = q.get(i) + p[i];
    }
    return a;
}

template<typename coordinate_type, size_t dimensions>
inline point<coordinate_type, dimensions> operator+(point<coordinate_type, dimensions> q, coordinate_type distance)
{
    std::array<coordinate_type, dimensions> a;
    for (size_t i = 0; i < q.count(); ++i)
    {
        a[i] = q.get(i) + distance;
    }
    return a;
}

template<typename coordinate_type, size_t dimensions>
inline point<coordinate_type, dimensions> operator-(point<coordinate_type, dimensions> q, point<coordinate_type, dimensions> p)
{
    size_t n = min(q.count(), p.count());
    std::array<coordinate_type, dimensions> a;
    for (size_t i = 0; i < n; ++i)
    {
        a[i] = q.get(i) - p[i];
    }
    return a;
}

template<typename coordinate_type, size_t dimensions>
inline point<coordinate_type, dimensions> operator-(point<coordinate_type, dimensions> p)
{
    std::array<coordinate_type, dimensions> a;
    for (size_t i = 0; i < p.count(); ++i)
    {
        a[i] = -p.get(i);
    }
    return a;
}

#endif /* my_point_hpp */
