#pragma once

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
class Point
{
public:
    Point(){}
    Point(std::array<coordinate_type, dimensions> c) : coords_(c)
    {
    }
    Point(std::initializer_list<coordinate_type> list)
    {
        size_t n = min(dimensions, list.size());
        std::copy_n(list.begin(), n, coords_.begin());
    }
    Point(coordinate_type x, coordinate_type y, coordinate_type z)
    {
        coords_[0] = x;
        coords_[1] = y;
        coords_[2] = z;
    }
    void operator=( const Point<coordinate_type, dimensions> &p)
    {
        size_t n = min(dimensions, p.count());
        for (size_t i = 0; i < n; ++i)
        {
            coords_[i] = p.get(i);
        }
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
    float distance(const Point<coordinate_type, dimensions>& pt) const
    {
        float dist = 0;
        for (size_t i = 0; i < dimensions; ++i)
        {
            float d = get(i) - pt.get(i);
            dist += d * d;
        }
        return sqrt(dist);
    }

    Point<coordinate_type, dimensions> operator-()
    {
        std::array<coordinate_type, dimensions> a;
        for (size_t i = 0; i < dimensions; ++i)
        {
            a[i] = -get(i);
        }
        return a;
    }

    bool operator ==(const Point<coordinate_type, dimensions>& p)
    {
        return std::equal(coords_.begin(), coords_.end(), p.coords_.begin());
    }
    
    coordinate_type& operator[](int i)
    {
        return coords_[i];
    }
    
    coordinate_type operator[](int i) const
    {
        return coords_[i];
    }
    
    Point<coordinate_type, dimensions> operator+(Point<coordinate_type, dimensions> &p)
    {
        size_t n = min(dimensions, p.coords_.size());
        std::array<coordinate_type, dimensions> a;
        for (size_t i = 0; i < n; ++i)
        {
            a[i] = get(i) + p[i];
        }
        return a;
    }

    Point<coordinate_type, dimensions> operator+(coordinate_type distance)
    {
        std::array<coordinate_type, dimensions> a;
        for (size_t i = 0; i < dimensions; ++i)
        {
            a[i] = get(i) + distance;
        }
        return a;
    }
    
    inline Point<coordinate_type, dimensions> operator-(const Point<coordinate_type, dimensions> q) const
    {
        size_t n = min(count(), q.count());
        std::array<coordinate_type, dimensions> a;
        for (size_t i = 0; i < n; ++i)
        {
            a[i] = get(i) - q[i];
        }
        return a;
    }

    Point<coordinate_type, dimensions> operator*(const coordinate_type m) const
    {
        std::array<coordinate_type, dimensions> a;
        for (size_t i = 0; i < dimensions; ++i)
        {
            a[i] = get(i) * m;
        }
        return a;
    }

    Point<coordinate_type, dimensions> operator*(Point<coordinate_type, dimensions> m)
    {
        std::array<coordinate_type, dimensions> a;
        for (size_t i = 0; i < dimensions; ++i)
        {
            a[i] = get(i) * m.get(i);
        }
        return a;
    }
    
    Point<coordinate_type, dimensions> operator/(coordinate_type m)
    {
        std::array<coordinate_type, dimensions> a;
        for (size_t i = 0; i < dimensions; ++i)
        {
            a[i] = get(i) / m;
        }
        return a;
    }
    
    Point<coordinate_type, dimensions> operator-(Point<coordinate_type, dimensions>& p)
    {
        size_t n = min(dimensions, p.coords_.size());
        std::array<coordinate_type, dimensions> a;
        for (size_t i = 0; i < n; ++i)
        {
            a[i] = get(i) - p[i];
        }
        return a;
    }
    
    void print(std::string prefix="")
    {
        std::cout << prefix << "( ";
        for (size_t i = 0; i < dimensions; ++i)
        {
            std::cout << get(i) << " ";
        }
        std::cout<< ")" << std::endl;
    }

    bool operator <(const Point<coordinate_type, dimensions>& d) const
    {
        return distance({0, 0, 0}) < d.distance({0, 0, 0});
    }
    
    inline coordinate_type mag() const
    {
        return sqrt(magSq());
    }

    inline coordinate_type magSq() const
    {
        float dist = 0;
        for (size_t i = 0; i < dimensions; ++i)
        {
            dist += pow(coords_[i], 2);
        }
        return dist;
    }
    
    inline Point<coordinate_type, dimensions> norm()
    {
        Point<coordinate_type, dimensions> res;
        float l = mag();
        if (l == 0.0f)
            return {0, 0, 0};
        return operator/(l);
    }

    inline Point<coordinate_type, dimensions> cross(Point<coordinate_type, dimensions> v)
    {
        Point<coordinate_type, dimensions> resVector;
        resVector[0] = coords_[1] * v[2] - coords_[2] * v[1];
        resVector[1] = coords_[2] * v[0] - coords_[0] * v[2];
        resVector[2] = coords_[0] * v[1] - coords_[1] * v[0];
        return resVector;
    }
    
    size_t count() const
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
    coordinate_type x() const
    {
        return coords_[0];
    }
    coordinate_type y() const
    {
        return coords_[1];
    }
    coordinate_type z() const
    {
        return coords_[2];
    }
    
    coordinate_type dot(const Point<coordinate_type, dimensions> & b) const
    {
        coordinate_type dist = 0;
        for (size_t i = 0; i < dimensions; ++i)
        {
            dist += get(i) * b.get(i);
        }
        return dist;
    }
    inline float dist(const Point<coordinate_type, dimensions> b) const
    {
        return operator-(b).mag();
    }
private:
    std::array<coordinate_type, dimensions> coords_;
};
 
template<typename coordinate_type, size_t dimensions>
std::ostream& operator<<(std::ostream& out, const Point<coordinate_type, dimensions>& pt)
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

typedef Point<float, 2> V2F;
typedef Point<int, 2> V2I;
typedef Point<double, 3> V3D;
typedef Point<float, 3> V3F;
typedef Point<float, 4> V4F;
typedef Point<double, 4> V4D;

template<typename coordinate_type, size_t dimensions>
inline Point<coordinate_type, dimensions> operator*(float a, Point<coordinate_type, dimensions> b)
{
    std::array<coordinate_type, dimensions> xa;
    for (size_t i = 0; i < dimensions; ++i)
    {
        xa[i] = b.get(i) * a;
    }
    return xa;
}

template<typename coordinate_type, size_t dimensions>
inline Point<coordinate_type, dimensions> operator+(Point<coordinate_type, dimensions> q, Point<coordinate_type, dimensions> p)
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
inline Point<coordinate_type, dimensions> operator+(Point<coordinate_type, dimensions> q, coordinate_type distance)
{
    std::array<coordinate_type, dimensions> a;
    for (size_t i = 0; i < q.count(); ++i)
    {
        a[i] = q.get(i) + distance;
    }
    return a;
}

template<typename coordinate_type, size_t dimensions>
inline Point<coordinate_type, dimensions> operator-(Point<coordinate_type, dimensions> q, Point<coordinate_type, dimensions> p)
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
inline Point<coordinate_type, dimensions> operator-(Point<coordinate_type, dimensions> p)
{
    std::array<coordinate_type, dimensions> a;
    for (size_t i = 0; i < p.count(); ++i)
    {
        a[i] = -p.get(i);
    }
    return a;
}

template<typename coordinate_type, size_t dimensions>
inline coordinate_type norm_2(const Point<coordinate_type, dimensions>& a)
{
  return a.magSq();

}
