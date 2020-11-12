#pragma once
#include "free_point.hpp"

template<typename coordinate_type>
class FreeRect
{
public:
    typedef point<coordinate_type, 2> point2DFR;
    FreeRect(const point2DFR topLeft, const point2DFR bottomRight): m_pTopLeft(topLeft), m_pBottomRight(bottomRight)
    {
        if (topLeft.get(0) > bottomRight.get(0))
        {
            m_pTopLeft = bottomRight;
            m_pBottomRight = topLeft;
        }
        else
        {
            m_pTopLeft = topLeft;
            m_pBottomRight = bottomRight;
        }
    }

    point2DFR getTopLeft()
    {
        return m_pTopLeft;
    }

    point2DFR getBottomRight()
    {
        return m_pBottomRight;
    }

    void setTopLeft(point2DFR topLeft)
    {
        m_pTopLeft = topLeft;
    }

    void setBottomRight(point2DFR bottomRight)
    {
        m_pBottomRight = bottomRight;
    }

    point2DFR center()
    {
        float cx = m_pTopLeft[0] + (m_pBottomRight[0] - m_pTopLeft[0]) / 2.0;
        float cy = m_pTopLeft[1] + (m_pBottomRight[1] - m_pTopLeft[1]) / 2.0;
        return point2DFR({cx, cy});
    }

    bool contains(point2DFR p)
    {
        if (p[0] >= m_pTopLeft[0] && p[0] <= m_pBottomRight[0]
            && p[1] >= m_pTopLeft[1] && p[1] <= m_pBottomRight[1])
        {
            return true;
        }
        return false;
    }

    bool crosses(FreeRect rect)
    {
        double minx = std::max(m_pTopLeft[0], rect.getTopLeft()[0]);
        double miny = std::max(m_pTopLeft[1], rect.getTopLeft()[1]);
        double maxx = std::min(m_pBottomRight[0], rect.getBottomRight()[0]);
        double maxy = std::min(m_pBottomRight[1], rect.getBottomRight()[1]);
        if (minx > maxx || miny > maxy)
        {
            return false;
        }
        return true;
    }
private:
    point2DFR m_pTopLeft;
    point2DFR m_pBottomRight;
};

template<typename coordinate_type>
class FreePolygon: public FreeRect<coordinate_type>
{
private:
    coordinate_type m_fHeight;
public:
    typedef point<coordinate_type, 2> point2DFR;
    FreePolygon(const point2DFR topLeft, const point2DFR bottomRight, coordinate_type height):FreeRect<coordinate_type>(topLeft, bottomRight), m_fHeight(height)
    {
    }

    coordinate_type getHeight()
    {
        return m_fHeight;
    }
    void setHeight(coordinate_type height)
    {
        m_fHeight = height;
    }

};
