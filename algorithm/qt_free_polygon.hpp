//
//  QTFreePolygon.hpp
//  freedomdrone
//
//  Created by kangzhiyong on 2020/11/12.
//

#ifndef QTFreePolygon_hpp
#define QTFreePolygon_hpp

#include <QPolygon>
#include <QLine>

template<typename coordinate_type>
class QTFreePolygon
{
public:
    QTFreePolygon() {}
    QTFreePolygon(QPolygonF _p, coordinate_type _h)
    {
        m_qPolygon = _p;
        m_cHeight = _h;
    }
    bool contains(QPointF p)
    {
        return m_qPolygon.containsPoint(p, Qt::OddEvenFill);
    }
    bool intersect(QPointF p0, QPointF p1)
    {
        if (contains(p0) || contains(p1))
        {
            return true;
        }
        QLineF line0(p0, p1);
        QLineF line1;
        for (size_t i = 1; i < m_qPolygon.size(); i++)
        {
            line1 = (m_qPolygon[i-1], m_qPolygon[i]);
            if (line1.intersect(line0, &QPoint()) == BoundedIntersection)
            {
                return true;
            }
        }
        line1 = (m_qPolygon[m_qPolygon.size()], m_qPolygon[0]);
        if (line1.intersect(line0, &QPoint()) == BoundedIntersection)
        {
            return true;
        }
        return false;
    }
    coordinate_type getHeight()
    {
        return m_cHeight;
    }
    QPolygonF getPolygon()
    {
        return m_qPolygon;
    }
private:
    coordinate_type m_cHeight;
    QPolygonF       m_qPolygon;
};

#endif /* QTFreePolygon_hpp */
