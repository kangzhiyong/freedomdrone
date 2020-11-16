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
        return m_qPolygon.containsPoint(p, Qt::WindingFill);
    }
    bool intersect(QPointF p0, QPointF p1)
    {
        if (contains(p0) != contains(p1))
        {
            return true;
        }
        QLineF line0(p0, p1);
        
//        QLineF line1;
        QPointF tmpP;
//        for (size_t i = 1; i < m_qPolygon.size(); i++)
//        {
//            line1 = QLineF(m_qPolygon[i-1], m_qPolygon[i]);
//            if (line1.intersects(line0, &tmpP) == QLineF::BoundedIntersection)
//            {
//                return true;
//            }
//        }
//        line1 = QLineF(m_qPolygon[m_qPolygon.size() - 1], m_qPolygon[0]);
//        if (line1.intersects(line0, &tmpP) == QLineF::BoundedIntersection)
//        {
//            return true;
//        }
        
        QPointF a = m_qPolygon.back();
        foreach(QPointF b, m_qPolygon)
        {
            if (QLineF::BoundedIntersection == line0.intersect(QLineF(a, b), &tmpP))
            {
                return  true;
            }
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
