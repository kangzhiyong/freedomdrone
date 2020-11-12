//
//  QTFreePolygon.hpp
//  freedomdrone
//
//  Created by kangzhiyong on 2020/11/12.
//

#ifndef QTFreePolygon_hpp
#define QTFreePolygon_hpp

#include <QPolygon>

template<typename coordinate_type>
class QTFreePolygon
{
public:
    QTFreePolygon(QPolygonF _p, coordinate_type _h)
    {
        m_qPolygon = _p;
        m_cHeight = _h;
    }
    bool contains(QPointF p)
    {
        return m_qPolygon.contains(p);
    }
    coordinate_type getHeight()
    {
        return m_cHeight;
    }
private:
    coordinate_type m_cHeight;
    QPolygonF       m_qPolygon;
};

#endif /* QTFreePolygon_hpp */
