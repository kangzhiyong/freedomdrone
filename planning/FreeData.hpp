#pragma once
#include <vector>
#include <string>
#include <fstream>
using namespace std;

#include "QTFreePolygon.hpp"
#include "DroneUtils.hpp"
#include "KDTree.hpp"
#include "FreeGraph.hpp"

template<typename coordinate_type>
class FreeData
{
public:
    typedef vector<coordinate_type> VCoordType;
    typedef vector<string> VString;

    static vector<Point<coordinate_type, 4>> loadtxt(string fileName, string delimiter)
    {
        vector<Point<coordinate_type, 4>> points;
        fstream file;
        file.open(fileName.c_str());
        if (file.is_open())
        {
            int i = 1;
            string str;
            while (getline(file, str))
            {
                VString strList = split(str, delimiter);
                if (strList.size() >= 4) {
                    points.push_back({(float)atof(strList[0].c_str()), (float)atof(strList[1].c_str()), (float)atof(strList[2].c_str()), (float)atof(strList[3].c_str())});
                }
            }
        }
        else
        {
            printf("Data file open failed!\r\n");
        }
        return points;
    }
    FreeData(std::string fileName, std::string delimiter)
    {
        fstream file;
        file.open(fileName.c_str());
        if (file.is_open())
        {
            int i = 1;
            string str;
            VCoordType northMin, northMax, eastMin, eastMax, altMin, altMax;
            while (getline(file, str))
            {
                VString strList = split(str, delimiter);
                if (i == 1 && strList.size() == 2)
                {
                    VString lat0 = split(strList[0], " ");
                    VString lon0 = split(strList[1], " ");
                    if (lat0.size() == 2)
                    {
                        m_dLat = atof(lat0[1].c_str());
                    }
                    if (lon0.size() == 2)
                    {
                        m_dLon = atof(lon0[1].c_str());
                    }
                }
                else if (i > 2 && strList.size() == 6)
                {
                    coordinate_type north, east, alt, dNorth, dEast, dAlt = 0;
                    north = atof(strList[0].c_str());
                    east = atof(strList[1].c_str());
                    alt = atof(strList[2].c_str());
                    dNorth = atof(strList[3].c_str());
                    dEast = atof(strList[4].c_str());
                    dAlt = atof(strList[5].c_str());
                    m_qvNorths.push_back(north);
                    m_qvEasts.push_back(east);
                    m_qvAlts.push_back(alt);
                    m_qvDNorths.push_back(dNorth);
                    m_qvDEasts.push_back(dEast);
                    m_qvDAlts.push_back(dAlt);

                    northMin.push_back(north - dNorth);
                    northMax.push_back(north + dNorth);
                    eastMin.push_back(east - dEast);
                    eastMax.push_back(east + dEast);
                    altMin.push_back(alt - dAlt);
                    altMax.push_back(alt + dAlt);
                }
                i++;
            }
            if (i > 1)
            {
                m_dNorthMin = std::floor(*(std::min_element(northMin.begin(), northMin.end())));
                m_dNorthMax = std::ceil(*(std::max_element(northMax.begin(), northMax.end())));
                m_dEastMin = std::floor(*(std::min_element(eastMin.begin(), eastMin.end())));
                m_dEastMax = std::ceil(*(std::max_element(eastMax.begin(), eastMax.end())));
                m_dAltMin = std::floor(*(std::min_element(altMin.begin(), altMin.end())));
                m_dAltMax = std::ceil(*(std::max_element(altMax.begin(), altMax.end())));
            }
        }
        else
        {
            printf("Data file open failed!\r\n");
        }
    }
    ~FreeData()
    {
        m_qvNorths.clear();
        m_qvEasts.clear();
        m_qvAlts.clear();
        m_qvDNorths.clear();
        m_qvDEasts.clear();
        m_qvDAlts.clear();
    }
    coordinate_type getNorthMin()
    {
        return m_dNorthMin;
    }
    coordinate_type getNorthMax()
    {
        return m_dNorthMax;
    }
    coordinate_type getEastMin()
    {
        return m_dEastMin;
    }
    coordinate_type getEastMax()
    {
        return m_dEastMax;
    }
    coordinate_type getAltMin()
    {
        return m_dAltMin;
    }
    coordinate_type getAltMax()
    {
        return m_dAltMax;
    }
    //获取障碍物中半径最大的
    coordinate_type getMaxPolyRadius()
    {
        coordinate_type maxN = 0, maxE = 0;
        if (m_qvDNorths.size() > 0)
        {
            maxN = *(std::max_element(m_qvDNorths.begin(), m_qvDNorths.end()));
        }
        if (m_qvDEasts.size() > 0)
        {
            maxE = *(std::max_element(m_qvDEasts.begin(), m_qvDEasts.end()));
        }
        return std::max(maxN, maxE);
    }
    size_t dataCount()
    {
        return m_qvNorths.size();
    }
    coordinate_type getNorth(int i)
    {
        return (i >= 0 && i < dataCount()) ? m_qvNorths[i] : 0;
    }
    coordinate_type getEast(int i)
    {
        return (i >= 0 && i < dataCount()) ? m_qvEasts[i] : 0;
    }
    coordinate_type getAlt(int i)
    {
        return (i >= 0 && i < dataCount()) ? m_qvAlts[i] : 0;
    }
    coordinate_type getDNorth(int i)
    {
        return (i >= 0 && i < dataCount()) ? m_qvDNorths[i] : 0;
    }
    coordinate_type getDEast(int i)
    {
        return (i >= 0 && i < dataCount()) ? m_qvDEasts[i] : 0;
    }
    coordinate_type getDAlt(int i)
    {
        return (i >= 0 && i < dataCount()) ? m_qvDAlts[i] : 0;
    }

    coordinate_type getLat()
    {
        return m_dLat;
    }

    coordinate_type getLon()
    {
        return m_dLon;
    }

    void createGrid(coordinate_type drone_altitude, coordinate_type safety_distance, vector<float> &grid, int &north_size, int &east_size, int &alt_size)
    {
        coordinate_type north, east, alt, d_north, d_east, d_alt = 0;
        int oNorthMin, oNorthMax, oEastMin, oEastMax = 0;
        vector<float>::iterator iColStart;
        north_size = std::ceil(m_dNorthMax - m_dNorthMin);
        east_size = std::ceil(m_dEastMax - m_dEastMin);
        grid.resize(east_size * north_size);

        std::for_each(grid.begin(), grid.end(), [](float& n) { n=0; });
        for (size_t i = 0; i < m_qvNorths.size(); i++)
        {
            north = m_qvNorths[i];
            east = m_qvEasts[i];
            alt = m_qvAlts[i];
            d_north = m_qvDNorths[i];
            d_east = m_qvDEasts[i];
            d_alt = m_qvDAlts[i];
            if ((alt + d_alt + safety_distance) > drone_altitude)
            {
                oNorthMin = clip(north - d_north - safety_distance - m_dNorthMin, 0, north_size - 1);
                oNorthMax = clip(north + d_north + safety_distance - m_dNorthMin, 0, north_size - 1);
                oEastMin = clip(east - d_east - safety_distance - m_dEastMin, 0, east_size - 1);
                oEastMax = clip(east + d_east + safety_distance - m_dEastMin, 0, east_size - 1);
                for (size_t m = oNorthMin; m <= oNorthMax; m++)
                {
                    iColStart = grid.begin() + m * east_size;
                    std::for_each(iColStart + oEastMin, iColStart + oEastMax, [](float& n) { n = 1; });
                }
            }
        }
    }

    /*
    Returns a grid representation of a 3D configuration space
    based on given obstacle data.
    
    The `voxel_size` argument sets the resolution of the voxel map. 
    */
    void create_voxmap(int voxel_size, vector<int>& voxmap, int &north_size, int &east_size, int &alt_size)
    {
        // given the minimumand maximum coordinates we can
        // calculate the size of the grid.
        north_size = floor(ceil(m_dNorthMax - m_dNorthMin) / voxel_size);
        east_size = floor(ceil(m_dEastMax - m_dEastMin) / voxel_size);
        alt_size = floor(m_dAltMax / voxel_size);
//        voxmap.resize(g_north_size * g_east_size * g_alt_size);
//        std::for_each(voxmap.begin(), voxmap.end(), [](int& n) { n = 0; });

        vector<vector<vector<int>>> _voxmap(north_size, vector<vector<int>>(east_size, vector<int>(alt_size, 0)));
        vector<int>::iterator iYStart, iXStart;
        float north, east, alt, d_north, d_east, d_alt = 0;
        int oNorthMin, oNorthMax, oEastMin, oEastMax, height = 0;

        for (size_t i = 0; i < m_qvNorths.size(); i++)
        {
            north = m_qvNorths[i];
            east = m_qvEasts[i];
            alt = m_qvAlts[i];
            d_north = m_qvDNorths[i];
            d_east = m_qvDEasts[i];
            d_alt = m_qvDAlts[i];
            oNorthMin = floor((north - d_north - m_dNorthMin) / voxel_size);
            oNorthMax = floor((north + d_north - m_dNorthMin) / voxel_size);
            oEastMin = floor((east - d_east - m_dEastMin) / voxel_size);
            oEastMax = floor((east + d_east - m_dEastMin) / voxel_size);
            height = floor((alt + d_alt) / voxel_size);
            for (size_t m = oNorthMin; m < oNorthMax; m++)
            {
                for (size_t n = oEastMin; n < oEastMax; n++)
                {
                    for (size_t p = 0; p < height; p++)
                    {
                        _voxmap[m][n][p] = 1;
                    }
                }
            }
        }
        for (size_t m = 0; m < north_size; m++)
        {
            for (size_t n = 0; n < east_size; n++)
            {
                for (size_t p = 0; p < alt_size; p++)
                {
                    voxmap.push_back(_voxmap[m][n][p]);
                }
            }
        }
    }

    void extract_polygons(coordinate_type safety_distance)
    {
        coordinate_type north, east, alt, d_north, d_east, d_alt = 0;
        vector<coordinate_type> obstacle;

        for (size_t i = 0; i < m_qvNorths.size(); i++)
        {
            north = m_qvNorths[i];
            east = m_qvEasts[i];
            alt = m_qvAlts[i];
            d_north = m_qvDNorths[i];
            d_east = m_qvDEasts[i];
            d_alt = m_qvDAlts[i];
            obstacle.push_back(north - d_north - safety_distance);
            obstacle.push_back(north + d_north + safety_distance);
            obstacle.push_back(east - d_east - safety_distance);
            obstacle.push_back(east + d_east + safety_distance);
            QPolygonF p;
            p.append(QPointF(obstacle[0], obstacle[2]));
            p.append(QPointF(obstacle[0], obstacle[3]));
            p.append(QPointF(obstacle[1], obstacle[3]));
            p.append(QPointF(obstacle[1], obstacle[2]));
            m_qvPolygons.push_back({p, alt + d_alt + safety_distance});
            obstacle.clear();
        }
    }
    
    bool collides(coordinate_type x, coordinate_type y, coordinate_type height)
    {
        QTFreePolygon<coordinate_type> polygon;
        for (int j = 0; j < m_qvPolygons.size(); j++) {
            polygon = m_qvPolygons[j];
            if (polygon.contains(QPointF(x, y)) && polygon.getHeight() >= height) {
                return true;
            }
        }
        return false;
    }
    void sample(int num)
    {
        m_vSamplePoints.clear();
        VCoordType xs = uniform(m_dNorthMin, m_dNorthMax, num);
        VCoordType ys = uniform(m_dEastMin, m_dEastMax, num);
        VCoordType zs = uniform((float)10.0, (float)30, num);
        for (int i = 0; i < num; i++) {
            if (!collides(xs[i], ys[i], zs[i])) {
                m_vSamplePoints.push_back({xs[i], ys[i], zs[i]});
            }
        }
    }

    void printPolygons()
    {
        QTFreePolygon<coordinate_type> polygon;
        for (int j = 0; j < m_qvPolygons.size(); j++) {
            polygon = m_qvPolygons[j];
            for (size_t i = 0; i < polygon.getPolygon().size(); i++)
            {
                cout << "(" << polygon.getPolygon()[i].x() << "," << polygon.getPolygon()[i].y() << ")";
            }
            cout << endl;
          
        }
    }

    bool can_connect(Point<coordinate_type, 3> p, Point<coordinate_type, 3> p1)
    {
        QTFreePolygon<coordinate_type> polygon;
        for (int j = 0; j < m_qvPolygons.size(); j++) {
            polygon = m_qvPolygons[j];
            if (polygon.intersect(QPointF(p[0], p[1]), QPointF(p1[0], p1[1])) && polygon.getHeight() >= min(p[2], p1[2])) {
                return false;
            }
        }
        return true;
    }

    void create_graph(int k)
    {
        Point<coordinate_type, 3> p, p1;
        KDTree<coordinate_type, 3> kdtree(m_vSamplePoints.begin(), m_vSamplePoints.end());
        for (size_t i = 0; i < m_vSamplePoints.size(); i++)
        {
            p = m_vSamplePoints[i];
            kdtree.nearest(p, k);

            for (size_t j = 0; j < kdtree.m_vNbrNodes.size(); j++)
            {
                p1 = kdtree.m_vNbrNodes[j].point_;
                if (p == p1)
                {
                    continue;
                }
                if (can_connect(p, p1))
                {
                    m_cGraph.add_edge(p, p1, {{"weight", "1"}});
                }
            }
        }
    }
    
    FreeGraph<coordinate_type, 3> getGraph()
    {
        return m_cGraph;
    }
    
    vector<Point<coordinate_type, 3>> getSamplePoints()
    {
        return m_vSamplePoints;
    }

    void setSamplePoints(vector<Point<coordinate_type, 3>> data)
    {
        m_vSamplePoints = data;
    }
private:
    coordinate_type m_dLat{ 0 };
    coordinate_type m_dLon{ 0 };
    // minimumand maximum north/east/alt coordinates
    coordinate_type m_dNorthMax{ 0 };
    coordinate_type m_dNorthMin{ 0 };
    coordinate_type m_dEastMax{ 0 };
    coordinate_type m_dEastMin{ 0 };
    coordinate_type m_dAltMax{ 0 };
    coordinate_type m_dAltMin{ 0 };
    VCoordType m_qvNorths;
    VCoordType m_qvEasts;
    VCoordType m_qvAlts;
    VCoordType m_qvDNorths;
    VCoordType m_qvDEasts;
    VCoordType m_qvDAlts;
    vector<QTFreePolygon<float>> m_qvPolygons;
    vector<Point<coordinate_type, 3>> m_vSamplePoints;
    FreeGraph<coordinate_type, 3> m_cGraph;
};
