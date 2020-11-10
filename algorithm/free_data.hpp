#pragma once
#include <vector>
#include <string>
#include <fstream>
using namespace std;

template<typename coordinate_type>
class FreeData
{
public:
    typedef vector<coordinate_type> VCoordType;
    typedef vector<string> VString;
    VString split(string str, string delimiter)
    {
        VString strList;
        char* tmp = nullptr;
        tmp = strtok((char*)str.c_str(), delimiter.c_str());
        while (tmp)
        {
            strList.emplace_back(tmp);
            tmp = strtok(nullptr, ",");
        }
        return strList;
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

    void createGrid(coordinate_type drone_altitude, coordinate_type safety_distance, vector<int> &grid, int &north_size, int &east_size)
    {
        coordinate_type north, east, alt, d_north, d_east, d_alt = 0;
        int oNorthMin, oNorthMax, oEastMin, oEastMax = 0;
        vector<int>::iterator iColStart;
        north_size = std::ceil(m_dNorthMax - m_dNorthMin);
        east_size = std::ceil(m_dEastMax - m_dEastMin);
        grid.resize(east_size * north_size);

        std::for_each(grid.begin(), grid.end(), [](int& n) { n=0; });
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
                    std::for_each(iColStart + oEastMin, iColStart + oEastMax, [](int& n) { n = 1; });
                }
                //for (size_t m = oEastMin; m <= oEastMax; m++)
                //{
                //    iColStart = grid.begin() + m * north_size;
                //    std::for_each(iColStart + oNorthMin, iColStart + oNorthMax, [](int& n) { n = 1; });
                //}
            }
        }
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
};