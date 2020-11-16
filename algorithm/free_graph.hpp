#pragma once
#include <map>
#include <set>
#include <string>
using namespace std;

#include "free_point.hpp"
template<typename coordinate_type, size_t dimensions>
class FreeEdge {
public:
    typedef point<coordinate_type, dimensions> PointType;
    FreeEdge(){}
    FreeEdge(PointType s, PointType e)
    {
        m_pStart = s;
        m_pEnd = e;
    }
    PointType getStart()
    {
        return m_pStart;
    }
    PointType getEnd()
    {
        return m_pEnd;
    }
private:
    PointType m_pStart;
    PointType m_pEnd;
};

template<typename coordinate_type, size_t dimensions>
class FreeGraph
{
public:
	typedef point<coordinate_type, dimensions> PointType;
	typedef map<PointType, map<string, string>> EdgesType;
	typedef map<PointType, EdgesType> GraphType;
	FreeGraph() {}
	~FreeGraph() {}
	void add_edge(PointType p0, PointType p1, const map<string, string>& keywords = {})
	{
		EdgesType edge;
        typename GraphType::iterator it = m_gEdges.find(p0);
		if (it == m_gEdges.end())
		{
			edge[p1] = keywords;
			m_gEdges[p0] = edge;
		}
		else
		{
			m_gEdges[p0].insert(pair<PointType, map<string, string>>(p1, keywords));
		}

		edge.clear();

		it = m_gEdges.find(p1);
		if (it == m_gEdges.end())
		{
			edge[p0] = keywords;
			m_gEdges[p1] = edge;
		}
		else
		{
			m_gEdges[p1][p0] = keywords;
		}
	}

	EdgesType edges(PointType vertice)
	{
		return m_gEdges[vertice];
	}
    
    GraphType getAllEdges()
    {
        return m_gEdges;
    }
    
    void getAllNodesAndEdges(vector<PointType> &nodes, vector<FreeEdge<coordinate_type, dimensions>> &_edges)
    {
        typename GraphType::iterator it = m_gEdges.begin();
        while (it != m_gEdges.end()) {
            PointType start = it->first;
            nodes.push_back(start);
            EdgesType edges = it->second;
            typename EdgesType::iterator eit = edges.begin();
            while (eit != edges.end()) {
                _edges.push_back({start, eit->first});
                eit++;
            }
            it++;
        }
    }
private:
	GraphType m_gEdges;
};
