#pragma once
#include <map>
#include <set>
#include <string>
using namespace std;

#include "free_point.hpp"

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
		GraphType::iterator it = m_gEdges.find(p0);
		if (it == m_gEdges.end())
		{
			edge[p1] = keywords;
			m_gEdges[p0] = edge;
		}
		else
		{
			m_gEdges[p0][p1] = keywords;
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
private:
	GraphType m_gEdges;
};