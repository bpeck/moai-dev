//
//  MOAIKDTree.h
//
// Copyright (c) 2013, Benjamin Peck
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
// Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef __MOAIKDTree_h__
#define __MOAIKDTree_h__

#include <moaicore/MOAILua.h>
#include <nanoflann.hpp>

using namespace nanoflann;
// a functor with some member data used by nanoflann:
// 3D points of type T, associated data values for each point of type T2
// This struct is mostly cribbed from nanoflann's pointcloud_kdd_radius.cpp example
template <typename T, typename T2>
struct PointCloud
{
	struct Point
	{
		T  x,y,z;
	};
	
	
	std::vector<Point>  pts;
	std::vector<T2> data;
	
	// Must return the number of data points
	inline size_t kdtree_get_point_count() const { return pts.size(); }
	
	// Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
	inline T kdtree_distance(const T *p1, const size_t idx_p2,size_t size) const
	{
		const T d0=p1[0]-pts[idx_p2].x;
		const T d1=p1[1]-pts[idx_p2].y;
		const T d2=p1[2]-pts[idx_p2].z;
		return d0*d0+d1*d1+d2*d2;
	}
	
	// Returns the dim'th component of the idx'th point in the class:
	// Since this is inlined and the "dim" argument is typically an immediate value, the
	//  "if/else's" are actually solved at compile time.
	inline T kdtree_get_pt(const size_t idx, int dim) const
	{
		if (dim==0) return pts[idx].x;
		else if (dim==1) return pts[idx].y;
		else return pts[idx].z;
	}
	
	// Optional bounding-box computation: return false to default to a standard bbox computation loop.
	//   Return true if the BBOX was already computed by the class and returned in "bb" so it can be avoided to redo it again.
	//   Look at bb.size() to find out the expected dimensionality (e.g. 2 or 3 for point clouds)
	template <class BBOX>
	bool kdtree_get_bbox(BBOX &bb) const { return false; }
	
};

// This index adaptor is used by nanoflann, telling it to use the functor I defined above.
typedef KDTreeSingleIndexAdaptor< L2_Simple_Adaptor<float, PointCloud<float, int> > ,PointCloud<float, int>, 3 /* dim */ > KDTree3fi;

// TODO: make a base class that is templated that has common methods like fill and KNN,
// then have concrete classes that inherits and specifies actual dimensionality / typing you want

//================================================================//
// MOAIKDTree3fi
//================================================================//
/**	@name	MOAIKDTree3fi
	A KD tree of float x,y,z points, and an integer value associated with each point
 */
class MOAIKDTree3fi :
public virtual MOAILuaObject {
private:
	
	// methods that get hooked into lua
	//----------------------------------------------------------------//
	static int			_setData( lua_State* L);
	static int			_KNN( lua_State* L);
	//----------------------------------------------------------------//
	
	PointCloud<float, int>	m_cloud;
	KDTree3fi				m_index;
	
	
public:
	
	DECL_LUA_FACTORY ( MOAIKDTree3fi )
	
	//----------------------------------------------------------------//
	MOAIKDTree3fi			();
	~MOAIKDTree3fi			();
	void			RegisterLuaClass	( MOAILuaState& state );
	void			RegisterLuaFuncs	( MOAILuaState& state );
};

#endif // __MOAIKDTree_h__
