//
//  MOAIKDTree.cpp
//
// Copyright (c) 2013, Benjamin Peck
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
//
// Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
// Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "pch.h"
#include <moaicore/MOAIKDTree.h>
#include <nanoflann.hpp>

//================================================================//
// lua
//================================================================//

//----------------------------------------------------------------//
/**	@name	_setData
 @text	fills our kd tree with a table of points (x,y,z) and associated data point "a" values
 @in t table of data formatted as { x, y, z, a, ... , xN, yN, zN, aN }
 
 @out	nil
 */
int MOAIKDTree3fi::_setData ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIKDTree3fi, "UT" ) // this macro initializes the 'self' variable and type checks arguments
	
	// TODO: having the input data be in nested tables might be nicer
    //       ie { { x, y, z, a }, ... , { xN, yN, zN, aN } }
	
	int iNumPts = luaL_getn(L, 2);
	self->m_cloud.pts.resize(iNumPts / 4);
	self->m_cloud.data.resize(iNumPts / 4);
	
	int itr = state.PushTableItr ( 2 );
	int iPt = 0;
	float x,y,z;
	int a;
	
	for ( ; state.TableItrNext ( itr ); iPt++ ) {
		x = state.GetValue < float >( -1, 0 );
		state.TableItrNext ( itr );
		y = state.GetValue < float >( -1, 0 );
		state.TableItrNext ( itr );
		z = state.GetValue < float >( -1, 0 );
		state.TableItrNext ( itr );
		a = state.GetValue < int >( -1, 0 );
		
		// printf("%f, %f, %f, [%d]\n", x, y, z, a);
		
		self->m_cloud.pts[iPt].x = x;
        self->m_cloud.pts[iPt].y = y;
        self->m_cloud.pts[iPt].z = z;
        self->m_cloud.data[iPt] = a;
	}
	
	// initialize kd tree
	self->m_index.buildIndex();

	return 0;
}

//----------------------------------------------------------------//
/**	@name	_KNNData
 @text	returns a table of triplets for nearest N points to (x,y,z): (index of point in cloud, distance from query, data value of point)
 @in N
 @in x
 @in y
 @in z
 
 @out	table of N triplets: { pt cloud idx, dist, data value }
 */
int MOAIKDTree3fi::_KNN ( lua_State* L ) {
	MOAI_LUA_SETUP ( MOAIKDTree3fi, "UNNNN" ) // this macro initializes the 'self' variable and type checks arguments
	
	int N;
	float aQueryPt[3];
	
	N = state.GetValue<int>(2, 0);
	aQueryPt[0] = state.GetValue<float>(3, 0);
	aQueryPt[1] = state.GetValue<float>(4, 0);
	aQueryPt[2] = state.GetValue<float>(5, 0);
	
	if ( N <= 0 ) {
		N = 0;
	}
	if ( N > (int)self->m_cloud.pts.size() ) {
		N = self->m_cloud.pts.size();
	}
	
	std::vector<size_t> aOutRetIndex(N);
	std::vector<float> aOutDistSqr(N);
	self->m_index.knnSearch(&aQueryPt[0], N, &aOutRetIndex[0], &aOutDistSqr[0]);
	
	// outer result table
	lua_newtable(state);
	
	for (size_t i = 1; i <= aOutRetIndex.size(); i++) {
		// inner triplet table
		lua_newtable(state);
		
		// push index in point cloud
		lua_pushnumber(state, 1);
		lua_pushnumber(state, aOutRetIndex[i-1]);
		lua_rawset(state, -3);
		
		// push distance
		lua_pushnumber(state, 2);
		lua_pushnumber(state,  aOutDistSqr[i-1]);
		lua_rawset(state, -3);
		
		// push data value
		lua_pushnumber(state, 3);
		lua_pushnumber(state,  self->m_cloud.data[aOutRetIndex[i-1]]);
		lua_rawset(state, -3);
		
		// push triplet to outer result table 
		lua_pushinteger(state, i);
		lua_insert(state, -2);
		lua_rawset(state, -3);
	}
	
	return 1;
}

//================================================================//
// MOAIKDTree3fi
//================================================================//

//----------------------------------------------------------------//
MOAIKDTree3fi::MOAIKDTree3fi () :
    m_index(3 /*dim*/, this->m_cloud, KDTreeSingleIndexAdaptorParams(10 /* max leaf */) )
{
	
	// register all classes MOAIKDTree3fi derives from
	// we need this for custom RTTI implementation
	RTTI_BEGIN
	RTTI_EXTEND ( MOAILuaObject )
	
	// and any other objects from multiple inheritance...
	// RTTI_EXTEND ( MOAIKDTree3fiBase )
	RTTI_END
	
	
}

//----------------------------------------------------------------//
MOAIKDTree3fi::~MOAIKDTree3fi () {
}

//----------------------------------------------------------------//
void MOAIKDTree3fi::RegisterLuaClass ( MOAILuaState& state ) {
	// here are the class methods:
	luaL_Reg regTable [] = {
		{ NULL, NULL }
	};
	
	luaL_register ( state, 0, regTable );
}

//----------------------------------------------------------------//
void MOAIKDTree3fi::RegisterLuaFuncs ( MOAILuaState& state ) {
	// here are the instance methods:
	luaL_Reg regTable [] = {
		{ "setData",		_setData },
		{ "KNN",			_KNN },
		{ NULL,				NULL }
	};
	
	luaL_register ( state, 0, regTable );
}
