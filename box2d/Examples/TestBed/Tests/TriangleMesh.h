/* Header file for TriangleMesh.cpp
 */
#ifndef __TRIANGLE_MESH_H
#define __TRIANGLE_MESH_H

#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <float.h>
#include <math.h>
#include <assert.h>

#ifndef TRIANGLEMESH_STANDALONE
#include "Box2D.h"
#else
   typedef float      float32;
   typedef signed int int32;
#endif

#define tmVERSION 1.002

#define tmAssert(condition) assert((condition)) 

// errors and warnings - TODO
#define tmE_OK    0
#define tmE_MEM   1
#define tmE_HOLES 2

// constants
#define tmC_ZEROTOL    0.00001
#define tmC_PI         3.14159265359f
#define tmC_PIx2       6.28318530718f
#define tmC_PI_3       1.04719755119f
#define tmC_SQRT2      1.41421356237f
// default big number to calculate a triangle covering all points (vertices[0-2])
#define tmC_BIGNUMBER       1.0e10f
// defaulat max. nodes number
#define tmC_DEFAULTMAXVERTEX 500
// default abort-inserting angle if tmO_GRADING set
#define tmC_DEFAULTGRADINGLOWERANGLE   30.0f
// options
// automatic segment all input vertices
#define tmO_SEGMENTBOUNDARY  2
// hull all vertices
#define tmO_CONVEXHULL       4
// abort inserting segments,if worst angle > minAngle
#define tmO_MINIMALGRID      8  // depreciated, use next
#define tmO_GRADING          8
// only for testing
#define tmO_BASICMESH       16
// internal option bit to mark, if there were enough maxVertexCount
#define tmO_ENOUGHVERTICES 128
// only for debug, testing
#define tmO_NOCALC         256
// only for debug, testing
#define tmO_BASICMESHNODEL 512

typedef struct
{
  float32 x,y;
} tmVertex;

typedef struct
{
  tmVertex *v[2];
} tmSegment;

typedef struct
{
  int32 i1,i2;
} tmSegmentId;

typedef struct
{
 tmVertex        *v[2];
 struct Triangle *t[2];
 bool   locked;
} tmEdge ;

typedef struct Triangle
{
 tmVertex *v[3];
 tmEdge   *e[3];
 float32 minAngle, angle;
 float32 area;
 bool    inside;
 // hold attributes for the triangles, external use only
 void    *userData;
} tmTriangle;

class TriangleMesh
{
  public:

   TriangleMesh(int32 aMaxVertexCount=tmC_DEFAULTMAXVERTEX,
                int32 aOptions=tmO_GRADING|tmO_CONVEXHULL);

   int32  Mesh(tmVertex *input, int32 n_input,
               tmSegmentId *segment=NULL, int32 n_segment=0,
               tmVertex *hole=NULL, int32 n_holes=0);

   void SegmentVertices(int32 startNode, int32 endNode, bool doclose);
   void SetOptions(int32 aOptions)     { options = aOptions;  }
   void AddOption(int32 aOptions)      { options |= aOptions; }
   void DeleteOption(int32 aOptions)   { options &= ~aOptions; }
   void SetMaxVertexCount(int32 count) {
       if ( count>3 )
       {
           maxVertexCount = count;
           options &= ~tmO_GRADING;
       }
   }
   void   SetGradingLowerAngle(float32 angle)
   {
       gradingLowerAngle = angle;
       options |= tmO_GRADING;
   }


   int32  GetVertexCount()          { return vertexCount;        }
   int32  GetInputVertexCount()     { return inputVertexCount;   }
   int32  GetEdgeCount()            { return edgeCount;          }
   int32  GetTriangleCount()        { return triangleCount;      }
   int32  GetSegmentCount()         { return segmentCount;       }
   int32  GetHoleCount()            { return holeCount;          }
   int32  GetInsideTriangleCount()  { return insideTriangleCount;}
   tmVertex*     GetVertices()  { return Vertices;  }
   tmEdge*       GetEdges()     { return Edges;     }
   tmTriangle*   GetTriangles() { return Triangles;  }
   tmSegment*    GetSegments()  { return Segments; }
   tmVertex*     GetHoles()     { return Holes;     }
   void PrintData(FILE* f = stdout);
   void FreeMemory();
   int32 PrintTriangles();

  private:

   /* Data */
   tmVertex   *Vertices;
   tmEdge     *Edges;
   tmTriangle* Triangles;
   tmSegment  *Segments;
   tmVertex   *Holes;

   int32       maxVertexCount, maxEdgeCount,
               maxTriangleCount,maxSegmentCount;
   int32       vertexCount, inputVertexCount;
   int32       edgeCount, triangleCount, segmentCount, holeCount;
   int32       insideTriangleCount;

   float32     gradingLowerAngle;
   int32       options;

   tmTriangle* lastTriangle;

   /* Functions */

   void        Triangulate();
   int32       MarkInsideTriangles(bool holes);
   void        InsertSegments();
   void        DeleteBadTriangles();
   void        DeleteTriangle(tmTriangle* t);
   //void        SegmentBoundary(int32 startNode, int32 endNode, bool doclose);
   tmVertex*   AddVertex();
   tmVertex*   GetClosestVertex(float32 x, float32 y);
   tmTriangle* FindVertex(tmVertex* v);
   bool        ContainsVertex(tmVertex* v0, tmVertex* v1, tmVertex* v);
   float32     GetVertexPosition(tmVertex* a, tmVertex* b, tmVertex* c);
   bool        InsertVertexAt(tmVertex* v, tmEdge* e);
   bool        InsertVertex(tmVertex* v);
   bool        SameVertex(tmVertex* v0, tmVertex* v1);
   tmVertex*   GetOppositeVertex(tmEdge* e, tmTriangle* t);

   tmEdge*     AddEdge();
   void        SetEdge(tmEdge* e, tmVertex* v0, tmVertex* v1, tmTriangle* t0, tmTriangle* t1);
   void        FixEdge(tmEdge* e, tmTriangle* t0, tmTriangle* t1);
   tmEdge*     GetEdge(tmVertex* v0, tmVertex* v1);
   bool        CheckEdge(tmEdge* e);
   tmSegment*  AddSegment();
   tmSegment*  GetSegment(tmVertex* v0, tmVertex* v1);

   tmTriangle* AddTriangle();
   void        SetTriangle(tmTriangle* t,
                           tmVertex* v0, tmVertex* v1, tmVertex* v2,
                           tmEdge* e0, tmEdge* e1, tmEdge* e2);
   bool        SetTriangleData(tmVertex* v0,tmVertex* v1,tmVertex* v2,
                               float32 *minAngle, float32 *angle, float32 *area);

   void        GetAdjacentEdges(tmEdge* e, tmTriangle* t,
                                tmEdge** e0, tmEdge** e1, tmVertex** v);
   bool        IsOppositeVertex(tmVertex* v0, tmVertex* v1, tmVertex* v2);
   bool        HasBoundingVertices(tmVertex* v0,tmVertex* v1,tmVertex* v2);
   void        CircumCenter(tmVertex* c, tmTriangle* t);
   void        GetSplitPosition(tmVertex* v, tmVertex* v0, tmVertex* v1);
   bool        SplitSegment(tmSegment* s);
   void        ConvexHull();

   void        Reset();
   void        CheckNumber(float32 x);
   float32     ArcTan2(float32 x, float32 y);
   float32     GetAngle(float32 a1, float32 a0);
};

#endif


