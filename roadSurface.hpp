#ifndef __ROAD_SURFACE__
#define __ROAD_SURFACE__

#include <pcl/kdtree/kdtree_flann.h>
//PointT variable as Point type is declared
typedef pcl::PointXYZ PointT;

enum eCellStatus{
	CELL_NOT_CHECKED,
	CELL_CHECKED,
	CELL_NOT_ROAD_SURFACE,
	CELL_PASSED_ONCE,
	CELL_PASSED_TWICE,
	CELL_ROAD_SURFACE
};

class roadSurface;

class gridMap{
	public:
		float numSquares;
		int numRows,numCols;
		pcl::PointCloud<PointT>::Ptr cloud;
		PointT minPt, maxPt;
		PointT mapCorners[4];
		float squareSideSize;
        //char** cellStatus;
        char cellStatus[500][500];
        float cellAvgZ[500][500];

	public:	
		gridMap(char* fileName,float squareSideSize);
		void drawGridOnMap();
        void fillColorOnMap(roadSurface*);
		void pointToCell(PointT& pt,int&,int&);
		void cellToPoint(int&,int&,PointT&);
};

class roadSurface{
	public:
		gridMap* map;
		PointT searchPoint;
		int cellRow,cellCol;
		pcl::KdTreeFLANN<PointT> kdtree;

	public:	
		roadSurface(char* fileName,float squareSideSize);
		~roadSurface();
		void detectRoadSurface();
		void findAvgZofCell(int& cellRow,int& cellCol);
		bool findCellsAndNeighbours(int& cellRow,int& cellCol,int& cr,int& cc,int& index);
		bool isCellRoadSurface(int& cellRow,int& cellCol);
		bool findAvgZofNeighbourCell(int& cellRow, int& cellCol);
};
#endif
