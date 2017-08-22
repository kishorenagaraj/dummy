#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include "roadSurface.hpp"
#include <string>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <fstream>

//PointT variable as Point type is declared
typedef pcl::PointXYZ PointT;

//Debug use only
char** debugArray=NULL;
int debugRows=0,debugCols=0;

//square side size
#define SQUARE_SIDE_SIZE 0.5
#define Z_THRESHOLD 0.1//0.1
#define NUM_POINTS_ON_CELL 50
#define PRCNTG_POINTS_ON_CELL 0.6 //0.6
pcl::visualization::PCLVisualizer viewer ("PCL visualizer");

ofstream myfile;

// void print_allCellStatus(gridMap *map){
// 	PointT pt;
// 	pt.x=0;pt.y=0;
// 	myfile.open ("del.txt");
// 	int cellIndex=0;
// 	for(int i=debugRows-1;i>=0;i--){
// 		for(int j=0;j<debugCols;j++){
// 			myfile << (int)debugArray[i][j];
// 			// if(debugArray[i][j] == CELL_PASSED_TWICE)
// 			// 	cout << "p=" << i <<", " << j << endl;
// 			if(debugArray[i][j] > 1){
// 				std::string cellName = "cell" + std::to_string(++cellIndex);
// 				map->cellToPoint(i,j,pt);
// 				viewer.addText3D(std::to_string(debugArray[i][j]),pt,0.2,1.0,0.0,0.0,cellName.c_str(),0);
// 			}
// 		}
// 		myfile << endl;
// 	}
// 	myfile.close();
// 	//viewer.addText3D(std::to_string(4),pt,1.0,1.0,0.0,0.0,"cell1",0);
// }
// void print_cellStatus(int i,int j){
// 			cout << "cell(" << i<<", " << j <<") = " << (int)debugArray[i][j] << endl;

// }

gridMap::gridMap(char* fileName,float squareSideSize)
{
	if(!boost::filesystem::exists(fileName)){
		cout << "file does not exist" << endl;
		exit(-1);
	}
	this->squareSideSize = squareSideSize;
	cout << "squareSideSize = " << this->squareSideSize << endl;
	cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr (new pcl::PointCloud<pcl::PointXYZ>);
	//load the PCD file
  	pcl::io::loadPCDFile<pcl::PointXYZ> (fileName, *cloud);
  	//find min and max points on the map
  	pcl::getMinMax3D (*cloud, minPt, maxPt);
  	//find the 4 corners of the map
  	mapCorners[0].x =minPt.x; mapCorners[0].y=minPt.y; mapCorners[0].z=0;
    mapCorners[1].x =minPt.x; mapCorners[1].y=maxPt.y; mapCorners[1].z=0; 
	mapCorners[2].x =maxPt.x; mapCorners[2].y=maxPt.y; mapCorners[2].z=0;
	mapCorners[3].x =maxPt.x; mapCorners[3].y=minPt.y; mapCorners[3].z=0;

}

void gridMap::drawGridOnMap()
{
	//4 border lines are drawn
	for(int i=0;i<4;i++){
		std::string lineName = "line" + std::to_string(i+1);
		viewer.addLine (mapCorners[i], mapCorners[(i+1)%4], lineName.c_str(), 0); 
    	viewer.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, lineName.c_str(), 0); 
    	viewer.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR,1.0, 1.0, 1.0 /* white */, lineName.c_str(), 0); 	
	}
	numRows=0;
	// loop that runs to draw the horizontal lines
	for(PointT p=mapCorners[0],q=mapCorners[0]; p.y < mapCorners[2].y; p.y+=squareSideSize,q=p)
	{
		q.x=mapCorners[2].x;
		std::string lineName = "lineH" + std::to_string(numRows);
		viewer.addLine(p,q,lineName.c_str(),0);
		viewer.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 0.5, lineName.c_str(), 0); 
        viewer.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR,0.5, 0.5, 0.5 /* white */, lineName.c_str(), 0);	
		numRows++;        
	}
	numCols=0;
	// loop that runs to draw the vertical lines
	for(PointT p=mapCorners[0],q=mapCorners[0]; p.x < mapCorners[2].x;p.x+=squareSideSize,q=p)
	{
		q.y=mapCorners[2].y;
		std::string lineName = "lineV" + std::to_string(numCols);
		viewer.addLine(p,q,lineName.c_str(),0);
		viewer.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 0.5, lineName.c_str(), 0); 
        viewer.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR,0.5, 0.5, 0.5 /* white */, lineName.c_str(), 0);	
		numCols++;        
	}

//	cellStatus = new char*[numRows];
//	for(int i = 0; i < numRows; ++i)
//    	cellStatus[i] = new char[numCols];
    
	for(int j = 0; j < numRows; j++){
   		for(int i = 0; i < numCols; i++){
   			cellStatus[j][i]=0;
   			cellAvgZ[j][i]=0.0;
   		}
	}

	//debug
//    debugArray = &cellStatus[0][0];
    debugCols=numCols;
    debugRows=numRows;

	viewer.addCoordinateSystem(1);
	viewer.setBackgroundColor(0, 0, 0);
	viewer.addPointCloud(cloud, "cloud");
}

void gridMap::fillColorOnMap(){
	static int circleIndex=0;
	float radius = squareSideSize*0.7071;
	PointT pt;

	for(int cr=0; cr < numRows; cr++){
		for(int cc=0; cc < numCols; cc++){
			if(cellStatus[cr][cc] == CELL_PASSED_TWICE){
				cellToPoint(cr,cc,pt);
				std::string circleName = "circle" + std::to_string(++circleIndex);

				viewer.addSphere(pt,(radius*0.7),circleName.c_str(),0);
				viewer.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE,circleName.c_str(),0); 
				viewer.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_SHADING, pcl::visualization::PCL_VISUALIZER_SHADING_FLAT, circleName.c_str(),0 );
				viewer.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.5, 0.5, 0, circleName.c_str(),0 );
			}
		}
	}
}

roadSurface::roadSurface(char* fileName,float squareSideSize)
{
	map = new gridMap(fileName,squareSideSize);
}

roadSurface::~roadSurface()
{
//	for(int i = 0; i < map->numCols; ++i) {
//    	delete [] map->cellStatus[i];
//	}
//	delete [] map->cellStatus;
	delete(map);
}

void gridMap::pointToCell(PointT& pt,int& cellRow, int& cellCol){
	cellCol = abs(ceil((mapCorners[0].x-pt.x)/squareSideSize));
	cellRow = abs(ceil((mapCorners[0].y-pt.y)/squareSideSize));
	//cout << cellRow << " ,"<< cellCol << endl;
}

void gridMap::cellToPoint(int& cellRow, int& cellCol,PointT& pt){
	pt.x = mapCorners[0].x + (cellCol * squareSideSize)+(squareSideSize/2.0);
	pt.y = mapCorners[0].y + (cellRow * squareSideSize)+(squareSideSize/2.0);
	//cout << pt.x << " ,"<< pt.y << endl;
}

void roadSurface::detectRoadSurface()
{
	float avgZofCell=0;
	int CR=0,CC=0;
	bool rowChangeFlag=false;

	map->pointToCell(searchPoint,cellRow,cellCol);
	map->cellToPoint(cellRow,cellCol,searchPoint);
	kdtree.setInputCloud(map->cloud);

	map->cellStatus[cellRow][cellCol] = CELL_PASSED_ONCE;

	for(int cr=cellRow; cr < map->numRows; cr++)
	{
		if(cr!=cellRow){
			rowChangeFlag=true;
		}
		for(int cc=cellCol; cc < map->numCols; cc++)
		{
			for(int nIndex=0;nIndex<9;nIndex++)
			{
				if(findCellsAndNeighbours(cr,cc,CR,CC,nIndex))
				{
					if((map->cellStatus[CR][CC] > CELL_PASSED_ONCE)
                        || (map->cellStatus[CR][CC] == CELL_NOT_ROAD_SURFACE)){

						if(map->cellStatus[CR][CC] == CELL_NOT_CHECKED)
								map->cellStatus[CR][CC] = CELL_CHECKED;
					}
					else{
							if (rowChangeFlag){
								findAvgZofNeighbourCell(CR,CC,avgZofCell);
								findAvgZofCell(CR,CC,avgZofCell);
								rowChangeFlag = false;
							}
							if(nIndex == 0){
								findAvgZofCell(CR,CC,avgZofCell);
							}
							isCellRoadSurface(CR,CC,avgZofCell);
							//cout << ".";
							if(map->cellStatus[CR][CC] == CELL_NOT_CHECKED)
								map->cellStatus[CR][CC] = CELL_CHECKED;
					}
				}
			}				
		}
	}
	//cout << endl;
	map->fillColorOnMap();
	//print_allCellStatus(map);
}

void roadSurface::findAvgZofCell(int& cellRow,int& cellCol,float& avgZ){
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	float radius = map->squareSideSize*0.7071;
	avgZ=0;

    map->cellToPoint(cellRow,cellCol,searchPoint);

	if (kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
	{
		for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i){
			avgZ += map->cloud->points[pointIdxRadiusSearch[i]].z;
		}
		avgZ /= (float)pointIdxRadiusSearch.size();
        searchPoint.z = avgZ;
        map->cellAvgZ[cellRow][cellCol]=avgZ;
	}
    //else
        //cout << "";
	//cout <<"avg z of cell = " << avgZ << endl;
}

bool roadSurface::isCellRoadSurface(int& cellRow,int& cellCol,float& avgZ)
{
	std::vector<int> pointIdxRadiusSearch;
	std::vector<float> pointRadiusSquaredDistance;
	float radius = map->squareSideSize*0.7071;

	map->cellToPoint(cellRow,cellCol,searchPoint);

	if (kdtree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
	{
		int count=0;
		for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i){
			if((map->cloud->points[pointIdxRadiusSearch[i]].z < (avgZ + Z_THRESHOLD))
				&& (map->cloud->points[pointIdxRadiusSearch[i]].z > (avgZ - Z_THRESHOLD))){
				count++;
			}
			map->cellAvgZ[cellRow][cellCol] += map->cloud->points[pointIdxRadiusSearch[i]].z;
		}
		if(count > 0 && count >= PRCNTG_POINTS_ON_CELL*pointIdxRadiusSearch.size())
		{
			map->cellAvgZ[cellRow][cellCol] /= pointIdxRadiusSearch.size();
			if(map->cellStatus[cellRow][cellCol] == CELL_NOT_CHECKED){
				map->cellStatus[cellRow][cellCol] = CELL_PASSED_ONCE;
			}
			else if(map->cellStatus[cellRow][cellCol] == CELL_PASSED_ONCE){
				map->cellStatus[cellRow][cellCol] = CELL_PASSED_TWICE;
			}
		}
        else if(map->cellStatus[cellRow][cellCol] < CELL_PASSED_ONCE){
            map->cellStatus[cellRow][cellCol] = CELL_NOT_ROAD_SURFACE;
        }
	}
}

bool roadSurface::findAvgZofNeighbourCell(int& cellRow,int& cellCol,float& avgZ)
{
	int cr=0,cc=0;
	for(int i=1;i<9;i++){
		findCellsAndNeighbours(cellRow,cellCol,cr,cc,i);
		if(map->cellStatus[cr][cc] == CELL_PASSED_TWICE){
			searchPoint.z = map->cellAvgZ[cr][cc];
			avgZ = searchPoint.z;
			break;
		}
	}

}

bool roadSurface::findCellsAndNeighbours(int& cellRow,int& cellCol,int& cr,int& cc,int& index)
{
	switch(index)
	{
		case 0:
			cr=cellRow;
			cc=cellCol;
		break;
		case 1:
			cr=cellRow;
			cc=cellCol-1;
		break;
		case 2:
			cr=cellRow+1;
			cc=cellCol-1;
		break;
		case 3:
			cr=cellRow+1;
			cc=cellCol;
		break;
		case 4:
			cr=cellRow+1;
			cc=cellCol+1;
		break;
		case 5:
			cr=cellRow;
			cc=cellCol+1;
		break;						
		case 6:
			cr=cellRow-1;
			cc=cellCol+1;
		break;
		case 7:
			cr=cellRow-1;
			cc=cellCol;
		break;
		case 8:
			cr=cellRow-1;
			cc=cellCol-1;
		break;
		default:
			cout << "error in findCellsAndNeighbours" << endl;
		break;									
	}
	if(cr < map->numRows && cc < map->numCols
		&& cr >= 0 && cc >= 0)
		return true;
	else
		return false;
}

void pp_callback(const pcl::visualization::PointPickingEvent& event, void* args)
{
	int dr,dc;
	roadSurface *surfaceRef=(roadSurface*)args;
	event.getPoint(surfaceRef->searchPoint.x,surfaceRef->searchPoint.y,surfaceRef->searchPoint.z);
	cout << "pt="<< surfaceRef->searchPoint.x << ", "<< surfaceRef->searchPoint.y << ", "<< surfaceRef->searchPoint.z << endl;
	surfaceRef->map->pointToCell(surfaceRef->searchPoint,dr,dc);
	cout << "cell="	<< dr <<", " << dc << endl;
}

void kb_callback(const pcl::visualization::KeyboardEvent& event, void* args)
{
	roadSurface *surfaceRef=(roadSurface*)args;
	if (event.getKeySym() == "r" && event.keyDown() && event.isCtrlPressed()){
		cout << "ctrl-r pressed and released" << endl;
		surfaceRef->detectRoadSurface();
	}
}

int main(int argc, char** argv){

	if(argc < 2){
		cout << "argument mising" << endl;
	}

	roadSurface surface(argv[1],SQUARE_SIDE_SIZE);
	surface.map->drawGridOnMap();
	viewer.registerPointPickingCallback(pp_callback,(void*)&surface);
	viewer.registerKeyboardCallback(kb_callback,(void*)&surface);
	viewer.spin();
}
