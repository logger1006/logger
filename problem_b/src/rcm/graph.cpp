#include "rcm.h"
#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include <stdlib.h>
#include <iomanip>
#include <set>
#include <omp.h>
using namespace std;

typedef vector<vector<vector<gGrid_C *>>> GRAPH3D;

bool constructGraph(design_C *pDesign)
{
	GRAPH3D *vGraph;
	vector<layer_C *> vLayer = pDesign->getLayer();

	//pDesign->getGraph( vGraph );
	vGraph = pDesign->getGraph();

	int nDX, nDY, nTX, nTY, nLayer;
	pDesign->getBoundary(nDX, nDY, nTX, nTY);
	nLayer = pDesign->getNumLayer();
	for (int l = 0; l < nLayer; l++)
	{
		vector<vector<gGrid_C *>> vGraph2D;
		for (int y = nDY; y <= nTY; y++)
		{
			vector<gGrid_C *> vLine;
			for (int x = nDX; x <= nTX; x++)
			{
				gGrid_C *pGrid = new gGrid_C;
				pGrid->m_nX = x;
				pGrid->m_nY = y;
				pGrid->m_nZ = vLayer[l]->getId();
				pGrid->setSupply(vLayer[l]->getSupply());
				//cout<<vLayer[l]->getSupply()<<endl;
				vLine.push_back(pGrid);
			}
			vGraph2D.push_back(vLine);
		}
		vGraph->push_back(vGraph2D);
	}
	return true;
}
/*
gGrid_C* getGrid(design_C *pDesign, int nX, int nY, int nZ)
{
	nX = nX - pDesign->m_vGraph[0][0][0]->m_nX;
	nY = nY - pDesign->m_vGraph[0][0][0]->m_nY;
	nZ = nZ - pDesign->m_vLayer[0]->getId();
	return pDesign->m_vGraph[nZ][nY][nX];
}
*/
bool addGGridSupply(design_C *pDesign, int nExtra, int nX, int nY, int nZ)
{
	gGrid_C *pGrid = getGrid(pDesign, nX, nY, nZ);
	int nSupply = pGrid->getSupply();
	nSupply = nSupply + nExtra;
	pGrid->setSupply(nSupply);
	return true;
}

bool delGGridSupply(design_C *pDesign, int nExtra, int nX, int nY, int nZ)
{
	gGrid_C *pGrid = getGrid(pDesign, nX, nY, nZ);
	int nSupply = pGrid->getSupply();
	nSupply = nSupply - nExtra;
	pGrid->setSupply(nSupply);
	return true;
}
/*
bool addCellDemand( gGrid_C* pGrid, instance_C* pInst )
{
	int nDemand = pGrid->getDemand();
	int nX, nY, nZ;
	pGrid->getPosition( nX, nY, nZ );	
	vector< blkg_C* > vBlkg = pInst->getBlkg();
	for( int i=0; i<vBlkg.size(); i++ )
	{
		blkg_C* pBlkg = vBlkg[i];
		nDemand = nDemand + vBlkg[i]->getDemand();	
	}
	pGrid->setDemand( nDemand );
	return true;
}

bool addCellDemand( design_C* pDesign, instance_C* pInst )
{
	//int nDemand = pGrid->getDemand();
	GRAPH3D vGraph;
	pDesign->getGraph( vGraph );
	//vector< layer_C* > vLayer = pDesign->getLayer();
	//int nLowestLayer = vLayer[0]->getId();
	int nX = pInst->getPlacedX();
	int nY = pInst->getPlacedY();
	vector< blkg_C* > vBlkg = pInst->getBlkg();
	if( pInst->getName() == "C1855")
		cout<<pInst->getName()<<endl;

	for( int i=0; i<vBlkg.size(); i++ )
	{
		blkg_C* pBlkg = vBlkg[i];
		int nLayerId = pBlkg->getLayerId();
		//cout<<nLayerId<<endl;
		gGrid_C* pGrid = getGrid( pDesign, nX, nY, nLayerId );
		int nDemand = pGrid->getDemand();
		//blkg_C* pAddGrid = graphTravel( )	
		pGrid->setDemand( nDemand + pBlkg->getDemand() );	
		if( pInst->getName() == "C1855" )
			cout<<pBlkg->getDemand()<<endl;
	}
	if( pInst->getName() == "C1855")
		cout<<endl;
	//pGrid->setDemand( nDemand );
	return true;
}
*/
bool calCellDemand(design_C *pDesign, gGrid_C *pGrid)
{
	//int nDemand = pGrid->getDemand();
	GRAPH3D vGraph;
	pDesign->getGraph(vGraph);
	//vector< layer_C* > vLayer = pDesign->getLayer();
	//int nLowestLayer = vLayer[0]->getId();
	//int nX = pInst->getPlacedX();
	//int nY = pInst->getPlacedY();
	vector<layer_C *> vLayer = pDesign->getLayer();
	int nZ = vLayer.front()->getId();
	//pGrid = getGrid( pDesign, nX, nY, nZ );

	int nTotalDemand = 0;
	vector<instance_C *> vInst = pGrid->getInstance();
	vector<int> vDemand;
	//cout<<vLayer.size()<<endl;
	for (int i = 0; i < vLayer.size(); i++)
	{
		vDemand.push_back(0);
	}

	for (int c = 0; c < vInst.size(); c++)
	{
		instance_C *pTmpInst = vInst[c];
		vector<blkg_C> vBlkg = pTmpInst->getBlkg();

		for (int i = 0; i < vBlkg.size(); i++)
		{
			blkg_C *pBlkg = &vBlkg[i];
			int nLayerId = pBlkg->getLayerId();
			//cout<<nLayerId<<endl;
			//gGrid_C* pTmpGrid = getGrid( pDesign, nX, nY, nLayerId );
			int nDemand = pBlkg->getDemand();
			//blkg_C* pAddGrid = graphTravel( )
			vDemand[nLayerId - nZ] = vDemand[nLayerId - nZ] + nDemand;
		}
	}

	for (int i = 0; i < vDemand.size(); i++)
	{
		pGrid->setDemand(vDemand[i]);
		pGrid = graphTravel(pDesign, pGrid, 0, 0, 1);
	}

	//pGrid->setDemand( nDemand );
	return true;
}

bool calCellDemand(design_C *pDesign, instance_C *pInst)
{
	//int nDemand = pGrid->getDemand();
	GRAPH3D vGraph;
	pDesign->getGraph(vGraph);
	//vector< layer_C* > vLayer = pDesign->getLayer();
	//int nLowestLayer = vLayer[0]->getId();
	int nX = pInst->getPlacedX();
	int nY = pInst->getPlacedY();
	vector<layer_C *> vLayer = pDesign->getLayer();
	int nZ = vLayer.front()->getId();
	gGrid_C *pGrid = getGrid(pDesign, nX, nY, nZ);

	int nTotalDemand = 0;
	vector<instance_C *> vInst = pGrid->getInstance();
	vector<int> vDemand;
	//cout<<vLayer.size()<<endl;
	for (int i = 0; i < vLayer.size(); i++)
	{
		vDemand.push_back(0);
	}

	for (int c = 0; c < vInst.size(); c++)
	{
		instance_C *pTmpInst = vInst[c];
		vector<blkg_C> vBlkg = pTmpInst->getBlkg();

		for (int i = 0; i < vBlkg.size(); i++)
		{
			blkg_C *pBlkg = &vBlkg[i];
			int nLayerId = pBlkg->getLayerId();
			//cout<<nLayerId<<endl;
			gGrid_C *pTmpGrid = getGrid(pDesign, nX, nY, nLayerId);
			int nDemand = pBlkg->getDemand();
			//blkg_C* pAddGrid = graphTravel( )
			vDemand[nLayerId - nZ] = vDemand[nLayerId - nZ] + nDemand;
		}
	}

	for (int i = 0; i < vDemand.size(); i++)
	{
		pGrid->setDemand(vDemand[i]);
		pGrid = graphTravel(pDesign, pGrid, 0, 0, 1);
		//nZ++;
		//pGrid = getGrid( pDesign, nX, nY, nZ );
	}

	//pGrid->setDemand( nDemand );
	return true;
}

/*
bool delCellDemand( gGrid_C* pGrid, instance_C* pInst )
{
	int nDemand = pGrid->getDemand();
	vector< blkg_C* > vBlkg = pInst->getBlkg();
	for( int i=0; i<vBlkg.size(); i++ )
	{
		nDemand = nDemand - vBlkg[i]->getDemand();	
	}
	pGrid->setDemand( nDemand );
	return true;
}

bool delCellDemand( design_C* pDesign, instance_C* pInst )
{
	//int nDemand = pGrid->getDemand();
	GRAPH3D vGraph;
	pDesign->getGraph( vGraph );
	vector< layer_C* > vLayer = pDesign->getLayer();
	int nLowestLayer = vLayer[0]->getId();
	int nX = pInst->getPlacedX();
	int nY = pInst->getPlacedY();
	vector< blkg_C* > vBlkg = pInst->getBlkg();
	for( int i=0; i<vBlkg.size(); i++ )
	{
		blkg_C* pBlkg = vBlkg[i];
		int nLayerId = pBlkg->getLayerId();
		gGrid_C* pGrid = getGrid( pDesign, nX, nY, nLayerId );
		int nDemand = pGrid->getDemand();
		//blkg_C* pAddGrid = graphTravel( )	
		pGrid->setDemand( nDemand - vBlkg[i]->getDemand() );	
	}
	//pGrid->setDemand( nDemand );
	return true;
}
*/
bool addCellOnGraph(design_C *pDesign, instance_C *pInst)
{
	GRAPH3D vGraph;
	pDesign->getGraph(vGraph);
	vector<layer_C *> vLayer = pDesign->getLayer();
	int nLowestLayer = vLayer[0]->getId();
	int nX = pInst->getPlacedX();
	int nY = pInst->getPlacedY();
	gGrid_C *pGrid = getGrid(pDesign, nX, nY, nLowestLayer);
	pGrid->addInstance(pInst);
	//addCellDemand( pDesign, pInst );
	calCellDemand(pDesign, pInst);
	return true;
}

bool addCellOnGraph(design_C *pDesign, vector<instance_C *> &vInst)
{
	for (int i = 0; i < vInst.size(); i++)
	{
		addCellOnGraph(pDesign, vInst[i]);
	}
	return true;
}

bool delCellOnGraph(design_C *pDesign, instance_C *pInst)
{
	GRAPH3D vGraph;
	pDesign->getGraph(vGraph);
	vector<layer_C *> vLayer = pDesign->getLayer();
	int nLowestLayer = vLayer[0]->getId();
	int nX = pInst->getPlacedX();
	int nY = pInst->getPlacedY();
	gGrid_C *pGrid = getGrid(pDesign, nX, nY, nLowestLayer);
	pGrid->delInstance(pInst);
	//delCellDemand( pDesign, pInst );
	calCellDemand(pDesign, pInst);
}

bool addExtraDemandOnGraph(design_C *pDesign, gGrid_C *pGrid)
{
	return true;
}

bool calculateCellDemand(design_C *pDesign, gGrid_C *pGrid)
{
	return true;
}

bool calculateCellDemand(design_C *pDesign)
{
	return true;
}

void dumpGraph(design_C *pDesign)
{
	cout << "Dumping graph information..." << endl;
	int nDX, nDY, nTX, nTY, nDL, nTL;
	pDesign->getBoundary(nDX, nDY, nTX, nTY);
	vector<layer_C *> vLayer = pDesign->getLayer();
	nDL = vLayer.front()->getId();
	nTL = vLayer.back()->getId();
	cout << nDX << " " << nDY << " " << nTX << " " << nTY << " " << nDL << " " << nTL << endl;

	cout << "Number of Cell" << endl;
	for (int x = nTX; x >= nDX; x--)
	{
		//for( int x=nDX; x<= nTX; x++ )
		for (int y = nDY; y <= nTY; y++)
		{
			gGrid_C *pGrid = getGrid(pDesign, x, y, nDL);
			cout << setw(2) << pGrid->getInstance().size() << " ";
		}
		cout << endl;
	}
	cout << endl;

	cout << "Remain" << endl;
	for (int l = nDL; l <= nTL; l++)
	{
		cout << "l = " << l << endl;
		//for( int y=nTY; y>=nDY; y-- )
		for (int x = nTX; x >= nDX; x--)
		{
			//for( int x=nDX; x<= nTX; x++ )
			for (int y = nDY; y <= nTY; y++)
			{
				gGrid_C *pGrid = getGrid(pDesign, x, y, l);
				cout << setw(2) << pGrid->getRemand() << " ";
			}
			cout << endl;
		}
		cout << endl;
	}
}

bool addNeighborCellDemand(design_C *pDesign, gGrid_C *pGrid, instance_C *pInst)
{

	unordered_map<string, int> *vInst_map; //map this grid cell
	string InstName = pInst->getType();
	unordered_map<string, unordered_map<string, extra_demand>> *v_map = pDesign->getExtra_map();
	unordered_map<string, unordered_map<string, extra_demand>> *v_map_2 = pDesign->getExtra_map_2();
	unordered_map<string, int> vlayer_map = pDesign->getLayer_map();
	int nX, nY, nZ, nZor; //nZor is pGird's nZ
	pGrid->getPosition(nX, nY, nZ);
	nZor = nZ;
	vInst_map = pGrid->getgGInstMap();
	auto iter = (*vInst_map).find(InstName);
	if (iter != (*vInst_map).end()) //update num of inst in this grid
		iter->second++;
	else
		(*vInst_map)[InstName] = 1;			//end of update
	pGrid->setgGInstMap((*vInst_map));		//set map to pGird
	auto iter2 = (*v_map).find(InstName);	//find pInst constraint(map[cell1][cell2])->map1
	auto iter5 = (*v_map_2).find(InstName); //find pInst constraint(map[cell2][cell1])->map2
	//constaint SameGrid
	if (iter2 != (*v_map).end()) //if find map1's index(find cell1)
	{
		string cell1_name = InstName;
		for (auto iter3 = (*vInst_map).begin(); iter3 != (*vInst_map).end(); iter3++)
		{
			string cell2_name = iter3->first;
			int N_pair = min((*vInst_map)[InstName], iter3->second);
			if (N_pair == (*vInst_map)[InstName]) // if N_pair = (*vInst_map)[InstName],that mean can generate new contraint pair;otherwise, it saturate
			{
				auto iter4 = iter2->second.find(cell2_name);
				if (iter4 != iter2->second.end()) //if find map1[cell1]'s index(find cell2)
				{
					for (int x = 0; x < (*v_map)[cell1_name][cell2_name].N; x++)
					{
						if ((*v_map)[cell1_name][cell2_name].type[x] == "sameGGrid")
						{
							nZ = vlayer_map[(*v_map)[cell1_name][cell2_name].layer[x]];
							gGrid_C *pTargetGrid = getGrid(pDesign, nX, nY, nZ);
							int nExtraDemand = pTargetGrid->getExtraDemand();
							nExtraDemand = nExtraDemand + (*v_map)[cell1_name][cell2_name].value[x];
							pTargetGrid->setExtraDemand(nExtraDemand);
							if (nX == 17 && nY == 26)
							{
								// if (cell1_name == "MC41" && cell2_name == "MC37")
								cout << endl
									 << cell1_name << " " << cell2_name << " " << x << ":1adadadadadada" << nExtraDemand - (*v_map)[cell1_name][cell2_name].value[x] << " :" << nExtraDemand;
							}
							// test t;
							// t.cell1 = cell1_name;
							// t.cell2 = cell2_name;
							// t.N = (*v_map)[cell1_name][cell2_name].value[x];
							// t.con = "same";
							// t.type = "add";
							// pTargetGrid->addcdebug(t);
						}
					}
				}
			}
		}
	}
	if (iter5 != (*v_map_2).end())
	{
		string cell1_name = iter5->first;
		for (auto iter3 = (*vInst_map).begin(); iter3 != (*vInst_map).end(); iter3++)
		{
			string cell2_name = iter3->first;
			if (cell1_name == cell2_name)
				continue;
			int N_pair = min((*vInst_map)[InstName], iter3->second);
			if (N_pair == (*vInst_map)[InstName])
			{
				auto iter4 = iter5->second.find(cell2_name);
				if (iter4 != iter5->second.end())
				{
					for (int x = 0; x < (*v_map_2)[cell1_name][cell2_name].N; x++)
					{
						if ((*v_map_2)[cell1_name][cell2_name].type[x] == "sameGGrid")
						{

							nZ = vlayer_map[(*v_map_2)[cell1_name][cell2_name].layer[x]];
							gGrid_C *pTargetGrid = getGrid(pDesign, nX, nY, nZ);
							int nExtraDemand = pTargetGrid->getExtraDemand();
							nExtraDemand = nExtraDemand + (*v_map_2)[cell1_name][cell2_name].value[x];
							pTargetGrid->setExtraDemand(nExtraDemand);

							// test t;
							// t.cell1 = cell1_name;
							// t.cell2 = cell2_name;
							// t.N = v_map_2[cell1_name][cell2_name].value[x];
							// t.con = "same";
							// t.type = "add";
							// pTargetGrid->addcdebug(t);
						}
					}
				}
			}
		}
	}
	// constraint "adjHGGrid" newInst
	bool bCheckT = false; // top
	bool bCheckD = false; // down

	int nX1, nY1, nX2, nY2;
	pDesign->getBoundary(nX1, nY1, nX2, nY2);
	int nTmpX, nTmpY, nTmpZ;
	pGrid->getPosition(nTmpX, nTmpY, nTmpZ);
	if (nTmpY > nY1)
		bCheckD = true;
	if (nTmpY < nY2)
		bCheckT = true;
	if (bCheckT) //top cell constraint : nX, nY+1, nzor
	{
		unordered_map<string, int> *vInst_map_H;
		gGrid_C *newGrid = getGrid(pDesign, nX, nY + 1, nZor);
		unordered_map<string, int>::iterator viter2;
		unordered_map<string, extra_demand>::iterator viter1;
		vInst_map_H = newGrid->getgGInstMap(); //map top grid's inst

		if (iter2 != (*v_map).end()) //if find map1's index(find cell1)
		{
			string cell1_name = InstName;
			for (viter2 = (*vInst_map_H).begin(); viter2 != (*vInst_map_H).end(); viter2++)
			{
				string cell2_name = viter2->first;
				int N_pair = min((*vInst_map)[InstName], viter2->second);
				if (N_pair == (*vInst_map)[InstName])
				{
					viter1 = (*v_map)[cell1_name].find(cell2_name);
					if (viter1 != (*v_map)[cell1_name].end()) //if find map1[cell1]'s index(find cell2)
					{
						for (int x = 0; x < (*v_map)[cell1_name][cell2_name].N; x++) //same cell1 ce112 diff layer
						{
							if ((*v_map)[cell1_name][cell2_name].type[x] == "adjHGGrid")
							{
								nZ = vlayer_map[(*v_map)[cell1_name][cell2_name].layer[x]]; //get layer_ID
								gGrid_C *pTargetGrid = getGrid(pDesign, nX, nY, nZ);		// find target grid
								int nExtraDemand = pTargetGrid->getExtraDemand();
								nExtraDemand = nExtraDemand + (*v_map)[cell1_name][cell2_name].value[x]; // add this rule demand
								pTargetGrid->setExtraDemand(nExtraDemand);
								// test t;
								// t.cell1 = cell1_name;
								// t.cell2 = cell2_name;
								// t.N = (*v_map)[cell1_name][cell2_name].value[x];
								// t.con = "adj";
								// t.type = "add";
								// pTargetGrid->addcdebug(t);
								//begin update nY+1 grid
								pTargetGrid = getGrid(pDesign, nX, nY + 1, nZ); // find target grid
								nExtraDemand = pTargetGrid->getExtraDemand();
								nExtraDemand = nExtraDemand + (*v_map)[cell1_name][cell2_name].value[x]; // add this rule demand
								pTargetGrid->setExtraDemand(nExtraDemand);
								// test t;
								// t.cell1 = cell1_name;
								// t.cell2 = cell2_name;
								// t.N = (*v_map)[cell1_name][cell2_name].value[x];
								// t.con = "adj";
								// t.type = "add";
								// pTargetGrid->addcdebug(t);
							}
						}
					}
				}
			}
		}
		if (iter5 != (*v_map_2).end()) //Opposite to above (map2[cell2][cell1]
		{
			string cell1_name = iter5->first;
			for (viter2 = (*vInst_map_H).begin(); viter2 != (*vInst_map_H).end(); viter2++)
			{
				string cell2_name = viter2->first;
				if (cell1_name == cell2_name)
					continue;
				int N_pair = min((*vInst_map)[InstName], viter2->second);
				if (N_pair == (*vInst_map)[InstName])
				{
					viter1 = (*v_map_2)[cell1_name].find(cell2_name);
					if (viter1 != (*v_map_2)[cell1_name].end()) // find extra constraint
					{
						for (int x = 0; x < (*v_map_2)[cell1_name][cell2_name].N; x++) //same cell1 ce112 diff layer
						{
							if ((*v_map_2)[cell1_name][cell2_name].type[x] == "adjHGGrid")
							{
								nZ = vlayer_map[(*v_map_2)[cell1_name][cell2_name].layer[x]]; //get layer_ID
								gGrid_C *pTargetGrid = getGrid(pDesign, nX, nY, nZ);		  // find target grid
								int nExtraDemand = pTargetGrid->getExtraDemand();
								nExtraDemand = nExtraDemand + (*v_map_2)[cell1_name][cell2_name].value[x]; // add this rule demand
								pTargetGrid->setExtraDemand(nExtraDemand);
								// test t;
								// t.cell1 = cell1_name;
								// t.cell2 = cell2_name;
								// t.N = v_map_2[cell1_name][cell2_name].value[x];
								// t.con = "adj";
								// t.type = "add";
								// pTargetGrid->addcdebug(t);
								// begin update nY+1 grid
								pTargetGrid = getGrid(pDesign, nX, nY + 1, nZ); // find target grid
								nExtraDemand = pTargetGrid->getExtraDemand();
								nExtraDemand = nExtraDemand + (*v_map_2)[cell1_name][cell2_name].value[x]; // add this rule demand
								pTargetGrid->setExtraDemand(nExtraDemand);
								// test t;
								// t.cell1 = cell1_name;
								// t.cell2 = cell2_name;
								// t.N = v_map_2[cell1_name][cell2_name].value[x];
								// t.con = "adj";
								// t.type = "add";
								// pTargetGrid->addcdebug(t);
							}
						}
					}
				}
			}
		}
	}
	if (bCheckD) //bottom cell constraint : nX, nY-1, nzor
	{
		unordered_map<string, int> *vInst_map_H;
		gGrid_C *newGrid = getGrid(pDesign, nX, nY - 1, nZor);
		unordered_map<string, int>::iterator viter2;
		unordered_map<string, extra_demand>::iterator viter1;
		vInst_map_H = newGrid->getgGInstMap(); //map top grid's inst

		if (iter2 != (*v_map).end()) //if find map1's index(find cell1)
		{
			string cell1_name = InstName;
			for (viter2 = (*vInst_map_H).begin(); viter2 != (*vInst_map_H).end(); viter2++)
			{
				string cell2_name = viter2->first;
				int N_pair = min((*vInst_map)[InstName], viter2->second);
				if (N_pair == (*vInst_map)[InstName])
				{
					viter1 = (*v_map)[cell1_name].find(cell2_name);
					if (viter1 != (*v_map)[cell1_name].end()) //if find map1[cell1]'s index(find cell2)
					{
						for (int x = 0; x < (*v_map)[cell1_name][cell2_name].N; x++) //same cell1 ce112 diff layer
						{
							if ((*v_map)[cell1_name][cell2_name].type[x] == "adjHGGrid")
							{
								nZ = vlayer_map[(*v_map)[cell1_name][cell2_name].layer[x]]; //get layer_ID
								gGrid_C *pTargetGrid = getGrid(pDesign, nX, nY, nZ);		// find target grid
								int nExtraDemand = pTargetGrid->getExtraDemand();
								nExtraDemand = nExtraDemand + (*v_map)[cell1_name][cell2_name].value[x]; // add this rule demand
								pTargetGrid->setExtraDemand(nExtraDemand);
								// test t;
								// t.cell1 = cell1_name;
								// t.cell2 = cell2_name;
								// t.N = (*v_map)[cell1_name][cell2_name].value[x];
								// t.con = "adj";
								// t.type = "add";
								// pTargetGrid->addcdebug(t);
								//begin update nY-1 grid
								pTargetGrid = getGrid(pDesign, nX, nY - 1, nZ); // find target grid
								nExtraDemand = pTargetGrid->getExtraDemand();
								nExtraDemand = nExtraDemand + (*v_map)[cell1_name][cell2_name].value[x]; // add this rule demand
								pTargetGrid->setExtraDemand(nExtraDemand);
								// test t;
								// t.cell1 = cell1_name;
								// t.cell2 = cell2_name;
								// t.N = (*v_map)[cell1_name][cell2_name].value[x];
								// t.con = "adj";
								// t.type = "add";
								// pTargetGrid->addcdebug(t);
							}
						}
					}
				}
				else
				{
					continue;
				}
			}
		}

		if (iter5 != (*v_map_2).end())
		{
			string cell1_name = iter5->first;
			for (viter2 = (*vInst_map_H).begin(); viter2 != (*vInst_map_H).end(); viter2++)
			{
				string cell2_name = viter2->first;
				if (cell1_name == cell2_name)
					continue;
				int N_pair = min((*vInst_map)[InstName], viter2->second);
				if (N_pair == (*vInst_map)[InstName])
				{
					viter1 = (*v_map_2)[cell1_name].find(cell2_name);
					if (viter1 != (*v_map_2)[cell1_name].end()) // find extra constraint
					{
						for (int x = 0; x < (*v_map_2)[cell1_name][cell2_name].N; x++) //same cell1 ce112 diff layer
						{
							if ((*v_map_2)[cell1_name][cell2_name].type[x] == "adjHGGrid")
							{
								nZ = vlayer_map[(*v_map_2)[cell1_name][cell2_name].layer[x]]; //get layer_ID
								gGrid_C *pTargetGrid = getGrid(pDesign, nX, nY, nZ);		  // find target grid
								int nExtraDemand = pTargetGrid->getExtraDemand();
								nExtraDemand = nExtraDemand + (*v_map_2)[cell1_name][cell2_name].value[x]; // add this rule demand
								pTargetGrid->setExtraDemand(nExtraDemand);
								// test t;
								// t.cell1 = cell1_name;
								// t.cell2 = cell2_name;
								// t.N = v_map_2[cell1_name][cell2_name].value[x];
								// t.con = "adj";
								// t.type = "add";
								// pTargetGrid->addcdebug(t);
								//begin update nY-1 grid
								pTargetGrid = getGrid(pDesign, nX, nY - 1, nZ); // find target grid
								nExtraDemand = pTargetGrid->getExtraDemand();
								nExtraDemand = nExtraDemand + (*v_map_2)[cell1_name][cell2_name].value[x]; // add this rule demand
								pTargetGrid->setExtraDemand(nExtraDemand);
								// test t;
								// t.cell1 = cell1_name;
								// t.cell2 = cell2_name;
								// t.N = v_map_2[cell1_name][cell2_name].value[x];
								// t.con = "adj";
								// t.type = "add";
								// pTargetGrid->addcdebug(t);
							}
						}
					}
				}
			}
		}
	}
}

bool removeNeighborCellDemand(design_C *pDesign, gGrid_C *pGrid, instance_C *pInst)
{

	unordered_map<string, int> *vInst_map; //map this grid cell
	string InstName = pInst->getType();
	unordered_map<string, unordered_map<string, extra_demand>> *v_map = pDesign->getExtra_map();
	unordered_map<string, unordered_map<string, extra_demand>> *v_map_2 = pDesign->getExtra_map_2();
	unordered_map<string, int> vlayer_map = pDesign->getLayer_map();
	int nX, nY, nZ, nZor; //nZor is pGird's nZ
	pGrid->getPosition(nX, nY, nZ);
	nZor = nZ;
	vInst_map = pGrid->getgGInstMap();
	auto iter = (*vInst_map).find(InstName);
	if (iter != (*vInst_map).end()) //update num of inst in this grid  //&&iter->second!=0
	{
		iter->second--;
		if (iter->second < 0)
			cout << "eriogfherilugehrugerghneruigfheruigreuigr" << endl;
		// 	(*vInst_map).erase(iter);
	}
	else
	{
		cout << endl
			 << "Remove Fail : can't find inst " << pInst->getName() << " in grid :(" << nX << "," << nY << "," << nZ << ")" << endl;
		getchar();
		auto test = pGrid->getInstance();
		if (test.size() == 0)
			cout << "zerooooooooooooo";
		else
		{
			for (int x = 0; x < test.size(); x++)
			{
				cout << " " << test[x]->getName();
			}
			for (auto x = (*vInst_map).begin(); x != (*vInst_map).end(); x++)
			{
				cout << " " << x->first;
			}
		}
		return false;
	}										//end of update
	pGrid->setgGInstMap((*vInst_map));		//set map to pGird
	auto iter2 = (*v_map).find(InstName);	//find pInst constraint(map[cell1][cell2])->map1
	auto iter5 = (*v_map_2).find(InstName); //find pInst constraint(map[cell2][cell1])->map2
	//constaint SameGrid
	if (iter2 != (*v_map).end()) //if find map1's index(find cell1)
	{
		string cell1_name = InstName;
		for (auto iter3 = (*vInst_map).begin(); iter3 != (*vInst_map).end(); iter3++)
		{
			string cell2_name = iter3->first;
			int N_pair = max((*vInst_map)[InstName], iter3->second);
			if (N_pair != (*vInst_map)[InstName]) // if N_pair = (*vInst_map)[InstName],that mean can generate new contraint pair;otherwise, it saturate
			{
				auto iter4 = iter2->second.find(cell2_name);
				if (iter4 != iter2->second.end()) //if find map1[cell1]'s index(find cell2)
				{
					for (int x = 0; x < (*v_map)[cell1_name][cell2_name].N; x++)
					{
						if ((*v_map)[cell1_name][cell2_name].type[x] == "sameGGrid")
						{
							nZ = vlayer_map[(*v_map)[cell1_name][cell2_name].layer[x]];
							gGrid_C *pTargetGrid = getGrid(pDesign, nX, nY, nZ);
							int nExtraDemand = pTargetGrid->getExtraDemand();
							nExtraDemand = nExtraDemand - (*v_map)[cell1_name][cell2_name].value[x];
							pTargetGrid->setExtraDemand(nExtraDemand);
							// test t;
							// t.cell1 = cell1_name;
							// t.cell2 = cell2_name;
							// t.N = (*v_map)[cell1_name][cell2_name].value[x];
							// t.con = "same";
							// t.type = "remove";
							// pTargetGrid->addcdebug(t);
						}
					}
				}
			}
		}
	}

	if (iter5 != (*v_map_2).end())
	{
		string cell1_name = iter5->first;
		for (auto iter3 = (*vInst_map).begin(); iter3 != (*vInst_map).end(); iter3++)
		{
			string cell2_name = iter3->first;
			if (cell1_name == cell2_name)
				continue;
			int N_pair = max((*vInst_map)[InstName], iter3->second);
			if (N_pair != (*vInst_map)[InstName])
			{
				auto iter4 = iter5->second.find(cell2_name);
				if (iter4 != iter5->second.end())
				{
					for (int x = 0; x < (*v_map_2)[cell1_name][cell2_name].N; x++)
					{
						if ((*v_map_2)[cell1_name][cell2_name].type[x] == "sameGGrid")
						{
							nZ = vlayer_map[(*v_map_2)[cell1_name][cell2_name].layer[x]];
							gGrid_C *pTargetGrid = getGrid(pDesign, nX, nY, nZ);
							int nExtraDemand = pTargetGrid->getExtraDemand();
							nExtraDemand = nExtraDemand - (*v_map_2)[cell1_name][cell2_name].value[x];
							pTargetGrid->setExtraDemand(nExtraDemand);
							// test t;
							// t.cell1 = cell1_name;
							// t.cell2 = cell2_name;
							// t.N = v_map_2[cell1_name][cell2_name].value[x];
							// t.con = "same";
							// t.type = "remove";
							// pTargetGrid->addcdebug(t);
						}
					}
				}
			}
		}
	}
	// constraint "adjHGGrid" newInst
	bool bCheckT = false; // top
	bool bCheckD = false; // down

	int nX1, nY1, nX2, nY2;
	pDesign->getBoundary(nX1, nY1, nX2, nY2);
	int nTmpX, nTmpY, nTmpZ;
	pGrid->getPosition(nTmpX, nTmpY, nTmpZ);
	if (nTmpY > nY1)
		bCheckD = true;
	if (nTmpY < nY2)
		bCheckT = true;
	if (bCheckT) //top cell constraint : nX, nY+1, nzor
	{
		unordered_map<string, int> *vInst_map_H;
		gGrid_C *newGrid = getGrid(pDesign, nX, nY + 1, nZor);
		unordered_map<string, int>::iterator viter2;
		unordered_map<string, extra_demand>::iterator viter1;
		vInst_map_H = newGrid->getgGInstMap(); //map top grid's inst

		if (iter2 != (*v_map).end()) //if find map1's index(find cell1)
		{
			string cell1_name = InstName;
			for (viter2 = (*vInst_map_H).begin(); viter2 != (*vInst_map_H).end(); viter2++)
			{
				string cell2_name = viter2->first;
				int N_pair = max((*vInst_map)[InstName], viter2->second);
				if (N_pair != (*vInst_map)[InstName])
				{
					viter1 = (*v_map)[cell1_name].find(cell2_name);
					if (viter1 != (*v_map)[cell1_name].end()) //if find map1[cell1]'s index(find cell2)
					{
						for (int x = 0; x < (*v_map)[cell1_name][cell2_name].N; x++) //same cell1 ce112 diff layer
						{
							if ((*v_map)[cell1_name][cell2_name].type[x] == "adjHGGrid")
							{
								nZ = vlayer_map[(*v_map)[cell1_name][cell2_name].layer[x]]; //get layer_ID
								gGrid_C *pTargetGrid = getGrid(pDesign, nX, nY, nZ);		// find target grid
								int nExtraDemand = pTargetGrid->getExtraDemand();
								nExtraDemand = nExtraDemand - (*v_map)[cell1_name][cell2_name].value[x]; // add this rule demand
								pTargetGrid->setExtraDemand(nExtraDemand);
								// test t;
								// t.cell1 = cell1_name;
								// t.cell2 = cell2_name;
								// t.N = (*v_map)[cell1_name][cell2_name].value[x];
								// t.con = "adj";
								// t.type = "remove";
								// pTargetGrid->addcdebug(t);
								//begin update nY+1 grid
								pTargetGrid = getGrid(pDesign, nX, nY + 1, nZ); // find target grid
								nExtraDemand = pTargetGrid->getExtraDemand();
								nExtraDemand = nExtraDemand - (*v_map)[cell1_name][cell2_name].value[x]; // add this rule demand
								pTargetGrid->setExtraDemand(nExtraDemand);
								// test t;
								// t.cell1 = cell1_name;
								// t.cell2 = cell2_name;
								// t.N = (*v_map)[cell1_name][cell2_name].value[x];
								// t.con = "adj";
								// t.type = "remove";
								// pTargetGrid->addcdebug(t);
							}
						}
					}
				}
				else
				{
					continue;
				}
			}
		}

		if (iter5 != (*v_map_2).end()) //Opposite to above (map2[cell2][cell1]
		{
			string cell1_name = iter5->first;
			for (viter2 = (*vInst_map_H).begin(); viter2 != (*vInst_map_H).end(); viter2++)
			{
				string cell2_name = viter2->first;
				if (cell1_name == cell2_name)
					continue;
				int N_pair = max((*vInst_map)[InstName], viter2->second);
				if (N_pair != (*vInst_map)[InstName])
				{
					viter1 = (*v_map_2)[cell1_name].find(cell2_name);
					if (viter1 != (*v_map_2)[cell1_name].end()) // find extra constraint
					{
						for (int x = 0; x < (*v_map_2)[cell1_name][cell2_name].N; x++) //same cell1 ce112 diff layer
						{
							if ((*v_map_2)[cell1_name][cell2_name].type[x] == "adjHGGrid")
							{
								nZ = vlayer_map[(*v_map_2)[cell1_name][cell2_name].layer[x]]; //get layer_ID
								gGrid_C *pTargetGrid = getGrid(pDesign, nX, nY, nZ);		  // find target grid
								int nExtraDemand = pTargetGrid->getExtraDemand();
								nExtraDemand = nExtraDemand - (*v_map_2)[cell1_name][cell2_name].value[x]; // add this rule demand
								pTargetGrid->setExtraDemand(nExtraDemand);
								// test t;
								// t.cell1 = cell1_name;
								// t.cell2 = cell2_name;
								// t.N = v_map_2[cell1_name][cell2_name].value[x];
								// t.con = "adj";
								// t.type = "remove";
								// pTargetGrid->addcdebug(t);
								// begin update nY+1 grid
								pTargetGrid = getGrid(pDesign, nX, nY + 1, nZ); // find target grid
								nExtraDemand = pTargetGrid->getExtraDemand();
								nExtraDemand = nExtraDemand - (*v_map_2)[cell1_name][cell2_name].value[x]; // add this rule demand
								pTargetGrid->setExtraDemand(nExtraDemand);
								// test t;
								// t.cell1 = cell1_name;
								// t.cell2 = cell2_name;
								// t.N = v_map_2[cell1_name][cell2_name].value[x];
								// t.con = "adj";
								// t.type = "remove";
								// pTargetGrid->addcdebug(t);
							}
						}
					}
				}
			}
		}
	}
	if (bCheckD) //bottom cell constraint : nX, nY-1, nzor
	{
		unordered_map<string, int> *vInst_map_H;
		gGrid_C *newGrid = getGrid(pDesign, nX, nY - 1, nZor);
		unordered_map<string, int>::iterator viter2;
		unordered_map<string, extra_demand>::iterator viter1;
		vInst_map_H = newGrid->getgGInstMap(); //map top grid's inst

		if (iter2 != (*v_map).end()) //if find map1's index(find cell1)
		{
			string cell1_name = InstName;
			for (viter2 = (*vInst_map_H).begin(); viter2 != (*vInst_map_H).end(); viter2++)
			{
				string cell2_name = viter2->first;
				int N_pair = max((*vInst_map)[InstName], viter2->second);
				if (N_pair != (*vInst_map)[InstName])
				{
					viter1 = (*v_map)[cell1_name].find(cell2_name);
					if (viter1 != (*v_map)[cell1_name].end()) //if find map1[cell1]'s index(find cell2)
					{
						for (int x = 0; x < (*v_map)[cell1_name][cell2_name].N; x++) //same cell1 ce112 diff layer
						{
							if ((*v_map)[cell1_name][cell2_name].type[x] == "adjHGGrid")
							{
								nZ = vlayer_map[(*v_map)[cell1_name][cell2_name].layer[x]]; //get layer_ID
								gGrid_C *pTargetGrid = getGrid(pDesign, nX, nY, nZ);		// find target grid
								int nExtraDemand = pTargetGrid->getExtraDemand();
								nExtraDemand = nExtraDemand - (*v_map)[cell1_name][cell2_name].value[x]; // add this rule demand
								pTargetGrid->setExtraDemand(nExtraDemand);
								// test t;
								// t.cell1 = cell1_name;
								// t.cell2 = cell2_name;
								// t.N = (*v_map)[cell1_name][cell2_name].value[x];
								// t.con = "adj";
								// t.type = "remove";
								// pTargetGrid->addcdebug(t);
								//begin update nY-1 grid
								pTargetGrid = getGrid(pDesign, nX, nY - 1, nZ); // find target grid
								nExtraDemand = pTargetGrid->getExtraDemand();
								nExtraDemand = nExtraDemand - (*v_map)[cell1_name][cell2_name].value[x]; // add this rule demand
								pTargetGrid->setExtraDemand(nExtraDemand);
								// test t;
								// t.cell1 = cell1_name;
								// t.cell2 = cell2_name;
								// t.N = (*v_map)[cell1_name][cell2_name].value[x];
								// t.con = "adj";
								// t.type = "remove";
								// pTargetGrid->addcdebug(t);
							}
						}
					}
				}
				else
				{
					continue;
				}
			}
		}

		if (iter5 != (*v_map_2).end())
		{
			string cell1_name = iter5->first;
			for (viter2 = (*vInst_map_H).begin(); viter2 != (*vInst_map_H).end(); viter2++)
			{
				string cell2_name = viter2->first;
				if (cell1_name == cell2_name)
					continue;
				int N_pair = max((*vInst_map)[InstName], viter2->second);
				if (N_pair != (*vInst_map)[InstName])
				{
					viter1 = (*v_map_2)[cell1_name].find(cell2_name);
					if (viter1 != (*v_map_2)[cell1_name].end()) // find extra constraint
					{
						for (int x = 0; x < (*v_map_2)[cell1_name][cell2_name].N; x++) //same cell1 ce112 diff layer
						{
							if ((*v_map_2)[cell1_name][cell2_name].type[x] == "adjHGGrid")
							{
								nZ = vlayer_map[(*v_map_2)[cell1_name][cell2_name].layer[x]]; //get layer_ID
								gGrid_C *pTargetGrid = getGrid(pDesign, nX, nY, nZ);		  // find target grid
								int nExtraDemand = pTargetGrid->getExtraDemand();
								nExtraDemand = nExtraDemand - (*v_map_2)[cell1_name][cell2_name].value[x]; // add this rule demand
								pTargetGrid->setExtraDemand(nExtraDemand);
								// test t;
								// t.cell1 = cell1_name;
								// t.cell2 = cell2_name;
								// t.N = v_map_2[cell1_name][cell2_name].value[x];
								// t.con = "adj";
								// t.type = "remove";
								// pTargetGrid->addcdebug(t);
								//begin update nY-1 grid
								pTargetGrid = getGrid(pDesign, nX, nY - 1, nZ); // find target grid
								nExtraDemand = pTargetGrid->getExtraDemand();
								nExtraDemand = nExtraDemand - (*v_map_2)[cell1_name][cell2_name].value[x]; // add this rule demand
								pTargetGrid->setExtraDemand(nExtraDemand);
								// test t;
								// t.cell1 = cell1_name;
								// t.cell2 = cell2_name;
								// t.N = v_map_2[cell1_name][cell2_name].value[x];
								// t.con = "adj";
								// t.type = "remove";
								// pTargetGrid->addcdebug(t);
							}
						}
					}
				}
			}
		}
	}

	if (iter->second == 0)
	{
		(*vInst_map).erase(iter);
		pGrid->setgGInstMap((*vInst_map));
	}
}

bool calNeighborCellDemand(design_C *pDesign, gGrid_C *pGrid)
{
	vector<extraDemandCell_C *> vExtra = pDesign->getExtraDemandCell();
	//vector< extraDemandCell_C* > vExtra;
	vector<instance_C *> vInst = pGrid->getInstance();
	int nX, nY, nZ, nZor;
	pGrid->getPosition(nX, nY, nZ);
	nZor = nZ;
	vector<layer_C *> vLayer = pDesign->getLayer();
	int nZ1, nZ2;
	nZ1 = vLayer.front()->getId();
	nZ2 = vLayer.back()->getId();
	// initial extra demand
	for (int z = nZ1; z <= nZ2; z++)
	{
		gGrid_C *pTmpGrid = getGrid(pDesign, nX, nY, z);
		pTmpGrid->setExtraDemand(0);
	}

	// chouchuu begin
	unordered_map<string, unordered_map<string, extra_demand>> *v_map = pDesign->getExtra_map();
	unordered_map<string, int>::iterator iter, iter2;
	unordered_map<string, int> vInst_map; //map this grid cell
	unordered_map<string, int> vlayer_map = pDesign->getLayer_map();
	unordered_map<string, extra_demand>::iterator iter3;
	for (int x = 0; x < vInst.size(); x++) //map this grid cell
	{
		string cell_name = vInst[x]->getType();
		iter = vInst_map.find(cell_name);
		if (iter != vInst_map.end())
		{
			vInst_map[cell_name]++;
		}
		else
		{
			vInst_map[cell_name] = 1;
		}
	}
	pGrid->setgGInstMap(vInst_map);
	// constraint "sameGGrid"
	for (iter = vInst_map.begin(); iter != vInst_map.end(); iter++)
	{
		string cell1_name = iter->first;

		for (iter2 = vInst_map.begin(); iter2 != vInst_map.end(); iter2++)
		{
			string cell2_name = iter2->first;
			iter3 = (*v_map)[cell1_name].find(cell2_name);
			if (iter3 != (*v_map)[cell1_name].end()) // find extra constraint
			{
				int N_pair = min(iter->second, iter2->second);
				for (int x = 0; x < (*v_map)[cell1_name][cell2_name].N; x++) //same cell1 ce112 diff layer
				{
					if ((*v_map)[cell1_name][cell2_name].type[x] == "sameGGrid")
					{
						nZ = vlayer_map[(*v_map)[cell1_name][cell2_name].layer[x]];		  //get layer_ID
						int nDemand = N_pair * (*v_map)[cell1_name][cell2_name].value[x]; // cal this rule demand
						gGrid_C *pTargetGrid = getGrid(pDesign, nX, nY, nZ);			  // find target grid
						int nExtraDemand = pTargetGrid->getExtraDemand();
						nExtraDemand = nExtraDemand + nDemand; // add this rule demand
						pTargetGrid->setExtraDemand(nExtraDemand);
						// if (nX == 10 && nY == 68 && nZ == 1)
						// {
						// 	cout << "same" << endl;
						// }
					}
				}
			}
			else // don't find constraint
			{
				continue;
			}
		}
	}

	// constraint "adjHGGrid" newInst
	bool bCheckT = false; // top
	bool bCheckD = false; // down

	int nX1, nY1, nX2, nY2;
	pDesign->getBoundary(nX1, nY1, nX2, nY2);
	int nTmpX, nTmpY, nTmpZ;
	pGrid->getPosition(nTmpX, nTmpY, nTmpZ);
	if (nTmpY > nY1)
		bCheckD = true;
	if (nTmpY < nY2)
		bCheckT = true;

	if (bCheckT) //top cell
	{
		unordered_map<string, int> vInst_map_H; //map top grid cell
		gGrid_C *newGrid = getGrid(pDesign, nX, nY + 1, nZor);
		vector<instance_C *> newInst = newGrid->getInstance();
		for (int x = 0; x < newInst.size(); x++) //map top grid cell
		{
			string cell_name = newInst[x]->getType();
			iter = vInst_map_H.find(cell_name);
			if (iter != vInst_map_H.end())
			{
				vInst_map_H[cell_name]++;
			}
			else
			{
				vInst_map_H[cell_name] = 1;
			}
		}
		for (iter = vInst_map.begin(); iter != vInst_map.end(); iter++) //cal this grid to top grid
		{
			string cell1_name = iter->first;
			for (iter2 = vInst_map_H.begin(); iter2 != vInst_map_H.end(); iter2++)
			{
				string cell2_name = iter2->first;
				iter3 = (*v_map)[cell1_name].find(cell2_name);
				if (iter3 != (*v_map)[cell1_name].end()) // find extra constraint
				{
					int N_pair = min(iter->second, iter2->second);
					for (int x = 0; x < (*v_map)[cell1_name][cell2_name].N; x++) //same cell1 ce112 diff layer
					{
						if ((*v_map)[cell1_name][cell2_name].type[x] == "adjHGGrid")
						{
							nZ = vlayer_map[(*v_map)[cell1_name][cell2_name].layer[x]];		  //get layer_ID
							int nDemand = N_pair * (*v_map)[cell1_name][cell2_name].value[x]; // cal this rule demand
							gGrid_C *pTargetGrid = getGrid(pDesign, nX, nY, nZ);			  // find target grid
							int nExtraDemand = pTargetGrid->getExtraDemand();
							nExtraDemand = nExtraDemand + nDemand; // add this rule demand
							pTargetGrid->setExtraDemand(nExtraDemand);
							// if (nX == 10 && nY == 68 && nZ == 1)
							// {
							// 	cout << "adj : " << cell1_name << " " << cell2_name << " " << x << " " << (*v_map)[cell1_name][cell2_name].layer[x] << endl;
							// }
						}
					}
				}
				else // don't find constraint
				{
					continue;
				}
			}
		}
		for (iter = vInst_map_H.begin(); iter != vInst_map_H.end(); iter++) //cal top grid to this grid
		{
			string cell1_name = iter->first;
			for (iter2 = vInst_map.begin(); iter2 != vInst_map.end(); iter2++)
			{
				string cell2_name = iter2->first;
				if (cell2_name == cell1_name)
					continue;
				iter3 = (*v_map)[cell1_name].find(cell2_name);
				if (iter3 != (*v_map)[cell1_name].end()) // find extra constraint
				{
					int N_pair = min(iter->second, iter2->second);
					for (int x = 0; x < (*v_map)[cell1_name][cell2_name].N; x++) //same cell1 ce112 diff layer
					{
						if ((*v_map)[cell1_name][cell2_name].type[x] == "adjHGGrid")
						{
							nZ = vlayer_map[(*v_map)[cell1_name][cell2_name].layer[x]];		  //get layer_ID
							int nDemand = N_pair * (*v_map)[cell1_name][cell2_name].value[x]; // cal this rule demand
							gGrid_C *pTargetGrid = getGrid(pDesign, nX, nY, nZ);			  // find target grid
							int nExtraDemand = pTargetGrid->getExtraDemand();
							nExtraDemand = nExtraDemand + nDemand; // add this rule demand
							pTargetGrid->setExtraDemand(nExtraDemand);
							// if (nX == 10 && nY == 68 && nZ == 1)
							// {
							// 	cout << "adj : " << cell1_name << " " << cell2_name << " " << x << " " << (*v_map)[cell1_name][cell2_name].layer[x] << endl;
							// }
						}
					}
				}
				else // don't find constraint
				{
					continue;
				}
			}
		}
	}
	if (bCheckD) //down cell
	{
		unordered_map<string, int> vInst_map_H; //map top grid cell
		gGrid_C *pNGrid = getGrid(pDesign, nX, nY - 1, nZor);
		vector<instance_C *> vNInst = pNGrid->getInstance();

		for (int x = 0; x < vNInst.size(); x++) //map top grid cell
		{
			string cell_name = vNInst[x]->getType();
			iter = vInst_map_H.find(cell_name);
			if (iter != vInst_map_H.end())
			{
				vInst_map_H[cell_name]++;
			}
			else
			{
				vInst_map_H[cell_name] = 1;
			}
		}
		for (iter = vInst_map.begin(); iter != vInst_map.end(); iter++) //cal this grid to down grid
		{
			string cell1_name = iter->first;
			// map<string,extra_demand> ::iterator iter3;
			for (iter2 = vInst_map_H.begin(); iter2 != vInst_map_H.end(); iter2++)
			{
				string cell2_name = iter2->first;
				iter3 = (*v_map)[cell1_name].find(cell2_name);
				if (iter3 != (*v_map)[cell1_name].end()) // find extra constraint
				{
					int N_pair = min(iter->second, iter2->second);
					for (int x = 0; x < (*v_map)[cell1_name][cell2_name].N; x++) //same cell1 ce112 diff layer
					{
						if ((*v_map)[cell1_name][cell2_name].type[x] == "adjHGGrid")
						{
							nZ = vlayer_map[(*v_map)[cell1_name][cell2_name].layer[x]];		  //get layer_ID
							int nDemand = N_pair * (*v_map)[cell1_name][cell2_name].value[x]; // cal this rule demand
							gGrid_C *pTargetGrid = getGrid(pDesign, nX, nY, nZ);			  // find target grid
							int nExtraDemand = pTargetGrid->getExtraDemand();
							nExtraDemand = nExtraDemand + nDemand; // add this rule demand
							pTargetGrid->setExtraDemand(nExtraDemand);
							// if (nX == 10 && nY == 68 && nZ == 1)
							// {
							// 	cout << "adj : " << cell1_name << " " << cell2_name << " " << x << " " << (*v_map)[cell1_name][cell2_name].layer[x] << endl;
							// }
						}
					}
				}
				else // don't find constraint
				{
					continue;
				}
			}
		}
		for (iter = vInst_map_H.begin(); iter != vInst_map_H.end(); iter++) //cal down grid to this grid
		{
			string cell1_name = iter->first;
			// map<string,extra_demand> ::iterator iter3;
			for (iter2 = vInst_map.begin(); iter2 != vInst_map.end(); iter2++)
			{
				string cell2_name = iter2->first;
				if (cell2_name == cell1_name)
					continue;
				iter3 = (*v_map)[cell1_name].find(cell2_name);
				if (iter3 != (*v_map)[cell1_name].end()) // find extra constraint
				{
					int N_pair = min(iter->second, iter2->second);
					for (int x = 0; x < (*v_map)[cell1_name][cell2_name].N; x++) //same cell1 ce112 diff layer
					{
						if ((*v_map)[cell1_name][cell2_name].type[x] == "adjHGGrid")
						{
							nZ = vlayer_map[(*v_map)[cell1_name][cell2_name].layer[x]];		  //get layer_ID
							int nDemand = N_pair * (*v_map)[cell1_name][cell2_name].value[x]; // cal this rule demand
							gGrid_C *pTargetGrid = getGrid(pDesign, nX, nY, nZ);			  // find target grid
							int nExtraDemand = pTargetGrid->getExtraDemand();
							nExtraDemand = nExtraDemand + nDemand; // add this rule demand
							pTargetGrid->setExtraDemand(nExtraDemand);
							// if (nX == 10 && nY == 68 && nZ == 1)
							// {
							// 	cout << "adj : " << cell1_name << " " << cell2_name << " " << x << " " << v_map[cell1_name][cell2_name].layer[x] << endl;
							// }
						}
					}
				}
				else // don't find constraint
				{
					continue;
				}
			}
		}
	}
	//chouchou end
	return true;
}

bool calNeighborCellDemand(design_C *pDesign) //check if need to calNeighborCell
{
	int nDX, nDY, nTX, nTY, nL;
	pDesign->getBoundary(nDX, nDY, nTX, nTY);
	vector<layer_C *> vLayer = pDesign->getLayer();
	nL = vLayer.front()->getId();

	// vector<extraDemandCell_C *> vExtra = pDesign->getExtraDemandCell();
	// set<string> sCell;
	// for (int i = 0; i < vExtra.size(); i++)
	// {
	// 	vector<cell_C *> vCell = vExtra[i]->getCell();
	// 	sCell.insert(vCell[0]->getName());
	// 	sCell.insert(vCell[1]->getName());
	// }

	// int count =0;

	for (int y = nDY; y <= nTY; y++)
	{
		for (int x = nDX; x <= nTX; x++)
		{
			gGrid_C *pGrid = getGrid(pDesign, x, y, nL);
			calCellDemand(pDesign, pGrid);
			vector<instance_C *> vInst = pGrid->getInstance();
			bool bHasCell = false;
			// for (int i = 0; i < vInst.size(); i++)
			// {
			// 	if (sCell.count(vInst[i]->getType()) > 0)
			// 	{
			// 		bHasCell = true;
			// 		break;
			// 	}
			// }
			if (vInst.size() != 0)
			// if (bHasCell)
			{
				calNeighborCellDemand(pDesign, pGrid);
				// cout<<"HI"<<endl;
				// int xx,yy,zz;
				// pGrid = getGrid( pDesign, x, y, 2 );
				// if(pGrid->getExtraDemand()!=0)
				// {
				// 	pGrid->getPosition(xx,yy,zz);
				// 	cout<<xx<<" "<<yy<<" "<<zz<<" "<<pGrid->getExtraDemand()<<endl;
				// 	count++;
				// }
			}
		}
	}
	// cout<<endl<<"total  "<<count<<endl;
}

int calJointCell(cell_C *pCell1, cell_C *pCell2, vector<instance_C *> &vInst)
{
	int nNumCell1 = 0;
	int nNumCell2 = 0;
	for (int i = 0; i < vInst.size(); i++)
	{
		instance_C *pInst = vInst[i];
		string strType = pInst->getType();
		if (pCell1->getName() == strType)
			nNumCell1++;
		else if (pCell2->getName() == strType)
			nNumCell2++;
		else
			continue;
	}
	return min(nNumCell1, nNumCell2);
}

int calJointCell(cell_C *pCell1, cell_C *pCell2, vector<instance_C *> &vInst1, vector<instance_C *> &vInst2)
{
	int nNumCell1_inst1 = 0;
	int nNumCell2_inst1 = 0;
	int nNumCell1_inst2 = 0;
	int nNumCell2_inst2 = 0;

	// calculate instance 1
	for (int i = 0; i < vInst1.size(); i++)
	{
		instance_C *pInst = vInst1[i];
		string strType = pInst->getType();
		if (pCell1->getName() == strType)
			nNumCell1_inst1++;
		else if (pCell2->getName() == strType)
			nNumCell2_inst1++;
		else
			continue;
	}

	// calculate instance 2
	for (int i = 0; i < vInst2.size(); i++)
	{
		instance_C *pInst = vInst2[i];
		string strType = pInst->getType();
		if (pCell1->getName() == strType)
			nNumCell1_inst2++;
		else if (pCell2->getName() == strType)
			nNumCell2_inst2++;
		else
			continue;
	}

	return min(nNumCell1_inst1, nNumCell2_inst2) + min(nNumCell1_inst2, nNumCell2_inst1);
}
/*
gGrid_C *graphTravel(design_C *pDesign, gGrid_C *pGrid, int dX, int dY, int dZ)
{
	int nX1, nY1, nX2, nY2;
	pDesign->getBoundary(nX1, nY1, nX2, nY2);
	vector<layer_C *> vLayer = pDesign->getLayer();
	int nZ1 = vLayer.front()->getId();
	int nZ2 = vLayer.back()->getId();
	int nX, nY, nZ;
	pGrid->getPosition(nX, nY, nZ);

	nX = nX + dX;
	nY = nY + dY;
	nZ = nZ + dZ;

	if (nX >= nX1 && nX <= nX2 && nY >= nY1 && nY <= nY2 && nZ >= nZ1 && nZ <= nZ2)
	{
		return getGrid(pDesign, nX, nY, nZ);
	}
	else
		return NULL;
}
*/
bool addNetOnGraph(design_C *pDesign, net_C *pNet)
{
	vector<gGrid_C *> vGrid;
	vector<wire_C *> vSegment;
	vSegment = pNet->getWire();
	//cout<<"test"<<endl;
	//cout << pNet << endl;
	for (int i = 0; i < vSegment.size(); i++)
	{
		wire_C *pWire = vSegment[i];
		gGrid_C *pGrid1 = pWire->getGrid1();
		gGrid_C *pGrid2 = pWire->getGrid2();
		net_C *pNet = pWire->getNet();
		//cout << pNet <<endl; 
		int nX1, nY1, nZ1;
		int nX2, nY2, nZ2;
		pGrid1->getPosition(nX1, nY1, nZ1);
		pGrid2->getPosition(nX2, nY2, nZ2);
		int ndX, ndY, ndZ;
		ndX = nX2 - nX1;
		ndY = nY2 - nY1;
		ndZ = nZ2 - nZ1;

		//if( pNet->getName() == "N13" )
		//	cout << nX1 << " " << nY1 << " " << nZ1 << " to " << nX2 << " " << nY2 << " " << nZ2 << endl;


		if (ndX != 0)
			ndX = ndX / abs(ndX);
		if (ndY != 0)
			ndY = ndY / abs(ndY);
		if (ndZ != 0)
			ndZ = ndZ / abs(ndZ);
		//cout<<ndX<<" "<<ndY<<" "<<ndZ<<endl;
		gGrid_C *pCurGrid = pGrid1;
		if (!pCurGrid->isRouted())
		{
			pCurGrid->setRouted();
			vGrid.push_back(pCurGrid);
		}

		while (pCurGrid != pGrid2)
		{
			pCurGrid = graphTravel(pDesign, pCurGrid, ndX, ndY, ndZ);
			if (!pCurGrid->isRouted())
			{
				pCurGrid->setRouted();
				vGrid.push_back(pCurGrid);
			}
		}
	}

	for (int g = 0; g < vGrid.size(); g++)
	{
		vGrid[g]->addNet(pNet);
		vGrid[g]->setUnrouted();
	}
	//addNetDemand( pDesign, vGrid );

	vGrid.clear();

	return true;
}

bool delNetOnGraph(design_C *pDesign, net_C *pNet)
{
	vector<gGrid_C *> vGrid;
	vector<wire_C *> vSegment;
	vSegment = pNet->getWire();
	//cout<<"test"<<endl;
	//cout << vSegment.size() << endl;
	for (int i = 0; i < vSegment.size(); i++)
	{
		//cout << "i " << i <<endl;
		//cout << vSegment[i] << endl;
		//if( vSegment[i] == NULL )
		//	cout << "no wire"<<endl;
		wire_C *pWire = vSegment[i];
		//cout << pWire << endl;
		gGrid_C *pGrid1 = pWire->getGrid1();
		gGrid_C *pGrid2 = pWire->getGrid2();
		//cout << "Here"<<endl;
		//if( pGrid1 == NULL || pGrid2 == NULL )
		//	cerr << "error here"<<endl;
		net_C *pNet = pWire->getNet();
		//cerr << pNet << endl;
		//if( pNet == NULL )
		//	cout << "No net"<<endl;
		//else 
		//	cout << "Has net"<<endl;
		//cerr << pNet->getId() << endl;

		int nX1, nY1, nZ1;
		int nX2, nY2, nZ2;
		pGrid1->getPosition(nX1, nY1, nZ1);
		pGrid2->getPosition(nX2, nY2, nZ2);
		int ndX, ndY, ndZ;
		ndX = nX2 - nX1;
		ndY = nY2 - nY1;
		ndZ = nZ2 - nZ1;
		//cout << nX1<<nY1<<nZ1<<endl;
		//cout << nX2<<nY2<<nZ2<<endl;

		if (ndX != 0)
			ndX = ndX / abs(ndX);
		if (ndY != 0)
			ndY = ndY / abs(ndY);
		if (ndZ != 0)
			ndZ = ndZ / abs(ndZ);
		//cout<<ndX<<" "<<ndY<<" "<<ndZ<<endl;
		gGrid_C *pCurGrid = pGrid1;
		if (!pCurGrid->isRouted())
		{
			pCurGrid->setRouted();
			vGrid.push_back(pCurGrid);
		}

		while (pCurGrid != pGrid2)
		{
			pCurGrid = graphTravel(pDesign, pCurGrid, ndX, ndY, ndZ);
			if (!pCurGrid->isRouted())
			{
				pCurGrid->setRouted();
				vGrid.push_back(pCurGrid);
			}
		}
	}

	for (int g = 0; g < vGrid.size(); g++)
	{
		vGrid[g]->delNet(pNet);
		vGrid[g]->setUnrouted();
	}
	//delNetDemand( pDesign, vGrid );

	vGrid.clear();

	return true;
}

bool addNetDemand(design_C *pDesign, vector<gGrid_C *> &vGrid)
{
	for (int i = 0; i < vGrid.size(); i++)
	{
		gGrid_C *pGrid = vGrid[i];
		int nDemand = pGrid->getDemand();
		pGrid->setDemand(nDemand + 1);
	}
	return true;
}

bool delNetDemand(design_C *pDesign, vector<gGrid_C *> &vGrid)
{
	for (int i = 0; i < vGrid.size(); i++)
	{
		gGrid_C *pGrid = vGrid[i];
		int nDemand = pGrid->getDemand();
		pGrid->setDemand(nDemand - 1);
	}
	return true;
}
