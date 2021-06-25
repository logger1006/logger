#include "parser.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <stdlib.h>
#include <map>
#include <sstream>

using namespace std;

extern time_t start;

bool parser(design_C *pDesign, char *pFileName)
{
	ifstream fin;
	fin.open(pFileName, ios::in);
	ios::sync_with_stdio(false);
	if (!fin)
	{
		return false;
	}

	string strBuffer;

	while (fin >> strBuffer)
	{
		if (strBuffer == "MaxCellMove")
		{
			parseMaxCell(pDesign, fin);
		}
		else if (strBuffer == "GGridBoundaryIdx")
		{
			parseBoundary(pDesign, fin);
		}
		else if (strBuffer == "NumLayer")
		{
			parseLayer(pDesign, fin);
		}
		else if (strBuffer == "NumNonDefaultSupplyGGrid")
		{
			parseNonDefaultSupplyGGrid(pDesign, fin);
			cout << "Non complete" << endl;
		}
		else if (strBuffer == "NumMasterCell")
		{
			parseCell(pDesign, fin);
			cout << "Cell complete" << endl;
		}
		else if (strBuffer == "NumNeighborCellExtraDemand")
		{
			parseCellExtraDemand(pDesign, fin);
			//dumpGraph( pDesign );
			cout << "Extra complete" << endl;
		}
		else if (strBuffer == "NumCellInst")
		{
			parseInstance(pDesign, fin);
			// cout<<"instance complete at "<<time(NULL) - start<<endl;
			cout << "Instance complete" << endl;
		}
		else if (strBuffer == "NumNets")
		{
			parseNet(pDesign, fin);
			cout << "Net complete" << endl;
		}
		else if (strBuffer == "NumRoutes")
		{
			parseRoute(pDesign, fin);
			cout << "parser route complete" << endl;
		}
		else if (strBuffer == "NumVoltageAreas")
		{
			parseVoltageArea( pDesign, fin );
			cout << "VoltageArea complete" << endl;
		}
	}

	//dumpGraph( pDesign );

	return true;
}

bool parseMaxCell(design_C *pDesign, ifstream &fin)
{
	int nMaxCell = 0;
	fin >> nMaxCell;
	pDesign->setMaxCellMove(nMaxCell);
	//cout << nMaxCell << endl;
	return true;
}

bool parseBoundary(design_C *pDesign, ifstream &fin)
{
	int nDX, nDY, nTX, nTY;
	fin >> nDX >> nDY >> nTX >> nTY;
	pDesign->setGridBoundry(nDX, nDY, nTX, nTY);
	//cout << nDX << nDY <<endl;
	return true;
}

bool parseLayer(design_C *pDesign, ifstream &fin)
{
	int nNumLayer = 0;
	fin >> nNumLayer;
	pDesign->setNumLayer(nNumLayer);
	string strBuf;
	string strLayer;
	int nIndex;
	char cDir;
	int nSupply;
	double dWeight;
	unordered_map<string, int> layer_map_temp;
	for (int i = 0; i < nNumLayer; i++)
	{
		fin >> strBuf >> strLayer >> nIndex >> cDir >> nSupply >> dWeight ;
		layer_C *pLayer = new layer_C;
		pLayer->setName(strLayer);
		pLayer->setId(nIndex);
		pLayer->setDir(cDir);
		pLayer->setSupply(nSupply);
		pLayer->setWeight( dWeight );
		pDesign->addLayer(pLayer);

		//chouchou
		layer_map_temp[strLayer] = nIndex;
	}
	// construct graph
	constructGraph(pDesign);
	pDesign->setLayer_map(layer_map_temp);
	return true;
}

bool parseNonDefaultSupplyGGrid(design_C *pDesign, ifstream &fin)
{
	int nNumGrid = 0;
	fin >> nNumGrid;
	int nX, nY, nZ, nNum;
	string strSupply;
	for (int i = 0; i < nNumGrid; i++)
	{
		fin >> nX >> nY >> nZ >> strSupply;
		nonDefault_C *pDef = new nonDefault_C;

		if (strSupply[0] == '+')
		{
			strSupply = strSupply.substr(1);
			nNum = atoi(strSupply.c_str());
			//cout<<nNum<<endl;
			addGGridSupply(pDesign, nNum, nX, nY, nZ);
			//cout<<"!"<<endl;
		}
		else if (strSupply[0] == '-')
		{
			strSupply = strSupply.substr(1);
			nNum = atoi(strSupply.c_str());
			delGGridSupply(pDesign, nNum, nX, nY, nZ);
			nNum = -1 * nNum;
			//cout<<nNum<<endl;
		}
		else
		{
			cout << "Error format" << endl;
			delete pDef;
			continue;
		}
		pDef->setId(i);
		pDef->setGrid(nX, nY, nZ);
		pDef->setNonDefaultSupply(nNum);
		pDesign->addNumNonDefSupGrid(pDef);
	}
	return true;
}

bool parseCell(design_C *pDesign, ifstream &fin)
{
	int nNumCell = 0;
	fin >> nNumCell;
	pDesign->setNumCell(nNumCell);

	string strBuf;
	string strCellName;
	int nNumPin = 0;
	int nNumBlkg = 0;
	// get layer information //
	vector<layer_C *> vLayer;
	vLayer = pDesign->getLayer();

	for (int i = 0; i < nNumCell; i++)
	{
		fin >> strBuf >> strCellName >> nNumPin >> nNumBlkg;
		cell_C *pCell = new cell_C;
		pCell->setName(strCellName);
		pCell->setNumPin(nNumPin);
		pCell->setNumBlkg(nNumBlkg);

		// parse pin
		string strPinName;
		string strPinLayer;
		unordered_map<string, int> temp_pin_map;
		for (int j = 0; j < nNumPin; j++)
		{
			fin >> strBuf >> strPinName >> strPinLayer;
			pin_C cPin;
			cPin.setName(strPinName);
			cPin.setCell(pCell);
			int nLayerId;
			for (int l = 0; l < vLayer.size(); l++)
			{
				if (vLayer[l]->getName() == strPinLayer)
				{
					nLayerId = vLayer[l]->getId();
					break;
				}
			}
			cPin.setLayerId(nLayerId);
			pCell->addPin(cPin);
			temp_pin_map[strPinName] = nLayerId;
			// cout << temp_pin_map[strPinName] << endl;
		}
		pCell->setPin_map(temp_pin_map);

		// parse blkg
		string strBlkgName;
		string strBlkgLayer;
		int nNumDemand;
		for (int j = 0; j < nNumBlkg; j++)
		{
			fin >> strBuf >> strBlkgName >> strBlkgLayer >> nNumDemand;
			blkg_C cBlkg;
			cBlkg.setCell(pCell);
			cBlkg.setName(strBlkgName);
			int nLayerId;
			for (int l = 0; l < vLayer.size(); l++)
			{
				if (vLayer[l]->getName() == strBlkgLayer)
				{
					nLayerId = vLayer[l]->getId();
					break;
				}
			}
			cBlkg.setLayerId(nLayerId);
			cBlkg.setDemand(nNumDemand);
			pCell->addBlkg(cBlkg);
		}
		pDesign->addCell(pCell);
	}
	return true;
}

bool parseCellExtraDemand(design_C *pDesign, ifstream &fin)
{
	int nDemand = 0;
	vector<cell_C *> vCell = pDesign->getCell();
	vector<layer_C *> vLayer = pDesign->getLayer();
	fin >> nDemand;
	string strType;
	string strCell1;
	string strCell2;
	string strLayer;
	int nExtraDemand;
	unordered_map<string, unordered_map<string, extra_demand>> temp_extra_map;
	unordered_map<string, unordered_map<string, extra_demand>> temp_extra_map_2;
	// map<string,map<string,extra_demand > >::iterator iter;
	// map<string,extra_demand > temp_map;
	for (int i = 0; i < nDemand; i++)
	{
		fin >> strType >> strCell1 >> strCell2 >> strLayer >> nExtraDemand;
		extraDemandCell_C *pExtra = new extraDemandCell_C;
		pExtra->setId(i);
		pExtra->setType(strType);
		cell_C *pCell = NULL;
		for (int c = 0; c < vCell.size(); c++)
		{
			if (vCell[c]->getName() == strCell1)
			{
				pCell = vCell[c];
				pCell->setConstraint();
				break;
			}
		}
		pExtra->addCell(pCell);

		for (int c = 0; c < vCell.size(); c++)
		{
			if (vCell[c]->getName() == strCell2)
			{
				pCell = vCell[c];
				pCell->setConstraint();
				break;
			}
		}
		pExtra->addCell(pCell);
		layer_C *pLayer = NULL;
		for (int l = 0; l < vLayer.size(); l++)
		{
			if (vLayer[l]->getName() == strLayer)
			{
				pLayer = vLayer[l];
				break;
			}
		}
		pExtra->setLayer(pLayer);
		pExtra->setExtraDemand(nExtraDemand);

		pDesign->addExtraDemandCell(pExtra);
		//create extra_map;

		// iter = temp_extra_map.find(strCell1);
		// if(iter != temp_extra_map.end())
		// {
		// 	iter->second[strCell2].layer.push_back(strLayer);
		// 	iter->second[strCell2].type.push_back(strType);
		// 	iter->second[strCell2].value.push_back(nExtraDemand);
		// 	iter->second[strCell2].N++;
		// }
		temp_extra_map[strCell1][strCell2].layer.push_back(strLayer);
		temp_extra_map[strCell1][strCell2].type.push_back(strType);
		temp_extra_map[strCell1][strCell2].value.push_back(nExtraDemand);
		temp_extra_map[strCell1][strCell2].N++;
		temp_extra_map_2[strCell2][strCell1].layer.push_back(strLayer);
		temp_extra_map_2[strCell2][strCell1].type.push_back(strType);
		temp_extra_map_2[strCell2][strCell1].value.push_back(nExtraDemand);
		temp_extra_map_2[strCell2][strCell1].N++;
	}
	pDesign->setExtra_map(temp_extra_map);
	pDesign->setExtra_map_2(temp_extra_map_2);
	return 0;
}

bool parseInstance(design_C *pDesign, ifstream &fin)
{
	vector<cell_C *> vCell = pDesign->getCell();
	int nNumCell = 0;
	fin >> nNumCell;

	string strBuf;
	string strInstName;
	string strCellName;
	int nX, nY;
	string strMovable;

	//map
	unordered_map<string, instance_C *> inst_map_temp;

	for (int i = 0; i < nNumCell; i++)
	{
		fin >> strBuf >> strInstName >> strCellName >> nX >> nY >> strMovable;
		cell_C *pCell;

		for (int c = 0; c < vCell.size(); c++)
		{
			if (strCellName == vCell[c]->getName())
			{
				pCell = vCell[c];
				break;
			}
		}

		instance_C *pInst = new instance_C(pCell);

		pInst->setName(strInstName);
		pInst->setPlaced(nX, nY);
		pInst->setId(i);
		if (strMovable == "Movable")
		{
			pInst->setMovable(true);
		}
		else
			pInst->setMovable(false);
		pDesign->addInstance(pInst);
		inst_map_temp[strInstName] = pInst;
		//addCellOnGraph( pDesign, pInst );
		int nLowestLayer = pDesign->getLayer()[0]->getId();
		gGrid_C *pGrid = getGrid(pDesign, nX, nY, nLowestLayer);
		//gGrid_C* pGrid = pDesign->getGrid( nX, nY, nLowestLayer );
		pGrid->addInstance(pInst);
	}

	pDesign->setInst_map(inst_map_temp);
	// calculate extra demand
	//cerr << "calculate extra demand" << endl;
	calNeighborCellDemand(pDesign);
	//cerr << "calculate extra complete" << endl;
	return true;
}

bool parseNet(design_C *pDesign, ifstream &fin)
{
	vector<instance_C *> vInst = pDesign->getInstance();
	unordered_map<string, instance_C *> vInst_map = pDesign->getInst_map();
	unordered_map<string, instance_C *>::iterator iter;
	unordered_map<string, net_C *> net_map_temp;

	int nNumNet = 0;
	fin >> nNumNet;
	pDesign->setNumNet(nNumNet);

	string strBuf;
	string strNetName;
	int nNumPin;
	string strConstraint;
	double dWeight;

	for (int i = 0; i < nNumNet; i++)
	{
		fin >> strBuf >> strNetName >> nNumPin >> strConstraint >> dWeight;
		net_C *pNet = new net_C;
		pNet->setName(strNetName);
		pNet->setConstraint(strConstraint);
		int nLayerConstraint = 1;
		if (strConstraint == "NoCstr")
		{
			nLayerConstraint = 1;
		}

		vector<layer_C *> vLayer = pDesign->getLayer();
		for (int l = 0; l < vLayer.size(); l++)
		{
			if (strConstraint == vLayer[l]->getName())
			{
				nLayerConstraint = vLayer[l]->getId();
				break;
			}
		}
		pNet->setConstraintLayerId( nLayerConstraint );
		pNet->setId(i);
		pNet->setWeight( dWeight );
		string strInst;
		string strPin;
		for (int p = 0; p < nNumPin; p++)
		{
			fin >> strBuf >> strBuf;
			strInst = strBuf.substr(0, strBuf.find("/"));
			strPin = strBuf.substr(strBuf.find("/") + 1);
			// find instance
			instance_C *pInst = NULL;
			// for (int x = 0; x < vInst.size(); x++)
			// {
			// 	if (vInst[x]->getName() == strInst)
			// 	{
			// 		pInst = vInst[x];
			// 		break;
			// 	}
			// }
			iter = vInst_map.find(strInst);
			if (iter != vInst_map.end())
			{
				pInst = iter->second;
			}

			//find pin
			//vector< pin_C* > vPin = pInst->getPin();
			pin_C *pPin = NULL;
			pPin = pInst->getPin(strPin);
			pPin->setNet( pNet );
			pNet->setnx(pInst->getPlacedX());
			pNet->setny(pInst->getPlacedY());
			pNet->addInst(pInst);

			// unordered_map<string, int> temp_map = pInst->getpin_map();
			// pNet->setnz(temp_map[strPin]);
			// for (auto iter = temp_map.begin(); iter != temp_map.end(); iter++)
			// {
			// 	cout << iter->first << " " << iter->second << endl;
			// }
			pin_C *temp_p = pInst->IgetPin(strPin);
			pNet->setnz(temp_p->getLayerId());
			/*
			for( int n=0; n<vPin.size(); n++ )
			{
				if( vPin[n]->getName() == strPin )
				{
					pPin = vPin[n];
					break;
				}
			}
			*/
			pNet->addPin(pPin);
		}
		pDesign->addNet(pNet);
		net_map_temp[strNetName] = pNet;
	}
	pDesign->setNet_map(net_map_temp);
	return true;
}

bool parseRoute(design_C *pDesign, ifstream &fin)
{
	int nNumRoute = 0;
	fin >> nNumRoute;
	vector<net_C *> vNet = pDesign->getNet();
	unordered_map<string, net_C *> vNet_map = pDesign->getNet_map();

	int nX1, nY1, nZ1, nX2, nY2, nZ2;
	string strNetName;
	for (int i = 0; i < nNumRoute; i++)
	{
		fin >> nX1 >> nY1 >> nZ1 >> nX2 >> nY2 >> nZ2 >> strNetName;
		net_C *pNet = NULL;
		// for (int n = 0; n < vNet.size(); n++)
		// {
		// 	if (vNet[n]->getName() == strNetName)
		// 	{
		// 		pNet = vNet[n];
		// 		break;
		// 	}
		// }
		auto iter = vNet_map.find(strNetName);
		if (iter != vNet_map.end())
		{
			pNet = iter->second;
		}

		wire_C *pWire = new wire_C;
		pWire->setId(i);
		pWire->setNet(pNet);
		gGrid_C *pGrid1 = getGrid(pDesign, nX1, nY1, nZ1);
		gGrid_C *pGrid2 = getGrid(pDesign, nX2, nY2, nZ2);
		pWire->setGrid1(pGrid1);
		pWire->setGrid2(pGrid2);
		pNet->addWire(pWire);
		//cout<<i<<endl;
	}

	for (int i = 0; i < vNet.size(); i++)
	{
		net_C *pNet = vNet[i];
		if (pNet->getPin().size() > 1 && pNet->getWire().size() == 0)
		{
			wire_C *pWire = new wire_C;
			vector<pin_C *> vTmpPin = pNet->getPin();
			pin_C *pPin = vTmpPin[0];

			instance_C *pInst = (instance_C *)pPin->getCell();
			int nX, nY;
			nX = pInst->getPlacedX();
			nY = pInst->getPlacedY();
			int nZ = pPin->getLayerId();
			gGrid_C *pGrid = getGrid(pDesign, nX, nY, nZ);
			pWire->setGrid1(pGrid);
			pWire->setGrid2(pGrid);
			pWire->setNet(pNet);
			pNet->addWire(pWire);
		}
		addNetOnGraph(pDesign, pNet);
	}

	return true;
}

bool parseVoltageArea( design_C* pDesign, ifstream &fin )
{
	vector< voltageArea_C* > vVolArea;
	string strName;
	string strInst;
	string strBuffer;
	int nNumVolArea = 0;
	int nX, nY;
	int nNumGrid = 0;
	int nNumInst = 0;
	

	fin >> nNumVolArea;
	for( int i=0; i<nNumVolArea; i++ )
	{
		set< gGrid_C* > sGrid;
		set< instance_C* > sInst;
		fin >> strBuffer >> strName;
		fin >> strBuffer >> nNumGrid;
		for( int g=0; g<nNumGrid; g++ )
		{
			fin >> nX >> nY;
			gGrid_C* pGrid = getGrid( pDesign, nX, nY, 1 );
			sGrid.insert( pGrid );
		}
		fin >> strBuffer >> nNumInst;
		for( int n=0; n<nNumInst; n++ )
		{
			fin >> strInst;
			instance_C* pInst = pDesign->getInst_map()[ strInst ];
			sInst.insert( pInst );
		}
		voltageArea_C* pVolArea = new voltageArea_C;
		//cout << strName << " " <<sGrid.size() << " " << sInst.size() << endl;
		pVolArea->setName( strName );
		pVolArea->setGrid( sGrid );
		pVolArea->setInst( sInst );
		//cout << strName << " " <<sGrid.size() << " " << sInst.size() << endl;
		vVolArea.push_back( pVolArea );
	}
	pDesign->setVolateArea( vVolArea );
	return true;

}
