
#ifndef _RCM_H_
#define _RCM_H_

#include <vector>
#include <string>
#include <fstream>
#include <unordered_map>
#include <iostream>

using namespace std;

class net_C;
class pin_C;
class blkg_C;
class nonDefault_C;
class cell_C;
class extraDemandCell_C;
class gGrid_C;
class layer_C;
class instance_C;
class wire_C;
class design_C;

class wire_C
{
private:
	int m_nId;

protected:
	gGrid_C *m_pGrid1;
	gGrid_C *m_pGrid2;
	net_C *m_pNet;

public:
	wire_C(){};
	~wire_C(){};
	// set data
	void setId(int &nId) { m_nId = nId; }
	void setGrid1(gGrid_C *pGrid) { m_pGrid1 = pGrid; }
	void setGrid2(gGrid_C *pGrid) { m_pGrid2 = pGrid; }
	void setNet(net_C *pNet) { m_pNet = pNet; }

	// get data
	int getId() { return m_nId; }
	gGrid_C *getGrid1() { return m_pGrid1; }
	gGrid_C *getGrid2() { return m_pGrid2; }
	void getGrid(gGrid_C *&gG1, gGrid_C *&gG2)
	{
		gG1 = m_pGrid1;
		gG2 = m_pGrid2;
	}
	net_C *getNet() { return m_pNet; }
};

class net_C
{
private:
	string m_strName;
	int m_nNumNet;
	string m_strConstraint;
	int m_nId;

protected:
	vector<pin_C *> m_vPin;
	vector<wire_C *> m_vWire;
	vector<instance_C *> m_vInst;
	int m_nReroute;
	vector<int> nx, ny, nz; //pin : x,y,z
	int m_nLength;

public:
	net_C() : m_nNumNet(0), m_nReroute(0), m_nLength(0){};
	~net_C(){};
	// set data
	void setName(string strName) { m_strName = strName; }
	void setNumNet(int &nNet) { m_nNumNet = nNet; }
	void addPin(pin_C *pPin) { m_vPin.push_back(pPin); }
	void setConstraint(string &strCon) { m_strConstraint = strCon; }
	void addWire(wire_C *pW) { m_vWire.push_back(pW); }
	void addReroute() { m_nReroute++; }
	void setId(int nId) { m_nId = nId; }
	void setnx(int temp) { nx.push_back(temp); }
	void setny(int temp) { ny.push_back(temp); }
	void setnz(int temp) { nz.push_back(temp); }
	void addInst(instance_C *temp) { m_vInst.push_back(temp); }
	void setLength(int nLength) { m_nLength = nLength; }

	// get data
	string getName()
	{
		return m_strName;
	}
	int getNumNet() { return m_nNumNet; }
	vector<pin_C *> getPin() { return m_vPin; };
	string getConstraint() { return m_strConstraint; }
	vector<wire_C *> getWire() { return m_vWire; }
	int getNumReroute() { return m_nReroute; }
	int getId() { return m_nId; }
	vector<int> getnx() { return nx; }
	vector<int> getny() { return ny; }
	vector<int> getnz() { return nz; }
	vector<instance_C *> getInst() { return m_vInst; }
	int getLength() { return m_nLength; }
	// other operation
	void cleanWire()
	{
		for (int i = 0; i < m_vWire.size(); i++)
		{
			wire_C *pWire = m_vWire[i];
			m_vWire[i] = NULL;
			delete pWire;
		}
		m_vWire.clear();
	}

	//vector<bool> m_vFL;
	//vector<bool> m_vFR;
	//vector<bool> m_vFT;
	//vector<bool> m_vFD;
	vector<vector<gGrid_C *>> m_vPath;
	vector<int> m_vFL;
	vector<int> m_vFR;
	vector<int> m_vFT;
	vector<int> m_vFD;
};

class pin_C
{
private:
	string m_strName;

protected:
	cell_C *m_pCell;
	int m_nLayerId;

public:
	pin_C(){};
	~pin_C(){};
	// set data
	void setName(string strName) { m_strName = strName; }
	void setCell(cell_C *pCell) { m_pCell = pCell; }
	void setLayerId(int nId) { m_nLayerId = nId; }

	// get data
	string getName() { return m_strName; }
	cell_C *getCell() { return m_pCell; }
	int getLayerId() { return m_nLayerId; }
};

class blkg_C
{
private:
	string m_strName;

protected:
	cell_C *m_pCell;
	int m_nLayerId;
	int m_nDemand;

public:
	blkg_C(){};
	~blkg_C(){};
	// set data
	void setName(string strName) { m_strName = strName; }
	void setCell(cell_C *pCell) { m_pCell = pCell; }
	void setLayerId(int nId) { m_nLayerId = nId; }
	void setDemand(int nD) { m_nDemand = nD; }

	// get data
	string getName() { return m_strName; }
	cell_C *getCell() { return m_pCell; }
	int getLayerId() { return m_nLayerId; }
	int getDemand() { return m_nDemand; }
};

class nonDefault_C
{
private:
	int m_nId;

protected:
	int m_nX;
	int m_nY;
	int m_nZ;
	int m_nExtraSupply;

public:
	nonDefault_C(){};
	~nonDefault_C(){};

	// set data
	void setId(int nId) { m_nId = nId; }
	void setGrid(int nX, int nY, int nZ)
	{
		m_nX = nX;
		m_nY = nY;
		m_nZ = nZ;
	}
	void setNonDefaultSupply(int nS) { m_nExtraSupply = nS; }
};

class cell_C
{
private:
	string m_strName;

protected:
	int m_nNumPin;
	int m_nNumBlkg;
	bool m_bConstraintCell;
	//int m_nId;

	vector<pin_C> m_vPin;
	vector<blkg_C> m_vBlkg;
	unordered_map<string, int> pin_map;

public:
	cell_C() : m_bConstraintCell(false){};
	cell_C(cell_C *pCell)
	{
		m_strName = pCell->getName();
		m_nNumPin = pCell->getNumPin();
		m_nNumBlkg = pCell->getNumBlkg();
		vector<pin_C> vPin = pCell->getPin();
		vector<blkg_C> vBlkg = pCell->getBlkg();
		pCell->setConstraint();

		for (int i = 0; i < m_nNumPin; i++)
		{
			pin_C cPin;
			cPin.setName(vPin[i].getName());
			cPin.setLayerId(vPin[i].getLayerId());
			cPin.setCell(this);
			m_vPin.push_back(cPin);
		}

		for (int i = 0; i < m_nNumBlkg; i++)
		{
			blkg_C cBlkg;
			cBlkg.setName(vBlkg[i].getName());
			cBlkg.setLayerId(vBlkg[i].getLayerId());
			cBlkg.setCell(this);
			cBlkg.setDemand(vBlkg[i].getDemand());
			m_vBlkg.push_back(cBlkg);
		}
	}
	~cell_C(){};
	// set data
	void setName(string &strName) { m_strName = strName; }
	void setNumPin(int &nPin) { m_nNumPin = nPin; }
	void setNumBlkg(int &nB) { m_nNumBlkg = nB; }
	void addPin(pin_C &pPin) { m_vPin.push_back(pPin); }
	void addBlkg(blkg_C &pB) { m_vBlkg.push_back(pB); }
	void setPin_map(unordered_map<string, int> &vPin) { pin_map = vPin; }
	void setConstraint() { m_bConstraintCell = true; }
	//void setId( int nId ){ m_nId = nId; }

	// get data
	bool isConstraintCell() { return true; }
	string getName()
	{
		return m_strName;
	}
	int getNumPin() { return m_nNumPin; }
	int getNumBlkg() { return m_nNumBlkg; }
	vector<pin_C> getPin() { return m_vPin; }
	vector<blkg_C> getBlkg() { return m_vBlkg; }
	pin_C *getPin(string strName)
	{
		pin_C *pPin = NULL;
		for (int i = 0; i < m_vPin.size(); i++)
		{
			if (m_vPin[i].getName() == strName)
			{
				pPin = &m_vPin[i];
				break;
			}
		}
		return pPin;
	}
	blkg_C *getBlkg(string strName)
	{
		blkg_C *pB = NULL;
		for (int i = 0; i < m_vBlkg.size(); i++)
		{
			if (m_vBlkg[i].getName() == strName)
			{
				pB = &m_vBlkg[i];
				break;
			}
		}
		return pB;
	}
	//int getId(){ return m_nId; }
};

class gGrid_C
{
private:
	int m_nId;

protected:
	int m_nSupply;
	int m_nDemand;
	int m_nExtraDemand;
	vector<instance_C *> m_vInst;
	vector<net_C *> m_vNet;
	bool m_bRouted;
	unordered_map<string, int> gGInstMap;
	int m_nPseudoPinDemand;

public:
	gGrid_C() : m_nSupply(0), m_nDemand(0), m_nExtraDemand(0), m_bRouted(false), m_nPseudoPinDemand(0){};
	~gGrid_C(){};
	int m_nX;
	int m_nY;
	int m_nZ;

	// set data
	void setId(int nId) { m_nId = nId; }
	void setSupply(int nS) { m_nSupply = nS; }
	void setDemand(int nD) { m_nDemand = nD; }
	void setExtraDemand(int nD) { m_nExtraDemand = nD; }
	void addInstance(instance_C *pInst) { m_vInst.push_back(pInst); }
	void delInstance(instance_C *pInst)
	{
		for (int i = m_vInst.size() - 1; i >= 0; i--)
		{
			if (pInst == m_vInst[i])
			{
				m_vInst.erase(m_vInst.begin() + i);
				break;
			}
		}
	}
	void addNet(net_C *pNet)
	{
		bool bFind = false;
		for (int i = 0; i < m_vNet.size(); i++)
		{
			if (m_vNet[i] == pNet)
			{
				bFind = true;
				break;
			}
		}
		if (!bFind)
			m_vNet.push_back(pNet);
	}
	void delNet(net_C *pNet)
	{
		for (int i = m_vNet.size() - 1; i >= 0; i--)
		{
			if (m_vNet[i] == pNet)
			{
				m_vNet.erase(m_vNet.begin() + i);
				break;
			}
		}
	}
	bool findNet(net_C *pNet)
	{
		bool bFind = false;
		for (int i = m_vNet.size() - 1; i >= 0; i--)
		{
			if (m_vNet[i] == pNet)
			{
				bFind = true;
				break;
			}
		}
		return bFind;
	}

	bool findInst(instance_C *pInst)
	{
		bool bFind = false;
		for (int i = 0; i < m_vInst.size(); i++)
		{
			if (m_vInst[i] == pInst)
			{
				bFind = true;
				break;
			}
		}
		return bFind;
	}
	void setgGInstMap(unordered_map<string, int> &temp)
	{
		gGInstMap = temp;
	}
	// for router
	void setRouted() { m_bRouted = true; }
	void setUnrouted() { m_bRouted = false; }

	// get data
	int getId() { return m_nId; }
	int getSupply() { return m_nSupply; }
	int getDemand() { return m_nDemand; }
	vector<net_C *> getNet() { return m_vNet; }
	int getExtraDemand() { return m_nExtraDemand; }
	int getTotalDemand() { return m_nDemand + m_nExtraDemand; }
	//int getRemand(){ return m_nSupply - m_nDemand - m_nExtraDemand; }
	int getRemand() { return m_nSupply - m_nDemand - m_nExtraDemand - (int)m_vNet.size(); }
	void getPosition(int &nX, int &nY, int &nZ)
	{
		nX = m_nX;
		nY = m_nY;
		nZ = m_nZ;
	}
	vector<instance_C *> getInstance() { return m_vInst; }
	bool isOverflow()
	{
		if ((int)(m_nSupply - m_nDemand - m_nExtraDemand - (int)m_vNet.size()) < 0)
			return true;
		else
			return false;
	}
	bool isLimit()
	{
		if ((int)(m_nSupply - m_nDemand - m_nExtraDemand - (int)m_vNet.size()) == 0)
			return true;
		else
			return false;
	}
	bool isRouted() { return m_bRouted; }
	unordered_map<string, int> *getgGInstMap()
	{
		return &gGInstMap;
	}

	int getPinDemand(){ return m_nPseudoPinDemand; }
	void addPinDemand(){ m_nPseudoPinDemand++; }
	void delPinDemand(){ m_nPseudoPinDemand--; }
	void resetPinDemand(){ m_nPseudoPinDemand = 0; }
};

class layer_C
{
private:
	string m_strName;

protected:
	int m_nId;
	char m_cDirection;
	int m_nSupply;

public:
	layer_C(){};
	~layer_C(){};
	// set data
	void setName(string strName) { m_strName = strName; }
	void setId(int nId) { m_nId = nId; }
	void setDir(char cDir) { m_cDirection = cDir; }
	void setSupply(int nSup) { m_nSupply = nSup; }

	// get data
	string getName() { return m_strName; }
	int getId() { return m_nId; }
	char getDir() { return m_cDirection; }
	int getSupply() { return m_nSupply; }
};

class instance_C : public cell_C
{
private:
	string m_strName;

protected:
	int m_nPlacedX;
	int m_nPlacedY;
	bool m_bMovable;
	int m_nId;
	bool m_bHasMoved;

public:
	instance_C() : m_bHasMoved(false){};
	instance_C(cell_C *pCell) : cell_C(pCell), m_bHasMoved(false){};
	~instance_C(){};
	// set data
	void setName(string strName) { m_strName = strName; }
	void setPlaced(int nX, int nY)
	{
		m_nPlacedX = nX;
		m_nPlacedY = nY;
	}
	void setMovable(bool bMove) { m_bMovable = bMove; }
	void setId(int nId) { m_nId = nId; }
	void hasMoved() { m_bHasMoved = true; }

	// get data
	string getName() { return m_strName; }
	string getType() { return cell_C::getName(); }
	int getPlacedX() { return m_nPlacedX; }
	int getPlacedY() { return m_nPlacedY; }
	bool isMovable() { return m_bMovable; }
	int getId() { return m_nId; }
	bool hasBeenMoved() { return m_bHasMoved; }
	pin_C *IgetPin(string name) { return cell_C::getPin(name); }
};

class extraDemandCell_C
{
private:
	int m_nId;

protected:
	string m_strType;
	vector<cell_C *> m_vCell;
	layer_C *m_pLayer;
	int m_nExtraDemand;

public:
	extraDemandCell_C(){};
	~extraDemandCell_C(){};

	// set data
	void setId(int nId) { m_nId = nId; }
	void setType(string strType) { m_strType = strType; }
	void addCell(cell_C *pCell) { m_vCell.push_back(pCell); }
	void setLayer(layer_C *pLayer) { m_pLayer = pLayer; }
	void setExtraDemand(int nD) { m_nExtraDemand = nD; }

	// get data
	string getType() { return m_strType; }
	vector<cell_C *> getCell() { return m_vCell; }
	layer_C *getLayer() { return m_pLayer; }
	int getExtraDemand() { return m_nExtraDemand; }
};

typedef struct extra_demand_
{
	int N = 0; //data num
	vector<string> layer;
	vector<string> type; // contraint typer
	vector<int> value;	 //demand
} extra_demand;

// typedef struct instance_
// {
// 	string maseterCellName;
// 	string move_cstr ;//movableCstr
// 	int x;//<gGridRowIdx
// 	int y;//gGridColIdx
// }inst;

class design_C
{
private:
protected:
	// move constraint
	int m_nMaxCellMove;

	// GGrid graph size
	int m_nGGridRowBegin;
	int m_nGGridRowEnd;
	int m_nGGridColBegin;
	int m_nGGridColEnd;

	// number of layer
	int m_nNumLayer;

	// number of non default supply GGrid
	int m_nNumNonDefSupGrid;

	// number of cell
	int m_nNumCell;

	// number of net
	int m_nNumNet;

	// design information
	vector<nonDefault_C *> m_vNonDefault;
	vector<extraDemandCell_C *> m_vExtraDemandCell;
	vector<cell_C *> m_vCell;
	vector<layer_C *> m_vLayer;
	vector<instance_C *> m_vInstance;
	vector<net_C *> m_vNet;

	// graph & map
	vector<vector<vector<gGrid_C *>>> m_vGraph;

	// moved cell information
	vector<instance_C *> m_vMovedInstance;

	//chouchou begin
	//extrademand map
	unordered_map<string, unordered_map<string, extra_demand>> extra_map; //cell1 cell2 type layer value
	//layer map
	unordered_map<string, int> layer_map;
	unordered_map<string, instance_C *> inst_map;
	unordered_map<string, net_C *> net_map;
	unordered_map<string, unordered_map<string, extra_demand>> extra_map_2;
	//chouchou end

public:
	design_C(){};
	~design_C(){};
	// set data
	void setMaxCellMove(int nM) { m_nMaxCellMove = nM; }
	void setGridBoundry(int nRB, int nCB, int nRE, int nCE)
	{
		m_nGGridRowBegin = nRB;
		m_nGGridRowEnd = nRE;
		m_nGGridColBegin = nCB;
		m_nGGridColEnd = nCE;
	}
	void setNumLayer(int nL) { m_nNumLayer = nL; }
	void setNumNonDefSupGrid(int nD) { m_nNumNonDefSupGrid = nD; }
	void addNumNonDefSupGrid(nonDefault_C *pD) { m_vNonDefault.push_back(pD); }
	void setNumCell(int nC) { m_nNumCell = nC; }
	void setNumNet(int nNet) { m_nNumNet = nNet; }
	void setExtraDemandCell(vector<extraDemandCell_C *> vExtra) { m_vExtraDemandCell = vExtra; }
	void addExtraDemandCell(extraDemandCell_C *pE) { m_vExtraDemandCell.push_back(pE); }
	void addCell(cell_C *pCell) { m_vCell.push_back(pCell); }
	void setCell(vector<cell_C *> vCell) { m_vCell = vCell; }
	void addLayer(layer_C *pLayer) { m_vLayer.push_back(pLayer); }
	void setLayer(vector<layer_C *> vLayer) { m_vLayer = vLayer; }
	void addInstance(instance_C *pInst) { m_vInstance.push_back(pInst); }
	void setInstance(vector<instance_C *> vInst) { m_vInstance = vInst; }
	void addNet(net_C *pNet) { m_vNet.push_back(pNet); }
	void setNet(vector<net_C *> vNet) { m_vNet = vNet; }
	void setExtra_map(unordered_map<string, unordered_map<string, extra_demand>> &vextra_map) { extra_map = vextra_map; }
	void setLayer_map(unordered_map<string, int> &vLayer_map) { layer_map = vLayer_map; }
	void setInst_map(unordered_map<string, instance_C *> &vinst_map) { inst_map = vinst_map; }
	void setNet_map(unordered_map<string, net_C *> &vnet_map) { net_map = vnet_map; }
	void setExtra_map_2(unordered_map<string, unordered_map<string, extra_demand>> &vextra_map)
	{
		extra_map_2 = vextra_map;
	}

	// get data
	int getMaxCellConstraint() { return m_nMaxCellMove; }
	int getNumLayer() { return m_nNumLayer; }
	vector<layer_C *> getLayer() { return m_vLayer; }
	vector<cell_C *> getCell() { return m_vCell; }
	vector<instance_C *> getInstance() { return m_vInstance; }
	vector<net_C *> getNet() { return m_vNet; }
	vector<extraDemandCell_C *> getExtraDemandCell() { return m_vExtraDemandCell; }
	void getBoundary(int &nRB, int &nCB, int &nRE, int &nCE)
	{
		nRB = m_nGGridRowBegin;
		nCB = m_nGGridColBegin;
		nRE = m_nGGridRowEnd;
		nCE = m_nGGridColEnd;
	}
	void getGraph(vector<vector<vector<gGrid_C *>>> &G) { G = m_vGraph; }
	vector<vector<vector<gGrid_C *>>> *getGraph() { return &m_vGraph; }
	unordered_map<string, unordered_map<string, extra_demand>> *getExtra_map() { return &extra_map; }
	unordered_map<string, unordered_map<string, extra_demand>> *getExtra_map_2() { return &extra_map_2; }

	unordered_map<string, int> getLayer_map()
	{
		return layer_map;
	}
	unordered_map<string, instance_C *> getInst_map() { return inst_map; }
	unordered_map<string, net_C *> getNet_map() { return net_map; }
	gGrid_C *getGrid(int nX, int nY, int nZ) { return m_vGraph[nZ - m_vLayer.front()->getId()][nY - m_nGGridColBegin][nX - m_nGGridRowBegin]; }
	// other information
	bool checkInfo();
	void dumpInfo();

	friend gGrid_C *getGrid(design_C *, int, int, int);
	//	friend gGrid_C* graphTravel( design_C*, gGrid_C*, int, int, int );
};

// operation function
int calWireLength(design_C *);
int calOverflow(design_C *);

// defined in graph.cpp
bool constructGraph(design_C *);

inline gGrid_C *getGrid(design_C *pDesign, int nX, int nY, int nZ)
{
	nX = nX - pDesign->m_vGraph[0][0][0]->m_nX;
	nY = nY - pDesign->m_vGraph[0][0][0]->m_nY;
	nZ = nZ - pDesign->m_vLayer[0]->getId();
	return pDesign->m_vGraph[nZ][nY][nX];
}
bool addGGridSupply(design_C *, int, int, int, int); // design, extra, x, y, z
bool delGGridSupply(design_C *, int, int, int, int);

inline gGrid_C *graphTravel(design_C *pDesign, gGrid_C *pGrid, int dX, int dY, int dZ)
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
} // design, current grid, moveX, moveY, moveZ

// cell operation
bool addCellOnGraph(design_C *, vector<instance_C *> &);
bool addCellOnGraph(design_C *, instance_C *);
bool delCellOnGraph(design_C *, instance_C *);
bool addCellDemand(design_C *, instance_C *);
bool addCellDemand(gGrid_C *, instance_C *);
bool delCellDemand(gGrid_C *, instance_C *);
bool delCellDemand(design_C *, instance_C *);
bool calCellDemand(design_C *, instance_C *);
bool calCellDemand(design_C *, gGrid_C *);
bool addNeighborCellDemand(design_C *, gGrid_C *, instance_C *);
bool removeNeighborCellDemand(design_C *, gGrid_C *, instance_C *);
bool calNeighborCellDemand(design_C *, gGrid_C *);
bool calNeighborCellDemand(design_C *, gGrid_C *, instance_C *);
bool calNeighborCellDemand(design_C *);
bool delNeighborCellDemand(design_C *, gGrid_C *, instance_C *);
int calJointCell(cell_C *, cell_C *, vector<instance_C *> &, vector<instance_C *> &);
int calJointCell(cell_C *, cell_C *, vector<instance_C *> &);
bool addNetDemand(design_C *, vector<gGrid_C *> &);
bool delNetDemand(design_C *, vector<gGrid_C *> &);

bool addExtraDemandOnGraph(design_C *, gGrid_C *);
bool calculateCellDemand(design_C *, gGrid_C *);
bool calculateCellDemand(design_C *);

// net operation
bool findNet(gGrid_C *, net_C *);
bool addNetOnGraph(design_C *, net_C *);
bool delNetOnGraph(design_C *, net_C *);
// display graph result
void dumpGraph(design_C *);

// output function
bool dumpResult(design_C *, char *);

#endif
