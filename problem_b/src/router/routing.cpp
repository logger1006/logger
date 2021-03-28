#include "routing.h"
#include <iostream>
#include <algorithm>
#include <climits>
#include <stdlib.h>
#include <set>
#include <iomanip>
#include <fstream>
#include <ctime>
#include <sstream>
#include "flute.h"
#include "3Dresult.h"
#include "assistant.h"
//#include "omp.h"

using namespace std;

extern time_t start;
extern time_t endt;

int nSglCount = 0;
int nMulCount = 0;
int nSglFCount = 0;
int nMulFCount = 0;
#define _DEBUG_MODE

#ifdef _DEBUG_MODE
int nTmpFailedRoute = 0;
int nTmpCantPlaced = 0;
#endif

#define COUTWIDTH 40
#define TIMECONST 3550

bool router_C::showSummary()
{
	cout << "SUMMARY" << endl;
	cout << setw(COUTWIDTH) << left << setfill(' ') << "Number of Iteration: " << m_nIteration << endl;
	cout << setw(COUTWIDTH) << left << setfill(' ') << "Number of success movement: " << m_nSuccess << endl;
	cout << setw(COUTWIDTH) << left << setfill(' ') << "Number of failed movement: " << m_nFailed << endl;
	cout << setw(COUTWIDTH) << left << setfill(' ') << "Number of routing refinement: " << m_nNumRefinement << endl;
	cout << setw(COUTWIDTH) << left << setfill(' ') << "Number of moved instance: " << m_vMovedInstance.size() << endl;
#ifdef _DEBUG_MODE
	cout << "Additional Information for debug" << endl;
	cout << setw(COUTWIDTH) << left << setfill(' ') << "Number of routing failed: " << nTmpFailedRoute << endl;
	cout << setw(COUTWIDTH) << left << setfill(' ') << "Number of placed failed: " << nTmpCantPlaced << endl;
#endif
	return true;
}

bool router_C::init()
{
	cout << "Initial routing engine" << endl;

	cout << setw(COUTWIDTH) << left << setfill('.') << "Loading design information";
	if (loadDesign())
		cout << "complete" << endl;

	cout << setw(COUTWIDTH) << left << setfill('.') << "Creating routing graph";
	if (createHistoryGraph() && createRoutingGraph() && create2DGraph() )
		cout << "complete" << endl;

	// create forced model
	cout << setw(COUTWIDTH) << left << setfill('.') << "Creating forced model";
	if (createForcedModel())
		cout << "complete" << endl;

	cout << setw(COUTWIDTH) << left << setfill('.') << "Creating network";
	if (createForcedNetwork())
		cout << "complete" << endl;

	cout << setw(COUTWIDTH) << left << setfill('.') << "Linking network";
	if (linkForcedModel())
		cout << "complete" << endl;

	//cout<<setw(COUTWIDTH)<<left<<setfill('.')<<"Creating net forced model";
	//if( createNetForcedModel() )
	//	cout<<"complete"<<endl;

	cerr << " Net Forced "<<endl;
	for (int i = 0; i < m_vNetworkForced.size(); i++)
	{
		//calForcedNetwork_ver2(m_vNetworkForced[i]);
		calForcedNetwork_ver3(m_vNetworkForced[i]);
	}
	cerr << "Forced "<<endl;
	for (int i = 0; i < m_vForced.size(); i++)
	{
		//calForcedModel_ver3(m_vForced[i]);
		calForcedModel_ver4(m_vForced[i]);
	}
	for (int i = 0; i < m_vNetworkForced.size(); i++)
	{
		calBoundryModel( &m_vNetworkForced[i].m_cLB );
		//calBoundryModel_ver2( &m_vNetworkForced[i].m_cLB );
		m_vBoundry.push_back( &m_vNetworkForced[i].m_cLB );
		calBoundryModel( &m_vNetworkForced[i].m_cRB );
		//calBoundryModel_ver2( &m_vNetworkForced[i].m_cRB );
		m_vBoundry.push_back( &m_vNetworkForced[i].m_cRB );
		calBoundryModel( &m_vNetworkForced[i].m_cTB );
		//calBoundryModel_ver2( &m_vNetworkForced[i].m_cTB );
		m_vBoundry.push_back( &m_vNetworkForced[i].m_cTB );
		calBoundryModel( &m_vNetworkForced[i].m_cDB );
		//calBoundryModel_ver2( &m_vNetworkForced[i].m_cDB );
		m_vBoundry.push_back( &m_vNetworkForced[i].m_cDB );
	}
	/*
	for( int i=0; i< m_vNetForced.size(); i++ )
	{
		calNetForcedModel( m_vNetForced[i] );
	}
	*/
	//for( int i=0; i<m_vNetForced.size(); i++ )
	//{
	//	calNetForcedModel( m_vNetForced[i] );
	//cout<<m_vNetForced[i].m_pInstance->getName()<<" ";
	//cout<<m_vNetForced[i].m_pInstance->getPlacedX()<<" "<<m_vNetForced[i].m_pInstance->getPlacedY()<<endl;
	//}
	//readLUT();
	// check information
	/*
	cout<<"Check net information"<<endl;
	vector< net_C* > vNet = m_pDesign->getNet();
	for( int i=0; i<vNet.size(); i++ )
	{
		//estimateHPWLwithoutLayer( vNet[i] );
		
		estimateHPWL( vNet[i] );
		calWireLength( vNet[i] );
		cout<<endl;
	}
	cout<<endl;
	cout<<"Check Networl model"<<endl;
	for( int i=0; i<m_vNetworkForced.size(); i++ )
	{
		networkForced_C &cNF = m_vNetworkForced[i];
		cout<<cNF.m_pNet->getName()<<endl;
		cout<<"X: "<<cNF.m_nCenterX<<"."<<cNF.m_nXHalf<<" Y: "<<cNF.m_nCenterY<<"."<<cNF.m_nYHalf<<endl;
	}
	cout<<endl;

	cout<<"Check forced model"<<endl;
	for( int i=0; i<m_vForced.size(); i++ )
	{
		forced_C &cF = m_vForced[i];
		cout<<cF.m_pInstance->getName()<<endl;
		cout<<"T: "<<cF.m_nT<<" D: "<<cF.m_nD<<" R: "<<cF.m_nR<<" L: "<<cF.m_nL<<endl;
	}
	cout<<endl;
	*/
	//pickInstanceToMove();
}

bool router_C::create2DGraph()
{
	vector< int > vIntGraph;
	vector< bool > vBoolGraph;
	for( int x = m_nDX; x<= m_nTX; x++ )
	{
		vIntGraph.push_back( 0 );
		vBoolGraph.push_back( false );
	}
	for( int y = m_nDY; y<= m_nTY; y++ )
	{
		m_v2DGraph.push_back( vBoolGraph );
		m_vTargetGraph.push_back( vIntGraph );
	}
	return true;
}

bool router_C::createRoutingGraph()
{
	for (int z = m_nDZ; z <= m_nTZ; z++)
	{
		vector<vector<rGrid_C *>> vMap;
		for (int y = m_nDY; y <= m_nTY; y++)
		{
			vector<rGrid_C *> vRow;
			for (int x = m_nDX; x <= m_nTX; x++)
			{
				rGrid_C *pRGrid = new rGrid_C;
				pRGrid->m_nX = x;
				pRGrid->m_nY = y;
				pRGrid->m_nZ = z;
				pRGrid->m_nH = 0;
				pRGrid->m_nMH = 0;
				pRGrid->m_nG = 0;
				pRGrid->m_nCost = 0;
				pRGrid->m_nBend = 0;
				pRGrid->m_pFrom = NULL;
				pRGrid->isRouted = false;
				pRGrid->isPath = false;
				pRGrid->isTarget = false;
				pRGrid->isObstacle = false;
				pRGrid->m_cDir = m_pDesign->getLayer()[z - m_nDZ]->getDir();
				vRow.push_back(pRGrid);
			}
			vMap.push_back(vRow);
		}
		m_vRoutingGraph.push_back(vMap);
	}
	return true;
}

int router_C::calTotalWireLength()
{
	int nLength = 0;
	for( int i=0; i<m_vNetworkForced.size(); i++ )
	{
		nLength = nLength + m_vNetworkForced[i].m_pNet->getLength();
	}
	return nLength;
}

_history router_C::getHGrid(int nX, int nY, int nZ)
{
	nX = nX - m_nOffsetX;
	nY = nY - m_nOffsetY;
	nZ = nZ - m_nOffsetZ;
	return m_vHistoryGraph[nZ][nY][nZ];
}

inline rGrid_C *router_C::getRGrid(int nX, int nY, int nZ)
{
	nX = nX - m_nOffsetX;
	nY = nY - m_nOffsetY;
	nZ = nZ - m_nOffsetZ;
	// added at 07/14 20:30
	if (nX < 0 || nX >= m_vRoutingGraph[0][0].size())
		return NULL;
	if (nY < 0 || nY >= m_vRoutingGraph[0].size())
		return NULL;
	if (nZ < 0 || nZ >= m_vRoutingGraph.size())
		return NULL;
	// end added at 07/14 20:30
	return m_vRoutingGraph[nZ][nY][nX];
}

bool router_C::loadDesign()
{
	m_pDesign->getGraph(m_vGraph);
	vector<layer_C *> vLayer = m_pDesign->getLayer();
	m_pDesign->getBoundary(m_nDX, m_nDY, m_nTX, m_nTY);
	m_nDZ = vLayer.front()->getId();
	m_nTZ = vLayer.back()->getId();

	m_nOffsetX = m_nDX;
	m_nOffsetY = m_nDY;
	m_nOffsetZ = m_nDZ;
	return true;
}

bool router_C::createHistoryGraph()
{
	for (int z = m_nDZ; z <= m_nTZ; z++)
	{
		vector<vector<_history>> vHMap;
		for (int y = m_nDY; y <= m_nTY; y++)
		{
			vector<_history> vHRow;
			for (int x = m_nDX; x <= m_nTX; x++)
			{
				_history nH = 0;
				vHRow.push_back(nH);
			}
			vHMap.push_back(vHRow);
		}
		m_vHistoryGraph.push_back(vHMap);
	}
	return true;
}

int router_C::calWireLength(net_C *pNet)
{
	vector<gGrid_C *> vGrid;
	vector<wire_C *> vSegment;
	vSegment = pNet->getWire();
	//cout<<"test"<<endl;
	int nLength = 0;
	for (int i = 0; i < vSegment.size(); i++)
	{
		wire_C *pWire = vSegment[i];
		gGrid_C *pGrid1 = pWire->getGrid1();
		gGrid_C *pGrid2 = pWire->getGrid2();
		net_C *pNet = pWire->getNet();

		int nX1, nY1, nZ1;
		int nX2, nY2, nZ2;
		pGrid1->getPosition(nX1, nY1, nZ1);
		pGrid2->getPosition(nX2, nY2, nZ2);
		int ndX, ndY, ndZ;
		ndX = nX2 - nX1;
		ndY = nY2 - nY1;
		ndZ = nZ2 - nZ1;

		//if( pNet->getName() == "N1482" )
		//{
		//	cout<<"("<<nX1<<" "<<nY1<<" "<<nZ1<<") "<<"("<<nX2<<" "<<nY2<<" "<<nZ2<<") "<<endl;
		//}
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
			pCurGrid = graphTravel(m_pDesign, pCurGrid, ndX, ndY, ndZ);
			if (!pCurGrid->isRouted())
			{
				pCurGrid->setRouted();
				vGrid.push_back(pCurGrid);
			}
		}
	}
	//if( pNet->getName() == "N1482" )
	//{
	//	cout<<"Cal Length: "<<vGrid.size()<<endl;
	//}

	for (int g = 0; g < vGrid.size(); g++)
	{
		vGrid[g]->setUnrouted();
	}
	return vGrid.size();
}

int router_C::estimateHPWLwithoutLayer(net_C *pNet)
{
	vector<pin_C *> vPin = pNet->getPin();
	int nMinX = INT_MAX;
	int nMaxX = INT_MIN;
	int nMinY = INT_MAX;
	int nMaxY = INT_MIN;
	for (int i = 0; i < vPin.size(); i++)
	{
		pin_C *pPin = vPin[i];
		int nX, nY, nZ;
		instance_C *pInst = (instance_C *)pPin->getCell();
		nX = pInst->getPlacedX();
		nY = pInst->getPlacedY();
		nZ = pPin->getLayerId();
		nMinX = min(nMinX, nX);
		nMaxX = max(nMaxX, nX);
		nMinY = min(nMinY, nY);
		nMaxY = max(nMaxY, nY);
	}
	int nHPWL = 0;
	vector<pin_C *> vInnerPin;
	vector<pin_C *> vBoundPin;

	for (int i = 0; i < vPin.size(); i++)
	{
		pin_C *pPin = vPin[i];
		int nX, nY;
		instance_C *pInst = (instance_C *)pPin->getCell();
		nX = pInst->getPlacedX();
		nY = pInst->getPlacedY();
		if (nX - nMaxX == 0 || nX - nMinX == 0 || nY - nMinY == 0 || nY - nMaxY == 0)
			vBoundPin.push_back(pPin);
		else
			vInnerPin.push_back(pPin);
	}

	/*
	for( int i=0; i<vPin.size(); i++ )
	{
		pin_C* pPin = vPin[i];
		int nX, nY, nZ;
		instance_C* pInst = (instance_C*)pPin->getCell();
		nX = pInst->getPlacedX();
		nY = pInst->getPlacedY();
		nZ = pPin->getLayerId();
		nHPWL = nHPWL + (nMaxX - nX) + (nX - nMinX) + (nMaxY - nY) + (nY - nMinY) + (nMaxZ - nZ) + (nZ - nMinZ);
	}
	*/

	for (int i = 0; i < vInnerPin.size(); i++)
	{
		pin_C *pInnerPin = vInnerPin[i];
		int nInX, nInY;
		instance_C *pInst = (instance_C *)pInnerPin->getCell();
		nInX = pInst->getPlacedX();
		nInY = pInst->getPlacedY();
		for (int j = 0; j < vBoundPin.size(); j++)
		{

			pin_C *pBoundPin = vBoundPin[j];
			int nBoX, nBoY;
			instance_C *pInst = (instance_C *)pBoundPin->getCell();
			nBoX = pInst->getPlacedX();
			nBoY = pInst->getPlacedY();

			nHPWL = nHPWL + abs(nInX - nBoX) + abs(nInY - nBoY) + 1;
		}
		//nHPWL++;
	}

	for (int i = 0; i < vBoundPin.size() - 1; i++)
	{
		pin_C *pBoundPin1 = vBoundPin[i];
		int nBoX1, nBoY1;
		instance_C *pInst = (instance_C *)pBoundPin1->getCell();
		nBoX1 = pInst->getPlacedX();
		nBoY1 = pInst->getPlacedY();
		for (int j = i + 1; j < vBoundPin.size(); j++)
		{

			pin_C *pBoundPin2 = vBoundPin[j];
			int nBoX2, nBoY2;
			instance_C *pInst = (instance_C *)pBoundPin2->getCell();
			nBoX2 = pInst->getPlacedX();
			nBoY2 = pInst->getPlacedY();

			nHPWL = nHPWL + abs(nBoX1 - nBoX2) + abs(nBoY1 - nBoY2) + 1;
		}
		//nHPWL++;
	}

	if (vPin.size() > 0)
	{
		nHPWL = nHPWL - (vPin.size() - 2) * vBoundPin.size() - (vBoundPin.size() - 1) * vInnerPin.size();
		cout << "Net " << pNet->getName() << " estimated HPWL = " << nHPWL * 2 / vPin.size() << endl;
		return nHPWL * 2 / vPin.size();
	}
	else
	{
		cout << "Net " << pNet->getName() << " has no pin!!" << endl;
		return 0;
	}
}

int router_C::estimateHPWL(net_C *pNet)
{
	vector<pin_C *> vPin = pNet->getPin();
	int nMinX = INT_MAX;
	int nMaxX = INT_MIN;
	int nMinY = INT_MAX;
	int nMaxY = INT_MIN;
	int nMinZ = INT_MAX;
	int nMaxZ = INT_MIN;
	for (int i = 0; i < vPin.size(); i++)
	{
		pin_C *pPin = vPin[i];
		int nX, nY, nZ;
		instance_C *pInst = (instance_C *)pPin->getCell();
		nX = pInst->getPlacedX();
		nY = pInst->getPlacedY();
		nZ = pPin->getLayerId();
		nMinX = min(nMinX, nX);
		nMaxX = max(nMaxX, nX);
		nMinY = min(nMinY, nY);
		nMaxY = max(nMaxY, nY);
		nMinZ = min(nMinZ, nZ);
		nMaxZ = max(nMaxZ, nZ);
	}
	int nHPWL = 0;
	vector<pin_C *> vInnerPin;
	vector<pin_C *> vBoundPin;

	for (int i = 0; i < vPin.size(); i++)
	{
		pin_C *pPin = vPin[i];
		int nX, nY, nZ;
		instance_C *pInst = (instance_C *)pPin->getCell();
		nX = pInst->getPlacedX();
		nY = pInst->getPlacedY();
		nZ = pPin->getLayerId();
		if (nX - nMaxX == 0 || nX - nMinX == 0 || nY - nMinY == 0 || nY - nMaxY == 0 || nZ - nMinZ == 0 || nZ - nMaxZ == 0)
			vBoundPin.push_back(pPin);
		else
			vInnerPin.push_back(pPin);
	}

	/*
	for( int i=0; i<vPin.size(); i++ )
	{
		pin_C* pPin = vPin[i];
		int nX, nY, nZ;
		instance_C* pInst = (instance_C*)pPin->getCell();
		nX = pInst->getPlacedX();
		nY = pInst->getPlacedY();
		nZ = pPin->getLayerId();
		nHPWL = nHPWL + (nMaxX - nX) + (nX - nMinX) + (nMaxY - nY) + (nY - nMinY) + (nMaxZ - nZ) + (nZ - nMinZ);
	}
	*/

	for (int i = 0; i < vInnerPin.size(); i++)
	{
		pin_C *pInnerPin = vInnerPin[i];
		int nInX, nInY, nInZ;
		instance_C *pInst = (instance_C *)pInnerPin->getCell();
		nInX = pInst->getPlacedX();
		nInY = pInst->getPlacedY();
		nInZ = pInnerPin->getLayerId();
		for (int j = 0; j < vBoundPin.size(); j++)
		{

			pin_C *pBoundPin = vBoundPin[j];
			int nBoX, nBoY, nBoZ;
			instance_C *pInst = (instance_C *)pBoundPin->getCell();
			nBoX = pInst->getPlacedX();
			nBoY = pInst->getPlacedY();
			nBoZ = pBoundPin->getLayerId();

			nHPWL = nHPWL + abs(nInX - nBoX) + abs(nInY - nBoY) + abs(nInZ - nBoZ) + 1;
		}
		//nHPWL++;
	}

	for (int i = 0; i < vBoundPin.size() - 1; i++)
	{
		pin_C *pBoundPin1 = vBoundPin[i];
		int nBoX1, nBoY1, nBoZ1;
		instance_C *pInst = (instance_C *)pBoundPin1->getCell();
		nBoX1 = pInst->getPlacedX();
		nBoY1 = pInst->getPlacedY();
		nBoZ1 = pBoundPin1->getLayerId();
		for (int j = i + 1; j < vBoundPin.size(); j++)
		{

			pin_C *pBoundPin2 = vBoundPin[j];
			int nBoX2, nBoY2, nBoZ2;
			instance_C *pInst = (instance_C *)pBoundPin2->getCell();
			nBoX2 = pInst->getPlacedX();
			nBoY2 = pInst->getPlacedY();
			nBoZ2 = pBoundPin2->getLayerId();

			nHPWL = nHPWL + abs(nBoX1 - nBoX2) + abs(nBoY1 - nBoY2) + abs(nBoZ1 - nBoZ2) + 1;
		}
		//nHPWL++;
	}

	if (vPin.size() > 0)
	{
		nHPWL = nHPWL - (vPin.size() - 2) * vBoundPin.size() - (vBoundPin.size() - 1) * vInnerPin.size();
		cout << "Net " << pNet->getName() << " estimated HPWL = " << nHPWL * 2 / vPin.size() << endl;
		return nHPWL * 2 / vPin.size();
	}
	else
	{
		cout << "Net " << pNet->getName() << " has no pin!!" << endl;
		return 0;
	}
}

inline bool router_C::removeInstOnGraph(instance_C *pInst)
{
	// delete the block demand
	// remove the instance
	// recalculate the demand caused by neighbor
	//cout << "Remove " << pInst->getName() << " at " << pInst->getPlacedX() << " " << pInst->getPlacedY() << endl;
	delCellOnGraph(m_pDesign, pInst);
	if (!pInst->isConstraintCell())
		return true;
	int nX = pInst->getPlacedX();
	int nY = pInst->getPlacedY();
	int nZ = m_nOffsetZ;
	gGrid_C *pGrid = getGrid(m_pDesign, nX, nY, nZ);
	removeNeighborCellDemand(m_pDesign, pGrid, pInst);
	// //#pragma omp parallel sections
	// //{
	// //#pragma omp section
	// //{
	// calNeighborCellDemand(m_pDesign, pGrid);
	// //}
	// //gGrid_C* pNGrid;
	// //#pragma omp section
	// //{
	// gGrid_C *pTGrid = graphTravel(m_pDesign, pGrid, 0, 1, 0);
	// if (pTGrid != NULL)
	// 	calNeighborCellDemand(m_pDesign, pTGrid);
	// //}
	// //#pragma omp section
	// //{
	// gGrid_C *pDGrid = graphTravel(m_pDesign, pGrid, 0, -1, 0);
	// if (pDGrid != NULL)
	// 	calNeighborCellDemand(m_pDesign, pDGrid);
	// //}
	// //}
	return true;
}

inline bool router_C::resetPseudoPinDemand( instance_C* pInst, int nX, int nY, int nZ )
{
	int nOrigX = pInst->getPlacedX();
	int nOrigY = pInst->getPlacedY();
	int nNumLayer = m_pDesign->getLayer().size();
	for( int i=0; i<nNumLayer; i++ )
	{
		gGrid_C* gG = getGrid( m_pDesign, nOrigX, nOrigY, i+1 );
		gG->resetPinDemand();
	}


}

inline bool router_C::delPseudoPinDemand( instance_C* pInst, int nX, int nY, int nZ )
{	
	//cout << "Del Pse" << endl;
	int nOrigX = pInst->getPlacedX();
	int nOrigY = pInst->getPlacedY();
	vector< gGrid_C* > vZGrid;
	int nNumLayer = m_pDesign->getLayer().size();
	for( int i=0; i<nNumLayer; i++ )
	{
		vZGrid.push_back( getGrid( m_pDesign, nOrigX, nOrigY, i+1 ) );
	}
	
	vector<pin_C> vPin = pInst->getPin();
	for (int i = 0; i < vPin.size(); i++)
	{
		pin_C *pPin = &vPin[i];
		if( pPin->getNet() == NULL )
			continue;
		int nLayerId = pPin->getLayerId();
//			
		int nLayerConstraint = nLayerId;
		if( pPin->getNet() != NULL )
		{
			nLayerConstraint = max( nLayerId, pPin->getNet()->getConstraintLayerId() );
//
		//vZGrid[nLayerId - nZ]->delPinDemand();
			for( int n=nLayerId; n<=nLayerConstraint; n++ )
				vZGrid[n - nZ]->delPinDemand( pPin->getNet() );
			
		}
	}
	//cout<< "end del"<<endl;
	return true;
}

inline vector< gGrid_C* > router_C::addPseudoPinDemand( net_C* pNet, set< instance_C* > &sUnplacedInst )
{
	//cout << "Add Pse" <<endl;
	vector< gGrid_C* > vGrid;
	vector< pin_C* > vPin = pNet->getPin();
	for( int i=0; i<vPin.size(); i++ )
	{
		pin_C* pPin = vPin[i];
		instance_C* pInst = (instance_C*)pPin->getCell();	
		int nLayer = pPin->getLayerId();
//
		int nLayerConstraint = nLayer;
		if( pPin->getNet() != NULL )
			nLayerConstraint = max( nLayer, pPin->getNet()->getConstraintLayerId() );
//		
		
		if( sUnplacedInst.count( pInst ) == 0 )
		{
		/*
			gGrid_C* pGrid = getGrid( m_pDesign, pInst->getPlacedX(), pInst->getPlacedY(), nLayer );	
			pGrid->addPinDemand();
			vGrid.push_back( pGrid );
		*/
			for( int n=nLayer; n<=nLayerConstraint; n++ )	
			{
				gGrid_C* pGrid = getGrid( m_pDesign, pInst->getPlacedX(), pInst->getPlacedY(), n );	
				pGrid->addPinDemand();
				vGrid.push_back( pGrid );
			}
		}
	}
	//cout << "end add"<<endl;
	return vGrid;
}

inline bool router_C::calPseudoPinDemand( instance_C* pInst, int nX, int nY, int nZ )
{	
	//cout << "Cal Pse"<<endl;
	//cout << pInst->getName() << endl;
	int nOrigX = pInst->getPlacedX();
	int nOrigY = pInst->getPlacedY();
	vector< gGrid_C* > vZGrid;
	int nNumLayer = m_pDesign->getLayer().size();
	for( int i=0; i<nNumLayer; i++ )
	{
		vZGrid.push_back( getGrid( m_pDesign, nOrigX, nOrigY, i+1 ) );
	}

	vector<pin_C> vPin = pInst->getPin();
	for (int i = 0; i < vPin.size(); i++)
	{
		pin_C *pPin = &vPin[i];
		if( pPin->getNet() == NULL )
			continue;
	//	cout << pPin->getName() << endl;
		int nLayerId = pPin->getLayerId();
//
		int nLayerConstraint = nLayerId;
		if( pPin->getNet() != NULL )
		{
			nLayerConstraint = max( nLayerId, pPin->getNet()->getConstraintLayerId() );
		//cout << pPin->getNet()->getConstraintLayerId() << endl;
//		

//		vZGrid[nLayerId - nZ]->addPinDemand();
			for( int n=nLayerId; n<=nLayerConstraint; n++ )
				vZGrid[ n - nZ ]->addPinDemand( pPin->getNet() );
		}
	}
	//cout << "end cal"<<endl;
	/*
	if( pInst->getName() == "C1112" )
	for( int i=0; i<vZGrid.size(); i++ )
		cout <<vZGrid[i]->getPinDemand() << " ";
	cout << endl;
	*/
}

inline bool router_C::putInstOnGraph(instance_C *pInst, int nX, int nY, int nZ)
{
	/*
	string strInstName = pInst->getName();
	string strInfo = "Put " + strInstName + " on graph";
	cout<<setw(COUTWIDTH)<<left<<setfill('.')<<strInfo;
	*/
	// put the instance on graph
	// add the instance demand on grid
	// recalculate the demand caused by neighbor
	//cout << "Put " << pInst->getName() << " at " << nX << " " << nY << endl;
	pInst->setPlaced(nX, nY);
	addCellOnGraph(m_pDesign, pInst);
	if (!pInst->isConstraintCell())
		return true;
	nZ = m_nOffsetZ;
	gGrid_C *pGrid = getGrid(m_pDesign, nX, nY, nZ);
	addNeighborCellDemand(m_pDesign, pGrid, pInst);

	// calNeighborCellDemand(m_pDesign, pGrid);

	// gGrid_C *pTGrid = graphTravel(m_pDesign, pGrid, 0, 1, 0);
	// if (pTGrid != NULL)
	// 	calNeighborCellDemand(m_pDesign, pTGrid);
	// //	addNeighborCellDemand( m_pDesign, pNGrid, pInst );

	// gGrid_C *pDGrid = graphTravel(m_pDesign, pGrid, 0, -1, 0);
	// if (pDGrid != NULL)
	// 	calNeighborCellDemand(m_pDesign, pDGrid);
	// //	addNeighborCellDemand( m_pDesign, pNGrid, pInst );

	// //	cout<<endl;
	return true;
}

inline bool router_C::backupInstance(instance_C *pInst)
{
	//version 1
	m_cBackupInstance.setPlaced(pInst->getPlacedX(), pInst->getPlacedY());

	//version 2
	/*
	instance_C cBackupInstance;
	cBackupInstance.setName( pInst->getName() );
	cBackupInstance.setPlaced( pInst->getPlacedX(), pInst->getPlacedY() );
	m_vBackupInstance.push_back( cBackupInstance );
	*/
	return true;
}

inline bool router_C::recoverInstance(instance_C *pInst)
{
	//version 1
	pInst->setPlaced(m_cBackupInstance.getPlacedX(), m_cBackupInstance.getPlacedY());
	//if( pInst->getName() == "C1" )
	//	cout << "C1: "<<pInst->getPlacedX() <<", "<<pInst->getPlacedY() << endl;
	//version 2
	/*
	for( int i=0; i<m_vBackupInstance.size(); i++ )
	{
		if( m_vBackupInstance[i].getName() == pInst->getName() )
		{
			pInst->setPlaced( m_vBackupInstance[i].getPlacedX(), m_vBackupInstance[i].getPlacedY() );
			break;
		}
	}
	*/
	return true;
}

inline bool router_C::backupNet(vector<net_C *> &vNet)
{
	for (int i = 0; i < vNet.size(); i++)
	{
		net_C cNet;
		net_C *pNet = vNet[i];
		vector<wire_C *> vWire = pNet->getWire();
		cNet.setName(pNet->getName());
		for (int j = 0; j < vWire.size(); j++)
		{
			wire_C *pWire = vWire[j];
			wire_C *pNewWire = new wire_C;
			//pNewWire->setId( pWire->getId() );
			pNewWire->setGrid1(pWire->getGrid1());
			pNewWire->setGrid2(pWire->getGrid2());
			pNewWire->setNet(pWire->getNet());
			cNet.addWire(pNewWire);
		}
		cNet.setLength(pNet->getLength());
		m_vBackupNet.push_back(cNet);
	}
}

inline bool router_C::backupNet( vector< net_C > &vBackupNet, net_C* pNet )
{
	net_C cNet;
	vector<wire_C *> vWire = pNet->getWire();
	cNet.setName(pNet->getName());
	for (int j = 0; j < vWire.size(); j++)
	{
		wire_C *pWire = vWire[j];
		wire_C *pNewWire = new wire_C;
		//pNewWire->setId( pWire->getId() );
		pNewWire->setGrid1(pWire->getGrid1());
		pNewWire->setGrid2(pWire->getGrid2());
		pNewWire->setNet(pWire->getNet());
		cNet.addWire(pNewWire);
	}
	cNet.setLength(pNet->getLength());
	vBackupNet.push_back(cNet);
}


inline bool router_C::backupNet( net_C* pNet )
{
	net_C cNet;
	vector<wire_C *> vWire = pNet->getWire();
	cNet.setName(pNet->getName());
	for (int j = 0; j < vWire.size(); j++)
	{
		wire_C *pWire = vWire[j];
		wire_C *pNewWire = new wire_C;
		//pNewWire->setId( pWire->getId() );
		pNewWire->setGrid1(pWire->getGrid1());
		pNewWire->setGrid2(pWire->getGrid2());
		pNewWire->setNet(pWire->getNet());
		cNet.addWire(pNewWire);
	}
	cNet.setLength(pNet->getLength());
	m_vBackupNet.push_back(cNet);
}

inline bool router_C::recoverNet( vector< net_C > &vBackupNet, net_C * pNet)
{ 
	bool bFind = false;
	for (int j = 0; j < vBackupNet.size(); j++)
	{
		net_C &cBNet = vBackupNet[j];
		if( pNet->getName() == cBNet.getName() )
		{
			bFind = true;
			pNet->cleanWire();
			vector<wire_C *> vBWire = cBNet.getWire();
			for (int k = 0; k < vBWire.size(); k++)
			{
				pNet->addWire(vBWire[k]);
			}
			pNet->setLength(cBNet.getLength());
			//if( pNet->getName() == "N4" )
			//	cout << vBWire.size() << endl;
			break;
		}
	}
	//if( pNet->getName() == "N4" )
	//	cout << "N4" << pNet->getWire().size() << endl;
	if (!bFind)
		cout << "Error: Net" << pNet->getName() << " failed to recover" << endl;

	//m_vBackupNet.clear();
	return bFind;
}

inline bool router_C::recoverNet( vector< net_C > &vBackupNet, vector<net_C *> &vNet)
{ 
	bool bFind = false;
	for (int i = 0; i < vNet.size(); i++)
	{
		bFind = false;
		net_C *pNet = vNet[i];
		//if( pNet->getName() == "N4" )
		//	cout << "HAHAHAHAHA" << endl;
		for (int j = 0; j < vBackupNet.size(); j++)
		{
			net_C &cBNet = vBackupNet[j];
			if( pNet->getName() == cBNet.getName() )
			{
				bFind = true;
				pNet->cleanWire();
				vector<wire_C *> vBWire = cBNet.getWire();
				for (int k = 0; k < vBWire.size(); k++)
				{
					pNet->addWire(vBWire[k]);
				}
				pNet->setLength(cBNet.getLength());
				//if( pNet->getName() == "N4" )
				//	cout << vBWire.size() << endl;
				break;
			}
		}
		//if( pNet->getName() == "N4" )
		//	cout << "N4" << pNet->getWire().size() << endl;
		if (!bFind)
			cout << "Error: Net" << pNet->getName() << " failed to recover" << endl;
	}

	//m_vBackupNet.clear();
	return bFind;
}


inline bool router_C::recoverNet(vector<net_C *> &vNet)
{
	bool bFind = false;
	for (int i = 0; i < vNet.size(); i++)
	{
		bFind = false;
		net_C *pNet = vNet[i];
		//if( pNet->getName() == "N4" )
		//	cout << "HAHAHAHAHA" << endl;
		for (int j = 0; j < m_vBackupNet.size(); j++)
		{
			net_C &cBNet = m_vBackupNet[j];
			if( pNet->getName() == cBNet.getName() )
			{
				bFind = true;
				pNet->cleanWire();
				vector<wire_C *> vBWire = cBNet.getWire();
				for (int k = 0; k < vBWire.size(); k++)
				{
					pNet->addWire(vBWire[k]);
				}
				pNet->setLength(cBNet.getLength());
				//if( pNet->getName() == "N4" )
				//	cout << vBWire.size() << endl;
				break;
			}
		}
		//if( pNet->getName() == "N4" )
		//	cout << "N4" << pNet->getWire().size() << endl;
		if (!bFind)
			cout << "Error: Net" << pNet->getName() << " failed to recover" << endl;
	}

	//m_vBackupNet.clear();
	return bFind;
}

bool router_C::calForceDirection(vector<instance_C *> &vInst)
{
}

bool router_C::cleanWire(vector<net_C *> &vNet)
{
	for (int i = 0; i < vNet.size(); i++)
		vNet[i]->cleanWire();
	return true;
}

bool router_C::ripupNet(vector<net_C *> &vNet)
{
	// store the grid that the net pass in the vector
	// decrease the demand grid stored in vector
	// remove the net on the grid
	for (int i = 0; i < vNet.size(); i++)
	{
		net_C *pNet = vNet[i];
		//cout<<pNet->getName()<<endl;
		delNetOnGraph(m_pDesign, pNet);
		//cout<<"complete"<<endl;
	}
	return true;
}

bool router_C::ripupNet(net_C *pNet)
{
	delNetOnGraph(m_pDesign, pNet);
	return true;
}

vector<gGrid_C *> router_C::routeNet(gGrid_C *pSource, gGrid_C *pTarget)
{
}

vector<gGrid_C *> router_C::routeNet(vector<gGrid_C *> &vSource, vector<gGrid_C *> &vTarget)
{
}

vector<gGrid_C *> router_C::routeNet(net_C *pNet, vector<gGrid_C *> &vObs)
{
	setRoutingConstraint(pNet);
	int nLConstraint = m_nConstraintLayerId;
	//cout<<nLConstraint<<endl;

	vector<gGrid_C *> vTerminal;
	vector<pin_C *> vPin = pNet->getPin();
	//vector< wire_C* > vWire;
	vector<gGrid_C *> vResult;

	for (int i = 0; i < vPin.size(); i++)
	{
		pin_C *pPin = vPin[i];
		int nX, nY, nZ;
		instance_C *pInst = (instance_C *)pPin->getCell();
		nX = pInst->getPlacedX();
		nY = pInst->getPlacedY();
		// project the z to the constraint layer
		nZ = pPin->getLayerId();
		if (nZ < nLConstraint)
		{
			for (int l = nZ; l <= nLConstraint; l++)
			{
				gGrid_C *pGrid = getGrid(m_pDesign, nX, nY, l);
				vResult.push_back(pGrid);
			}
			nZ = nLConstraint;
		}

		gGrid_C *pTerminal = getGrid(m_pDesign, nX, nY, nZ);
		rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
		pRGrid->isTarget = true;
		vTerminal.push_back(pTerminal);
	}

	// check same position of terminal
	//cout<<"Check terminal"<<endl;
	for (int i = 0; i < vTerminal.size(); i++)
	{
		for (int j = i + 1; j < vTerminal.size(); j++)
		{
			if (vTerminal[i] == vTerminal[j])
			{
				vTerminal.erase(vTerminal.begin() + j);
				j--;
			}
		}
	}
	//cout<<"Num of terminals: "<<vTerminal.size()<<endl;
	if (vTerminal.size() == 1)
	{
		if (vTerminal[0]->getRemand() <= 0)
		{

			int nX, nY, nZ;
			vTerminal[0]->getPosition(nX, nY, nZ);
			rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
			pRGrid->isTarget = false;
			return vResult;
		}
		int nX, nY, nZ;
		vTerminal[0]->getPosition(nX, nY, nZ);
		rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
		pRGrid->isTarget = false;

		bool bTisObs = false;
		for (int i = 0; i < vObs.size(); i++)
		{
			if (vObs[i] == vTerminal[0])
			{
				bTisObs = true;
				break;
			}
		}

		if (bTisObs)
			return vResult;
		else
		{
			vResult.push_back(vTerminal[0]);
			return vResult;
		}
	}
	else
	{
		bool bFindOverflow = false;
		for (int i = 0; i < vTerminal.size(); i++)
		{
			if (vTerminal[i]->getRemand() <= 0)
			{
				bFindOverflow = true;
				break;
			}
		}

		if (bFindOverflow)
		{
			for (int i = 0; i < vTerminal.size(); i++)
			{
				int nX, nY, nZ;
				vTerminal[i]->getPosition(nX, nY, nZ);
				rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
				pRGrid->isTarget = false;
			}
			if (vResult.size() != 0)
				vResult.clear();

			return vResult;
		}
	}

	vector<rGrid_C *> vRoutedGrid;
	//vector< rGrid_C* > vPriorityQueue;
	vector<rGrid_C *> vHistory;
	vector<rGrid_C *> vNotInPath;
	// find first terminal
	gGrid_C *pFirst;
	int nMinX = m_nTX;
	for (int i = 0; i < vTerminal.size(); i++)
	{
		gGrid_C *pGrid = vTerminal[i];
		int nX, nY, nZ;
		pGrid->getPosition(nX, nY, nZ);
		if (nX <= nMinX)
			nMinX = nX;
	}

	int nMinY = m_nTY;
	for (int i = 0; i < vTerminal.size(); i++)
	{
		gGrid_C *pGrid = vTerminal[i];
		int nX, nY, nZ;
		pGrid->getPosition(nX, nY, nZ);
		if (nX == nMinX && nY <= nMinY)
		{
			nMinY = nY;
			pFirst = vTerminal[i];
		}
	}
	//cout<<"Here"<<endl;
	int nX, nY, nZ;
	pFirst->getPosition(nX, nY, nZ);
	rGrid_C *pCurRGrid = getRGrid(nX, nY, nZ);
	gGrid_C *pGGrid = pFirst;
	if (pGGrid->getRemand() <= 0)
	{
		pCurRGrid->isTarget = false;
		vResult.clear();
		return vResult;
	}

	vHistory.push_back(pCurRGrid);
	pCurRGrid->isPath = true;
	for (int i = 0; i < vTerminal.size(); i++)
	{
		if (vTerminal[i] == pGGrid)
		{
			vTerminal.erase(vTerminal.begin() + i);
			break;
		}
	}
	int nNumTermanals = vTerminal.size();
	//nNumTermanals--; // because the first grid is a termianl
	vector<layer_C *> vLayer = m_pDesign->getLayer();
	//cout<<"Start routing"<<endl;

	// setting obstacle
	for (int i = 0; i < vObs.size(); i++)
	{
		int nX, nY, nZ;
		gGrid_C *pGrid = vObs[i];
		pGrid->getPosition(nX, nY, nZ);
		rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
		pRGrid->isObstacle = true;
	}

	vector<rGrid_C *> vHaveAddTarget;
	while (nNumTermanals > 0)
	{
		//cout<<nNumTermanals<<endl;
		//cout<<pCurRGrid->m_nX<<" "<<pCurRGrid->m_nY<<" "<<pCurRGrid->m_nZ<<endl;
		// A* propogation
		if (vHistory.size() == 0)
		{
			vResult.clear();
			break;
		}

		cout << vHistory.size() << endl;

		vector<rGrid_C *> vToPropogate;
		int nMinCost = 0;
		bool bPick = false;
		for (int i = 0; i < vHistory.size(); i++)
		{
			if (!vHistory[i]->isRouted)
			{
				nMinCost = vHistory[i]->m_nCost;
				bPick = true;
				break;
			}
		}
		if (!bPick)
		{
			vResult.clear();
			//cout<<"Can't find result"<<endl;
			break;
		}
		/*
		cout<<nMinCost<<" "<<vHistory.size()<<endl;
		for( int i=0; i<vHistory.size(); i++ )
		{
			if( !vHistory[i]->isRouted )	
			{
				cout << vHistory[i]->m_nCost <<" ";
			}
		}
		//cout<<endl;
		*/
		for (int i = 0; i < vHistory.size(); i++)
		{
			if (!vHistory[i]->isRouted)
			{
				if (vHistory[i]->m_nCost == nMinCost)
				{
					vToPropogate.push_back(vHistory[i]);
				}
				else
					break;
			}
		}

		vector<rGrid_C *> vAddGrid;

		for (int i = 0; i < vToPropogate.size(); i++)
		{
			pCurRGrid = vToPropogate[i];
			pCurRGrid->isRouted = true;
			int nCurDistance = pCurRGrid->m_nG;
			int nNewDistance = nCurDistance + 1;
			int nX, nY, nZ;
			nX = pCurRGrid->m_nX;
			nY = pCurRGrid->m_nY;
			nZ = pCurRGrid->m_nZ;
			//cout<<"P: "<<nX<<" "<<nY<<" "<<nZ<<endl;
			pGGrid = getGrid(m_pDesign, nX, nY, nZ);
			char cDir = vLayer[nZ - m_nOffsetZ]->getDir();
			//cout<<cDir<<endl;
			int nDX, nDY, nDZ;
			if (cDir == 'H')
			{
				nDX = 0;
				nDY = 1;
				nDZ = 0;
			}
			else if (cDir == 'V')
			{
				nDX = 1;
				nDY = 0;
				nDZ = 0;
			}
			else
				cout << "Unknown routing direction" << endl;
			// go in the same layer;
			gGrid_C *pNextGrid = graphTravel(m_pDesign, pGGrid, nDX, nDY, nDZ);
			if (pNextGrid != NULL && pNextGrid->getRemand())
			{
				rGrid_C *pNextRGrid = getRGrid(nX + nDX, nY + nDY, nZ + nDZ);
				if (!pNextRGrid->isRouted && !pNextRGrid->isObstacle)
				{
					//pNextRGrid->isRouted = true;
					layer_C *pLayer = vLayer[nZ + nDZ - m_nOffsetZ];
					pNextRGrid->m_nG = nNewDistance;
					int nSumH = 0;
					//for( int t=0; t<vTerminal.size(); t++ )
					//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
					nSumH = nEularDistance(vTerminal, pNextGrid, pLayer);
					pNextRGrid->m_nH = nSumH;
					pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
					pNextRGrid->m_pFrom = pCurRGrid;
					bool bInsert = false;
					for (int j = 0; j < vHistory.size(); j++)
					{
						if (pNextRGrid->m_nCost < vHistory[j]->m_nCost)
						{
							vHistory.insert(vHistory.begin() + j, pNextRGrid);
							bInsert = true;
							break;
						}
					}
					if (!bInsert)
					{
						vHistory.push_back(pNextRGrid);
					}
					vAddGrid.push_back(pNextRGrid);
				}
			}
			//cout<<"test"<<endl;
			pNextGrid = graphTravel(m_pDesign, pGGrid, -nDX, -nDY, -nDZ);
			if (pNextGrid != NULL && pNextGrid->getRemand() > 0)
			{
				rGrid_C *pNextRGrid = getRGrid(nX - nDX, nY - nDY, nZ - nDZ);
				if (!pNextRGrid->isRouted && !pNextRGrid->isObstacle)
				{
					//pNextRGrid->isRouted = true;
					layer_C *pLayer = vLayer[nZ + nDZ - m_nOffsetZ];
					pNextRGrid->m_nG = nNewDistance;
					int nSumH = 0;
					//for( int t=0; t<vTerminal.size(); t++ )
					//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
					nSumH = nEularDistance(vTerminal, pNextGrid, pLayer);
					pNextRGrid->m_nH = nSumH;
					pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
					pNextRGrid->m_pFrom = pCurRGrid;
					bool bInsert = false;
					for (int j = 0; j < vHistory.size(); j++)
					{
						if (pNextRGrid->m_nCost < vHistory[j]->m_nCost)
						{
							vHistory.insert(vHistory.begin() + j, pNextRGrid);
							bInsert = true;
							break;
						}
					}
					if (!bInsert)
					{
						vHistory.push_back(pNextRGrid);
					}
					vAddGrid.push_back(pNextRGrid);
				}
			}

			// go to different layer
			//cout<<"different layer"<<endl;
			pNextGrid = graphTravel(m_pDesign, pGGrid, 0, 0, 1);
			if (pNextGrid != NULL && pNextGrid->getRemand() > 0)
			{
				rGrid_C *pNextRGrid = getRGrid(nX, nY, nZ + 1);
				if (!pNextRGrid->isRouted && !pNextRGrid->isObstacle)
				{
					//pNextRGrid->isRouted = true;
					layer_C *pLayer = vLayer[nZ + nDZ - m_nOffsetZ];
					pNextRGrid->m_nG = nNewDistance;
					int nSumH = 0;
					//for( int t=0; t<vTerminal.size(); t++ )
					//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
					nSumH = nEularDistance(vTerminal, pNextGrid, pLayer);
					pNextRGrid->m_nH = nSumH;
					pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
					pNextRGrid->m_pFrom = pCurRGrid;
					bool bInsert = false;
					for (int j = 0; j < vHistory.size(); j++)
					{
						if (pNextRGrid->m_nCost < vHistory[j]->m_nCost)
						{
							vHistory.insert(vHistory.begin() + j, pNextRGrid);
							bInsert = true;
							break;
						}
					}
					if (!bInsert)
					{
						vHistory.push_back(pNextRGrid);
					}
					vAddGrid.push_back(pNextRGrid);
				}
			}

			if (nZ - 1 >= nLConstraint)
			{
				pNextGrid = graphTravel(m_pDesign, pGGrid, 0, 0, -1);
				if (pNextGrid != NULL && pNextGrid->getRemand() > 0)
				{
					rGrid_C *pNextRGrid = getRGrid(nX, nY, nZ - 1);
					if (!pNextRGrid->isRouted && !pNextRGrid->isObstacle)
					{
						//pNextRGrid->isRouted = true;
						layer_C *pLayer = vLayer[nZ + nDZ - m_nOffsetZ];
						pNextRGrid->m_nG = nNewDistance;
						int nSumH = 0;
						//for( int t=0; t<vTerminal.size(); t++ )
						//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
						nSumH = nEularDistance(vTerminal, pNextGrid, pLayer);
						pNextRGrid->m_nH = nSumH;
						pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
						pNextRGrid->m_pFrom = pCurRGrid;
						bool bInsert = false;
						for (int j = 0; j < vHistory.size(); j++)
						{
							if (pNextRGrid->m_nCost < vHistory[j]->m_nCost)
							{
								vHistory.insert(vHistory.begin() + j, pNextRGrid);
								bInsert = true;
								break;
							}
						}
						if (!bInsert)
						{
							vHistory.push_back(pNextRGrid);
						}
						vAddGrid.push_back(pNextRGrid);
					}
				}
			}
		}
		//cout<<"Check target"<<endl;
		// if there is a target in addgrid;
		//vector< rGrid_C* > vHaveAddTarget;
		for (int i = 0; i < vAddGrid.size(); i++)
		{
			if (vAddGrid[i]->isTarget)
			{
				bool bHaveSearch = false;
				for (int j = 0; j < vHaveAddTarget.size(); j++)
				{
					if (vHaveAddTarget[j] == vAddGrid[i])
					{
						bHaveSearch = true;
						break;
					}
				}
				if (bHaveSearch)
					continue;

				vHaveAddTarget.push_back(vAddGrid[i]);
				//cout<<"Back trace"<<endl;
				// find the path
				vector<gGrid_C *> vReversePath;

				int nPath = vAddGrid[i]->m_nG;
				rGrid_C *pRPath = vAddGrid[i];
				pRPath->isPath = true;
				rGrid_C *pPreGrid = vAddGrid[i]->m_pFrom;
				gGrid_C *pPathGrid = getGrid(m_pDesign, pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ);
				//vResult.push_back( pPathGrid );
				vReversePath.push_back(pPathGrid);
				while (!pPreGrid->isPath)
				{
					pRPath = pPreGrid;
					pRPath->isPath = true;
					pPathGrid = getGrid(m_pDesign, pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ);
					//vResult.push_back( pPathGrid );
					vReversePath.push_back(pPathGrid);
					pPreGrid = pRPath->m_pFrom;
				}

				pPathGrid = getGrid(m_pDesign, pPreGrid->m_nX, pPreGrid->m_nY, pPreGrid->m_nZ);
				//vResult.push_back( pPathGrid );
				vReversePath.push_back(pPathGrid);
				for (int g = vReversePath.size() - 1; g >= 0; g--)
				{
					vResult.push_back(vReversePath[g]);
				}

				// find the terminal and remove
				for (int t = 0; t < vTerminal.size(); t++)
				{
					int nX, nY, nZ;
					vTerminal[t]->getPosition(nX, nY, nZ);
					if (vAddGrid[i]->m_nX == nX && vAddGrid[i]->m_nY == nY && vAddGrid[i]->m_nZ == nZ)
					{
						vTerminal.erase(vTerminal.begin() + t);
						nNumTermanals--;
						break;
					}
				}
				// update the nG in all the searched node ( without path )
				for (int h = 0; h < vHistory.size(); h++)
				{
					rGrid_C *pTmpGrid = vHistory[h];
					if (!pTmpGrid->isPath)
						calDistanceFromPath(pTmpGrid, nPath);
				}

				// update the nH in all the node in queue
				for (int h = 0; h < vHistory.size(); h++)
				{
					rGrid_C *pTmpGrid = vHistory[h];
					if (!pTmpGrid->isRouted)
					{
						int nH = 0;
						gGrid_C *pTmpGGrid = getGrid(m_pDesign, pTmpGrid->m_nX, pTmpGrid->m_nY, pTmpGrid->m_nZ);
						int nX, nY, nZ;
						pTmpGGrid->getPosition(nX, nY, nZ);
						layer_C *pLayer = vLayer[nZ - m_nOffsetZ];
						//for( int t=0; t<vTerminal.size(); t++ )
						//{
						//	nH = nH + nEularDistance( vTerminal[t], pTmpGGrid, pLayer );
						//}
						nH = nEularDistance(vTerminal, pTmpGGrid, pLayer);
						pTmpGrid->m_nH = nH;
						pTmpGrid->m_nCost = nH + pTmpGrid->m_nG;
					}
				}
				//cout<<endl;
			}
		}
	}

	for (int i = 0; i < vObs.size(); i++)
	{
		int nX, nY, nZ;
		gGrid_C *pGrid = vObs[i];
		pGrid->getPosition(nX, nY, nZ);
		rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
		pRGrid->isObstacle = false;
	}

	for (int i = 0; i < vHistory.size(); i++)
	{
		rGrid_C *pRGrid = vHistory[i];
		pRGrid->m_nH = 0;
		pRGrid->m_nG = 0;
		pRGrid->m_nCost = 0;
		pRGrid->m_nBend = 0;
		pRGrid->m_pFrom = NULL;
		pRGrid->isRouted = false;
		pRGrid->isPath = false;
		pRGrid->isTarget = false;
	}

	for (int i = 0; i < vResult.size(); i++)
	{
		int nX, nY, nZ;
		gGrid_C *pTmpGrid = vResult[i];
		pTmpGrid->getPosition(nX, nY, nZ);
		rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
		pRGrid->m_nH = 0;
		pRGrid->m_nG = 0;
		pRGrid->m_nCost = 0;
		pRGrid->m_nBend = 0;
		pRGrid->m_pFrom = NULL;
		pRGrid->isRouted = false;
		pRGrid->isPath = false;
		pRGrid->isTarget = false;
	}

	if (nNumTermanals != 0)
		vResult.clear();

	return vResult;
}

vector<gGrid_C *> router_C::routeNet_ver2(net_C *pNet)
{
	setRoutingConstraint(pNet);
	int nLConstraint = m_nConstraintLayerId;
	//cout<<nLConstraint<<endl;

	vector<gGrid_C *> vTerminal;
	vector<pin_C *> vPin = pNet->getPin();
	//vector< wire_C* > vWire;
	vector<gGrid_C *> vResult;
	for (int i = 0; i < vPin.size(); i++)
	{
		pin_C *pPin = vPin[i];
		int nX, nY, nZ;
		instance_C *pInst = (instance_C *)pPin->getCell();
		nX = pInst->getPlacedX();
		nY = pInst->getPlacedY();
		// project the z to the constraint layer
		nZ = pPin->getLayerId();
		if (nZ < nLConstraint)
		{
			for (int l = nZ; l <= nLConstraint; l++)
			{
				gGrid_C *pGrid = getGrid(m_pDesign, nX, nY, l);
				vResult.push_back(pGrid);
			}
			nZ = nLConstraint;
		}

		gGrid_C *pTerminal = getGrid(m_pDesign, nX, nY, nZ);
		rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
		pRGrid->isTarget = true;
		vTerminal.push_back(pTerminal);
	}

	// check same position of terminal
	//cout<<"Check terminal"<<endl;
	for (int i = 0; i < vTerminal.size(); i++)
	{
		for (int j = i + 1; j < vTerminal.size(); j++)
		{
			if (vTerminal[i] == vTerminal[j])
			{
				vTerminal.erase(vTerminal.begin() + j);
				j--;
			}
		}
	}
	//cout<<"Num of terminals: "<<vTerminal.size()<<endl;
	if (vTerminal.size() == 1)
	{
		if (vTerminal[0]->getRemand() <= 0)
		{

			int nX, nY, nZ;
			vTerminal[0]->getPosition(nX, nY, nZ);
			rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
			pRGrid->isTarget = false;
			return vResult;
		}
		int nX, nY, nZ;
		vTerminal[0]->getPosition(nX, nY, nZ);
		rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
		pRGrid->isTarget = false;

		vResult.push_back(vTerminal[0]);
		return vResult;
	}
	else
	{
		bool bFindOverflow = false;
		for (int i = 0; i < vTerminal.size(); i++)
		{
			if (vTerminal[i]->getRemand() <= 0)
			{
				bFindOverflow = true;
				break;
			}
		}

		if (bFindOverflow)
		{
			for (int i = 0; i < vTerminal.size(); i++)
			{
				int nX, nY, nZ;
				vTerminal[i]->getPosition(nX, nY, nZ);
				rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
				pRGrid->isTarget = false;
			}
			if (vResult.size() != 0)
				vResult.clear();

			return vResult;
		}
	}

	vector<rGrid_C *> vRoutedGrid;
	//vector< rGrid_C* > vPriorityQueue;
	vector<rGrid_C *> vHistory;
	vector<rGrid_C *> vNotInPath;
	// find first terminal
	gGrid_C *pFirst = NULL;
	// change to terminal set, and simultaneous multi-terminal routing
	/*
	rGrid_C* pCurRGrid = NULL;
	gGrid_C* pGGrid = NULL;
	vector< vector< gGrid_C* > > vTerminalSet;
	for( int i=0; i<vTerminal.size(); i++ )
	{
		gGrid_C* pGrid = vTerminal[i];
		vector< gGrid_C* > vTmpTerminal;
		vTmpTerminal.push_back( pGrid );
		vTerminalSet.push_back( vTmpTerminal );
	}

	for( int i=0; i<vTerminalSet.size(); i++ )
	{
		pGGrid = vTerminalSet[i][0];
		int nX, nY, nZ;
		pGGrid->getPosition( nX, nY, nZ );
		pCurRGrid = getRGrid( nX, nY, nZ );
		pCurRGrid->isPath = true;
		vHistory.push_back( pCurRGrid );
	}

	int nNumTerminalSet = vTerminalSet.size();
	vector< layer_C* > vLayer = m_pDesign->getLayer();
	while( nNumTerminalSet > 1 )
	{
		if( vHistory.size() == 0 )
		{
			vResult.clear();
			break;
		}
		vector< rGrid_C* > vToPropogate;
		int nMinCost = 0;
		bool bPick = false;
		for( int i=0; i<vHistory.size(); i++ )
		{
			if( !vHistory[i]->isRouted )	
			{
				nMinCost = vHistory[i]->m_nCost;
				bPick = true;
				break;
			}
		}
		if( !bPick )
		{
			vResult.clear();
			//cout<<"Can't find result"<<endl;
			break;
		}
		
		for( int i=0; i<vHistory.size(); i++ )
		{
			if( !vHistory[i]->isRouted )	
			{
				if( vHistory[i]->m_nCost == nMinCost )
				{
					vToPropogate.push_back( vHistory[i] );
				}
				//else
				//	break;
			}
		}
		
		for( int i=0; i<vToPropogate.size(); i++ )
		{
			pCurRGrid = vToPropogate[i];
			pCurRGrid->isRouted = true;
			int nCurDistance = pCurRGrid->m_nG;
			int nNewDistance = nCurDistance + 1;
			int nX, nY, nZ;
			nX = pCurRGrid->m_nX;
			nY = pCurRGrid->m_nY;
			nZ = pCurRGrid->m_nZ;
			//cout<<"P: "<<nX<<" "<<nY<<" "<<nZ<<endl;
			pGGrid = getGrid( m_pDesign, nX, nY, nZ );
			char cDir = vLayer[ nZ - m_nOffsetZ ]->getDir();
			//cout<<cDir<<endl;
			int nDX, nDY, nDZ;
			if( cDir == 'H' )
			{
				nDX = 0; nDY = 1; nDZ = 0;
			}
			else if( cDir == 'V')
			{
				nDX = 1; nDY = 0; nDZ = 0;
			}
			else
				cout<<"Unknown routing direction"<<endl;
			// go in the same layer;
			gGrid_C* pNextGrid = graphTravel( m_pDesign, pGGrid, nDX, nDY, nDZ );
			if( pNextGrid != NULL && pNextGrid->getRemand() > 0 )
			{
				rGrid_C* pNextRGrid = getRGrid( nX + nDX, nY + nDY, nZ + nDZ );
				if( !pNextRGrid->isRouted )
				{
					//pNextRGrid->isRouted = true;
					layer_C* pLayer = vLayer[ nZ + nDZ - m_nOffsetZ ];
					pNextRGrid->m_nG = nNewDistance;  	
					int nSumH = 0;
					//for( int t=0; t<vTerminal.size(); t++ )
					//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
					//nSumH = nEularDistance( vTerminal, pNextGrid, pLayer );
					//pNextRGrid->m_nH = nSumH;
					nSumH = nEularDistance_FaS( vTerminal, pNextGrid, pLayer, pNextRGrid );
					pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
					pNextRGrid->m_pFrom = pCurRGrid;
					bool bInsert = false;
					for( int j=0; j<vHistory.size(); j++ )
					{
						if( pNextRGrid->m_nCost < vHistory[j]->m_nCost )
						{
							vHistory.insert( vHistory.begin() + j, pNextRGrid );
							bInsert = true;
							break;
						}
					}
					if( !bInsert )
					{
						vHistory.push_back( pNextRGrid );
					}
					vAddGrid.push_back( pNextRGrid );
				}
			}
			//cout<<"test"<<endl;
			pNextGrid = graphTravel( m_pDesign, pGGrid, -nDX, -nDY, -nDZ );
			if( pNextGrid != NULL && pNextGrid->getRemand() > 0 )
			{
				rGrid_C* pNextRGrid = getRGrid( nX - nDX, nY - nDY, nZ - nDZ );
				if( !pNextRGrid->isRouted )
				{	
					//pNextRGrid->isRouted = true;
					layer_C* pLayer = vLayer[ nZ + nDZ - m_nOffsetZ ];
					pNextRGrid->m_nG = nNewDistance;  	
					int nSumH = 0;
					//for( int t=0; t<vTerminal.size(); t++ )
					//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
					//nSumH = nEularDistance( vTerminal, pNextGrid, pLayer );
					//pNextRGrid->m_nH = nSumH;
					nSumH = nEularDistance_FaS( vTerminal, pNextGrid, pLayer, pNextRGrid );
					pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
					pNextRGrid->m_pFrom = pCurRGrid;
					bool bInsert = false;
					for( int j=0; j<vHistory.size(); j++ )
					{
						if( pNextRGrid->m_nCost < vHistory[j]->m_nCost )
						{
							vHistory.insert( vHistory.begin() + j, pNextRGrid );
							bInsert = true;
							break;
						}
					}
					if( !bInsert )
					{
						vHistory.push_back( pNextRGrid );
					}
					vAddGrid.push_back( pNextRGrid );
				}
			}

			// go to different layer
			//cout<<"different layer"<<endl;
			pNextGrid = graphTravel( m_pDesign, pGGrid, 0, 0, 1 );
			if( pNextGrid != NULL && pNextGrid->getRemand() > 0 )
			{
				rGrid_C* pNextRGrid = getRGrid( nX, nY, nZ + 1 );
				if( !pNextRGrid->isRouted )
				{
					//pNextRGrid->isRouted = true;
					layer_C* pLayer = vLayer[ nZ + nDZ - m_nOffsetZ ];
					pNextRGrid->m_nG = nNewDistance;  	
					int nSumH = 0;
					//for( int t=0; t<vTerminal.size(); t++ )
					//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
					//nSumH = nEularDistance( vTerminal, pNextGrid, pLayer );
					//pNextRGrid->m_nH = nSumH;
					nSumH = nEularDistance_FaS( vTerminal, pNextGrid, pLayer, pNextRGrid );
					pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
					pNextRGrid->m_pFrom = pCurRGrid;
					bool bInsert = false;
					for( int j=0; j<vHistory.size(); j++ )
					{
						if( pNextRGrid->m_nCost < vHistory[j]->m_nCost )
						{
							vHistory.insert( vHistory.begin() + j, pNextRGrid );
							bInsert = true;
							break;
						}
					}
					if( !bInsert )
					{
						vHistory.push_back( pNextRGrid );
					}
					vAddGrid.push_back( pNextRGrid );
				}
			}
		
			if( nZ - 1 >= nLConstraint )
			{
				pNextGrid = graphTravel( m_pDesign, pGGrid, 0, 0, -1 );
				if( pNextGrid != NULL && pNextGrid->getRemand() > 0 )
				{
					rGrid_C* pNextRGrid = getRGrid( nX, nY, nZ - 1 );
					if( !pNextRGrid->isRouted )
					{
						//pNextRGrid->isRouted = true;
						layer_C* pLayer = vLayer[ nZ + nDZ - m_nOffsetZ ];
						pNextRGrid->m_nG = nNewDistance;  	
						int nSumH = 0;
						//for( int t=0; t<vTerminal.size(); t++ )
						//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
						//nSumH = nEularDistance( vTerminal, pNextGrid, pLayer );
						//pNextRGrid->m_nH = nSumH;
						nSumH = nEularDistance_FaS( vTerminal, pNextGrid, pLayer, pNextRGrid );
						pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
						pNextRGrid->m_pFrom = pCurRGrid;
						bool bInsert = false;
						for( int j=0; j<vHistory.size(); j++ )
						{
							if( pNextRGrid->m_nCost < vHistory[j]->m_nCost )
							{
								vHistory.insert( vHistory.begin() + j, pNextRGrid );
								bInsert = true;
								break;
							}
						}
						if( !bInsert )
						{
							vHistory.push_back( pNextRGrid );
						}
						vAddGrid.push_back( pNextRGrid );
					}
				}
			}	
		}
		
		vector< rGrid_C* > vHaveAddTarget;
		for( int i=0; i<vAddGrid.size(); i++ )
		{
			if( vAddGrid[i]->isTarget )
			{
				bool bHaveSearch = false;
				for( int j=0; j<vHaveAddTarget.size(); j++ )
				{
					if( vHaveAddTarget[j] == vAddGrid[i] )
					{
						bHaveSearch = true;
						break;
					}
				}
				if( bHaveSearch )
					continue;

				vHaveAddTarget.push_back( vAddGrid[i] );
				//cout<<"Back trace"<<endl;
				// find the path
				vector< gGrid_C* > vReversePath;
				
				int nPath = vAddGrid[i]->m_nG;
				rGrid_C* pRPath = vAddGrid[i];
				pRPath->isPath = true;
				gGrid_C* pPathGrid = getGrid( m_pDesign, pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ );
				rGrid_C* pPreGrid = pRPath->m_pFrom;
				//gGrid_C* pPathGrid = getGrid( m_pDesign, pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ );
				//vResult.push_back( pPathGrid );
				vReversePath.push_back( pPathGrid );
				while( !pPreGrid->isPath )
				{
					pRPath = pPreGrid;
					pRPath->isPath = true;
					pPathGrid = getGrid( m_pDesign, pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ );
					//vResult.push_back( pPathGrid );
					vReversePath.push_back( pPathGrid );
					pPreGrid = pRPath->m_pFrom;
				}
				
				// added at 07/04 20:30
				//nPath = pPreGrid->m_nG + nAddPath;
				// end added at 07/04 20:30
				

				pPathGrid = getGrid( m_pDesign, pPreGrid->m_nX, pPreGrid->m_nY, pPreGrid->m_nZ );
				//vResult.push_back( pPathGrid );
				vReversePath.push_back( pPathGrid );
				for( int g=vReversePath.size()-1; g>=0; g-- )
				{
					vResult.push_back( vReversePath[g] );
				}

				// find the terminal and remove
				for( int t=0; t<vTerminal.size(); t++ )
				{
					int nX, nY, nZ;
					vTerminal[t]->getPosition( nX, nY, nZ );
					if( vAddGrid[i]->m_nX == nX &&  vAddGrid[i]->m_nY == nY && vAddGrid[i]->m_nZ == nZ )
					{
						vTerminal.erase( vTerminal.begin() + t );
						nNumTermanals--;
						break;
					}
				}
				// update the nG in all the searched node ( without path )
				for( int h=0; h<vHistory.size(); h++ )
				{
					rGrid_C* pTmpGrid = vHistory[h];
					if( !pTmpGrid->isPath )
						calDistanceFromPath( pTmpGrid, nPath );	
				}
				
				// update the nH in all the node in queue
				for( int h=0; h<vHistory.size(); h++ )
				{
					rGrid_C* pTmpGrid = vHistory[h];
					if( !pTmpGrid->isRouted )
					{
						int nH = 0;
						gGrid_C* pTmpGGrid = getGrid( m_pDesign, pTmpGrid->m_nX, pTmpGrid->m_nY, pTmpGrid->m_nZ );
						int nX, nY, nZ;
						pTmpGGrid->getPosition( nX, nY, nZ );
						layer_C* pLayer = vLayer[ nZ - m_nOffsetZ ];
						//for( int t=0; t<vTerminal.size(); t++ )
						//{
						//	nH = nH + nEularDistance( vTerminal[t], pTmpGGrid, pLayer );
						//}
						nH = nEularDistance( vTerminal, pTmpGGrid, pLayer );
						pTmpGrid->m_nH = nH;
						pTmpGrid->m_nCost = nH + pTmpGrid->m_nG;
					}	
				}
				//cout<<endl;
			}
		}

	}
*/
	// end change

	int nMinX = m_nTX;
	for (int i = 0; i < vTerminal.size(); i++)
	{
		gGrid_C *pGrid = vTerminal[i];
		int nX, nY, nZ;
		pGrid->getPosition(nX, nY, nZ);
		if (nX <= nMinX)
			nMinX = nX;
	}

	int nMinY = m_nTY;
	for (int i = 0; i < vTerminal.size(); i++)
	{
		gGrid_C *pGrid = vTerminal[i];
		int nX, nY, nZ;
		pGrid->getPosition(nX, nY, nZ);
		if (nX == nMinX && nY <= nMinY)
		{
			nMinY = nY;
			pFirst = vTerminal[i];
		}
	}

	//
	//pFirst = vTerminal[0];

	//cout<<"Here"<<endl;
	int nX, nY, nZ;
	pFirst->getPosition(nX, nY, nZ);
	rGrid_C *pCurRGrid = getRGrid(nX, nY, nZ);
	gGrid_C *pGGrid = pFirst;
	if (pGGrid->getRemand() <= 0)
	{
		pCurRGrid->isTarget = false;
		vResult.clear();
		return vResult;
	}

	vHistory.push_back(pCurRGrid);
	pCurRGrid->isPath = true;
	for (int i = 0; i < vTerminal.size(); i++)
	{
		if (vTerminal[i] == pGGrid)
		{
			vTerminal.erase(vTerminal.begin() + i);
			break;
		}
	}
	int nNumTermanals = vTerminal.size();
	//nNumTermanals--; // because the first grid is a termianl
	vector<layer_C *> vLayer = m_pDesign->getLayer();
	//cout<<"Start routing"<<endl;
	while (nNumTermanals > 0)
	{
		//cout<<nNumTermanals<<endl;
		//cout<<pCurRGrid->m_nX<<" "<<pCurRGrid->m_nY<<" "<<pCurRGrid->m_nZ<<endl;
		// A* propogation
		if (vHistory.size() == 0)
		{
			vResult.clear();
			break;
		}

		vector<rGrid_C *> vToPropogate;
		int nMinCost = 0;
		bool bPick = false;
		for (int i = 0; i < vHistory.size(); i++)
		{
			if (!vHistory[i]->isRouted)
			{
				nMinCost = vHistory[i]->m_nCost;
				bPick = true;
				break;
			}
		}
		if (!bPick)
		{
			vResult.clear();
			//cout<<"Can't find result"<<endl;
			break;
		}
		/*
		cout<<nMinCost<<" "<<vHistory.size()<<endl;
		for( int i=0; i<vHistory.size(); i++ )
		{
			if( !vHistory[i]->isRouted )	
			{
				cout << vHistory[i]->m_nCost <<" ";
			}
		}
		//cout<<endl;
		*/
		for (int i = 0; i < vHistory.size(); i++)
		{
			if (!vHistory[i]->isRouted)
			{
				if (vHistory[i]->m_nCost == nMinCost)
				{
					vToPropogate.push_back(vHistory[i]);
				}
				else
					break;
			}
		}

		vector<rGrid_C *> vAddGrid;

		for (int i = 0; i < vToPropogate.size(); i++)
		{
			pCurRGrid = vToPropogate[i];
			pCurRGrid->isRouted = true;
			int nCurDistance = pCurRGrid->m_nG;
			int nNewDistance = nCurDistance + 1;
			int nX, nY, nZ;
			nX = pCurRGrid->m_nX;
			nY = pCurRGrid->m_nY;
			nZ = pCurRGrid->m_nZ;
			//cout<<"P: "<<nX<<" "<<nY<<" "<<nZ<<endl;
			pGGrid = getGrid(m_pDesign, nX, nY, nZ);
			char cDir = vLayer[nZ - m_nOffsetZ]->getDir();
			//cout<<cDir<<endl;
			int nDX, nDY, nDZ;
			if (cDir == 'H')
			{
				nDX = 0;
				nDY = 1;
				nDZ = 0;
			}
			else if (cDir == 'V')
			{
				nDX = 1;
				nDY = 0;
				nDZ = 0;
			}
			else
				cout << "Unknown routing direction" << endl;
			// go in the same layer;
			gGrid_C *pNextGrid = graphTravel(m_pDesign, pGGrid, nDX, nDY, nDZ);
			if (pNextGrid != NULL && pNextGrid->getRemand() > 0)
			{
				rGrid_C *pNextRGrid = getRGrid(nX + nDX, nY + nDY, nZ + nDZ);
				if (!pNextRGrid->isRouted)
				{
					//pNextRGrid->isRouted = true;
					layer_C *pLayer = vLayer[nZ + nDZ - m_nOffsetZ];
					pNextRGrid->m_nG = nNewDistance;
					int nSumH = 0;
					//for( int t=0; t<vTerminal.size(); t++ )
					//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
					nSumH = nEularDistance(vTerminal, pNextGrid, pLayer);
					pNextRGrid->m_nH = nSumH;
					pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
					pNextRGrid->m_pFrom = pCurRGrid;
					bool bInsert = false;
					for (int j = 0; j < vHistory.size(); j++)
					{
						if (pNextRGrid->m_nCost < vHistory[j]->m_nCost)
						{
							vHistory.insert(vHistory.begin() + j, pNextRGrid);
							bInsert = true;
							break;
						}
					}
					if (!bInsert)
					{
						vHistory.push_back(pNextRGrid);
					}
					vAddGrid.push_back(pNextRGrid);
				}
			}
			//cout<<"test"<<endl;
			pNextGrid = graphTravel(m_pDesign, pGGrid, -nDX, -nDY, -nDZ);
			if (pNextGrid != NULL && pNextGrid->getRemand() > 0)
			{
				rGrid_C *pNextRGrid = getRGrid(nX - nDX, nY - nDY, nZ - nDZ);
				if (!pNextRGrid->isRouted)
				{
					//pNextRGrid->isRouted = true;
					layer_C *pLayer = vLayer[nZ + nDZ - m_nOffsetZ];
					pNextRGrid->m_nG = nNewDistance;
					int nSumH = 0;
					//for( int t=0; t<vTerminal.size(); t++ )
					//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
					nSumH = nEularDistance(vTerminal, pNextGrid, pLayer);
					pNextRGrid->m_nH = nSumH;
					pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
					pNextRGrid->m_pFrom = pCurRGrid;
					bool bInsert = false;
					for (int j = 0; j < vHistory.size(); j++)
					{
						if (pNextRGrid->m_nCost < vHistory[j]->m_nCost)
						{
							vHistory.insert(vHistory.begin() + j, pNextRGrid);
							bInsert = true;
							break;
						}
					}
					if (!bInsert)
					{
						vHistory.push_back(pNextRGrid);
					}
					vAddGrid.push_back(pNextRGrid);
				}
			}

			// go to different layer
			//cout<<"different layer"<<endl;
			pNextGrid = graphTravel(m_pDesign, pGGrid, 0, 0, 1);
			if (pNextGrid != NULL && pNextGrid->getRemand() > 0)
			{
				rGrid_C *pNextRGrid = getRGrid(nX, nY, nZ + 1);
				if (!pNextRGrid->isRouted)
				{
					//pNextRGrid->isRouted = true;
					layer_C *pLayer = vLayer[nZ + nDZ - m_nOffsetZ];
					pNextRGrid->m_nG = nNewDistance;
					int nSumH = 0;
					//for( int t=0; t<vTerminal.size(); t++ )
					//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
					nSumH = nEularDistance(vTerminal, pNextGrid, pLayer);
					pNextRGrid->m_nH = nSumH;
					pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
					pNextRGrid->m_pFrom = pCurRGrid;
					bool bInsert = false;
					for (int j = 0; j < vHistory.size(); j++)
					{
						if (pNextRGrid->m_nCost < vHistory[j]->m_nCost)
						{
							vHistory.insert(vHistory.begin() + j, pNextRGrid);
							bInsert = true;
							break;
						}
					}
					if (!bInsert)
					{
						vHistory.push_back(pNextRGrid);
					}
					vAddGrid.push_back(pNextRGrid);
				}
			}

			if (nZ - 1 >= nLConstraint)
			{
				pNextGrid = graphTravel(m_pDesign, pGGrid, 0, 0, -1);
				if (pNextGrid != NULL && pNextGrid->getRemand() > 0)
				{
					rGrid_C *pNextRGrid = getRGrid(nX, nY, nZ - 1);
					if (!pNextRGrid->isRouted)
					{
						//pNextRGrid->isRouted = true;
						layer_C *pLayer = vLayer[nZ + nDZ - m_nOffsetZ];
						pNextRGrid->m_nG = nNewDistance;
						int nSumH = 0;
						//for( int t=0; t<vTerminal.size(); t++ )
						//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
						nSumH = nEularDistance(vTerminal, pNextGrid, pLayer);
						pNextRGrid->m_nH = nSumH;
						pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
						pNextRGrid->m_pFrom = pCurRGrid;
						bool bInsert = false;
						for (int j = 0; j < vHistory.size(); j++)
						{
							if (pNextRGrid->m_nCost < vHistory[j]->m_nCost)
							{
								vHistory.insert(vHistory.begin() + j, pNextRGrid);
								bInsert = true;
								break;
							}
						}
						if (!bInsert)
						{
							vHistory.push_back(pNextRGrid);
						}
						vAddGrid.push_back(pNextRGrid);
					}
				}
			}
		}
		//cout<<"Check target"<<endl;
		// if there is a target in addgrid;
		vector<rGrid_C *> vHaveAddTarget;
		for (int i = 0; i < vAddGrid.size(); i++)
		{
			if (vAddGrid[i]->isTarget)
			{
				bool bHaveSearch = false;
				for (int j = 0; j < vHaveAddTarget.size(); j++)
				{
					if (vHaveAddTarget[j] == vAddGrid[i])
					{
						bHaveSearch = true;
						break;
					}
				}
				if (bHaveSearch)
					continue;

				vHaveAddTarget.push_back(vAddGrid[i]);
				//cout<<"Back trace"<<endl;
				// find the path
				vector<gGrid_C *> vReversePath;

				int nPath = vAddGrid[i]->m_nG;
				rGrid_C *pRPath = vAddGrid[i];
				pRPath->isPath = true;
				gGrid_C *pPathGrid = getGrid(m_pDesign, pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ);
				rGrid_C *pPreGrid = pRPath->m_pFrom;
				/*
				//  added at 07/20 20:30
				layer_C* pTmpLayer = m_pDesign->getLayer()[ pRPath->m_nZ - m_nOffsetZ ];
				int nPDX;
				int nPDY;
				if( pTmpLayer->getDir() == 'H' )
				{
					nPDX = 0; nPDY = 1;
				}
				else
				{
					nPDX = 1; nPDY = 0;
				}
				
				rGrid_C* pRGrid1 = getRGrid( pRPath->m_nX + nPDX, pRPath->m_nY + nPDY, pRPath->m_nZ );
				rGrid_C* pRGrid2 = getRGrid( pRPath->m_nX - nPDX, pRPath->m_nY - nPDY, pRPath->m_nZ );	
				rGrid_C* pRGrid3 = getRGrid( pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ + 1 );
				rGrid_C* pRGrid4 = getRGrid( pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ - 1 );
				vector< rGrid_C* > vTmpRGrid;
				//for( int rg=0; rg<4; rg++ )
				//{
					if( pRGrid1 != NULL && pRGrid1->isRouted )
						vTmpRGrid.push_back( pRGrid1 );
					if( pRGrid2 != NULL && pRGrid2->isRouted )
						vTmpRGrid.push_back( pRGrid2 );
					if( pRGrid3 != NULL && pRGrid3->isRouted )
						vTmpRGrid.push_back( pRGrid3 );
					if( pRGrid4 != NULL && pRGrid4->isRouted )
						vTmpRGrid.push_back( pRGrid4 );
				//}
				int nTmpPath = nPath;
				rGrid_C* pMinRGrid = vAddGrid[i]->m_pFrom;
				for( int rg=0; rg<vTmpRGrid.size(); rg++ )
				{
					if( nTmpPath > vTmpRGrid[ rg ]->m_nG )
					{
						nTmpPath = vTmpRGrid[ rg ]->m_nG;
						pMinRGrid = vTmpRGrid[ rg ];
					}	
				}
				vTmpRGrid.clear();
				vAddGrid[i]->m_pFrom = pMinRGrid;
				int nAddPath = 1;
				rGrid_C* pPreGrid = vAddGrid[i]->m_pFrom;
				//end added at 07/04 20:30
				*/
				//gGrid_C* pPathGrid = getGrid( m_pDesign, pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ );
				//vResult.push_back( pPathGrid );
				vReversePath.push_back(pPathGrid);
				while (!pPreGrid->isPath)
				{
					pRPath = pPreGrid;
					pRPath->isPath = true;
					pPathGrid = getGrid(m_pDesign, pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ);
					//vResult.push_back( pPathGrid );
					vReversePath.push_back(pPathGrid);
					/*
					// added at 07/04 20:30	
					pTmpLayer = m_pDesign->getLayer()[ pRPath->m_nZ - m_nOffsetZ ];
					
					if( pTmpLayer->getDir() == 'H' )
					{
						nPDX = 0; nPDY = 1;
					}
					else
					{
						nPDX = 1; nPDY = 0;
					}
					
					pRGrid1 = getRGrid( pRPath->m_nX + nPDX, pRPath->m_nY + nPDY, pRPath->m_nZ );
					pRGrid2 = getRGrid( pRPath->m_nX - nPDX, pRPath->m_nY - nPDY, pRPath->m_nZ );	
					pRGrid3 = getRGrid( pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ + 1 );
					pRGrid4 = getRGrid( pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ - 1 );
					//vector< rGrid_C* > vTmpRGrid;
					//for( int rg=0; rg<4; rg++ )
					//{
						if( pRGrid1 != NULL && pRGrid1->isRouted )
							vTmpRGrid.push_back( pRGrid1 );
						if( pRGrid2 != NULL && pRGrid2->isRouted )
							vTmpRGrid.push_back( pRGrid2 );
						if( pRGrid3 != NULL && pRGrid3->isRouted )
							vTmpRGrid.push_back( pRGrid3 );
						if( pRGrid4 != NULL && pRGrid4->isRouted )
							vTmpRGrid.push_back( pRGrid4 );
					//}
					int nTmpPath = nPath;
					rGrid_C* pMinRGrid = pRPath->m_pFrom;
					for( int rg=0; rg<vTmpRGrid.size(); rg++ )
					{
						if( nTmpPath > vTmpRGrid[ rg ]->m_nG )
						{
							nTmpPath = vTmpRGrid[ rg ]->m_nG;
							pMinRGrid = vTmpRGrid[ rg ];
						}	
					}
					vTmpRGrid.clear();
					pRPath->m_pFrom = pMinRGrid;
					nAddPath++;
					// end added at 07/04 20:30
					*/
					pPreGrid = pRPath->m_pFrom;
				}
				/*
				// added at 07/04 20:30
				nPath = pPreGrid->m_nG + nAddPath;
				// end added at 07/04 20:30
				*/

				pPathGrid = getGrid(m_pDesign, pPreGrid->m_nX, pPreGrid->m_nY, pPreGrid->m_nZ);
				//vResult.push_back( pPathGrid );
				vReversePath.push_back(pPathGrid);
				for (int g = vReversePath.size() - 1; g >= 0; g--)
				{
					vResult.push_back(vReversePath[g]);
				}

				// find the terminal and remove
				for (int t = 0; t < vTerminal.size(); t++)
				{
					int nX, nY, nZ;
					vTerminal[t]->getPosition(nX, nY, nZ);
					if (vAddGrid[i]->m_nX == nX && vAddGrid[i]->m_nY == nY && vAddGrid[i]->m_nZ == nZ)
					{
						vTerminal.erase(vTerminal.begin() + t);
						nNumTermanals--;
						break;
					}
				}
				// update the nG in all the searched node ( without path )
				for (int h = 0; h < vHistory.size(); h++)
				{
					rGrid_C *pTmpGrid = vHistory[h];
					if (!pTmpGrid->isPath)
						calDistanceFromPath(pTmpGrid, nPath);
				}

				// update the nH in all the node in queue
				for (int h = 0; h < vHistory.size(); h++)
				{
					rGrid_C *pTmpGrid = vHistory[h];
					if (!pTmpGrid->isRouted)
					{
						int nH = 0;
						gGrid_C *pTmpGGrid = getGrid(m_pDesign, pTmpGrid->m_nX, pTmpGrid->m_nY, pTmpGrid->m_nZ);
						int nX, nY, nZ;
						pTmpGGrid->getPosition(nX, nY, nZ);
						layer_C *pLayer = vLayer[nZ - m_nOffsetZ];
						//for( int t=0; t<vTerminal.size(); t++ )
						//{
						//	nH = nH + nEularDistance( vTerminal[t], pTmpGGrid, pLayer );
						//}
						nH = nEularDistance(vTerminal, pTmpGGrid, pLayer);
						pTmpGrid->m_nH = nH;
						pTmpGrid->m_nCost = nH + pTmpGrid->m_nG;
					}
				}
				//cout<<endl;
			}
		}
	}

	for (int i = 0; i < vHistory.size(); i++)
	{
		rGrid_C *pRGrid = vHistory[i];
		pRGrid->m_nH = 0;
		pRGrid->m_nG = 0;
		pRGrid->m_nCost = 0;
		pRGrid->m_nBend = 0;
		pRGrid->m_pFrom = NULL;
		pRGrid->isRouted = false;
		pRGrid->isPath = false;
		pRGrid->isTarget = false;
	}

	for (int i = 0; i < vResult.size(); i++)
	{
		int nX, nY, nZ;
		gGrid_C *pTmpGrid = vResult[i];
		pTmpGrid->getPosition(nX, nY, nZ);
		rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
		pRGrid->m_nH = 0;
		pRGrid->m_nG = 0;
		pRGrid->m_nCost = 0;
		pRGrid->m_nBend = 0;
		pRGrid->m_pFrom = NULL;
		pRGrid->isRouted = false;
		pRGrid->isPath = false;
		pRGrid->isTarget = false;
	}

	for (int i = 0; i < vTerminal.size(); i++)
	{
		int nX, nY, nZ;
		gGrid_C *pTmpGrid = vTerminal[i];
		pTmpGrid->getPosition(nX, nY, nZ);
		rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
		pRGrid->m_nH = 0;
		pRGrid->m_nG = 0;
		pRGrid->m_nCost = 0;
		pRGrid->m_nBend = 0;
		pRGrid->m_pFrom = NULL;
		pRGrid->isRouted = false;
		pRGrid->isPath = false;
		pRGrid->isTarget = false;
	}
	if (nNumTermanals != 0)
		vResult.clear();

	return vResult;
}

vector<gGrid_C *> router_C::routeNet_neg_length_constraint(net_C *pNet, const int nConstraint, vector<int> &vConstraint, vector<gGrid_C *> &vOverflowGrid)
{
	setRoutingConstraint(pNet);
	int nLConstraint = m_nConstraintLayerId;
	//cout<<nLConstraint<<endl;

	vector<gGrid_C *> vTerminal;
	vector<pin_C *> vPin = pNet->getPin();
	//vector< wire_C* > vWire;
	vector<gGrid_C *> vResult;
	for (int i = 0; i < vPin.size(); i++)
	{
		pin_C *pPin = vPin[i];
		int nX, nY, nZ;
		instance_C *pInst = (instance_C *)pPin->getCell();
		nX = pInst->getPlacedX();
		nY = pInst->getPlacedY();
		// project the z to the constraint layer
		nZ = pPin->getLayerId();
		if (nZ < nLConstraint)
		{
			for (int l = nZ; l <= nLConstraint; l++)
			{
				gGrid_C *pGrid = getGrid(m_pDesign, nX, nY, l);
				vResult.push_back(pGrid);
			}
			nZ = nLConstraint;
		}

		gGrid_C *pTerminal = getGrid(m_pDesign, nX, nY, nZ);
		rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
		pRGrid->isTarget = true;
		vTerminal.push_back(pTerminal);
	}

	// check same position of terminal
	//cout<<"Check terminal"<<endl;
	for (int i = 0; i < vTerminal.size(); i++)
	{
		for (int j = i + 1; j < vTerminal.size(); j++)
		{
			if (vTerminal[i] == vTerminal[j])
			{
				vTerminal.erase(vTerminal.begin() + j);
				j--;
			}
		}
	}
	//cout<<"Num of terminals: "<<vTerminal.size()<<endl;
	if (vTerminal.size() == 1)
	{
		if (vTerminal[0]->getRemand() <= 0)
		{

			int nX, nY, nZ;
			vTerminal[0]->getPosition(nX, nY, nZ);
			rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
			pRGrid->isTarget = false;
			return vResult;
		}
		int nX, nY, nZ;
		vTerminal[0]->getPosition(nX, nY, nZ);
		rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
		pRGrid->isTarget = false;

		vResult.push_back(vTerminal[0]);
		return vResult;
	}
	else
	{
		bool bFindOverflow = false;
		for (int i = 0; i < vTerminal.size(); i++)
		{
			if (vTerminal[i]->getRemand() <= 0)
			{
				bFindOverflow = true;
				break;
			}
		}

		if (bFindOverflow)
		{
			for (int i = 0; i < vTerminal.size(); i++)
			{
				int nX, nY, nZ;
				vTerminal[i]->getPosition(nX, nY, nZ);
				rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
				pRGrid->isTarget = false;
			}
			if (vResult.size() != 0)
				vResult.clear();

			return vResult;
		}
	}

	vector<rGrid_C *> vRoutedGrid;
	//vector< rGrid_C* > vPriorityQueue;
	vector<rGrid_C *> vHistory;
	vector<rGrid_C *> vNotInPath;
	// find first terminal
	gGrid_C *pFirst = NULL;

	int nMinX = m_nTX;
	for (int i = 0; i < vTerminal.size(); i++)
	{
		gGrid_C *pGrid = vTerminal[i];
		int nX, nY, nZ;
		pGrid->getPosition(nX, nY, nZ);
		if (nX <= nMinX)
			nMinX = nX;
	}

	int nMinY = m_nTY;
	for (int i = 0; i < vTerminal.size(); i++)
	{
		gGrid_C *pGrid = vTerminal[i];
		int nX, nY, nZ;
		pGrid->getPosition(nX, nY, nZ);
		if (nX == nMinX && nY <= nMinY)
		{
			nMinY = nY;
			pFirst = vTerminal[i];
		}
	}

	//
	//pFirst = vTerminal[0];

	//chech the terminal is available to route the net
	int nX, nY, nZ;
	pFirst->getPosition(nX, nY, nZ);
	rGrid_C *pCurRGrid = getRGrid(nX, nY, nZ);
	gGrid_C *pGGrid = pFirst;
	if (pGGrid->getRemand() <= 0)
	{
		pCurRGrid->isTarget = false;
		// added at 0705 22:00
		for (int i = 0; i < vTerminal.size(); i++)
		{
			vTerminal[i]->getPosition(nX, nY, nZ);
			pCurRGrid = getRGrid(nX, nY, nZ);
			pCurRGrid->isTarget = false;
		}
		// end added at 0705 22:00
		vResult.clear();
		return vResult;
	}

	vHistory.push_back(pCurRGrid);
	pCurRGrid->isPath = true;
	for (int i = 0; i < vTerminal.size(); i++)
	{
		if (vTerminal[i] == pGGrid)
		{
			vTerminal.erase(vTerminal.begin() + i);
			break;
		}
	}
	int nNumTermanals = vTerminal.size();
	//nNumTermanals--; // because the first grid is a termianl
	vector<layer_C *> vLayer = m_pDesign->getLayer();
	//cout<<"Start routing"<<endl;
	vector<rGrid_C *> vHaveAddTarget;
	int nHistoryIndex = 0;
	while (nNumTermanals > 0)
	{
		//cout<<nNumTermanals<<endl;
		//cout<<pCurRGrid->m_nX<<" "<<pCurRGrid->m_nY<<" "<<pCurRGrid->m_nZ<<endl;
		// A* propogation
		if (vHistory.size() == 0)
		{
			vResult.clear();
			break;
		}
		//cout<<vHistory.size()<<endl;

		vector<rGrid_C *> vToPropogate;
		int nMinCost = 0;
		/*
		bool bPick = false;
		
		for( int i=0; i<vHistory.size(); i++ )
		{
			if( !vHistory[i]->isRouted )	
			{
				nMinCost = vHistory[i]->m_nCost;
				bPick = true;
				break;
			}
		}
		if( !bPick )
		{
			vResult.clear();
			//cout<<"Can't find result"<<endl;
			break;
		}
		*/
		if (nHistoryIndex == vHistory.size())
			break;
		else
		{
			nMinCost = vHistory[nHistoryIndex]->m_nCost;
			vToPropogate.push_back(vHistory[nHistoryIndex]);
			//nHistoryIndex++;
		}
		/*
		cout<<nMinCost<<" "<<vHistory.size()<<endl;
		for( int i=0; i<vHistory.size(); i++ )
		{
			if( !vHistory[i]->isRouted )	
			{
				cout << vHistory[i]->m_nCost <<" ";
			}
		}
		//cout<<endl;
		*/
		/*
		for( int i=0; i<vHistory.size(); i++ )
		{
			if( !vHistory[i]->isRouted )	
			{
				if( vHistory[i]->m_nCost == nMinCost )
				{
					vToPropogate.push_back( vHistory[i] );
					break;
				}
				//else
				//	break;
			}
		}
		*/

		//----------------------------------------------------------------------------choubegin TODO:
		vector<rGrid_C *> samecost;
		int vsum = INT_MAX;
		rGrid_C *temprG;
		gGrid_C *tempgG;
		for (int i = nHistoryIndex; i < vHistory.size(); i++)
		{
			if (!vHistory[i]->isRouted)
			{
				if (vHistory[i]->m_nCost == nMinCost)
				{
					samecost.push_back(vHistory[i]);
				}
				else
					break;
			}
		}
		//cout << "iter--------" << endl;
		for (int x = 0; x < samecost.size(); x++)
		{
			int az = samecost[x]->m_nZ;
			layer_C *pLayer = vLayer[az - m_nOffsetZ];
			//if (pNet->getName() == "N2")
			//	cout << "same : " << samecost[x]->m_nX << " " << samecost[x]->m_nY << " " << samecost[x]->m_nZ << endl;
			tempgG = m_pDesign->getGrid(samecost[x]->m_nX, samecost[x]->m_nY, az);
			if (vsum > nEularDistance_sum(vTerminal, tempgG, pLayer))
			{
				vsum = nEularDistance_sum(vTerminal, tempgG, pLayer);
				temprG = samecost[x];
			}
		}
		for (int i = nHistoryIndex; i < vHistory.size(); i++)
		{
			if (temprG == vHistory[i])
			{
				vHistory[i] = vHistory[nHistoryIndex];
				vHistory[nHistoryIndex] = temprG;
				break;
			}
		}
		nHistoryIndex++;
		vToPropogate.clear();
		vToPropogate.push_back(temprG);
		samecost.clear();
		//------------------------------------------------------------------------------chouend

		/*
		if (pNet->getName() == "N93")
		{
			cout<<endl;
			cout << "Routed: ";
			for (int i = 0; i < vHistory.size(); i++)
			{
				if( vHistory[i]->isRouted )
				cout << "(" << vHistory[i]->m_nX << " " << vHistory[i]->m_nY << " " << vHistory[i]->m_nZ << " " << vHistory[i]->m_nCost << " )";
			}
			cout << endl;
			cout<< "His: ";
			for (int i = 0; i < vHistory.size(); i++)
			{
				if( !vHistory[i]->isRouted )
				cout << "(" << vHistory[i]->m_nX << " " << vHistory[i]->m_nY << " " << vHistory[i]->m_nZ << " " << vHistory[i]->m_nCost << " )";
			}
			cout<<endl;
		}
		*/
		vector<rGrid_C *> vAddGrid;

		for (int i = 0; i < vToPropogate.size(); i++)
		{
			pCurRGrid = vToPropogate[i];
			pCurRGrid->isRouted = true;
			int nCurDistance = pCurRGrid->m_nG;
			int nNewDistance = nCurDistance + 1;
			if (nNewDistance > nConstraint)
				continue;

			int nX, nY, nZ;
			nX = pCurRGrid->m_nX;
			nY = pCurRGrid->m_nY;
			nZ = pCurRGrid->m_nZ;
			//cout<<"P: "<<nX<<" "<<nY<<" "<<nZ<<endl;
			pGGrid = getGrid(m_pDesign, nX, nY, nZ);
			char cDir = vLayer[nZ - m_nOffsetZ]->getDir();
			//cout<<cDir<<endl;
			int nDX, nDY, nDZ;
			if (cDir == 'H')
			{
				nDX = 0;
				nDY = 1;
				nDZ = 0;
			}
			else if (cDir == 'V')
			{
				nDX = 1;
				nDY = 0;
				nDZ = 0;
			}
			else
				cout << "Unknown routing direction" << endl;
			// go in the same layer;
			gGrid_C *pNextGrid = graphTravel(m_pDesign, pGGrid, nDX, nDY, nDZ);
			if (pNextGrid != NULL && pNextGrid->getRemand() > 0)
			{
				rGrid_C *pNextRGrid = getRGrid(nX + nDX, nY + nDY, nZ + nDZ);
				if (!pNextRGrid->isRouted)
				{
					//pNextRGrid->isRouted = true;
					layer_C *pLayer = vLayer[nZ + nDZ - m_nOffsetZ];
					pNextRGrid->m_nG = nNewDistance;
					int nSumH = 0;
					//for( int t=0; t<vTerminal.size(); t++ )
					//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
					nSumH = nEularDistance(vTerminal, pNextGrid, pLayer);
					pNextRGrid->m_nH = nSumH;
					pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
					pNextRGrid->m_pFrom = pCurRGrid;
					bool bInsert = false;
					//for( int j=0; j<vHistory.size(); j++ )
					for (int j = nHistoryIndex; j < vHistory.size(); j++)
					{
						if (pNextRGrid->m_nCost <= vHistory[j]->m_nCost)
						{
							vHistory.insert(vHistory.begin() + j, pNextRGrid);
							bInsert = true;
							break;
						}
					}
					if (!bInsert)
					{
						vHistory.push_back(pNextRGrid);
					}
					vAddGrid.push_back(pNextRGrid);
				}
			}
			//cout<<"test"<<endl;
			pNextGrid = graphTravel(m_pDesign, pGGrid, -nDX, -nDY, -nDZ);
			if (pNextGrid != NULL && pNextGrid->getRemand() > 0)
			{
				rGrid_C *pNextRGrid = getRGrid(nX - nDX, nY - nDY, nZ - nDZ);
				if (!pNextRGrid->isRouted)
				{
					//pNextRGrid->isRouted = true;
					layer_C *pLayer = vLayer[nZ - nDZ - m_nOffsetZ];
					pNextRGrid->m_nG = nNewDistance;
					int nSumH = 0;
					//for( int t=0; t<vTerminal.size(); t++ )
					//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
					nSumH = nEularDistance(vTerminal, pNextGrid, pLayer);
					pNextRGrid->m_nH = nSumH;
					pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
					pNextRGrid->m_pFrom = pCurRGrid;
					bool bInsert = false;
					//for( int j=0; j<vHistory.size(); j++ )
					for (int j = nHistoryIndex; j < vHistory.size(); j++)
					{
						if (pNextRGrid->m_nCost <= vHistory[j]->m_nCost)
						{
							vHistory.insert(vHistory.begin() + j, pNextRGrid);
							bInsert = true;
							break;
						}
					}
					if (!bInsert)
					{
						vHistory.push_back(pNextRGrid);
					}
					vAddGrid.push_back(pNextRGrid);
				}
			}

			// go to different layer
			//cout<<"different layer"<<endl;
			pNextGrid = graphTravel(m_pDesign, pGGrid, 0, 0, 1);
			if (pNextGrid != NULL && pNextGrid->getRemand() > 0)
			{
				rGrid_C *pNextRGrid = getRGrid(nX, nY, nZ + 1);
				if (!pNextRGrid->isRouted)
				{
					//pNextRGrid->isRouted = true;
					layer_C *pLayer = vLayer[nZ + 1 - m_nOffsetZ];
					pNextRGrid->m_nG = nNewDistance;
					int nSumH = 0;
					//for( int t=0; t<vTerminal.size(); t++ )
					//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
					nSumH = nEularDistance(vTerminal, pNextGrid, pLayer);
					pNextRGrid->m_nH = nSumH;
					pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
					pNextRGrid->m_pFrom = pCurRGrid;
					bool bInsert = false;
					//for( int j=0; j<vHistory.size(); j++ )
					for (int j = nHistoryIndex; j < vHistory.size(); j++)
					{
						if (pNextRGrid->m_nCost <= vHistory[j]->m_nCost)
						{
							vHistory.insert(vHistory.begin() + j, pNextRGrid);
							bInsert = true;
							break;
						}
					}
					if (!bInsert)
					{
						vHistory.push_back(pNextRGrid);
					}
					vAddGrid.push_back(pNextRGrid);
				}
			}

			if (nZ - 1 >= nLConstraint)
			{
				pNextGrid = graphTravel(m_pDesign, pGGrid, 0, 0, -1);
				if (pNextGrid != NULL && pNextGrid->getRemand() > 0)
				{
					rGrid_C *pNextRGrid = getRGrid(nX, nY, nZ - 1);
					if (!pNextRGrid->isRouted)
					{
						//pNextRGrid->isRouted = true;
						layer_C *pLayer = vLayer[nZ - 1 - m_nOffsetZ];
						pNextRGrid->m_nG = nNewDistance;
						int nSumH = 0;
						//for( int t=0; t<vTerminal.size(); t++ )
						//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
						nSumH = nEularDistance(vTerminal, pNextGrid, pLayer);
						pNextRGrid->m_nH = nSumH;
						pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
						pNextRGrid->m_pFrom = pCurRGrid;
						bool bInsert = false;
						//for( int j=0; j<vHistory.size(); j++ )
						for (int j = nHistoryIndex; j < vHistory.size(); j++)
						{
							if (pNextRGrid->m_nCost <= vHistory[j]->m_nCost)
							{
								vHistory.insert(vHistory.begin() + j, pNextRGrid);
								bInsert = true;
								break;
							}
						}
						if (!bInsert)
						{
							vHistory.push_back(pNextRGrid);
						}
						vAddGrid.push_back(pNextRGrid);
					}
				}
			}
		}
		//cout<<"Check target"<<endl;
		// if there is a target in addgrid;
		//vector< rGrid_C* > vHaveAddTarget;
		bool bFindTarget = false;
		for (int i = 0; i < vAddGrid.size(); i++)
		{
			if (vAddGrid[i]->isTarget)
			{
				bFindTarget = true;
				bool bHaveSearch = false;
				for (int j = 0; j < vHaveAddTarget.size(); j++)
				{
					if (vHaveAddTarget[j] == vAddGrid[i])
					{
						bHaveSearch = true;
						break;
					}
				}
				if (bHaveSearch)
					continue;

				vHaveAddTarget.push_back(vAddGrid[i]);
				//cout<<"Back trace"<<endl;
				// find the path
				vector<gGrid_C *> vReversePath;

				int nPath = vAddGrid[i]->m_nG;
				rGrid_C *pRPath = vAddGrid[i];
				pRPath->isPath = true;
				gGrid_C *pPathGrid = getGrid(m_pDesign, pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ);
				rGrid_C *pPreGrid = pRPath->m_pFrom;
				vReversePath.push_back(pPathGrid);
				while (!pPreGrid->isPath)
				{
					pRPath = pPreGrid;
					pRPath->isPath = true;
					pPathGrid = getGrid(m_pDesign, pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ);
					//vResult.push_back( pPathGrid );
					vReversePath.push_back(pPathGrid);
					pPreGrid = pRPath->m_pFrom;
				}

				pPathGrid = getGrid(m_pDesign, pPreGrid->m_nX, pPreGrid->m_nY, pPreGrid->m_nZ);
				//vResult.push_back( pPathGrid );
				vReversePath.push_back(pPathGrid);
				for (int g = vReversePath.size() - 1; g >= 0; g--)
				{
					vResult.push_back(vReversePath[g]);
				}

				// find the terminal and remove
				for (int t = 0; t < vTerminal.size(); t++)
				{
					int nTX, nTY, nTZ;
					vTerminal[t]->getPosition(nTX, nTY, nTZ);
					if (vAddGrid[i]->m_nX == nTX && vAddGrid[i]->m_nY == nTY && vAddGrid[i]->m_nZ == nTZ)
					{
						vTerminal.erase(vTerminal.begin() + t);
						nNumTermanals--;
						break;
					}
				}
				// update the nG in all the searched node ( without path )
				for (int h = 0; h < vHistory.size(); h++)
				{
					rGrid_C *pTmpGrid = vHistory[h];
					if (!pTmpGrid->isPath)
						calDistanceFromPath(pTmpGrid, nPath);
				}

				// update the nH in all the node in queue
				for (int h = nHistoryIndex; h < vHistory.size(); h++)
				{
					rGrid_C *pTmpGrid = vHistory[h];
					if (!pTmpGrid->isRouted)
					{
						int nH = 0;
						gGrid_C *pTmpGGrid = getGrid(m_pDesign, pTmpGrid->m_nX, pTmpGrid->m_nY, pTmpGrid->m_nZ);
						int nX, nY, nZ;
						pTmpGGrid->getPosition(nX, nY, nZ);
						layer_C *pLayer = vLayer[nZ - m_nOffsetZ];
						//for( int t=0; t<vTerminal.size(); t++ )
						//{
						//	nH = nH + nEularDistance( vTerminal[t], pTmpGGrid, pLayer );
						//}
						nH = nEularDistance(vTerminal, pTmpGGrid, pLayer);
						pTmpGrid->m_nH = nH;
						pTmpGrid->m_nCost = nH + pTmpGrid->m_nG;
					}
				}
				//cout<<endl;
			}
		}

		if (bFindTarget)
		{
			for (int i = nHistoryIndex; i < vHistory.size(); i++)
			{
				for (int j = i - 1; j >= 0; j--)
				{
					if (vHistory[j]->m_nCost > vHistory[j + 1]->m_nCost)
					{
						rGrid_C *pFGrid = vHistory[j];
						rGrid_C *pBGrid = vHistory[j + 1];
						vHistory[j] = pBGrid;
						vHistory[j + 1] = pFGrid;
					}
					else
						break;
				}
			}
		}
	}

	for (int i = 0; i < vHistory.size(); i++)
	{
		rGrid_C *pRGrid = vHistory[i];
		pRGrid->m_nH = 0;
		pRGrid->m_nG = 0;
		pRGrid->m_nCost = 0;
		pRGrid->m_nBend = 0;
		pRGrid->m_pFrom = NULL;
		pRGrid->isRouted = false;
		pRGrid->isPath = false;
		pRGrid->isTarget = false;
	}

	for (int i = 0; i < vResult.size(); i++)
	{
		int nX, nY, nZ;
		gGrid_C *pTmpGrid = vResult[i];
		pTmpGrid->getPosition(nX, nY, nZ);
		rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
		pRGrid->m_nH = 0;
		pRGrid->m_nG = 0;
		pRGrid->m_nCost = 0;
		pRGrid->m_nBend = 0;
		pRGrid->m_pFrom = NULL;
		pRGrid->isRouted = false;
		pRGrid->isPath = false;
		pRGrid->isTarget = false;
	}

	for (int i = 0; i < vTerminal.size(); i++)
	{
		int nX, nY, nZ;
		gGrid_C *pTmpGrid = vTerminal[i];
		pTmpGrid->getPosition(nX, nY, nZ);
		rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
		pRGrid->m_nH = 0;
		pRGrid->m_nG = 0;
		pRGrid->m_nCost = 0;
		pRGrid->m_nBend = 0;
		pRGrid->m_pFrom = NULL;
		pRGrid->isRouted = false;
		pRGrid->isPath = false;
		pRGrid->isTarget = false;
	}
	if (nNumTermanals != 0)
		vResult.clear();

	return vResult;
}

bool router_C::reduceOverflow( vector< net_C* > &vRoutedNet, gGrid_C* pOGrid, int &nLengthConstraint )
{
	net_C* pRipNet = NULL;
	bool bFix = false;
	vector< net_C* > vInGridNet = pOGrid->getNet();
	set< net_C* > sInGridNet;
	for( int n=0; n<vInGridNet.size(); n++ )
	{
		sInGridNet.insert( vInGridNet[n] );
	}
	for( int r=0; r<vRoutedNet.size(); r++ )
	{
		if( sInGridNet.count( vRoutedNet[r] ) == 0 ) // the net is not in the grid
			continue;

		vector< pin_C* > vPin = vRoutedNet[r]->getPin();
		bool bFind = false;
		for( int p=0; p<vPin.size(); p++ )
		{
			instance_C* pPInst = (instance_C*)vPin[p]->getCell();
			int nLayer = vPin[p]->getLayerId();
			gGrid_C* pPGrid = getGrid( m_pDesign, pPInst->getPlacedX(), pPInst->getPlacedY(), nLayer );
			if( pPGrid == pOGrid )
			{
				bFind = true;
				break;
			}
		}
		if( !bFind )
		{
			pRipNet = vRoutedNet[r];
			break;
		}
	
	}
	
	if( pRipNet != NULL )
	{
		nLengthConstraint = nLengthConstraint + pRipNet->getLength();
		ripupNet( pRipNet );
		pRipNet->cleanWire();
		//cout << "After Rip:" ;
		//cout << pOGrid->getRemand() << endl;
		vector< vector<gGrid_C *> > vNewResult = routeNet_length_constraint_ver2( pRipNet, nLengthConstraint);
		
		if( vNewResult.size() == 0 )
		{
			bFix = false;
		}
		else
		{
			bool bStillOverflow = false;
			for (int g = 0; g < vNewResult.size(); g++)
			{
				for (int gg = 0; gg < vNewResult[g].size(); gg++)
				{
					int nPinDemand = vNewResult[g][gg]->getPinDemand();
					
					if (vNewResult[g][gg]->getRemand() - 1 - vNewResult[g][gg]->getPinDemand() < 0)
					{
						bStillOverflow = true;
						break;
					}
				}
				if( bStillOverflow )
					break;
			}
			if( bStillOverflow )
			{
				// for the clean up, so save the net even it has overflow
				saveNet( pRipNet, vNewResult );
				addNetOnGraph( m_pDesign, pRipNet );
				nLengthConstraint = nLengthConstraint - pRipNet->getLength();
				bFix = false;
			}
			else
			{
				saveNet( pRipNet, vNewResult );
				addNetOnGraph( m_pDesign, pRipNet );
				nLengthConstraint = nLengthConstraint - pRipNet->getLength();
				bFix = true;
			}
		}
	}
	else
	{
		bFix = false;
	}
	return bFix;
}

//bool router_C::multiNetRouting_ver2( vector< net_C* > vNet, const int nConstraint, vector< net_C* > &vTmpRip )
bool router_C::multiNetRouting_ver2( vector< net_C* > vNet, int &nLengthConstraint, vector< net_C* > &vTmpRip )
{
	//cout << "Length constraint: "<<nConstraint << endl;
	bool bRouteAllNet = true;
	bool bSuccess = false;
	vector< net_C* > vRoutedNet;
	vector< net_C* > vRRRNet;
	//int nLengthConstraint = nConstraint;
	int nSuccessCount = 0;
	unordered_map< net_C*, int > mLengthTable;
	for (int i = 1; i < vNet.size(); i++)
	{
		for (int j = i - 1; j >= 0; j--)
		{
			int nLength_b = vNet[j + 1]->getLength();
			int nLength_f = vNet[j]->getLength();
			if( nLength_f <= nLength_b )
				break;
			else
			{
				net_C *pNet = vNet[j + 1];
				vNet[j + 1] = vNet[j];
				vNet[j] = pNet;
			}
		}
	}
	//for( int i=0; i<vNet.size(); i++ )
	//{
	//	cout << vNet[i]->getName() << " " << vNet[i]->getLength() << endl;
	//}

	for( int n=0; n < vNet.size(); n++ )
	{
		net_C* pNet = vNet[n];
		cout << pNet->getName() << endl;	
		vector<vector<gGrid_C *>> vResult = routeNet_length_constraint_ver3( pNet, nLengthConstraint );
		if (vResult.size() == 0)
		{
			bRouteAllNet = false;
			cout << "No routing path"<<endl;
			break;
		}
		else
		{
			/*
			cout << pNet->getName () << endl;
			for( int i=0; i<vResult.size(); i++ )
			{
				for( int j=0; j<vResult[i].size(); j++ )
				{
					int nX, nY, nZ;
					vResult[i][j]->getPosition( nX, nY, nZ );
					cout << "(" << nX <<" " << nY << " " << nZ << ") ";
				}
				cout << endl;
			}
			cout << endl;
			*/
			saveNet( pNet, vResult );
			addNetOnGraph( m_pDesign, pNet );
			nLengthConstraint = nLengthConstraint - pNet->getLength();
			
			vector< gGrid_C* > vOGrid = checkOverflow( vResult );
			bool bOverflow = false;	
			if( vOGrid.size() > 0  )
			{
				cout << "Fix overflow"<<endl;
				bOverflow = rrr_ver2( vRoutedNet, vOGrid, mLengthTable, nLengthConstraint, vRRRNet );
			}
			if( bOverflow )
			{
				cout << "Still overflow"<<endl;
				vRoutedNet.push_back( pNet );
				bRouteAllNet = false;
				break;
			}
			else	
				vRoutedNet.push_back( pNet );
		}
		
		nSuccessCount++;
	}

	bool bBetterSol = false;
	int nNewLength = 0;
	int nConstraint = 0;
	for( int i=0; i<m_vBackupNet.size(); i++ )
	{
		nConstraint = nConstraint + m_vBackupNet[i].getLength();
	}
	if( bRouteAllNet )
	{
		//cout << "After routing: "<<endl;
		for( int n=0; n<vNet.size(); n++ )
		{
			//if( n== 0 ) matlab_graph( "Before", vNet[n]->getName(), m_pDesign, false, 0 );		
			//else matlab_graph( "Before", vNet[n]->getName(), m_pDesign, false, 1 );		
			nNewLength = nNewLength + vNet[n]->getLength();
		//	cout << vNet[n]->getName() << " " << vNet[n]->getLength() << endl;	
		}
		for( int n=0; n<vRRRNet.size(); n++ )
		{
			nNewLength = nNewLength + vRRRNet[n]->getLength();
		//	cout << vRRRNet[n]->getName() << " " << vRRRNet[n]->getLength() << endl;
		}

		if( nNewLength < nConstraint )
		{
			bBetterSol = true;
		}
		else
		{
			bBetterSol = false;
		}
		//getchar();
	}
	
	// deal with the routing result
	if( !bRouteAllNet || !bBetterSol )
	{
		if( !bRouteAllNet )
		{
			//cout << "Some net failed"<<endl;
			//cout << "Complete "<< nSuccessCount << "/"  << vNet.size() << " nets"<<endl;
		}
		else
		{
			cout << "No better solution in routing "<<endl;
			cout << "Original Length: " << nConstraint << " New Length: "<< nNewLength<<endl;
		}
		/*
		vector< net_C* > vTmpCNet;
		for( int n=0; n<nSuccessCount; n++ )
		{
			ripupNet( vNet[n] );
			vTmpCNet.push_back( vNet[n] );
		}
		*/
		for( int i=0; i<vRoutedNet.size(); i++ )
		{
			for( int j=0; j<m_vBackupNet.size(); j++ )
			{
				if( vRoutedNet[i]->getName() == m_vBackupNet[j].getName() )
				{
					vRoutedNet[i]->setLength( m_vBackupNet[j].getLength() );
					break;
				}
			}
		}
		ripupNet( vRoutedNet );
		//cleanWire( vTmpCNet );
		cleanWire( vRoutedNet );
		//cout << "After recover: " << endl; 
		//for( int i=0; i<vRoutedNet.size(); i++ )
		//{
		//	cout << vRoutedNet[i]->getName() << " " << vRoutedNet[i]->getLength() << endl;
		//}
	
		ripupNet( vRRRNet );
		recoverNet( vRRRNet );
		
		for( int i=0; i<vRRRNet.size(); i++ )
		{
			addNetOnGraph( m_pDesign, vRRRNet[i] );
			for( int b=m_vBackupNet.size()-1; b>=0; b-- )
			{
				if( m_vBackupNet[b].getName() == vRRRNet[i]->getName() )
				{
					m_vBackupNet.erase( m_vBackupNet.begin() + b );
					break;
				}
			}
		}

		bSuccess = false;
	}
	else //( bRouteAllNet && bBetterSol )
	{	
		bSuccess = true;
		cout << "Original Length: " << nConstraint << " New Length: "<< nNewLength<<endl;
		for( int i=0; i<vRoutedNet.size(); i++ )
			cout << vRoutedNet[i]->getName() << " " << vRoutedNet[i]->getLength() << endl;
	}

	return bSuccess;
}


bool router_C::multiNetRouting( vector< net_C* > vNet, const int nConstraint  )
{
	//cout << "Length constraint: "<<nConstraint << endl;
	bool bRouteAllNet = true;
	bool bSuccess = false;
	vector< net_C* > vRoutedNet;
	int nLengthConstraint = nConstraint;
	int nSuccessCount = 0;
	for (int i = 1; i < vNet.size(); i++)
	{
		for (int j = i - 1; j >= 0; j--)
		{
			int nLength_b = vNet[j + 1]->getLength();
			int nLength_f = vNet[j]->getLength();
			if( nLength_f <= nLength_b )
				break;
			else
			{
				net_C *pNet = vNet[j + 1];
				vNet[j + 1] = vNet[j];
				vNet[j] = pNet;
			}
		}
	}

	for( int n=0; n < vNet.size(); n++ )
	{
		net_C* pNet = vNet[n];
		
		vector<vector<gGrid_C *>> vResult = routeNet_length_constraint_ver2( pNet, nLengthConstraint);
		if (vResult.size() == 0)
		{
			bRouteAllNet = false;
			cout << "No routing path"<<endl;
			break;
		}
		else
		{
			gGrid_C* pOGrid = NULL;
			bool bOverflow = false;
			int nOX, nOY, nOZ;
			vector< gGrid_C* > vOGrid;
			vector< int > vDemand;
			for (int g = 0; g < vResult.size(); g++)
			{
				for (int gg = 0; gg < vResult[g].size(); gg++)
					if (vResult[g][gg]->getRemand() - 1 < 0)
					{
						bOverflow = true;
						pOGrid = vResult[g][gg];
						pOGrid->getPosition( nOX, nOY, nOZ );
						//cout << "Overflow at: " << nOX << " " << nOY << " " << nOZ << endl;
						pOGrid->addPinDemand();
						vOGrid.push_back( pOGrid );
						//cout <<"Overflow here: ";
						//cout << pOGrid->getRemand() << endl;				
						bool bFix = reduceOverflow( vRoutedNet, pOGrid, nLengthConstraint );
						//pOGrid->delPinDemand();
						if( !bFix )
							break;
						else
						{
							if( pOGrid->getRemand() - 1 < 0 )
							{
								//cout << "Still overflow" <<endl;
								//getchar();
								break;
							}
							else
							{
								g = 0;
								gg = -1;
								bOverflow = false;
							}
							//getchar();
						}
					}
				if( bOverflow )
					break;
			}
			for( int o=0; o<vOGrid.size(); o++ )
			{
				vOGrid[o]->delPinDemand();
			}

			if( bOverflow )
			{
				bRouteAllNet = false;
				break;	
			}
		}
		
		saveNet( pNet, vResult );
		vRoutedNet.push_back( pNet );
		addNetOnGraph( m_pDesign, pNet );
		nLengthConstraint = nLengthConstraint - pNet->getLength();
		nSuccessCount++;
	}

	bool bBetterSol = false;
	int nNewLength = 0;
	if( bRouteAllNet )
	{
		for( int n=0; n<vNet.size(); n++ )
		{
			nNewLength = nNewLength + vNet[n]->getLength();
		}
		if( nNewLength < nConstraint )
		{
			bBetterSol = true;
		}
		else
		{
			bBetterSol = false;
		}
	}
	
	// deal with the routing result
	if( !bRouteAllNet || !bBetterSol )
	{
		if( !bRouteAllNet )
		{
			cout << "Some net failed"<<endl;
			cout << "Complete "<< nSuccessCount << "/"  << vNet.size() << " nets"<<endl;
		}
		else
		{
			cout << "No better solution in routing "<<endl;
			cout << "Original Length: " << nConstraint << " New Length: "<< nNewLength<<endl;
		}
		/*
		vector< net_C* > vTmpCNet;
		for( int n=0; n<nSuccessCount; n++ )
		{
			ripupNet( vNet[n] );
			vTmpCNet.push_back( vNet[n] );
		}
		*/
		for( int i=0; i<vRoutedNet.size(); i++ )
		{
			for( int j=0; j<m_vBackupNet.size(); j++ )
			{
				if( vRoutedNet[i]->getName() == m_vBackupNet[j].getName() )
				{
					vRoutedNet[i]->setLength( m_vBackupNet[j].getLength() );
					break;
				}
			}
		}
		ripupNet( vRoutedNet );
		//cleanWire( vTmpCNet );
		cleanWire( vRoutedNet );
		bSuccess = false;
	}
	else //( bRouteAllNet && bBetterSol )
	{	
		bSuccess = true;
		cout << "Original Length: " << nConstraint << " New Length: "<< nNewLength<<endl;
		//for( int i=0; i<vRoutedNet.size(); i++ )
		//	cout << vRoutedNet[i]->getName() << " " << vRoutedNet[i]->getLength() << endl;
	}

	return bSuccess;
}


bool router_C::multiNetRouting( vector< net_C* > &vNet, const int nConstraint, int &nCount )
{
	bool bRouteAllNet = true;
	vector< net_C* > vRoutedNet;
	int nLengthConstraint = nConstraint;
	for( int n=0; n < vNet.size(); n++ )
	{
		net_C* pNet = vNet[n];
		
		vector<vector<gGrid_C *>> vResult = routeNet_length_constraint_ver2( pNet, nLengthConstraint);
		if (vResult.size() == 0)
		{
			bRouteAllNet = false;
			cout << "No routing path"<<endl;
			break;
		}
		else
		{
			gGrid_C* pOGrid = NULL;
			bool bOverflow = false;
			int nOX, nOY, nOZ;
			vector< gGrid_C* > vOGrid;
			vector< int > vDemand;
			for (int g = 0; g < vResult.size(); g++)
			{
				for (int gg = 0; gg < vResult[g].size(); gg++)
					if (vResult[g][gg]->getRemand() - 1 < 0)
					{
						bOverflow = true;
						
						pOGrid = vResult[g][gg];
						pOGrid->getPosition( nOX, nOY, nOZ );
						cout << "Overflow at: " << nOX << " " << nOY << " " << nOZ << endl;
						pOGrid->addPinDemand();
						vOGrid.push_back( pOGrid );
						cout <<"Overflow here: ";
						cout << pOGrid->getRemand() << endl;				
						bool bFix = reduceOverflow( vRoutedNet, pOGrid, nLengthConstraint );
						//pOGrid->delPinDemand();
						if( !bFix )
							break;
						else
						{
							if( pOGrid->getRemand() - 1 < 0 )
							{
								cout << "Still overflow" <<endl;
								//getchar();
								break;
							}
							else
							{
								g = 0;
								gg = -1;
								bOverflow = false;
							}
							//getchar();
						}
						
						break;
					}
				if( bOverflow )
					break;
			}
			for( int o=0; o<vOGrid.size(); o++ )
			{
				vOGrid[o]->delPinDemand();
			}

			if( bOverflow )
			{
				bRouteAllNet = false;
				break;	
			}
		}
		
		saveNet( pNet, vResult );
		vRoutedNet.push_back( pNet );
		addNetOnGraph( m_pDesign, pNet );
		nLengthConstraint = nLengthConstraint - pNet->getLength();
		nCount++;
	}
	
	return bRouteAllNet;

}

vector< gGrid_C* > router_C::checkOverflow( vector< vector< gGrid_C* > > &vResult )
{
	vector< gGrid_C* > vOGrid;
	gGrid_C* pOGrid = NULL;
	int nOX, nOY, nOZ;
	for( int g = 0; g < vResult.size(); g++ )
	{
		for( int gg = 0; gg < vResult[g].size(); gg++ )
		{
			if( vResult[g][gg]->getRemand()  < 0 )
			{
				pOGrid = vResult[g][gg];
				pOGrid->getPosition( nOX, nOY, nOZ );
				vOGrid.push_back( pOGrid );
				//cout <<"Overflow at: "<< nOX << " " << nOY << " " << nOZ << endl;
				//cout << pOGrid->getRemand() << endl;				
			}
		}
	}
	return vOGrid;	
}

bool router_C::rrr_ver2( vector< net_C* > &vRoutedNet, vector< gGrid_C* > &vOGrid, unordered_map< net_C*, int > &mLengthTable, int &nLengthConstraint, vector< net_C* > &vTmpRip )
{
	bool bOverflow = true;
	set< net_C* > sRoutedNet;
	vector< net_C* > vRerouteNet;
	set< net_C* > sRerouteNet;
	vector< net_C > vLocalBackup;
	set< string > sLocalBackupNet;
	set< string > sBackupNet;
	for( int i=0; i<vRoutedNet.size(); i++ )
	{
		sRoutedNet.insert( vRoutedNet[i] );
		//mLengthTable[ vRoutedNet[i] ] = vRoutedNet[i]->getLength() - min( vRoutedNet[i]->getLength(), routeNet_ideal( vRoutedNet[i] ) ) ; 
		mLengthTable[ vRoutedNet[i] ] = 0 ; 
	}

	for( int i=0; i<m_vBackupNet.size(); i++ )
	{
		sBackupNet.insert( m_vBackupNet[i].getName() );
	}

	for( int i=0; i<vOGrid.size(); i++ )
	{
		if( vOGrid[i]->getRemand() < 0 )
		{
			//cout << "Num overflow: "<< vOGrid.size() << " at: "<< i << ": "; 
			bOverflow = true;
			gGrid_C* pOGrid = vOGrid[i];
			int nOX, nOY, nOZ;
			pOGrid->getPosition( nOX, nOY, nOZ );
			cout << "Overflow at: "<< nOX << " " << nOY << " " << nOZ << endl;
			vector< net_C* > vONet = pOGrid->getNet();
			vector< instance_C* > vInst = getGrid(m_pDesign, nOX, nOY, m_nOffsetZ )->getInstance();
			set< net_C* > sPinNet;
			/*
			for( int j=0; j<vInst.size(); j++ )
			{
				instance_C* pInst = vInst[j];
				forced_C* pF = &m_vForced[ pInst->getId() ];
				vector< networkForced_C* > &vNF = pF->m_vNetwork;
				for( int n=0; n<vNF.size(); n++ )
					sPinNet.insert( vNF[n]->m_pNet );
			}
			*/
			for( int j=0; j<vInst.size(); j++ )
			{
				instance_C* pInst = vInst[j];
				vector< pin_C > vPin = pInst->getPin();
				for( int k=0; k<vPin.size(); k++ )
				{
					if( vPin[k].getNet() == NULL )
						continue;
					int nLayer = vPin[k].getLayerId();
					int nLayerConstraint = vPin[k].getNet()->getConstraintLayerId();
					if( nLayerConstraint >= nOZ && nLayer <= nOZ || nLayer == nOZ )
						sPinNet.insert( vPin[k].getNet() );
				}
			}

			vector< net_C* > vCanRip;
			cout << "Net at grid " << nOX << " " << nOY << " " << nOZ <<" : ";
			for( int j=0; j<vONet.size(); j++ )
			{
				cout << vONet[j]->getName() << " ";
				if( sPinNet.count( vONet[j] ) == 0 )
					vCanRip.push_back( vONet[j] );
			}
			cout << endl;
			//cout << "Candidate nets: " << vCanRip.size() << endl;
			if( vCanRip.size() == 0 )
			{
				cout << "No net can rip" << endl;
				break;
			}
			else
			{
				do
				{
					net_C* pRipNet = NULL;
					int nMaxLength = 0;
					int nMinDLength = INT_MAX;
					int nIndex = -1;
					for( int j=0; j<vCanRip.size(); j++ )
					{
						if( sRerouteNet.count( vCanRip[j] ) == 0 && mLengthTable.find( vCanRip[j] ) == mLengthTable.end() )
						{
							//mLengthTable[ vCanRip[j] ] = vCanRip[j]->getLength() - min( vCanRip[j]->getLength(), routeNet_ideal( vCanRip[j]) );
							mLengthTable[ vCanRip[j] ] = 0;
						}
						int nDLength = mLengthTable.find( vCanRip[j] )->second;
						
						if( nDLength < nMinDLength )
						{
							nMinDLength = nDLength;
							nMaxLength = vCanRip[j]->getLength();
							pRipNet = vCanRip[j];
							nIndex = j;
						}
						else if( nDLength == nMinDLength && nMaxLength < vCanRip[j]->getLength() )
						{
							nMaxLength = vCanRip[j]->getLength();
							pRipNet = vCanRip[j];	
							nIndex = j;
						}

					}
					//cout << "Rip: " << pRipNet->getName() << " in length " << nMaxLength << endl;
					cout << "Reroute net: " << pRipNet->getName() << endl;
					nLengthConstraint = nLengthConstraint + nMaxLength;
					if( sLocalBackupNet.count( pRipNet->getName() ) == 0 )
					{
						backupNet( vLocalBackup, pRipNet );
						sLocalBackupNet.insert( pRipNet->getName() );
					}
					ripupNet( pRipNet );
					pRipNet->cleanWire();
					cout << "Detour length: " << nMinDLength << endl;	
					//vector< vector< gGrid_C* > > vReroute = routeNet_length_constraint_ver3( pRipNet, nLengthConstraint, nMinDLength );
					//vector< vector< gGrid_C* > > vReroute = routeNet_length_constraint_ver4( pRipNet, nMaxLength + nMinDLength );
					int nIdealLength = routeNet_ideal( pRipNet );
					vector< vector< gGrid_C* > > vReroute = routeNet_length_constraint_ver4( pRipNet, nIdealLength + nMinDLength );
					if( vReroute.size() != 0 )
					{
						
						/*
						for( int p=0; p<vReroute.size(); p++ )
						{
							for( int pp=0; pp<vReroute[p].size(); pp++ )
							{
								gGrid_C* pPGrid = vReroute[p][pp];
								int nPX, nPY, nPZ;
								pPGrid->getPosition( nPX, nPY, nPZ );
								cout << "("<<nPX << " " << nPY << " " << nPZ << ") ";
							}
							cout << endl;
						}
						cout << endl;
						*/
						vRerouteNet.push_back( pRipNet );
						sRerouteNet.insert( pRipNet );
						saveNet( pRipNet, vReroute );
						addNetOnGraph( m_pDesign, pRipNet );
						nLengthConstraint = nLengthConstraint - pRipNet->getLength();
						/*	
						if( nLengthConstraint < 0 )
						{
							nLengthConstraint = nLengthConstraint + pRipNet->getLength();
							nLengthConstraint = nLengthConstraint - nMaxLength;
							ripupNet( pRipNet );
							pRipNet->cleanWire();
							recoverNet( vLocalBackup, pRipNet );
							addNetOnGraph( m_pDesign, pRipNet );
							vLocalBackup.erase( vLocalBackup.end() - 1 );
							sLocalBackupNet.erase( pRipNet->getName() );
							vCanRip.erase( vCanRip.begin() + nIndex );
							vRerouteNet.erase( vRerouteNet.end() - 1 );
							sRoutedNet.erase( pRipNet );
							bOverflow = true;
							if( vCanRip.size() == 0 )
							{
								cout << "No net can ripup" << endl;
								break;
							}
							else
								continue;
						}
						*/
						//cout << "New length: " << pRipNet->getLength() << endl;
						
						vector< gGrid_C* > vNewOGrid = checkOverflow( vReroute );
						if( vNewOGrid.size() == 0 )
							bOverflow = false;
							
						else if( vOGrid[i]->getRemand() < 0 )
						{
							bOverflow = true;
						}
						else
						{
							bOverflow = false;
							for( int o=0; o<vNewOGrid.size(); o++ )
								vOGrid.push_back( vNewOGrid[o] );
						}
						break;
					}
					else // recover the net
					{
						nLengthConstraint = nLengthConstraint - nMaxLength;
						nMinDLength = nMinDLength + max( 1, ( nLengthConstraint - nMinDLength - 1 ) );
						//nMinDLength = nMinDLength + 1;
						mLengthTable[ pRipNet ] = nMinDLength;
						cout << pRipNet->getName() << " " << nMinDLength << " " << nLengthConstraint << endl;
						if( nMinDLength > nLengthConstraint )
							vCanRip.erase( vCanRip.begin() + nIndex );
						recoverNet( vLocalBackup, pRipNet );
						addNetOnGraph( m_pDesign, pRipNet );
						vLocalBackup.erase( vLocalBackup.end() - 1 );
						sLocalBackupNet.erase( pRipNet->getName() );
						if( vCanRip.size() == 0 )
						{
							cout << "No net can ripup" << endl;
							break;
						}
					}
				}
				while( bOverflow );

			}
			if( bOverflow )
				break;

		}
	}

	if( bOverflow )
	{
		ripupNet( vRerouteNet );
		recoverNet( vLocalBackup, vRerouteNet );
		//cout << "RRR recover: "<<endl;
		for( int i=0; i<vRerouteNet.size(); i++ )
		{
			//cout << vRerouteNet[i]->getName() << endl;
			addNetOnGraph( m_pDesign, vRerouteNet[i] );
		}
	}
	else
	{
		for( int i=0; i<vLocalBackup.size(); i++ )
		{
			if( sBackupNet.count( vLocalBackup[i].getName() ) == 0 )
			{
				//cout << "Add " << vLocalBackup[i].getName() << " in Backup Lib" << endl; 
				m_vBackupNet.push_back( vLocalBackup[i] );
				for( int j=0; j<vRerouteNet.size(); j++ )
				{
					if( vRerouteNet[j]->getName() == vLocalBackup[i].getName() )
					{
						vTmpRip.push_back( vRerouteNet[j] );
						break;
					}
				}
			}
		}
		for( int i=0; i<vRerouteNet.size(); i++ )
		{
			if( sRoutedNet.count( vRerouteNet[i] ) == 0 )
			{
				vRoutedNet.push_back( vRerouteNet[i] );
				sRoutedNet.insert( vRerouteNet[i] );
			}
		}
	}
	
	return bOverflow;
}


bool router_C::rrr( vector< net_C* > &vRoutedNet, vector< gGrid_C* > &vOGrid, unordered_map< net_C*, int > &mLengthTable, int &nLengthConstraint, vector< net_C* > &vTmpRip )
{
	bool bOverflow = true;
	set< net_C* > sRoutedNet;
	vector< net_C* > vRerouteNet;
	set< net_C* > sRerouteNet;
	vector< net_C > vLocalBackup;
	set< string > sLocalBackupNet;
	set< string > sBackupNet;
	for( int i=0; i<vRoutedNet.size(); i++ )
	{
		sRoutedNet.insert( vRoutedNet[i] );
		mLengthTable[ vRoutedNet[i] ] = vRoutedNet[i]->getLength(); 
	}

	for( int i=0; i<m_vBackupNet.size(); i++ )
	{
		sBackupNet.insert( m_vBackupNet[i].getName() );
	}

	for( int i=0; i<vOGrid.size(); i++ )
	{
		if( vOGrid[i]->getRemand() < 0 )
		{
			//cout << "Num overflow: "<< vOGrid.size() << " at: "<< i << ": "; 
			bOverflow = true;
			gGrid_C* pOGrid = vOGrid[i];
			int nOX, nOY, nOZ;
			pOGrid->getPosition( nOX, nOY, nOZ );
			cout << "Overflow at: "<< nOX << " " << nOY << " " << nOZ << endl;
			vector< net_C* > vONet = pOGrid->getNet();
			vector< instance_C* > vInst = getGrid(m_pDesign, nOX, nOY, m_nOffsetZ )->getInstance();
			set< net_C* > sPinNet;
			for( int j=0; j<vInst.size(); j++ )
			{
				instance_C* pInst = vInst[j];
				forced_C* pF = &m_vForced[ pInst->getId() ];
				vector< networkForced_C* > &vNF = pF->m_vNetwork;
				for( int n=0; n<vNF.size(); n++ )
					sPinNet.insert( vNF[n]->m_pNet );
			}

			vector< net_C* > vCanRip;
			for( int j=0; j<vONet.size(); j++ )
			{
				if( sPinNet.count( vONet[j] ) == 0 )
					vCanRip.push_back( vONet[j] );
			}
			//cout << "Candidate nets: " << vCanRip.size() << endl;
			if( vCanRip.size() == 0 )
			{
				break;
			}
			else
			{
				do
				{
					net_C* pRipNet = NULL;
					int nMaxLength = 0;
					int nMinDLength = INT_MAX;
					int nIndex = -1;
					for( int j=0; j<vCanRip.size(); j++ )
					{
						if( sRerouteNet.count( vCanRip[j] ) == 0 )
						{
							mLengthTable[ vCanRip[j] ] = vCanRip[j]->getLength();
						}
						int nDLength = vCanRip[j]->getLength() - mLengthTable.find( vCanRip[j] )->second;
						
						if( nDLength < nMinDLength )
						{
							nMinDLength = nDLength;
							nMaxLength = vCanRip[j]->getLength();
							pRipNet = vCanRip[j];
							nIndex = j;
						}
						else if( nDLength == nMinDLength && nMaxLength < vCanRip[j]->getLength() )
						{
							nMaxLength = vCanRip[j]->getLength();
							pRipNet = vCanRip[j];	
							nIndex = j;
						}

					}
					//cout << "Rip: " << pRipNet->getName() << " in length " << nMaxLength << endl;
					nLengthConstraint = nLengthConstraint + nMaxLength;
					if( sLocalBackupNet.count( pRipNet->getName() ) == 0 )
					{
						backupNet( vLocalBackup, pRipNet );
						sLocalBackupNet.insert( pRipNet->getName() );
					}
					ripupNet( pRipNet );
					pRipNet->cleanWire();
					//cout << "Detour length: " << nMinDLength << endl;	
					//vector< vector< gGrid_C* > > vReroute = routeNet_length_constraint_ver3( pRipNet, nLengthConstraint, nMinDLength );
					vector< vector< gGrid_C* > > vReroute = routeNet_length_constraint_ver4( pRipNet, nLengthConstraint );
					if( vReroute.size() != 0 )
					{
						
						for( int p=0; p<vReroute.size(); p++ )
						{
							for( int pp=0; pp<vReroute[p].size(); pp++ )
							{
								gGrid_C* pPGrid = vReroute[p][pp];
								int nPX, nPY, nPZ;
								pPGrid->getPosition( nPX, nPY, nPZ );
								cout << "("<<nPX << " " << nPY << " " << nPZ << ") ";
							}
							cout << endl;
						}
						cout << endl;
						
						vRerouteNet.push_back( pRipNet );
						sRerouteNet.insert( pRipNet );
						saveNet( pRipNet, vReroute );
						addNetOnGraph( m_pDesign, pRipNet );
						nLengthConstraint = nLengthConstraint - pRipNet->getLength();
						//cout << "New length: " << pRipNet->getLength() << endl;
						vector< gGrid_C* > vNewOGrid = checkOverflow( vReroute );
						if( vNewOGrid.size() == 0 )
							bOverflow = false;
							
						else
						{
							for( int o=0; o<vNewOGrid.size(); o++ )
								vOGrid.push_back( vNewOGrid[o] );
						}
						break;
					}
					else // recover the net
					{
						nLengthConstraint = nLengthConstraint - nMaxLength;
						vCanRip.erase( vCanRip.begin() + nIndex );
						recoverNet( vLocalBackup, pRipNet );
						addNetOnGraph( m_pDesign, pRipNet );
						vLocalBackup.erase( vLocalBackup.end() - 1 );
						sLocalBackupNet.erase( pRipNet->getName() );
						if( vCanRip.size() == 0 )
							break;
					}
				}
				while( bOverflow );

			}
			if( bOverflow )
				break;

		}
	}

	if( bOverflow )
	{
		ripupNet( vRerouteNet );
		recoverNet( vLocalBackup, vRerouteNet );
		//cout << "RRR recover: "<<endl;
		for( int i=0; i<vRerouteNet.size(); i++ )
		{
			//cout << vRerouteNet[i]->getName() << endl;
			addNetOnGraph( m_pDesign, vRerouteNet[i] );
		}
	}
	else
	{
		for( int i=0; i<vLocalBackup.size(); i++ )
		{
			if( sBackupNet.count( vLocalBackup[i].getName() ) == 0 )
			{
				//cout << "Add " << vLocalBackup[i].getName() << " in Backup Lib" << endl; 
				m_vBackupNet.push_back( vLocalBackup[i] );
				for( int j=0; j<vRerouteNet.size(); j++ )
				{
					if( vRerouteNet[j]->getName() == vLocalBackup[i].getName() )
					{
						vTmpRip.push_back( vRerouteNet[j] );
						break;
					}
				}
			}
		}
		for( int i=0; i<vRerouteNet.size(); i++ )
		{
			if( sRoutedNet.count( vRerouteNet[i] ) == 0 )
			{
				vRoutedNet.push_back( vRerouteNet[i] );
				sRoutedNet.insert( vRerouteNet[i] );
			}
		}
	}
	
	return bOverflow;
}


int router_C::routeNet_ideal(net_C *pNet )
{
	//cout<<pNet->getName()<<endl;
	setRoutingConstraint(pNet);
	int &nLConstraint = m_nConstraintLayerId;
	//cout<<nLConstraint<<endl;

	vector<gGrid_C *> vTerminal;
	vector<pin_C *> vPin = pNet->getPin();
	//vector< wire_C* > vWire;
	vector<vector<gGrid_C *>> vResult;
	set< gGrid_C* > sLength;
	for (int i = 0; i < vPin.size(); i++)
	{
		pin_C *pPin = vPin[i];
		int nX, nY, nZ;
		instance_C *pInst = (instance_C *)pPin->getCell();
		nX = pInst->getPlacedX();
		nY = pInst->getPlacedY();
		// project the z to the constraint layer
		nZ = pPin->getLayerId();
		if (nZ < nLConstraint)
		{
			vector<gGrid_C *> vTmpResult;
			for (int l = nZ; l <= nLConstraint; l++)
			{
				gGrid_C *pGrid = getGrid(m_pDesign, nX, nY, l);
				vTmpResult.push_back(pGrid);
			}
			nZ = nLConstraint;
			vResult.push_back(vTmpResult);
		}

		gGrid_C *pTerminal = getGrid(m_pDesign, nX, nY, nZ);
		rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
		if (!pRGrid->isTarget) // if pRGrid is already put in vTerminal
		{
			pRGrid->isTarget = true;
			vTerminal.push_back(pTerminal);
		}
	}

	if (vTerminal.size() == 1)
	{
		int nX, nY, nZ;
		vTerminal[0]->getPosition(nX, nY, nZ);
		rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
		pRGrid->isTarget = false;
		vector<gGrid_C *> vTmpResult;
		vTmpResult.push_back(vTerminal[0]);
		vResult.push_back(vTmpResult);
		for( int i=0; i<vResult.size(); i++ )
		{
			for( int j=0; j<vResult[i].size(); j++ )
			{
				sLength.insert( vResult[i][j] );
			}
		}
		
		return sLength.size();
	}

	vector<rGrid_C *> vRoutedGrid;
	//vector< rGrid_C* > vPriorityQueue;
	vector<rGrid_C *> vHistory;
	vector<rGrid_C *> vNotInPath;
	// find first terminal
	gGrid_C *pFirst = NULL;

	int nMinX = m_nTX;
	for (int i = 0; i < vTerminal.size(); i++)
	{
		gGrid_C *pGrid = vTerminal[i];
		int nX, nY, nZ;
		pGrid->getPosition(nX, nY, nZ);
		if (nX <= nMinX)
			nMinX = nX;
	}

	int nMinY = m_nTY;
	for (int i = 0; i < vTerminal.size(); i++)
	{
		gGrid_C *pGrid = vTerminal[i];
		int nX, nY, nZ;
		pGrid->getPosition(nX, nY, nZ);
		if (nX == nMinX && nY <= nMinY)
		{
			nMinY = nY;
			pFirst = vTerminal[i];
		}
	}

	//
	//pFirst = vTerminal[0];

	//chech the terminal is available to route the net
	int nX, nY, nZ;
	pFirst->getPosition(nX, nY, nZ);
	rGrid_C *pCurRGrid = getRGrid(nX, nY, nZ);
	gGrid_C *pGGrid = pFirst;

	vHistory.push_back(pCurRGrid);
	pCurRGrid->isPath = true;
	for (int i = 0; i < vTerminal.size(); i++)
	{
		if (vTerminal[i] == pGGrid)
		{
			vTerminal.erase(vTerminal.begin() + i);
			break;
		}
	}
	int nNumTermanals = vTerminal.size();
	vector<layer_C *> vLayer = m_pDesign->getLayer();
	//cout<<"Start routing"<<endl;
	vector<rGrid_C *> vHaveAddTarget;
	int nHistoryIndex = 0;
	while (nNumTermanals > 0)
	{
		//cout<<nNumTermanals<<endl;
		//cout<<pCurRGrid->m_nX<<" "<<pCurRGrid->m_nY<<" "<<pCurRGrid->m_nZ<<endl;
		// A* propogation
		if (vHistory.size() == 0)
		{
			//cout << "break at size"<<endl;
			vResult.clear();
			break;
		}
		//cout<<vHistory.size()<<endl;

		vector<rGrid_C *> vToPropogate;
		int nMinCost = 0;
		
		if (nHistoryIndex == vHistory.size())
		{
			//cout << "break at index" << endl;
			break;
		}
		else
		{
			nMinCost = vHistory[nHistoryIndex]->m_nCost;
			vToPropogate.push_back(vHistory[nHistoryIndex]);
			//nHistoryIndex++;
		}
		nHistoryIndex++;
		//cout << "now: " << nHistoryIndex << endl;	
		vector<rGrid_C *> vAddGrid;

		for (int i = 0; i < vToPropogate.size(); i++)
		{
			pCurRGrid = vToPropogate[i];
			pCurRGrid->isRouted = true;
			int nCurDistance = pCurRGrid->m_nG;
			int nNewDistance = nCurDistance + 1;
			
			int nX, nY, nZ;
			nX = pCurRGrid->m_nX;
			nY = pCurRGrid->m_nY;
			nZ = pCurRGrid->m_nZ;
			//cout<<"P: "<<nX<<" "<<nY<<" "<<nZ<<endl;
			pGGrid = getGrid(m_pDesign, nX, nY, nZ);
			char cDir = vLayer[nZ - m_nOffsetZ]->getDir();
			//cout<<cDir<<endl;
			int nDX, nDY, nDZ;
			if (cDir == 'H')
			{
				nDX = 0;
				nDY = 1;
				nDZ = 0;
			}
			else if (cDir == 'V')
			{
				nDX = 1;
				nDY = 0;
				nDZ = 0;
			}
			else
				cout << "Unknown routing direction" << endl;
			// go in the same layer;
			gGrid_C *pNextGrid = graphTravel(m_pDesign, pGGrid, nDX, nDY, nDZ);
			if( pNextGrid != NULL )
			{
				rGrid_C *pNextRGrid = getRGrid(nX + nDX, nY + nDY, nZ + nDZ);
				if (!pNextRGrid->isRouted)
				{
					//pNextRGrid->isRouted = true;
					layer_C *pLayer = vLayer[nZ + nDZ - m_nOffsetZ];
					pNextRGrid->m_nG = nNewDistance;
					int nSumH = 0;
					//for( int t=0; t<vTerminal.size(); t++ )
					//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
					nSumH = nEularDistance(vTerminal, pNextGrid, pLayer);
					pNextRGrid->m_nH = nSumH;
					pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;

					pNextRGrid->m_pFrom = pCurRGrid;
					bool bInsert = false;
					//for( int j=0; j<vHistory.size(); j++ )
					for (int j = nHistoryIndex; j < vHistory.size(); j++)
					{
						if (pNextRGrid->m_nCost <= vHistory[j]->m_nCost)
						{
							//cout << "insert at " << j << endl;
							vHistory.insert(vHistory.begin() + j, pNextRGrid);
							bInsert = true;
							break;
						}
					}
					if (!bInsert)
					{
						vHistory.push_back(pNextRGrid);
							//cout << "insert at last"<<endl;
					}
					vAddGrid.push_back(pNextRGrid);
				}
			}
			//cout<<"test"<<endl;
			pNextGrid = graphTravel(m_pDesign, pGGrid, -nDX, -nDY, -nDZ);
			if( pNextGrid != NULL )
			{
				rGrid_C *pNextRGrid = getRGrid(nX - nDX, nY - nDY, nZ - nDZ);
				if (!pNextRGrid->isRouted)
				{
					//pNextRGrid->isRouted = true;
					layer_C *pLayer = vLayer[nZ - nDZ - m_nOffsetZ];
					pNextRGrid->m_nG = nNewDistance;
					int nSumH = 0;
					//for( int t=0; t<vTerminal.size(); t++ )
					//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
					nSumH = nEularDistance(vTerminal, pNextGrid, pLayer);
					pNextRGrid->m_nH = nSumH;
					pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
					pNextRGrid->m_pFrom = pCurRGrid;
					bool bInsert = false;
					//for( int j=0; j<vHistory.size(); j++ )
					for (int j = nHistoryIndex; j < vHistory.size(); j++)
					{
						if (pNextRGrid->m_nCost <= vHistory[j]->m_nCost)
						{
							vHistory.insert(vHistory.begin() + j, pNextRGrid);
							//cout << "insert at " << j << endl;
							bInsert = true;
							break;
						}
					}
					if (!bInsert)
					{
						vHistory.push_back(pNextRGrid);
							//cout << "insert at last"<<endl;
					}
					vAddGrid.push_back(pNextRGrid);
				}
			}

			// go to different layer
			//cout<<"different layer"<<endl;
			pNextGrid = graphTravel(m_pDesign, pGGrid, 0, 0, 1);
			if (pNextGrid != NULL )
			{
				rGrid_C *pNextRGrid = getRGrid(nX, nY, nZ + 1);
				if (!pNextRGrid->isRouted)
				{
					//pNextRGrid->isRouted = true;
					layer_C *pLayer = vLayer[nZ + 1 - m_nOffsetZ];
					pNextRGrid->m_nG = nNewDistance;
					int nSumH = 0;
					//for( int t=0; t<vTerminal.size(); t++ )
					//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
					nSumH = nEularDistance(vTerminal, pNextGrid, pLayer);
					pNextRGrid->m_nH = nSumH;
					pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
					pNextRGrid->m_pFrom = pCurRGrid;
					bool bInsert = false;
					//for( int j=0; j<vHistory.size(); j++ )
					for (int j = nHistoryIndex; j < vHistory.size(); j++)
					{
						if (pNextRGrid->m_nCost <= vHistory[j]->m_nCost)
						{
							vHistory.insert(vHistory.begin() + j, pNextRGrid);
							//cout << "insert at " << j << endl;
							bInsert = true;
							break;
						}
					}
					if (!bInsert)
					{
						vHistory.push_back(pNextRGrid);
							//cout << "insert at last"<<endl;
					}
					vAddGrid.push_back(pNextRGrid);
				}
			}

			if (nZ - 1 >= nLConstraint)
			{
				pNextGrid = graphTravel(m_pDesign, pGGrid, 0, 0, -1);
				if (pNextGrid != NULL )
				{
					rGrid_C *pNextRGrid = getRGrid(nX, nY, nZ - 1);
					if (!pNextRGrid->isRouted)
					{
						//pNextRGrid->isRouted = true;
						layer_C *pLayer = vLayer[nZ - 1 - m_nOffsetZ];
						pNextRGrid->m_nG = nNewDistance;
						int nSumH = 0;
						//for( int t=0; t<vTerminal.size(); t++ )
						//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
						nSumH = nEularDistance(vTerminal, pNextGrid, pLayer);
						pNextRGrid->m_nH = nSumH;
						pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
						pNextRGrid->m_pFrom = pCurRGrid;
						bool bInsert = false;
						//for( int j=0; j<vHistory.size(); j++ )
						for (int j = nHistoryIndex; j < vHistory.size(); j++)
						{
							if (pNextRGrid->m_nCost <= vHistory[j]->m_nCost)
							{
								vHistory.insert(vHistory.begin() + j, pNextRGrid);
								//cout << "insert at " << j << endl;
								bInsert = true;
								break;
							}
						}
						if (!bInsert)
						{
							vHistory.push_back(pNextRGrid);
							//cout << "insert at last"<<endl;
						}
						vAddGrid.push_back(pNextRGrid);
					}
				}
			}
		}
		//cout<<"Check target"<<endl;
		// if there is a target in addgrid;
		//vector< rGrid_C* > vHaveAddTarget;
		bool bFindTarget = false;
		for (int i = 0; i < vAddGrid.size(); i++)
		{
			if (vAddGrid[i]->isTarget)
			{
				//if( pNet->getName() == "N1482")
				//	cout<<"Target is: "<<vAddGrid[i]->m_nX<<" "<<vAddGrid[i]->m_nY<<" "<<vAddGrid[i]->m_nZ<<" "<<endl;
				bFindTarget = true;
				bool bHaveSearch = false;
				for (int j = 0; j < vHaveAddTarget.size(); j++)
				{
					if (vHaveAddTarget[j] == vAddGrid[i])
					{
						bHaveSearch = true;
						break;
					}
				}
				if (bHaveSearch)
					continue;

				vHaveAddTarget.push_back(vAddGrid[i]);
				//cout<<"Back trace"<<endl;
				// find the path
				vector<gGrid_C *> vReversePath;

				int nPath = vAddGrid[i]->m_nG;
				//if( pNet->getName() == "N1482")
				//	cout<<"Path cost: "<<nPath<<endl;
				rGrid_C *pRPath = vAddGrid[i];
				pRPath->isPath = true;
				gGrid_C *pPathGrid = getGrid(m_pDesign, pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ);
				rGrid_C *pPreGrid = pRPath->m_pFrom;
				//gGrid_C* pPathGrid = getGrid( m_pDesign, pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ );
				//vResult.push_back( pPathGrid );
				vReversePath.push_back(pPathGrid);
				while (!pPreGrid->isPath)
				{
					pRPath = pPreGrid;
					pRPath->isPath = true;
					pPathGrid = getGrid(m_pDesign, pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ);
					//vResult.push_back( pPathGrid );
					vReversePath.push_back(pPathGrid);
					pPreGrid = pRPath->m_pFrom;
				}

				pPathGrid = getGrid(m_pDesign, pPreGrid->m_nX, pPreGrid->m_nY, pPreGrid->m_nZ);
				//vResult.push_back( pPathGrid );
				vReversePath.push_back(pPathGrid);
				vector<gGrid_C *> vTmpResult;
				for (int g = vReversePath.size() - 1; g >= 0; g--)
				{
					vTmpResult.push_back(vReversePath[g]);
				}

				vResult.push_back(vTmpResult);
				// find the terminal and remove
				for (int t = 0; t < vTerminal.size(); t++)
				{
					int nTX, nTY, nTZ;
					vTerminal[t]->getPosition(nTX, nTY, nTZ);
					if (vAddGrid[i]->m_nX == nTX && vAddGrid[i]->m_nY == nTY && vAddGrid[i]->m_nZ == nTZ)
					{
						vTerminal.erase(vTerminal.begin() + t);
						nNumTermanals--;
						break;
					}
				}
				// update the nG in all the searched node ( without path )
				for (int h = 0; h < vHistory.size(); h++)
				{
					rGrid_C *pTmpGrid = vHistory[h];
					if (!pTmpGrid->isPath)
						calDistanceFromPath(pTmpGrid, nPath);
					else
						pTmpGrid->m_nG = nPath;
				}

				// update the nH in all the node in queue
				for (int h = nHistoryIndex; h < vHistory.size(); h++)
				{
					rGrid_C *pTmpGrid = vHistory[h];
					if (!pTmpGrid->isRouted)
					{
						int nH = 0;
						gGrid_C *pTmpGGrid = getGrid(m_pDesign, pTmpGrid->m_nX, pTmpGrid->m_nY, pTmpGrid->m_nZ);
						int nX, nY, nZ;
						pTmpGGrid->getPosition(nX, nY, nZ);
						layer_C *pLayer = vLayer[nZ - m_nOffsetZ];
						nH = nEularDistance(vTerminal, pTmpGGrid, pLayer);
						pTmpGrid->m_nH = nH;
						pTmpGrid->m_nCost = nH + pTmpGrid->m_nG;
					}
				}
				//cout<<endl;
			}
		}

		if (bFindTarget)
		{
			for (int i = nHistoryIndex; i < vHistory.size(); i++)
			{
				for (int j = i - 1; j >= nHistoryIndex; j--)
				{
					if (vHistory[j]->m_nCost > vHistory[j + 1]->m_nCost)
					{
						rGrid_C *pFGrid = vHistory[j];
						rGrid_C *pBGrid = vHistory[j + 1];
						vHistory[j] = pBGrid;
						vHistory[j + 1] = pFGrid;
					}
					else if (vHistory[j]->m_nCost ==  vHistory[j + 1]->m_nCost)
					{
						rGrid_C *pFGrid = vHistory[j];
						rGrid_C *pBGrid = vHistory[j + 1];
						if( pFGrid->m_nZ == pCurRGrid->m_nZ )
							break;
						else if( pBGrid->m_nZ == pCurRGrid->m_nZ )
						{	
							vHistory[j] = pBGrid;
							vHistory[j + 1] = pFGrid;	
						}
						else
							break;
					}
					else
						break;
				}
			}
		}
	}

	for (int i = 0; i < vHistory.size(); i++)
	{
		rGrid_C *pRGrid = vHistory[i];
		pRGrid->m_nH = 0;
		pRGrid->m_nG = 0;
		pRGrid->m_nCost = 0;
		pRGrid->m_nBend = 0;
		pRGrid->m_pFrom = NULL;
		pRGrid->isRouted = false;
		pRGrid->isPath = false;
		pRGrid->isTarget = false;
	}

	for (int i = 0; i < vTerminal.size(); i++)
	{
		int nX, nY, nZ;
		gGrid_C *pTmpGrid = vTerminal[i];
		pTmpGrid->getPosition(nX, nY, nZ);
		rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
		pRGrid->m_nH = 0;
		pRGrid->m_nG = 0;
		pRGrid->m_nCost = 0;
		pRGrid->m_nBend = 0;
		pRGrid->m_pFrom = NULL;
		pRGrid->isRouted = false;
		pRGrid->isPath = false;
		pRGrid->isTarget = false;
	}

	//if( pNet->getName() == "N92" )
	//	cout << "N92" << endl;

	for( int i=0; i<vResult.size(); i++ )
	{
		for( int j=0; j<vResult[i].size(); j++ )
		{
			//if( pNet->getName() == "N92" )
			//{
			//	int nX, nY, nZ;
			//	vResult[i][j]->getPosition( nX, nY, nZ );
			//	cout << nX << " " << nY << " " << nZ << endl;
			//}
			sLength.insert( vResult[i][j] );
		}
	}
	
	return sLength.size();
}

int router_C::routeNet_ideal(net_C *pNet, set< instance_C* > &sUnplacedInst )
{
	//cout<<pNet->getName()<<endl;
	setRoutingConstraint(pNet);
	int &nLConstraint = m_nConstraintLayerId;
	//cout<<nLConstraint<<endl;

	vector<gGrid_C *> vTerminal;
	vector<pin_C *> vPin = pNet->getPin();
	//vector< wire_C* > vWire;
	vector<vector<gGrid_C *>> vResult;
	set< gGrid_C* > sLength;
	for (int i = 0; i < vPin.size(); i++)
	{
		pin_C *pPin = vPin[i];
		int nX, nY, nZ;
		instance_C *pInst = (instance_C *)pPin->getCell();
		if( sUnplacedInst.count( pInst ) != 0 )
			continue;
		nX = pInst->getPlacedX();
		nY = pInst->getPlacedY();
		// project the z to the constraint layer
		nZ = pPin->getLayerId();
		if (nZ < nLConstraint)
		{
			vector<gGrid_C *> vTmpResult;
			for (int l = nZ; l <= nLConstraint; l++)
			{
				gGrid_C *pGrid = getGrid(m_pDesign, nX, nY, l);
				vTmpResult.push_back(pGrid);
			}
			nZ = nLConstraint;
			vResult.push_back(vTmpResult);
		}

		gGrid_C *pTerminal = getGrid(m_pDesign, nX, nY, nZ);
		rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
		if (!pRGrid->isTarget) // if pRGrid is already put in vTerminal
		{
			pRGrid->isTarget = true;
			vTerminal.push_back(pTerminal);
		}
	}

	if (vTerminal.size() == 1)
	{
		int nX, nY, nZ;
		vTerminal[0]->getPosition(nX, nY, nZ);
		rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
		pRGrid->isTarget = false;
		vector<gGrid_C *> vTmpResult;
		vTmpResult.push_back(vTerminal[0]);
		vResult.push_back(vTmpResult);
		for( int i=0; i<vResult.size(); i++ )
		{
			for( int j=0; j<vResult[i].size(); j++ )
			{
				sLength.insert( vResult[i][j] );
			}
		}
		
		return sLength.size();
	}

	vector<rGrid_C *> vRoutedGrid;
	//vector< rGrid_C* > vPriorityQueue;
	vector<rGrid_C *> vHistory;
	vector<rGrid_C *> vNotInPath;
	// find first terminal
	gGrid_C *pFirst = NULL;

	int nMinX = m_nTX;
	for (int i = 0; i < vTerminal.size(); i++)
	{
		gGrid_C *pGrid = vTerminal[i];
		int nX, nY, nZ;
		pGrid->getPosition(nX, nY, nZ);
		if (nX <= nMinX)
			nMinX = nX;
	}

	int nMinY = m_nTY;
	for (int i = 0; i < vTerminal.size(); i++)
	{
		gGrid_C *pGrid = vTerminal[i];
		int nX, nY, nZ;
		pGrid->getPosition(nX, nY, nZ);
		if (nX == nMinX && nY <= nMinY)
		{
			nMinY = nY;
			pFirst = vTerminal[i];
		}
	}

	//
	//pFirst = vTerminal[0];

	//chech the terminal is available to route the net
	int nX, nY, nZ;
	pFirst->getPosition(nX, nY, nZ);
	rGrid_C *pCurRGrid = getRGrid(nX, nY, nZ);
	gGrid_C *pGGrid = pFirst;

	vHistory.push_back(pCurRGrid);
	pCurRGrid->isPath = true;
	for (int i = 0; i < vTerminal.size(); i++)
	{
		if (vTerminal[i] == pGGrid)
		{
			vTerminal.erase(vTerminal.begin() + i);
			break;
		}
	}
	int nNumTermanals = vTerminal.size();
	vector<layer_C *> vLayer = m_pDesign->getLayer();
	//cout<<"Start routing"<<endl;
	vector<rGrid_C *> vHaveAddTarget;
	int nHistoryIndex = 0;
	while (nNumTermanals > 0)
	{
		//cout<<nNumTermanals<<endl;
		//cout<<pCurRGrid->m_nX<<" "<<pCurRGrid->m_nY<<" "<<pCurRGrid->m_nZ<<endl;
		// A* propogation
		if (vHistory.size() == 0)
		{
			//cout << "break at size"<<endl;
			vResult.clear();
			break;
		}
		//cout<<vHistory.size()<<endl;

		vector<rGrid_C *> vToPropogate;
		int nMinCost = 0;
		
		if (nHistoryIndex == vHistory.size())
		{
			//cout << "break at index" << endl;
			break;
		}
		else
		{
			nMinCost = vHistory[nHistoryIndex]->m_nCost;
			vToPropogate.push_back(vHistory[nHistoryIndex]);
			//nHistoryIndex++;
		}
		nHistoryIndex++;
		//cout << "now: " << nHistoryIndex << endl;	
		vector<rGrid_C *> vAddGrid;

		for (int i = 0; i < vToPropogate.size(); i++)
		{
			pCurRGrid = vToPropogate[i];
			pCurRGrid->isRouted = true;
			int nCurDistance = pCurRGrid->m_nG;
			int nNewDistance = nCurDistance + 1;
			
			int nX, nY, nZ;
			nX = pCurRGrid->m_nX;
			nY = pCurRGrid->m_nY;
			nZ = pCurRGrid->m_nZ;
			//cout<<"P: "<<nX<<" "<<nY<<" "<<nZ<<endl;
			pGGrid = getGrid(m_pDesign, nX, nY, nZ);
			char cDir = vLayer[nZ - m_nOffsetZ]->getDir();
			//cout<<cDir<<endl;
			int nDX, nDY, nDZ;
			if (cDir == 'H')
			{
				nDX = 0;
				nDY = 1;
				nDZ = 0;
			}
			else if (cDir == 'V')
			{
				nDX = 1;
				nDY = 0;
				nDZ = 0;
			}
			else
				cout << "Unknown routing direction" << endl;
			// go in the same layer;
			gGrid_C *pNextGrid = graphTravel(m_pDesign, pGGrid, nDX, nDY, nDZ);
			if( pNextGrid != NULL )
			{
				rGrid_C *pNextRGrid = getRGrid(nX + nDX, nY + nDY, nZ + nDZ);
				if (!pNextRGrid->isRouted)
				{
					//pNextRGrid->isRouted = true;
					layer_C *pLayer = vLayer[nZ + nDZ - m_nOffsetZ];
					pNextRGrid->m_nG = nNewDistance;
					int nSumH = 0;
					//for( int t=0; t<vTerminal.size(); t++ )
					//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
					nSumH = nEularDistance(vTerminal, pNextGrid, pLayer);
					pNextRGrid->m_nH = nSumH;
					pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;

					pNextRGrid->m_pFrom = pCurRGrid;
					bool bInsert = false;
					//for( int j=0; j<vHistory.size(); j++ )
					for (int j = nHistoryIndex; j < vHistory.size(); j++)
					{
						if (pNextRGrid->m_nCost <= vHistory[j]->m_nCost)
						{
							//cout << "insert at " << j << endl;
							vHistory.insert(vHistory.begin() + j, pNextRGrid);
							bInsert = true;
							break;
						}
					}
					if (!bInsert)
					{
						vHistory.push_back(pNextRGrid);
							//cout << "insert at last"<<endl;
					}
					vAddGrid.push_back(pNextRGrid);
				}
			}
			//cout<<"test"<<endl;
			pNextGrid = graphTravel(m_pDesign, pGGrid, -nDX, -nDY, -nDZ);
			if( pNextGrid != NULL )
			{
				rGrid_C *pNextRGrid = getRGrid(nX - nDX, nY - nDY, nZ - nDZ);
				if (!pNextRGrid->isRouted)
				{
					//pNextRGrid->isRouted = true;
					layer_C *pLayer = vLayer[nZ - nDZ - m_nOffsetZ];
					pNextRGrid->m_nG = nNewDistance;
					int nSumH = 0;
					//for( int t=0; t<vTerminal.size(); t++ )
					//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
					nSumH = nEularDistance(vTerminal, pNextGrid, pLayer);
					pNextRGrid->m_nH = nSumH;
					pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
					pNextRGrid->m_pFrom = pCurRGrid;
					bool bInsert = false;
					//for( int j=0; j<vHistory.size(); j++ )
					for (int j = nHistoryIndex; j < vHistory.size(); j++)
					{
						if (pNextRGrid->m_nCost <= vHistory[j]->m_nCost)
						{
							vHistory.insert(vHistory.begin() + j, pNextRGrid);
							//cout << "insert at " << j << endl;
							bInsert = true;
							break;
						}
					}
					if (!bInsert)
					{
						vHistory.push_back(pNextRGrid);
							//cout << "insert at last"<<endl;
					}
					vAddGrid.push_back(pNextRGrid);
				}
			}

			// go to different layer
			//cout<<"different layer"<<endl;
			pNextGrid = graphTravel(m_pDesign, pGGrid, 0, 0, 1);
			if (pNextGrid != NULL )
			{
				rGrid_C *pNextRGrid = getRGrid(nX, nY, nZ + 1);
				if (!pNextRGrid->isRouted)
				{
					//pNextRGrid->isRouted = true;
					layer_C *pLayer = vLayer[nZ + 1 - m_nOffsetZ];
					pNextRGrid->m_nG = nNewDistance;
					int nSumH = 0;
					//for( int t=0; t<vTerminal.size(); t++ )
					//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
					nSumH = nEularDistance(vTerminal, pNextGrid, pLayer);
					pNextRGrid->m_nH = nSumH;
					pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
					pNextRGrid->m_pFrom = pCurRGrid;
					bool bInsert = false;
					//for( int j=0; j<vHistory.size(); j++ )
					for (int j = nHistoryIndex; j < vHistory.size(); j++)
					{
						if (pNextRGrid->m_nCost <= vHistory[j]->m_nCost)
						{
							vHistory.insert(vHistory.begin() + j, pNextRGrid);
							//cout << "insert at " << j << endl;
							bInsert = true;
							break;
						}
					}
					if (!bInsert)
					{
						vHistory.push_back(pNextRGrid);
							//cout << "insert at last"<<endl;
					}
					vAddGrid.push_back(pNextRGrid);
				}
			}

			if (nZ - 1 >= nLConstraint)
			{
				pNextGrid = graphTravel(m_pDesign, pGGrid, 0, 0, -1);
				if (pNextGrid != NULL )
				{
					rGrid_C *pNextRGrid = getRGrid(nX, nY, nZ - 1);
					if (!pNextRGrid->isRouted)
					{
						//pNextRGrid->isRouted = true;
						layer_C *pLayer = vLayer[nZ - 1 - m_nOffsetZ];
						pNextRGrid->m_nG = nNewDistance;
						int nSumH = 0;
						//for( int t=0; t<vTerminal.size(); t++ )
						//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
						nSumH = nEularDistance(vTerminal, pNextGrid, pLayer);
						pNextRGrid->m_nH = nSumH;
						pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
						pNextRGrid->m_pFrom = pCurRGrid;
						bool bInsert = false;
						//for( int j=0; j<vHistory.size(); j++ )
						for (int j = nHistoryIndex; j < vHistory.size(); j++)
						{
							if (pNextRGrid->m_nCost <= vHistory[j]->m_nCost)
							{
								vHistory.insert(vHistory.begin() + j, pNextRGrid);
								//cout << "insert at " << j << endl;
								bInsert = true;
								break;
							}
						}
						if (!bInsert)
						{
							vHistory.push_back(pNextRGrid);
							//cout << "insert at last"<<endl;
						}
						vAddGrid.push_back(pNextRGrid);
					}
				}
			}
		}
		//cout<<"Check target"<<endl;
		// if there is a target in addgrid;
		//vector< rGrid_C* > vHaveAddTarget;
		bool bFindTarget = false;
		for (int i = 0; i < vAddGrid.size(); i++)
		{
			if (vAddGrid[i]->isTarget)
			{
				//if( pNet->getName() == "N1482")
				//	cout<<"Target is: "<<vAddGrid[i]->m_nX<<" "<<vAddGrid[i]->m_nY<<" "<<vAddGrid[i]->m_nZ<<" "<<endl;
				bFindTarget = true;
				bool bHaveSearch = false;
				for (int j = 0; j < vHaveAddTarget.size(); j++)
				{
					if (vHaveAddTarget[j] == vAddGrid[i])
					{
						bHaveSearch = true;
						break;
					}
				}
				if (bHaveSearch)
					continue;

				vHaveAddTarget.push_back(vAddGrid[i]);
				//cout<<"Back trace"<<endl;
				// find the path
				vector<gGrid_C *> vReversePath;

				int nPath = vAddGrid[i]->m_nG;
				//if( pNet->getName() == "N1482")
				//	cout<<"Path cost: "<<nPath<<endl;
				rGrid_C *pRPath = vAddGrid[i];
				pRPath->isPath = true;
				gGrid_C *pPathGrid = getGrid(m_pDesign, pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ);
				rGrid_C *pPreGrid = pRPath->m_pFrom;
				//gGrid_C* pPathGrid = getGrid( m_pDesign, pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ );
				//vResult.push_back( pPathGrid );
				vReversePath.push_back(pPathGrid);
				while (!pPreGrid->isPath)
				{
					pRPath = pPreGrid;
					pRPath->isPath = true;
					pPathGrid = getGrid(m_pDesign, pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ);
					//vResult.push_back( pPathGrid );
					vReversePath.push_back(pPathGrid);
					pPreGrid = pRPath->m_pFrom;
				}

				pPathGrid = getGrid(m_pDesign, pPreGrid->m_nX, pPreGrid->m_nY, pPreGrid->m_nZ);
				//vResult.push_back( pPathGrid );
				vReversePath.push_back(pPathGrid);
				vector<gGrid_C *> vTmpResult;
				for (int g = vReversePath.size() - 1; g >= 0; g--)
				{
					vTmpResult.push_back(vReversePath[g]);
				}

				vResult.push_back(vTmpResult);
				// find the terminal and remove
				for (int t = 0; t < vTerminal.size(); t++)
				{
					int nTX, nTY, nTZ;
					vTerminal[t]->getPosition(nTX, nTY, nTZ);
					if (vAddGrid[i]->m_nX == nTX && vAddGrid[i]->m_nY == nTY && vAddGrid[i]->m_nZ == nTZ)
					{
						vTerminal.erase(vTerminal.begin() + t);
						nNumTermanals--;
						break;
					}
				}
				// update the nG in all the searched node ( without path )
				for (int h = 0; h < vHistory.size(); h++)
				{
					rGrid_C *pTmpGrid = vHistory[h];
					if (!pTmpGrid->isPath)
						calDistanceFromPath(pTmpGrid, nPath);
					else
						pTmpGrid->m_nG = nPath;
				}

				// update the nH in all the node in queue
				for (int h = nHistoryIndex; h < vHistory.size(); h++)
				{
					rGrid_C *pTmpGrid = vHistory[h];
					if (!pTmpGrid->isRouted)
					{
						int nH = 0;
						gGrid_C *pTmpGGrid = getGrid(m_pDesign, pTmpGrid->m_nX, pTmpGrid->m_nY, pTmpGrid->m_nZ);
						int nX, nY, nZ;
						pTmpGGrid->getPosition(nX, nY, nZ);
						layer_C *pLayer = vLayer[nZ - m_nOffsetZ];
						nH = nEularDistance(vTerminal, pTmpGGrid, pLayer);
						pTmpGrid->m_nH = nH;
						pTmpGrid->m_nCost = nH + pTmpGrid->m_nG;
					}
				}
				//cout<<endl;
			}
		}

		if (bFindTarget)
		{
			for (int i = nHistoryIndex; i < vHistory.size(); i++)
			{
				for (int j = i - 1; j >= 0; j--)
				{
					if (vHistory[j]->m_nCost > vHistory[j + 1]->m_nCost)
					{
						rGrid_C *pFGrid = vHistory[j];
						rGrid_C *pBGrid = vHistory[j + 1];
						vHistory[j] = pBGrid;
						vHistory[j + 1] = pFGrid;
					}
					else
						break;
				}
			}
		}
	}

	for (int i = 0; i < vHistory.size(); i++)
	{
		rGrid_C *pRGrid = vHistory[i];
		pRGrid->m_nH = 0;
		pRGrid->m_nG = 0;
		pRGrid->m_nCost = 0;
		pRGrid->m_nBend = 0;
		pRGrid->m_pFrom = NULL;
		pRGrid->isRouted = false;
		pRGrid->isPath = false;
		pRGrid->isTarget = false;
	}

	for (int i = 0; i < vTerminal.size(); i++)
	{
		int nX, nY, nZ;
		gGrid_C *pTmpGrid = vTerminal[i];
		pTmpGrid->getPosition(nX, nY, nZ);
		rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
		pRGrid->m_nH = 0;
		pRGrid->m_nG = 0;
		pRGrid->m_nCost = 0;
		pRGrid->m_nBend = 0;
		pRGrid->m_pFrom = NULL;
		pRGrid->isRouted = false;
		pRGrid->isPath = false;
		pRGrid->isTarget = false;
	}

	for( int i=0; i<vResult.size(); i++ )
	{
		for( int j=0; j<vResult[i].size(); j++ )
		{
			sLength.insert( vResult[i][j] );
		}
	}
	
	return sLength.size();
}


vector< vector< gGrid_C* > > router_C::routeNet_length_constraint_ver3(net_C *pNet, int &nConstraint, int nDetour )
{
	//cout<<pNet->getName()<<endl;
	setRoutingConstraint(pNet);
	int &nLConstraint = m_nConstraintLayerId;
	//cout<<nLConstraint<<endl;

	vector<gGrid_C *> vTerminal;
	vector<pin_C *> vPin = pNet->getPin();
	//vector< wire_C* > vWire;
	vector<vector<gGrid_C *>> vResult;
	for (int i = 0; i < vPin.size(); i++)
	{
		pin_C *pPin = vPin[i];
		int nX, nY, nZ;
		instance_C *pInst = (instance_C *)pPin->getCell();
		nX = pInst->getPlacedX();
		nY = pInst->getPlacedY();
		// project the z to the constraint layer
		nZ = pPin->getLayerId();
		if (nZ < nLConstraint)
		{
			vector<gGrid_C *> vTmpResult;
			for (int l = nZ; l <= nLConstraint; l++)
			{
				gGrid_C *pGrid = getGrid(m_pDesign, nX, nY, l);
				vTmpResult.push_back(pGrid);
			}
			nZ = nLConstraint;
			vResult.push_back(vTmpResult);
		}

		gGrid_C *pTerminal = getGrid(m_pDesign, nX, nY, nZ);
		rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
		if (!pRGrid->isTarget) // if pRGrid is already put in vTerminal
		{
			pRGrid->isTarget = true;
			vTerminal.push_back(pTerminal);
		}
	}

	if (vTerminal.size() == 1)
	{
		int nX, nY, nZ;
		vTerminal[0]->getPosition(nX, nY, nZ);
		rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
		pRGrid->isTarget = false;
		vector<gGrid_C *> vTmpResult;
		vTmpResult.push_back(vTerminal[0]);
		vResult.push_back(vTmpResult);
		return vResult;
	}

	vector<rGrid_C *> vRoutedGrid;
	//vector< rGrid_C* > vPriorityQueue;
	vector<rGrid_C *> vHistory;
	vector<rGrid_C *> vNotInPath;
	// find first terminal
	gGrid_C *pFirst = NULL;

	int nMinX = m_nTX;
	for (int i = 0; i < vTerminal.size(); i++)
	{
		gGrid_C *pGrid = vTerminal[i];
		int nX, nY, nZ;
		pGrid->getPosition(nX, nY, nZ);
		if (nX <= nMinX)
			nMinX = nX;
	}

	int nMinY = m_nTY;
	for (int i = 0; i < vTerminal.size(); i++)
	{
		gGrid_C *pGrid = vTerminal[i];
		int nX, nY, nZ;
		pGrid->getPosition(nX, nY, nZ);
		if (nX == nMinX && nY <= nMinY)
		{
			nMinY = nY;
			pFirst = vTerminal[i];
		}
	}

	//
	//pFirst = vTerminal[0];

	//chech the terminal is available to route the net
	int nX, nY, nZ;
	pFirst->getPosition(nX, nY, nZ);
	rGrid_C *pCurRGrid = getRGrid(nX, nY, nZ);
	gGrid_C *pGGrid = pFirst;

	vHistory.push_back(pCurRGrid);
	pCurRGrid->isPath = true;
	for (int i = 0; i < vTerminal.size(); i++)
	{
		if (vTerminal[i] == pGGrid)
		{
			vTerminal.erase(vTerminal.begin() + i);
			break;
		}
	}
	int nNumTermanals = vTerminal.size();
	vector<layer_C *> vLayer = m_pDesign->getLayer();
	//cout<<"Start routing"<<endl;
	vector<rGrid_C *> vHaveAddTarget;
	int nHistoryIndex = 0;
	while (nNumTermanals > 0)
	{
		//cout<<nNumTermanals<<endl;
		//cout<<pCurRGrid->m_nX<<" "<<pCurRGrid->m_nY<<" "<<pCurRGrid->m_nZ<<endl;
		// A* propogation
		if (vHistory.size() == 0)
		{
			//cout << "break at size"<<endl;
			vResult.clear();
			break;
		}
		//cout<<vHistory.size()<<endl;

		vector<rGrid_C *> vToPropogate;
		int nMinCost = 0;
		
		if (nHistoryIndex == vHistory.size())
		{
			//cout << "break at index" << endl;
			break;
		}
		else
		{
			nMinCost = vHistory[nHistoryIndex]->m_nCost;
			vToPropogate.push_back(vHistory[nHistoryIndex]);
			//nHistoryIndex++;
		}
		nHistoryIndex++;
		//cout << "now: " << nHistoryIndex << endl;	
		vector<rGrid_C *> vAddGrid;

		for (int i = 0; i < vToPropogate.size(); i++)
		{
			pCurRGrid = vToPropogate[i];
			pCurRGrid->isRouted = true;
			int nCurDistance = pCurRGrid->m_nG;
			int nNewDistance = nCurDistance + 1;
			if (nNewDistance > nConstraint)
				continue;

			int nX, nY, nZ;
			nX = pCurRGrid->m_nX;
			nY = pCurRGrid->m_nY;
			nZ = pCurRGrid->m_nZ;
			//cout<<"P: "<<nX<<" "<<nY<<" "<<nZ<<endl;
			pGGrid = getGrid(m_pDesign, nX, nY, nZ);
			char cDir = vLayer[nZ - m_nOffsetZ]->getDir();
			//cout<<cDir<<endl;
			int nDX, nDY, nDZ;
			if (cDir == 'H')
			{
				nDX = 0;
				nDY = 1;
				nDZ = 0;
			}
			else if (cDir == 'V')
			{
				nDX = 1;
				nDY = 0;
				nDZ = 0;
			}
			else
				cout << "Unknown routing direction" << endl;
			// go in the same layer;
			gGrid_C *pNextGrid = graphTravel(m_pDesign, pGGrid, nDX, nDY, nDZ);
			if( pNextGrid != NULL )
			{
				rGrid_C *pNextRGrid = getRGrid(nX + nDX, nY + nDY, nZ + nDZ);
				if (!pNextRGrid->isRouted)
				{
					//pNextRGrid->isRouted = true;
					layer_C *pLayer = vLayer[nZ + nDZ - m_nOffsetZ];
					pNextRGrid->m_nG = nNewDistance;
					int nSumH = 0;
					//for( int t=0; t<vTerminal.size(); t++ )
					//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
					nSumH = nEularDistance(vTerminal, pNextGrid, pLayer);
					pNextRGrid->m_nH = nSumH;
					if( pNextGrid->getRemand() <= 0 )
					{
						if( nDetour == 0 )
							pNextRGrid->m_nH = pNextRGrid->m_nH + 1;
						else
						{
							int nPenalty = nDetour;
							if( nPenalty <= 0 )
								nPenalty = 1;
							
							pNextRGrid->m_nH = pNextRGrid->m_nH + nPenalty;
						}
					}
					pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;

					pNextRGrid->m_pFrom = pCurRGrid;
					bool bInsert = false;
					//for( int j=0; j<vHistory.size(); j++ )
					for (int j = nHistoryIndex; j < vHistory.size(); j++)
					{
						if (pNextRGrid->m_nCost <= vHistory[j]->m_nCost)
						{
							//cout << "insert at " << j << endl;
							vHistory.insert(vHistory.begin() + j, pNextRGrid);
							bInsert = true;
							break;
						}
					}
					if (!bInsert)
					{
						vHistory.push_back(pNextRGrid);
							//cout << "insert at last"<<endl;
					}
					vAddGrid.push_back(pNextRGrid);
				}
			}
			//cout<<"test"<<endl;
			pNextGrid = graphTravel(m_pDesign, pGGrid, -nDX, -nDY, -nDZ);
			if( pNextGrid != NULL )
			{
				rGrid_C *pNextRGrid = getRGrid(nX - nDX, nY - nDY, nZ - nDZ);
				if (!pNextRGrid->isRouted)
				{
					//pNextRGrid->isRouted = true;
					layer_C *pLayer = vLayer[nZ - nDZ - m_nOffsetZ];
					pNextRGrid->m_nG = nNewDistance;
					int nSumH = 0;
					//for( int t=0; t<vTerminal.size(); t++ )
					//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
					nSumH = nEularDistance(vTerminal, pNextGrid, pLayer);
					pNextRGrid->m_nH = nSumH;
					if( pNextGrid->getRemand() <= 0 )
					{
						if( nDetour == 0 )
							pNextRGrid->m_nH = pNextRGrid->m_nH + 1;
						else
						{
							int nPenalty = nDetour;
							if( nPenalty <= 0 )
								nPenalty = 1;
							
							pNextRGrid->m_nH = pNextRGrid->m_nH + nPenalty;
						}
					}
					pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
					pNextRGrid->m_pFrom = pCurRGrid;
					bool bInsert = false;
					//for( int j=0; j<vHistory.size(); j++ )
					for (int j = nHistoryIndex; j < vHistory.size(); j++)
					{
						if (pNextRGrid->m_nCost <= vHistory[j]->m_nCost)
						{
							vHistory.insert(vHistory.begin() + j, pNextRGrid);
							//cout << "insert at " << j << endl;
							bInsert = true;
							break;
						}
					}
					if (!bInsert)
					{
						vHistory.push_back(pNextRGrid);
							//cout << "insert at last"<<endl;
					}
					vAddGrid.push_back(pNextRGrid);
				}
			}

			// go to different layer
			//cout<<"different layer"<<endl;
			pNextGrid = graphTravel(m_pDesign, pGGrid, 0, 0, 1);
			if (pNextGrid != NULL )
			{
				rGrid_C *pNextRGrid = getRGrid(nX, nY, nZ + 1);
				if (!pNextRGrid->isRouted)
				{
					//pNextRGrid->isRouted = true;
					layer_C *pLayer = vLayer[nZ + 1 - m_nOffsetZ];
					pNextRGrid->m_nG = nNewDistance;
					int nSumH = 0;
					//for( int t=0; t<vTerminal.size(); t++ )
					//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
					nSumH = nEularDistance(vTerminal, pNextGrid, pLayer);
					pNextRGrid->m_nH = nSumH;
					if( pNextGrid->getRemand() <= 0 )
					{
						if( nDetour == 0 )
							pNextRGrid->m_nH = pNextRGrid->m_nH + 1;
						else
						{
							int nPenalty = nDetour;
							if( nPenalty <= 0 )
								nPenalty = 1;
							
							pNextRGrid->m_nH = pNextRGrid->m_nH + nPenalty;
						}
					}
					pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
					pNextRGrid->m_pFrom = pCurRGrid;
					bool bInsert = false;
					//for( int j=0; j<vHistory.size(); j++ )
					for (int j = nHistoryIndex; j < vHistory.size(); j++)
					{
						if (pNextRGrid->m_nCost <= vHistory[j]->m_nCost)
						{
							vHistory.insert(vHistory.begin() + j, pNextRGrid);
							//cout << "insert at " << j << endl;
							bInsert = true;
							break;
						}
					}
					if (!bInsert)
					{
						vHistory.push_back(pNextRGrid);
							//cout << "insert at last"<<endl;
					}
					vAddGrid.push_back(pNextRGrid);
				}
			}

			if (nZ - 1 >= nLConstraint)
			{
				pNextGrid = graphTravel(m_pDesign, pGGrid, 0, 0, -1);
				if (pNextGrid != NULL )
				{
					rGrid_C *pNextRGrid = getRGrid(nX, nY, nZ - 1);
					if (!pNextRGrid->isRouted)
					{
						//pNextRGrid->isRouted = true;
						layer_C *pLayer = vLayer[nZ - 1 - m_nOffsetZ];
						pNextRGrid->m_nG = nNewDistance;
						int nSumH = 0;
						//for( int t=0; t<vTerminal.size(); t++ )
						//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
						nSumH = nEularDistance(vTerminal, pNextGrid, pLayer);
						pNextRGrid->m_nH = nSumH;
						if( pNextGrid->getRemand() <= 0 )
						{
							if( nDetour == 0 )
								pNextRGrid->m_nH = pNextRGrid->m_nH + 1;
							else
							{
								int nPenalty = nDetour;
								if( nPenalty <= 0 )
									nPenalty = 1;
								
								pNextRGrid->m_nH = pNextRGrid->m_nH + nPenalty;
							}
						}
						pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
						pNextRGrid->m_pFrom = pCurRGrid;
						bool bInsert = false;
						//for( int j=0; j<vHistory.size(); j++ )
						for (int j = nHistoryIndex; j < vHistory.size(); j++)
						{
							if (pNextRGrid->m_nCost <= vHistory[j]->m_nCost)
							{
								vHistory.insert(vHistory.begin() + j, pNextRGrid);
								//cout << "insert at " << j << endl;
								bInsert = true;
								break;
							}
						}
						if (!bInsert)
						{
							vHistory.push_back(pNextRGrid);
							//cout << "insert at last"<<endl;
						}
						vAddGrid.push_back(pNextRGrid);
					}
				}
			}
		}
		//cout<<"Check target"<<endl;
		// if there is a target in addgrid;
		//vector< rGrid_C* > vHaveAddTarget;
		bool bFindTarget = false;
		for (int i = 0; i < vAddGrid.size(); i++)
		{
			if (vAddGrid[i]->isTarget)
			{
				//if( pNet->getName() == "N1482")
				//	cout<<"Target is: "<<vAddGrid[i]->m_nX<<" "<<vAddGrid[i]->m_nY<<" "<<vAddGrid[i]->m_nZ<<" "<<endl;
				bFindTarget = true;
				bool bHaveSearch = false;
				for (int j = 0; j < vHaveAddTarget.size(); j++)
				{
					if (vHaveAddTarget[j] == vAddGrid[i])
					{
						bHaveSearch = true;
						break;
					}
				}
				if (bHaveSearch)
					continue;

				vHaveAddTarget.push_back(vAddGrid[i]);
				//cout<<"Back trace"<<endl;
				// find the path
				vector<gGrid_C *> vReversePath;

				int nPath = vAddGrid[i]->m_nG;
				//if( pNet->getName() == "N1482")
				//	cout<<"Path cost: "<<nPath<<endl;
				rGrid_C *pRPath = vAddGrid[i];
				pRPath->isPath = true;
				gGrid_C *pPathGrid = getGrid(m_pDesign, pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ);
				rGrid_C *pPreGrid = pRPath->m_pFrom;
				//gGrid_C* pPathGrid = getGrid( m_pDesign, pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ );
				//vResult.push_back( pPathGrid );
				vReversePath.push_back(pPathGrid);
				while (!pPreGrid->isPath)
				{
					pRPath = pPreGrid;
					pRPath->isPath = true;
					pPathGrid = getGrid(m_pDesign, pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ);
					//vResult.push_back( pPathGrid );
					vReversePath.push_back(pPathGrid);
					pPreGrid = pRPath->m_pFrom;
				}

				pPathGrid = getGrid(m_pDesign, pPreGrid->m_nX, pPreGrid->m_nY, pPreGrid->m_nZ);
				//vResult.push_back( pPathGrid );
				vReversePath.push_back(pPathGrid);
				vector<gGrid_C *> vTmpResult;
				for (int g = vReversePath.size() - 1; g >= 0; g--)
				{
					vTmpResult.push_back(vReversePath[g]);
				}

				vResult.push_back(vTmpResult);
				// find the terminal and remove
				for (int t = 0; t < vTerminal.size(); t++)
				{
					int nTX, nTY, nTZ;
					vTerminal[t]->getPosition(nTX, nTY, nTZ);
					if (vAddGrid[i]->m_nX == nTX && vAddGrid[i]->m_nY == nTY && vAddGrid[i]->m_nZ == nTZ)
					{
						vTerminal.erase(vTerminal.begin() + t);
						nNumTermanals--;
						break;
					}
				}
				// update the nG in all the searched node ( without path )
				for (int h = 0; h < vHistory.size(); h++)
				{
					rGrid_C *pTmpGrid = vHistory[h];
					if (!pTmpGrid->isPath)
						calDistanceFromPath(pTmpGrid, nPath);
					else
						pTmpGrid->m_nG = nPath;
				}

				// update the nH in all the node in queue
				for (int h = nHistoryIndex; h < vHistory.size(); h++)
				{
					rGrid_C *pTmpGrid = vHistory[h];
					if (!pTmpGrid->isRouted)
					{
						int nH = 0;
						gGrid_C *pTmpGGrid = getGrid(m_pDesign, pTmpGrid->m_nX, pTmpGrid->m_nY, pTmpGrid->m_nZ);
						int nX, nY, nZ;
						pTmpGGrid->getPosition(nX, nY, nZ);
						layer_C *pLayer = vLayer[nZ - m_nOffsetZ];
						nH = nEularDistance(vTerminal, pTmpGGrid, pLayer);
						pTmpGrid->m_nH = nH;
						pTmpGrid->m_nCost = nH + pTmpGrid->m_nG;
					}
				}
				//cout<<endl;
			}
		}

		if (bFindTarget)
		{
			for (int i = nHistoryIndex; i < vHistory.size(); i++)
			{
				for (int j = i - 1; j >= nHistoryIndex; j--)
				{
					if (vHistory[j]->m_nCost > vHistory[j + 1]->m_nCost)
					{
						rGrid_C *pFGrid = vHistory[j];
						rGrid_C *pBGrid = vHistory[j + 1];
						vHistory[j] = pBGrid;
						vHistory[j + 1] = pFGrid;
					}
					else if (vHistory[j]->m_nCost ==  vHistory[j + 1]->m_nCost)
					{
						rGrid_C *pFGrid = vHistory[j];
						rGrid_C *pBGrid = vHistory[j + 1];
						if( pFGrid->m_nZ == pCurRGrid->m_nZ )
							break;
						else if( pBGrid->m_nZ == pCurRGrid->m_nZ )
						{	
							vHistory[j] = pBGrid;
							vHistory[j + 1] = pFGrid;	
						}
						else
							break;
					}
					else
						break;
				}
			}
		}
	}

	for (int i = 0; i < vHistory.size(); i++)
	{
		rGrid_C *pRGrid = vHistory[i];
		pRGrid->m_nH = 0;
		pRGrid->m_nG = 0;
		pRGrid->m_nCost = 0;
		pRGrid->m_nBend = 0;
		pRGrid->m_pFrom = NULL;
		pRGrid->isRouted = false;
		pRGrid->isPath = false;
		pRGrid->isTarget = false;
	}

	/*	
	for( int i=0; i<vResult.size(); i++ )
	{
		int nX, nY, nZ;
		gGrid_C* pTmpGrid = vResult[i];
		pTmpGrid->getPosition( nX, nY, nZ );
		rGrid_C* pRGrid = getRGrid( nX, nY, nZ );
		pRGrid->m_nH = 0;
		pRGrid->m_nG = 0;
		pRGrid->m_nCost = 0;
		pRGrid->m_nBend = 0;
		pRGrid->m_pFrom = NULL;
		pRGrid->isRouted = false;
		pRGrid->isPath = false;
		pRGrid->isTarget = false;
	}
	*/
	for (int i = 0; i < vTerminal.size(); i++)
	{
		int nX, nY, nZ;
		gGrid_C *pTmpGrid = vTerminal[i];
		pTmpGrid->getPosition(nX, nY, nZ);
		rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
		pRGrid->m_nH = 0;
		pRGrid->m_nG = 0;
		pRGrid->m_nCost = 0;
		pRGrid->m_nBend = 0;
		pRGrid->m_pFrom = NULL;
		pRGrid->isRouted = false;
		pRGrid->isPath = false;
		pRGrid->isTarget = false;
	}
	if (nNumTermanals != 0)
	{
		//cout << "Some net out of constraint" << endl;
		vResult.clear();
	}
	return vResult;
}


vector<vector<gGrid_C *>> router_C::routeNet_length_constraint_ver4(net_C *pNet, const int nConstraint)
{
	//cout<<pNet->getName()<<endl;
	setRoutingConstraint(pNet);
	int &nLConstraint = m_nConstraintLayerId;
	//cout<<nLConstraint<<endl;

	vector<gGrid_C *> vTerminal;
	vector<pin_C *> vPin = pNet->getPin();
	//vector< wire_C* > vWire;
	vector<vector<gGrid_C *>> vResult;
	for (int i = 0; i < vPin.size(); i++)
	{
		pin_C *pPin = vPin[i];
		int nX, nY, nZ;
		instance_C *pInst = (instance_C *)pPin->getCell();
		nX = pInst->getPlacedX();
		nY = pInst->getPlacedY();
		// project the z to the constraint layer
		nZ = pPin->getLayerId();
		if (nZ < nLConstraint)
		{
			vector<gGrid_C *> vTmpResult;
			for (int l = nZ; l <= nLConstraint; l++)
			{
				gGrid_C *pGrid = getGrid(m_pDesign, nX, nY, l);
				vTmpResult.push_back(pGrid);
			}
			nZ = nLConstraint;
			vResult.push_back(vTmpResult);
		}

		gGrid_C *pTerminal = getGrid(m_pDesign, nX, nY, nZ);
		rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
		if (!pRGrid->isTarget) // if pRGrid is already put in vTerminal
		{
			pRGrid->isTarget = true;
			vTerminal.push_back(pTerminal);
		}
	}

	// check same position of terminal
	//cout<<"Check terminal"<<endl;
	/*
	for( int i=0; i<vTerminal.size(); i++ )
	{
		
		for( int j=i+1; j<vTerminal.size(); j++ )
		{
			if( vTerminal[i] == vTerminal[j] )
			{
				vTerminal.erase( vTerminal.begin() + j );
				j--;
			}
		}
		

	}
	*/
	//cout<<"Num of terminals: "<<vTerminal.size()<<endl;
	bool bFindOverflow = false;
	for( int i=0; i<vResult.size(); i++ )
	{
		for( int j=0; j<vResult[i].size(); j++ )
		{
			if( vResult[i][j]->getRemand() <= 0 )
			{
				bFindOverflow = true;
				break;
			}
		}
		if( bFindOverflow )
			break;
	}
	if( bFindOverflow )
	{
		//cout << "Failed at target"<<endl;
		for (int i = 0; i < vTerminal.size(); i++)
		{
			int nX, nY, nZ;
			vTerminal[i]->getPosition(nX, nY, nZ);
			rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
			pRGrid->isTarget = false;
		}
		vResult.clear();
		return vResult;
	}

	if (vTerminal.size() == 1)
	{
		if (vTerminal[0]->getRemand() <= 0)
		{

			int nX, nY, nZ;
			vTerminal[0]->getPosition(nX, nY, nZ);
			rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
			pRGrid->isTarget = false;
			vResult.clear();
			return vResult;
		}
		int nX, nY, nZ;
		vTerminal[0]->getPosition(nX, nY, nZ);
		rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
		pRGrid->isTarget = false;
		vector<gGrid_C *> vTmpResult;
		vTmpResult.push_back(vTerminal[0]);
		vResult.push_back(vTmpResult);
		return vResult;
	}
	else
	{
		bool bFindOverflow = false;
		for (int i = 0; i < vTerminal.size(); i++)
		{
			if (vTerminal[i]->getRemand() <= 0)
			{
				bFindOverflow = true;
				break;
			}
		}

		if (bFindOverflow)
		{
			//cout << "Failed at target"<<endl;
			for (int i = 0; i < vTerminal.size(); i++)
			{
				int nX, nY, nZ;
				vTerminal[i]->getPosition(nX, nY, nZ);
				rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
				pRGrid->isTarget = false;
			}
			if (vResult.size() != 0)
				vResult.clear();

			return vResult;
		}
	}

	vector<rGrid_C *> vRoutedGrid;
	//vector< rGrid_C* > vPriorityQueue;
	vector<rGrid_C *> vHistory;
	vector<rGrid_C *> vNotInPath;
	// find first terminal
	gGrid_C *pFirst = NULL;

	int nMinX = m_nTX;
	for (int i = 0; i < vTerminal.size(); i++)
	{
		gGrid_C *pGrid = vTerminal[i];
		int nX, nY, nZ;
		pGrid->getPosition(nX, nY, nZ);
		if (nX <= nMinX)
			nMinX = nX;
	}

	int nMinY = m_nTY;
	for (int i = 0; i < vTerminal.size(); i++)
	{
		gGrid_C *pGrid = vTerminal[i];
		int nX, nY, nZ;
		pGrid->getPosition(nX, nY, nZ);
		if (nX == nMinX && nY <= nMinY)
		{
			nMinY = nY;
			pFirst = vTerminal[i];
		}
	}

	//
	//pFirst = vTerminal[0];

	//chech the terminal is available to route the net
	int nX, nY, nZ;
	pFirst->getPosition(nX, nY, nZ);
	rGrid_C *pCurRGrid = getRGrid(nX, nY, nZ);
	gGrid_C *pGGrid = pFirst;
	if (pGGrid->getRemand() <= 0)
	{
		pCurRGrid->isTarget = false;
		// added at 0705 22:00
		for (int i = 0; i < vTerminal.size(); i++)
		{
			vTerminal[i]->getPosition(nX, nY, nZ);
			pCurRGrid = getRGrid(nX, nY, nZ);
			pCurRGrid->isTarget = false;
		}
		// end added at 0705 22:00
		vResult.clear();
		return vResult;
	}

	vHistory.push_back(pCurRGrid);
	pCurRGrid->isPath = true;
	for (int i = 0; i < vTerminal.size(); i++)
	{
		if (vTerminal[i] == pGGrid)
		{
			vTerminal.erase(vTerminal.begin() + i);
			break;
		}
	}
	int nNumTermanals = vTerminal.size();
	//nNumTermanals--; // because the first grid is a termianl
	vector<layer_C *> vLayer = m_pDesign->getLayer();
	//cout<<"Start routing"<<endl;
	vector<rGrid_C *> vHaveAddTarget;
	int nHistoryIndex = 0;
	while (nNumTermanals > 0)
	{
		//cout<<nNumTermanals<<endl;
		//cout<<pCurRGrid->m_nX<<" "<<pCurRGrid->m_nY<<" "<<pCurRGrid->m_nZ<<endl;
		// A* propogation
		if (vHistory.size() == 0)
		{
			vResult.clear();
			break;
		}
		//cout<<vHistory.size()<<endl;

		vector<rGrid_C *> vToPropogate;
		int nMinCost = 0;
		/*
		bool bPick = false;
		
		for( int i=0; i<vHistory.size(); i++ )
		{
			if( !vHistory[i]->isRouted )	
			{
				nMinCost = vHistory[i]->m_nCost;
				bPick = true;
				break;
			}
		}
		if( !bPick )
		{
			vResult.clear();
			//cout<<"Can't find result"<<endl;
			break;
		}
		*/
		if (nHistoryIndex == vHistory.size())
			break;
		else
		{
			nMinCost = vHistory[nHistoryIndex]->m_nCost;
			vToPropogate.push_back(vHistory[nHistoryIndex]);
			//nHistoryIndex++;
		}
		/*
		cout<<nMinCost<<" "<<vHistory.size()<<endl;
		for( int i=0; i<vHistory.size(); i++ )
		{
			if( !vHistory[i]->isRouted )	
			{
				cout << vHistory[i]->m_nCost <<" ";
			}
		}
		//cout<<endl;
		*/
		/*
		for( int i=0; i<vHistory.size(); i++ )
		{
			if( !vHistory[i]->isRouted )	
			{
				if( vHistory[i]->m_nCost == nMinCost )
				{
					vToPropogate.push_back( vHistory[i] );
					break;
				}
				//else
				//	break;
			}
		}
		*/
		/*	
		//----------------------------------------------------------------------------choubegin TODO:
		vector<rGrid_C *> samecost;
		int vsum = INT_MAX;
		rGrid_C *temprG;
		gGrid_C *tempgG;
		for (int i = nHistoryIndex; i < vHistory.size(); i++)
		{
			if (!vHistory[i]->isRouted)
			{
				if (vHistory[i]->m_nCost == nMinCost)
				{
					samecost.push_back(vHistory[i]);
				}
				else
					break;
			}
		}
		//cout << "iter--------" << endl;
		for (int x = 0; x < samecost.size(); x++)
		{
			int az = samecost[x]->m_nZ;
			layer_C *pLayer = vLayer[az - m_nOffsetZ];
			//if (pNet->getName() == "N2")
			//	cout << "same : " << samecost[x]->m_nX << " " << samecost[x]->m_nY << " " << samecost[x]->m_nZ << endl;
			tempgG = m_pDesign->getGrid(samecost[x]->m_nX, samecost[x]->m_nY, az);
			if (vsum > nEularDistance_sum(vTerminal, tempgG, pLayer))
			{
				vsum = nEularDistance_sum(vTerminal, tempgG, pLayer);
				temprG = samecost[x];
			}
		}
		for( int i=nHistoryIndex; i<vHistory.size(); i++ )
		{
			if( temprG == vHistory[i] )
			{
				vHistory[i] = vHistory[ nHistoryIndex ];
				vHistory[ nHistoryIndex ] = temprG;
				break;
			}
		}
		nHistoryIndex++;
		vToPropogate.clear();
		vToPropogate.push_back(temprG);
		samecost.clear();
		//------------------------------------------------------------------------------chouend 
 		*/
		nHistoryIndex++;
		/*	
		if (pNet->getName() == "N1482")
		{
			cout<<endl;
			cout << "Routed: ";
			for (int i = 0; i < vHistory.size(); i++)
			{
				if( vHistory[i]->isRouted )
				cout << "(" << vHistory[i]->m_nX << " " << vHistory[i]->m_nY << " " << vHistory[i]->m_nZ << " " << vHistory[i]->m_nCost << " )";
			}
			cout << endl;
			cout<< "His: ";
			for (int i = 0; i < vHistory.size(); i++)
			{
				if( !vHistory[i]->isRouted )
				cout << "(" << vHistory[i]->m_nX << " " << vHistory[i]->m_nY << " " << vHistory[i]->m_nZ << " " << vHistory[i]->m_nG <<" " << vHistory[i]->m_nH <<" " << vHistory[i]->m_nCost << " )";
			}
			cout<<endl;
		}
		*/
		vector<rGrid_C *> vAddGrid;

		for (int i = 0; i < vToPropogate.size(); i++)
		{
			pCurRGrid = vToPropogate[i];
			pCurRGrid->isRouted = true;
			int nCurDistance = pCurRGrid->m_nG;
			int nNewDistance = nCurDistance + 1;
			if (nNewDistance > nConstraint)
				continue;

			int nX, nY, nZ;
			nX = pCurRGrid->m_nX;
			nY = pCurRGrid->m_nY;
			nZ = pCurRGrid->m_nZ;
			//cout<<"P: "<<nX<<" "<<nY<<" "<<nZ<<endl;
			pGGrid = getGrid(m_pDesign, nX, nY, nZ);
			char cDir = vLayer[nZ - m_nOffsetZ]->getDir();
			//cout<<cDir<<endl;
			int nDX, nDY, nDZ;
			if (cDir == 'H')
			{
				nDX = 0;
				nDY = 1;
				nDZ = 0;
			}
			else if (cDir == 'V')
			{
				nDX = 1;
				nDY = 0;
				nDZ = 0;
			}
			else
				cout << "Unknown routing direction" << endl;
			// go in the same layer;
			gGrid_C *pNextGrid = graphTravel(m_pDesign, pGGrid, nDX, nDY, nDZ);
			if (pNextGrid != NULL && pNextGrid->getRemand() > 0)
			{
				rGrid_C *pNextRGrid = getRGrid(nX + nDX, nY + nDY, nZ + nDZ);
				if (!pNextRGrid->isRouted)
				{
					//pNextRGrid->isRouted = true;
					layer_C *pLayer = vLayer[nZ + nDZ - m_nOffsetZ];
					pNextRGrid->m_nG = nNewDistance;
					int nSumH = 0;
					//for( int t=0; t<vTerminal.size(); t++ )
					//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
					nSumH = nEularDistance(vTerminal, pNextGrid, pLayer);
					pNextRGrid->m_nH = nSumH;
					pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
					pNextRGrid->m_pFrom = pCurRGrid;
					bool bInsert = false;
					//for( int j=0; j<vHistory.size(); j++ )
					for (int j = nHistoryIndex; j < vHistory.size(); j++)
					{
						if (pNextRGrid->m_nCost <= vHistory[j]->m_nCost)
						{
							vHistory.insert(vHistory.begin() + j, pNextRGrid);
							bInsert = true;
							break;
						}
					}
					if (!bInsert)
					{
						vHistory.push_back(pNextRGrid);
					}
					vAddGrid.push_back(pNextRGrid);
				}
			}
			//cout<<"test"<<endl;
			pNextGrid = graphTravel(m_pDesign, pGGrid, -nDX, -nDY, -nDZ);
			if (pNextGrid != NULL && pNextGrid->getRemand() > 0)
			{
				rGrid_C *pNextRGrid = getRGrid(nX - nDX, nY - nDY, nZ - nDZ);
				if (!pNextRGrid->isRouted)
				{
					//pNextRGrid->isRouted = true;
					layer_C *pLayer = vLayer[nZ - nDZ - m_nOffsetZ];
					pNextRGrid->m_nG = nNewDistance;
					int nSumH = 0;
					//for( int t=0; t<vTerminal.size(); t++ )
					//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
					nSumH = nEularDistance(vTerminal, pNextGrid, pLayer);
					pNextRGrid->m_nH = nSumH;
					pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
					pNextRGrid->m_pFrom = pCurRGrid;
					bool bInsert = false;
					//for( int j=0; j<vHistory.size(); j++ )
					for (int j = nHistoryIndex; j < vHistory.size(); j++)
					{
						if (pNextRGrid->m_nCost <= vHistory[j]->m_nCost)
						{
							vHistory.insert(vHistory.begin() + j, pNextRGrid);
							bInsert = true;
							break;
						}
					}
					if (!bInsert)
					{
						vHistory.push_back(pNextRGrid);
					}
					vAddGrid.push_back(pNextRGrid);
				}
			}

			// go to different layer
			//cout<<"different layer"<<endl;
			pNextGrid = graphTravel(m_pDesign, pGGrid, 0, 0, 1);
			if (pNextGrid != NULL && pNextGrid->getRemand() > 0)
			{
				rGrid_C *pNextRGrid = getRGrid(nX, nY, nZ + 1);
				if (!pNextRGrid->isRouted)
				{
					//pNextRGrid->isRouted = true;
					layer_C *pLayer = vLayer[nZ + 1 - m_nOffsetZ];
					pNextRGrid->m_nG = nNewDistance;
					int nSumH = 0;
					//for( int t=0; t<vTerminal.size(); t++ )
					//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
					nSumH = nEularDistance(vTerminal, pNextGrid, pLayer);
					pNextRGrid->m_nH = nSumH;
					pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
					pNextRGrid->m_pFrom = pCurRGrid;
					bool bInsert = false;
					//for( int j=0; j<vHistory.size(); j++ )
					for (int j = nHistoryIndex; j < vHistory.size(); j++)
					{
						if (pNextRGrid->m_nCost <= vHistory[j]->m_nCost)
						{
							vHistory.insert(vHistory.begin() + j, pNextRGrid);
							bInsert = true;
							break;
						}
					}
					if (!bInsert)
					{
						vHistory.push_back(pNextRGrid);
					}
					vAddGrid.push_back(pNextRGrid);
				}
			}

			if (nZ - 1 >= nLConstraint)
			{
				pNextGrid = graphTravel(m_pDesign, pGGrid, 0, 0, -1);
				if (pNextGrid != NULL && pNextGrid->getRemand() > 0)
				{
					rGrid_C *pNextRGrid = getRGrid(nX, nY, nZ - 1);
					if (!pNextRGrid->isRouted)
					{
						//pNextRGrid->isRouted = true;
						layer_C *pLayer = vLayer[nZ - 1 - m_nOffsetZ];
						pNextRGrid->m_nG = nNewDistance;
						int nSumH = 0;
						//for( int t=0; t<vTerminal.size(); t++ )
						//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
						nSumH = nEularDistance(vTerminal, pNextGrid, pLayer);
						pNextRGrid->m_nH = nSumH;
						pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
						pNextRGrid->m_pFrom = pCurRGrid;
						bool bInsert = false;
						//for( int j=0; j<vHistory.size(); j++ )
						for (int j = nHistoryIndex; j < vHistory.size(); j++)
						{
							if (pNextRGrid->m_nCost <= vHistory[j]->m_nCost)
							{
								vHistory.insert(vHistory.begin() + j, pNextRGrid);
								bInsert = true;
								break;
							}
						}
						if (!bInsert)
						{
							vHistory.push_back(pNextRGrid);
						}
						vAddGrid.push_back(pNextRGrid);
					}
				}
			}
		}
		//cout<<"Check target"<<endl;
		// if there is a target in addgrid;
		//vector< rGrid_C* > vHaveAddTarget;
		bool bFindTarget = false;
		for (int i = 0; i < vAddGrid.size(); i++)
		{
			if (vAddGrid[i]->isTarget)
			{
				//if( pNet->getName() == "N1482")
				//	cout<<"Target is: "<<vAddGrid[i]->m_nX<<" "<<vAddGrid[i]->m_nY<<" "<<vAddGrid[i]->m_nZ<<" "<<endl;
				bFindTarget = true;
				bool bHaveSearch = false;
				for (int j = 0; j < vHaveAddTarget.size(); j++)
				{
					if (vHaveAddTarget[j] == vAddGrid[i])
					{
						bHaveSearch = true;
						break;
					}
				}
				if (bHaveSearch)
					continue;

				vHaveAddTarget.push_back(vAddGrid[i]);
				//cout<<"Back trace"<<endl;
				// find the path
				vector<gGrid_C *> vReversePath;

				int nPath = vAddGrid[i]->m_nG;
				//if( pNet->getName() == "N1482")
				//	cout<<"Path cost: "<<nPath<<endl;
				rGrid_C *pRPath = vAddGrid[i];
				pRPath->isPath = true;
				gGrid_C *pPathGrid = getGrid(m_pDesign, pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ);
				rGrid_C *pPreGrid = pRPath->m_pFrom;
				/*
				//  added at 07/20 20:30
				layer_C* pTmpLayer = m_pDesign->getLayer()[ pRPath->m_nZ - m_nOffsetZ ];
				int nPDX;
				int nPDY;
				if( pTmpLayer->getDir() == 'H' )
				{
					nPDX = 0; nPDY = 1;
				}
				else
				{
					nPDX = 1; nPDY = 0;
				}
				
				rGrid_C* pRGrid1 = getRGrid( pRPath->m_nX + nPDX, pRPath->m_nY + nPDY, pRPath->m_nZ );
				rGrid_C* pRGrid2 = getRGrid( pRPath->m_nX - nPDX, pRPath->m_nY - nPDY, pRPath->m_nZ );	
				rGrid_C* pRGrid3 = getRGrid( pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ + 1 );
				rGrid_C* pRGrid4 = getRGrid( pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ - 1 );
				vector< rGrid_C* > vTmpRGrid;
				//for( int rg=0; rg<4; rg++ )
				//{
					if( pRGrid1 != NULL && pRGrid1->isRouted )
						vTmpRGrid.push_back( pRGrid1 );
					if( pRGrid2 != NULL && pRGrid2->isRouted )
						vTmpRGrid.push_back( pRGrid2 );
					if( pRGrid3 != NULL && pRGrid3->isRouted )
						vTmpRGrid.push_back( pRGrid3 );
					if( pRGrid4 != NULL && pRGrid4->isRouted )
						vTmpRGrid.push_back( pRGrid4 );
				//}
				int nTmpPath = nPath;
				rGrid_C* pMinRGrid = vAddGrid[i]->m_pFrom;
				for( int rg=0; rg<vTmpRGrid.size(); rg++ )
				{
					if( nTmpPath > vTmpRGrid[ rg ]->m_nG )
					{
						nTmpPath = vTmpRGrid[ rg ]->m_nG;
						pMinRGrid = vTmpRGrid[ rg ];
					}	
				}
				vTmpRGrid.clear();
				vAddGrid[i]->m_pFrom = pMinRGrid;
				int nAddPath = 1;
				rGrid_C* pPreGrid = vAddGrid[i]->m_pFrom;
				//end added at 07/04 20:30
				*/
				//gGrid_C* pPathGrid = getGrid( m_pDesign, pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ );
				//vResult.push_back( pPathGrid );
				vReversePath.push_back(pPathGrid);
				while (!pPreGrid->isPath)
				{
					pRPath = pPreGrid;
					pRPath->isPath = true;
					pPathGrid = getGrid(m_pDesign, pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ);
					//vResult.push_back( pPathGrid );
					vReversePath.push_back(pPathGrid);
					/*
					// added at 07/04 20:30	
					pTmpLayer = m_pDesign->getLayer()[ pRPath->m_nZ - m_nOffsetZ ];
					
					if( pTmpLayer->getDir() == 'H' )
					{
						nPDX = 0; nPDY = 1;
					}
					else
					{
						nPDX = 1; nPDY = 0;
					}
					
					pRGrid1 = getRGrid( pRPath->m_nX + nPDX, pRPath->m_nY + nPDY, pRPath->m_nZ );
					pRGrid2 = getRGrid( pRPath->m_nX - nPDX, pRPath->m_nY - nPDY, pRPath->m_nZ );	
					pRGrid3 = getRGrid( pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ + 1 );
					pRGrid4 = getRGrid( pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ - 1 );
					//vector< rGrid_C* > vTmpRGrid;
					//for( int rg=0; rg<4; rg++ )
					//{
						if( pRGrid1 != NULL && pRGrid1->isRouted )
							vTmpRGrid.push_back( pRGrid1 );
						if( pRGrid2 != NULL && pRGrid2->isRouted )
							vTmpRGrid.push_back( pRGrid2 );
						if( pRGrid3 != NULL && pRGrid3->isRouted )
							vTmpRGrid.push_back( pRGrid3 );
						if( pRGrid4 != NULL && pRGrid4->isRouted )
							vTmpRGrid.push_back( pRGrid4 );
					//}
					int nTmpPath = nPath;
					rGrid_C* pMinRGrid = pRPath->m_pFrom;
					for( int rg=0; rg<vTmpRGrid.size(); rg++ )
					{
						if( nTmpPath > vTmpRGrid[ rg ]->m_nG )
						{
							nTmpPath = vTmpRGrid[ rg ]->m_nG;
							pMinRGrid = vTmpRGrid[ rg ];
						}	
					}
					vTmpRGrid.clear();
					pRPath->m_pFrom = pMinRGrid;
					nAddPath++;
					// end added at 07/04 20:30
					*/
					pPreGrid = pRPath->m_pFrom;
				}
				/*
				// added at 07/04 20:30
				nPath = pPreGrid->m_nG + nAddPath;
				// end added at 07/04 20:30
				*/

				pPathGrid = getGrid(m_pDesign, pPreGrid->m_nX, pPreGrid->m_nY, pPreGrid->m_nZ);
				//vResult.push_back( pPathGrid );
				vReversePath.push_back(pPathGrid);
				vector<gGrid_C *> vTmpResult;
				for (int g = vReversePath.size() - 1; g >= 0; g--)
				{
					vTmpResult.push_back(vReversePath[g]);
				}

				vResult.push_back(vTmpResult);
				/*
				if( pNet->getName() == "N1482" )
				{
					for( int p=0; p<vReversePath.size(); p++ )
					{
						int pX, pY, pZ;
						vReversePath[p]->getPosition( pX, pY, pZ );
						cout<<"("<<pX<<","<<pY<<","<<pZ<<") ";
					}
					cout<<endl;
				}
				*/
				// find the terminal and remove
				for (int t = 0; t < vTerminal.size(); t++)
				{
					int nTX, nTY, nTZ;
					vTerminal[t]->getPosition(nTX, nTY, nTZ);
					if (vAddGrid[i]->m_nX == nTX && vAddGrid[i]->m_nY == nTY && vAddGrid[i]->m_nZ == nTZ)
					{
						vTerminal.erase(vTerminal.begin() + t);
						nNumTermanals--;
						break;
					}
				}
				// update the nG in all the searched node ( without path )
				for (int h = 0; h < vHistory.size(); h++)
				{
					rGrid_C *pTmpGrid = vHistory[h];
					if (!pTmpGrid->isPath)
						calDistanceFromPath(pTmpGrid, nPath);
					else
						pTmpGrid->m_nG = nPath;
				}

				// update the nH in all the node in queue
				for (int h = nHistoryIndex; h < vHistory.size(); h++)
				{
					rGrid_C *pTmpGrid = vHistory[h];
					if (!pTmpGrid->isRouted)
					{
						int nH = 0;
						gGrid_C *pTmpGGrid = getGrid(m_pDesign, pTmpGrid->m_nX, pTmpGrid->m_nY, pTmpGrid->m_nZ);
						int nX, nY, nZ;
						pTmpGGrid->getPosition(nX, nY, nZ);
						layer_C *pLayer = vLayer[nZ - m_nOffsetZ];
						//for( int t=0; t<vTerminal.size(); t++ )
						//{
						//	nH = nH + nEularDistance( vTerminal[t], pTmpGGrid, pLayer );
						//}
						nH = nEularDistance(vTerminal, pTmpGGrid, pLayer);
						pTmpGrid->m_nH = nH;
						pTmpGrid->m_nCost = nH + pTmpGrid->m_nG;
					}
				}
				//cout<<endl;
			}
		}

		if (bFindTarget)
		{
			for (int i = nHistoryIndex; i < vHistory.size(); i++)
			{
				for (int j = i - 1; j >= nHistoryIndex; j--)
				{
					if (vHistory[j]->m_nCost > vHistory[j + 1]->m_nCost)
					{
						rGrid_C *pFGrid = vHistory[j];
						rGrid_C *pBGrid = vHistory[j + 1];
						vHistory[j] = pBGrid;
						vHistory[j + 1] = pFGrid;
					}
					else if (vHistory[j]->m_nCost ==  vHistory[j + 1]->m_nCost)
					{
						rGrid_C *pFGrid = vHistory[j];
						rGrid_C *pBGrid = vHistory[j + 1];
						if( pFGrid->m_nZ == pCurRGrid->m_nZ )
							break;
						else if( pBGrid->m_nZ == pCurRGrid->m_nZ )
						{	
							vHistory[j] = pBGrid;
							vHistory[j + 1] = pFGrid;	
						}
						else
							break;
					}
					else
						break;
				}
			}
		}
	}

	for (int i = 0; i < vHistory.size(); i++)
	{
		rGrid_C *pRGrid = vHistory[i];
		pRGrid->m_nH = 0;
		pRGrid->m_nG = 0;
		pRGrid->m_nCost = 0;
		pRGrid->m_nBend = 0;
		pRGrid->m_pFrom = NULL;
		pRGrid->isRouted = false;
		pRGrid->isPath = false;
		pRGrid->isTarget = false;
	}

	/*	
	for( int i=0; i<vResult.size(); i++ )
	{
		int nX, nY, nZ;
		gGrid_C* pTmpGrid = vResult[i];
		pTmpGrid->getPosition( nX, nY, nZ );
		rGrid_C* pRGrid = getRGrid( nX, nY, nZ );
		pRGrid->m_nH = 0;
		pRGrid->m_nG = 0;
		pRGrid->m_nCost = 0;
		pRGrid->m_nBend = 0;
		pRGrid->m_pFrom = NULL;
		pRGrid->isRouted = false;
		pRGrid->isPath = false;
		pRGrid->isTarget = false;
	}
	*/
	for (int i = 0; i < vTerminal.size(); i++)
	{
		int nX, nY, nZ;
		gGrid_C *pTmpGrid = vTerminal[i];
		pTmpGrid->getPosition(nX, nY, nZ);
		rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
		pRGrid->m_nH = 0;
		pRGrid->m_nG = 0;
		pRGrid->m_nCost = 0;
		pRGrid->m_nBend = 0;
		pRGrid->m_pFrom = NULL;
		pRGrid->isRouted = false;
		pRGrid->isPath = false;
		pRGrid->isTarget = false;
	}
	if (nNumTermanals != 0)
	{
		//cout << "Some net out of constraint" << endl;
		vResult.clear();
	}
	return vResult;
}


vector<vector<gGrid_C *>> router_C::routeNet_length_constraint_ver2(net_C *pNet, const int nConstraint)
{
	//cout<<pNet->getName()<<endl;
	setRoutingConstraint(pNet);
	int &nLConstraint = m_nConstraintLayerId;
	//cout<<nLConstraint<<endl;

	vector<gGrid_C *> vTerminal;
	vector<pin_C *> vPin = pNet->getPin();
	//vector< wire_C* > vWire;
	vector<vector<gGrid_C *>> vResult;
	for (int i = 0; i < vPin.size(); i++)
	{
		pin_C *pPin = vPin[i];
		int nX, nY, nZ;
		instance_C *pInst = (instance_C *)pPin->getCell();
		nX = pInst->getPlacedX();
		nY = pInst->getPlacedY();
		// project the z to the constraint layer
		nZ = pPin->getLayerId();
		if (nZ < nLConstraint)
		{
			vector<gGrid_C *> vTmpResult;
			for (int l = nZ; l <= nLConstraint; l++)
			{
				gGrid_C *pGrid = getGrid(m_pDesign, nX, nY, l);
				vTmpResult.push_back(pGrid);
			}
			nZ = nLConstraint;
			vResult.push_back(vTmpResult);
		}

		gGrid_C *pTerminal = getGrid(m_pDesign, nX, nY, nZ);
		rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
		if (!pRGrid->isTarget) // if pRGrid is already put in vTerminal
		{
			pRGrid->isTarget = true;
			vTerminal.push_back(pTerminal);
		}
	}

	// check same position of terminal
	//cout<<"Check terminal"<<endl;
	/*
	for( int i=0; i<vTerminal.size(); i++ )
	{
		
		for( int j=i+1; j<vTerminal.size(); j++ )
		{
			if( vTerminal[i] == vTerminal[j] )
			{
				vTerminal.erase( vTerminal.begin() + j );
				j--;
			}
		}
		

	}
	*/
	//cout<<"Num of terminals: "<<vTerminal.size()<<endl;
	bool bFindOverflow = false;
	for( int i=0; i<vResult.size(); i++ )
	{
		for( int j=0; j<vResult[i].size(); j++ )
		{
			if( vResult[i][j]->getRemand() - vResult[i][j]->getPinDemand() <= 0 )
			{
				bFindOverflow = true;
				break;
			}
		}
		if( bFindOverflow )
			break;
	}
	if( bFindOverflow )
	{
		cout << "Failed at target"<<endl;
		for (int i = 0; i < vTerminal.size(); i++)
		{
			int nX, nY, nZ;
			vTerminal[i]->getPosition(nX, nY, nZ);
			rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
			pRGrid->isTarget = false;
		}
		vResult.clear();
		return vResult;
	}

	if (vTerminal.size() == 1)
	{
		if (vTerminal[0]->getRemand() <= 0)
		{

			int nX, nY, nZ;
			vTerminal[0]->getPosition(nX, nY, nZ);
			rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
			pRGrid->isTarget = false;
			vResult.clear();
			return vResult;
		}
		int nX, nY, nZ;
		vTerminal[0]->getPosition(nX, nY, nZ);
		rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
		pRGrid->isTarget = false;
		vector<gGrid_C *> vTmpResult;
		vTmpResult.push_back(vTerminal[0]);
		vResult.push_back(vTmpResult);
		return vResult;
	}
	else
	{
		bool bFindOverflow = false;
		for (int i = 0; i < vTerminal.size(); i++)
		{
			if (vTerminal[i]->getRemand() <= 0)
			{
				bFindOverflow = true;
				break;
			}
		}

		if (bFindOverflow)
		{
			cout << "Failed at target"<<endl;
			for (int i = 0; i < vTerminal.size(); i++)
			{
				int nX, nY, nZ;
				vTerminal[i]->getPosition(nX, nY, nZ);
				rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
				pRGrid->isTarget = false;
			}
			if (vResult.size() != 0)
				vResult.clear();

			return vResult;
		}
	}

	vector<rGrid_C *> vRoutedGrid;
	//vector< rGrid_C* > vPriorityQueue;
	vector<rGrid_C *> vHistory;
	vector<rGrid_C *> vNotInPath;
	// find first terminal
	gGrid_C *pFirst = NULL;

	int nMinX = m_nTX;
	for (int i = 0; i < vTerminal.size(); i++)
	{
		gGrid_C *pGrid = vTerminal[i];
		int nX, nY, nZ;
		pGrid->getPosition(nX, nY, nZ);
		if (nX <= nMinX)
			nMinX = nX;
	}

	int nMinY = m_nTY;
	for (int i = 0; i < vTerminal.size(); i++)
	{
		gGrid_C *pGrid = vTerminal[i];
		int nX, nY, nZ;
		pGrid->getPosition(nX, nY, nZ);
		if (nX == nMinX && nY <= nMinY)
		{
			nMinY = nY;
			pFirst = vTerminal[i];
		}
	}

	//
	//pFirst = vTerminal[0];

	//chech the terminal is available to route the net
	int nX, nY, nZ;
	pFirst->getPosition(nX, nY, nZ);
	rGrid_C *pCurRGrid = getRGrid(nX, nY, nZ);
	gGrid_C *pGGrid = pFirst;
	if (pGGrid->getRemand() <= 0)
	{
		pCurRGrid->isTarget = false;
		// added at 0705 22:00
		for (int i = 0; i < vTerminal.size(); i++)
		{
			vTerminal[i]->getPosition(nX, nY, nZ);
			pCurRGrid = getRGrid(nX, nY, nZ);
			pCurRGrid->isTarget = false;
		}
		// end added at 0705 22:00
		vResult.clear();
		return vResult;
	}

	vHistory.push_back(pCurRGrid);
	pCurRGrid->isPath = true;
	for (int i = 0; i < vTerminal.size(); i++)
	{
		if (vTerminal[i] == pGGrid)
		{
			vTerminal.erase(vTerminal.begin() + i);
			break;
		}
	}
	int nNumTermanals = vTerminal.size();
	//nNumTermanals--; // because the first grid is a termianl
	vector<layer_C *> vLayer = m_pDesign->getLayer();
	//cout<<"Start routing"<<endl;
	vector<rGrid_C *> vHaveAddTarget;
	int nHistoryIndex = 0;
	while (nNumTermanals > 0)
	{
		//cout<<nNumTermanals<<endl;
		//cout<<pCurRGrid->m_nX<<" "<<pCurRGrid->m_nY<<" "<<pCurRGrid->m_nZ<<endl;
		// A* propogation
		if (vHistory.size() == 0)
		{
			vResult.clear();
			break;
		}
		//cout<<vHistory.size()<<endl;

		vector<rGrid_C *> vToPropogate;
		int nMinCost = 0;
		/*
		bool bPick = false;
		
		for( int i=0; i<vHistory.size(); i++ )
		{
			if( !vHistory[i]->isRouted )	
			{
				nMinCost = vHistory[i]->m_nCost;
				bPick = true;
				break;
			}
		}
		if( !bPick )
		{
			vResult.clear();
			//cout<<"Can't find result"<<endl;
			break;
		}
		*/
		if (nHistoryIndex == vHistory.size())
			break;
		else
		{
			nMinCost = vHistory[nHistoryIndex]->m_nCost;
			vToPropogate.push_back(vHistory[nHistoryIndex]);
			//nHistoryIndex++;
		}
		/*
		cout<<nMinCost<<" "<<vHistory.size()<<endl;
		for( int i=0; i<vHistory.size(); i++ )
		{
			if( !vHistory[i]->isRouted )	
			{
				cout << vHistory[i]->m_nCost <<" ";
			}
		}
		//cout<<endl;
		*/
		/*
		for( int i=0; i<vHistory.size(); i++ )
		{
			if( !vHistory[i]->isRouted )	
			{
				if( vHistory[i]->m_nCost == nMinCost )
				{
					vToPropogate.push_back( vHistory[i] );
					break;
				}
				//else
				//	break;
			}
		}
		*/
		/*	
		//----------------------------------------------------------------------------choubegin TODO:
		vector<rGrid_C *> samecost;
		int vsum = INT_MAX;
		rGrid_C *temprG;
		gGrid_C *tempgG;
		for (int i = nHistoryIndex; i < vHistory.size(); i++)
		{
			if (!vHistory[i]->isRouted)
			{
				if (vHistory[i]->m_nCost == nMinCost)
				{
					samecost.push_back(vHistory[i]);
				}
				else
					break;
			}
		}
		//cout << "iter--------" << endl;
		for (int x = 0; x < samecost.size(); x++)
		{
			int az = samecost[x]->m_nZ;
			layer_C *pLayer = vLayer[az - m_nOffsetZ];
			//if (pNet->getName() == "N2")
			//	cout << "same : " << samecost[x]->m_nX << " " << samecost[x]->m_nY << " " << samecost[x]->m_nZ << endl;
			tempgG = m_pDesign->getGrid(samecost[x]->m_nX, samecost[x]->m_nY, az);
			if (vsum > nEularDistance_sum(vTerminal, tempgG, pLayer))
			{
				vsum = nEularDistance_sum(vTerminal, tempgG, pLayer);
				temprG = samecost[x];
			}
		}
		for( int i=nHistoryIndex; i<vHistory.size(); i++ )
		{
			if( temprG == vHistory[i] )
			{
				vHistory[i] = vHistory[ nHistoryIndex ];
				vHistory[ nHistoryIndex ] = temprG;
				break;
			}
		}
		nHistoryIndex++;
		vToPropogate.clear();
		vToPropogate.push_back(temprG);
		samecost.clear();
		//------------------------------------------------------------------------------chouend 
 		*/
		nHistoryIndex++;
		/*	
		if (pNet->getName() == "N1482")
		{
			cout<<endl;
			cout << "Routed: ";
			for (int i = 0; i < vHistory.size(); i++)
			{
				if( vHistory[i]->isRouted )
				cout << "(" << vHistory[i]->m_nX << " " << vHistory[i]->m_nY << " " << vHistory[i]->m_nZ << " " << vHistory[i]->m_nCost << " )";
			}
			cout << endl;
			cout<< "His: ";
			for (int i = 0; i < vHistory.size(); i++)
			{
				if( !vHistory[i]->isRouted )
				cout << "(" << vHistory[i]->m_nX << " " << vHistory[i]->m_nY << " " << vHistory[i]->m_nZ << " " << vHistory[i]->m_nG <<" " << vHistory[i]->m_nH <<" " << vHistory[i]->m_nCost << " )";
			}
			cout<<endl;
		}
		*/
		vector<rGrid_C *> vAddGrid;

		for (int i = 0; i < vToPropogate.size(); i++)
		{
			pCurRGrid = vToPropogate[i];
			pCurRGrid->isRouted = true;
			int nCurDistance = pCurRGrid->m_nG;
			int nNewDistance = nCurDistance + 1;
			if (nNewDistance > nConstraint)
				continue;

			int nX, nY, nZ;
			nX = pCurRGrid->m_nX;
			nY = pCurRGrid->m_nY;
			nZ = pCurRGrid->m_nZ;
			//cout<<"P: "<<nX<<" "<<nY<<" "<<nZ<<endl;
			pGGrid = getGrid(m_pDesign, nX, nY, nZ);
			char cDir = vLayer[nZ - m_nOffsetZ]->getDir();
			//cout<<cDir<<endl;
			int nDX, nDY, nDZ;
			if (cDir == 'H')
			{
				nDX = 0;
				nDY = 1;
				nDZ = 0;
			}
			else if (cDir == 'V')
			{
				nDX = 1;
				nDY = 0;
				nDZ = 0;
			}
			else
				cout << "Unknown routing direction" << endl;
			// go in the same layer;
			gGrid_C *pNextGrid = graphTravel(m_pDesign, pGGrid, nDX, nDY, nDZ);
			if (pNextGrid != NULL && pNextGrid->getRemand() - pNextGrid->getPinDemand() > 0)
			{
				rGrid_C *pNextRGrid = getRGrid(nX + nDX, nY + nDY, nZ + nDZ);
				if (!pNextRGrid->isRouted)
				{
					//pNextRGrid->isRouted = true;
					layer_C *pLayer = vLayer[nZ + nDZ - m_nOffsetZ];
					pNextRGrid->m_nG = nNewDistance;
					int nSumH = 0;
					//for( int t=0; t<vTerminal.size(); t++ )
					//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
					nSumH = nEularDistance(vTerminal, pNextGrid, pLayer);
					pNextRGrid->m_nH = nSumH;
					pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
					pNextRGrid->m_pFrom = pCurRGrid;
					bool bInsert = false;
					//for( int j=0; j<vHistory.size(); j++ )
					for (int j = nHistoryIndex; j < vHistory.size(); j++)
					{
						if (pNextRGrid->m_nCost <= vHistory[j]->m_nCost)
						{
							vHistory.insert(vHistory.begin() + j, pNextRGrid);
							bInsert = true;
							break;
						}
					}
					if (!bInsert)
					{
						vHistory.push_back(pNextRGrid);
					}
					vAddGrid.push_back(pNextRGrid);
				}
			}
			//cout<<"test"<<endl;
			pNextGrid = graphTravel(m_pDesign, pGGrid, -nDX, -nDY, -nDZ);
			if (pNextGrid != NULL && pNextGrid->getRemand() - pNextGrid->getPinDemand() > 0)
			{
				rGrid_C *pNextRGrid = getRGrid(nX - nDX, nY - nDY, nZ - nDZ);
				if (!pNextRGrid->isRouted)
				{
					//pNextRGrid->isRouted = true;
					layer_C *pLayer = vLayer[nZ - nDZ - m_nOffsetZ];
					pNextRGrid->m_nG = nNewDistance;
					int nSumH = 0;
					//for( int t=0; t<vTerminal.size(); t++ )
					//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
					nSumH = nEularDistance(vTerminal, pNextGrid, pLayer);
					pNextRGrid->m_nH = nSumH;
					pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
					pNextRGrid->m_pFrom = pCurRGrid;
					bool bInsert = false;
					//for( int j=0; j<vHistory.size(); j++ )
					for (int j = nHistoryIndex; j < vHistory.size(); j++)
					{
						if (pNextRGrid->m_nCost <= vHistory[j]->m_nCost)
						{
							vHistory.insert(vHistory.begin() + j, pNextRGrid);
							bInsert = true;
							break;
						}
					}
					if (!bInsert)
					{
						vHistory.push_back(pNextRGrid);
					}
					vAddGrid.push_back(pNextRGrid);
				}
			}

			// go to different layer
			//cout<<"different layer"<<endl;
			pNextGrid = graphTravel(m_pDesign, pGGrid, 0, 0, 1);
			if (pNextGrid != NULL && pNextGrid->getRemand() - pNextGrid->getPinDemand() > 0)
			{
				rGrid_C *pNextRGrid = getRGrid(nX, nY, nZ + 1);
				if (!pNextRGrid->isRouted)
				{
					//pNextRGrid->isRouted = true;
					layer_C *pLayer = vLayer[nZ + 1 - m_nOffsetZ];
					pNextRGrid->m_nG = nNewDistance;
					int nSumH = 0;
					//for( int t=0; t<vTerminal.size(); t++ )
					//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
					nSumH = nEularDistance(vTerminal, pNextGrid, pLayer);
					pNextRGrid->m_nH = nSumH;
					pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
					pNextRGrid->m_pFrom = pCurRGrid;
					bool bInsert = false;
					//for( int j=0; j<vHistory.size(); j++ )
					for (int j = nHistoryIndex; j < vHistory.size(); j++)
					{
						if (pNextRGrid->m_nCost <= vHistory[j]->m_nCost)
						{
							vHistory.insert(vHistory.begin() + j, pNextRGrid);
							bInsert = true;
							break;
						}
					}
					if (!bInsert)
					{
						vHistory.push_back(pNextRGrid);
					}
					vAddGrid.push_back(pNextRGrid);
				}
			}

			if (nZ - 1 >= nLConstraint)
			{
				pNextGrid = graphTravel(m_pDesign, pGGrid, 0, 0, -1);
				if (pNextGrid != NULL && pNextGrid->getRemand() - pNextGrid->getPinDemand() > 0)
				{
					rGrid_C *pNextRGrid = getRGrid(nX, nY, nZ - 1);
					if (!pNextRGrid->isRouted)
					{
						//pNextRGrid->isRouted = true;
						layer_C *pLayer = vLayer[nZ - 1 - m_nOffsetZ];
						pNextRGrid->m_nG = nNewDistance;
						int nSumH = 0;
						//for( int t=0; t<vTerminal.size(); t++ )
						//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
						nSumH = nEularDistance(vTerminal, pNextGrid, pLayer);
						pNextRGrid->m_nH = nSumH;
						pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
						pNextRGrid->m_pFrom = pCurRGrid;
						bool bInsert = false;
						//for( int j=0; j<vHistory.size(); j++ )
						for (int j = nHistoryIndex; j < vHistory.size(); j++)
						{
							if (pNextRGrid->m_nCost <= vHistory[j]->m_nCost)
							{
								vHistory.insert(vHistory.begin() + j, pNextRGrid);
								bInsert = true;
								break;
							}
						}
						if (!bInsert)
						{
							vHistory.push_back(pNextRGrid);
						}
						vAddGrid.push_back(pNextRGrid);
					}
				}
			}
		}
		//cout<<"Check target"<<endl;
		// if there is a target in addgrid;
		//vector< rGrid_C* > vHaveAddTarget;
		bool bFindTarget = false;
		for (int i = 0; i < vAddGrid.size(); i++)
		{
			if (vAddGrid[i]->isTarget)
			{
				//if( pNet->getName() == "N1482")
				//	cout<<"Target is: "<<vAddGrid[i]->m_nX<<" "<<vAddGrid[i]->m_nY<<" "<<vAddGrid[i]->m_nZ<<" "<<endl;
				bFindTarget = true;
				bool bHaveSearch = false;
				for (int j = 0; j < vHaveAddTarget.size(); j++)
				{
					if (vHaveAddTarget[j] == vAddGrid[i])
					{
						bHaveSearch = true;
						break;
					}
				}
				if (bHaveSearch)
					continue;

				vHaveAddTarget.push_back(vAddGrid[i]);
				//cout<<"Back trace"<<endl;
				// find the path
				vector<gGrid_C *> vReversePath;

				int nPath = vAddGrid[i]->m_nG;
				//if( pNet->getName() == "N1482")
				//	cout<<"Path cost: "<<nPath<<endl;
				rGrid_C *pRPath = vAddGrid[i];
				pRPath->isPath = true;
				gGrid_C *pPathGrid = getGrid(m_pDesign, pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ);
				rGrid_C *pPreGrid = pRPath->m_pFrom;
				/*
				//  added at 07/20 20:30
				layer_C* pTmpLayer = m_pDesign->getLayer()[ pRPath->m_nZ - m_nOffsetZ ];
				int nPDX;
				int nPDY;
				if( pTmpLayer->getDir() == 'H' )
				{
					nPDX = 0; nPDY = 1;
				}
				else
				{
					nPDX = 1; nPDY = 0;
				}
				
				rGrid_C* pRGrid1 = getRGrid( pRPath->m_nX + nPDX, pRPath->m_nY + nPDY, pRPath->m_nZ );
				rGrid_C* pRGrid2 = getRGrid( pRPath->m_nX - nPDX, pRPath->m_nY - nPDY, pRPath->m_nZ );	
				rGrid_C* pRGrid3 = getRGrid( pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ + 1 );
				rGrid_C* pRGrid4 = getRGrid( pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ - 1 );
				vector< rGrid_C* > vTmpRGrid;
				//for( int rg=0; rg<4; rg++ )
				//{
					if( pRGrid1 != NULL && pRGrid1->isRouted )
						vTmpRGrid.push_back( pRGrid1 );
					if( pRGrid2 != NULL && pRGrid2->isRouted )
						vTmpRGrid.push_back( pRGrid2 );
					if( pRGrid3 != NULL && pRGrid3->isRouted )
						vTmpRGrid.push_back( pRGrid3 );
					if( pRGrid4 != NULL && pRGrid4->isRouted )
						vTmpRGrid.push_back( pRGrid4 );
				//}
				int nTmpPath = nPath;
				rGrid_C* pMinRGrid = vAddGrid[i]->m_pFrom;
				for( int rg=0; rg<vTmpRGrid.size(); rg++ )
				{
					if( nTmpPath > vTmpRGrid[ rg ]->m_nG )
					{
						nTmpPath = vTmpRGrid[ rg ]->m_nG;
						pMinRGrid = vTmpRGrid[ rg ];
					}	
				}
				vTmpRGrid.clear();
				vAddGrid[i]->m_pFrom = pMinRGrid;
				int nAddPath = 1;
				rGrid_C* pPreGrid = vAddGrid[i]->m_pFrom;
				//end added at 07/04 20:30
				*/
				//gGrid_C* pPathGrid = getGrid( m_pDesign, pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ );
				//vResult.push_back( pPathGrid );
				vReversePath.push_back(pPathGrid);
				while (!pPreGrid->isPath)
				{
					pRPath = pPreGrid;
					pRPath->isPath = true;
					pPathGrid = getGrid(m_pDesign, pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ);
					//vResult.push_back( pPathGrid );
					vReversePath.push_back(pPathGrid);
					/*
					// added at 07/04 20:30	
					pTmpLayer = m_pDesign->getLayer()[ pRPath->m_nZ - m_nOffsetZ ];
					
					if( pTmpLayer->getDir() == 'H' )
					{
						nPDX = 0; nPDY = 1;
					}
					else
					{
						nPDX = 1; nPDY = 0;
					}
					
					pRGrid1 = getRGrid( pRPath->m_nX + nPDX, pRPath->m_nY + nPDY, pRPath->m_nZ );
					pRGrid2 = getRGrid( pRPath->m_nX - nPDX, pRPath->m_nY - nPDY, pRPath->m_nZ );	
					pRGrid3 = getRGrid( pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ + 1 );
					pRGrid4 = getRGrid( pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ - 1 );
					//vector< rGrid_C* > vTmpRGrid;
					//for( int rg=0; rg<4; rg++ )
					//{
						if( pRGrid1 != NULL && pRGrid1->isRouted )
							vTmpRGrid.push_back( pRGrid1 );
						if( pRGrid2 != NULL && pRGrid2->isRouted )
							vTmpRGrid.push_back( pRGrid2 );
						if( pRGrid3 != NULL && pRGrid3->isRouted )
							vTmpRGrid.push_back( pRGrid3 );
						if( pRGrid4 != NULL && pRGrid4->isRouted )
							vTmpRGrid.push_back( pRGrid4 );
					//}
					int nTmpPath = nPath;
					rGrid_C* pMinRGrid = pRPath->m_pFrom;
					for( int rg=0; rg<vTmpRGrid.size(); rg++ )
					{
						if( nTmpPath > vTmpRGrid[ rg ]->m_nG )
						{
							nTmpPath = vTmpRGrid[ rg ]->m_nG;
							pMinRGrid = vTmpRGrid[ rg ];
						}	
					}
					vTmpRGrid.clear();
					pRPath->m_pFrom = pMinRGrid;
					nAddPath++;
					// end added at 07/04 20:30
					*/
					pPreGrid = pRPath->m_pFrom;
				}
				/*
				// added at 07/04 20:30
				nPath = pPreGrid->m_nG + nAddPath;
				// end added at 07/04 20:30
				*/

				pPathGrid = getGrid(m_pDesign, pPreGrid->m_nX, pPreGrid->m_nY, pPreGrid->m_nZ);
				//vResult.push_back( pPathGrid );
				vReversePath.push_back(pPathGrid);
				vector<gGrid_C *> vTmpResult;
				for (int g = vReversePath.size() - 1; g >= 0; g--)
				{
					vTmpResult.push_back(vReversePath[g]);
				}

				vResult.push_back(vTmpResult);
				/*
				if( pNet->getName() == "N1482" )
				{
					for( int p=0; p<vReversePath.size(); p++ )
					{
						int pX, pY, pZ;
						vReversePath[p]->getPosition( pX, pY, pZ );
						cout<<"("<<pX<<","<<pY<<","<<pZ<<") ";
					}
					cout<<endl;
				}
				*/
				// find the terminal and remove
				for (int t = 0; t < vTerminal.size(); t++)
				{
					int nTX, nTY, nTZ;
					vTerminal[t]->getPosition(nTX, nTY, nTZ);
					if (vAddGrid[i]->m_nX == nTX && vAddGrid[i]->m_nY == nTY && vAddGrid[i]->m_nZ == nTZ)
					{
						vTerminal.erase(vTerminal.begin() + t);
						nNumTermanals--;
						break;
					}
				}
				// update the nG in all the searched node ( without path )
				for (int h = 0; h < vHistory.size(); h++)
				{
					rGrid_C *pTmpGrid = vHistory[h];
					if (!pTmpGrid->isPath)
						calDistanceFromPath(pTmpGrid, nPath);
					else
						pTmpGrid->m_nG = nPath;
				}

				// update the nH in all the node in queue
				for (int h = nHistoryIndex; h < vHistory.size(); h++)
				{
					rGrid_C *pTmpGrid = vHistory[h];
					if (!pTmpGrid->isRouted)
					{
						int nH = 0;
						gGrid_C *pTmpGGrid = getGrid(m_pDesign, pTmpGrid->m_nX, pTmpGrid->m_nY, pTmpGrid->m_nZ);
						int nX, nY, nZ;
						pTmpGGrid->getPosition(nX, nY, nZ);
						layer_C *pLayer = vLayer[nZ - m_nOffsetZ];
						//for( int t=0; t<vTerminal.size(); t++ )
						//{
						//	nH = nH + nEularDistance( vTerminal[t], pTmpGGrid, pLayer );
						//}
						nH = nEularDistance(vTerminal, pTmpGGrid, pLayer);
						pTmpGrid->m_nH = nH;
						pTmpGrid->m_nCost = nH + pTmpGrid->m_nG;
					}
				}
				//cout<<endl;
			}
		}

		if (bFindTarget)
		{
			for (int i = nHistoryIndex; i < vHistory.size(); i++)
			{
				for (int j = i - 1; j >= 0; j--)
				{
					if (vHistory[j]->m_nCost > vHistory[j + 1]->m_nCost)
					{
						rGrid_C *pFGrid = vHistory[j];
						rGrid_C *pBGrid = vHistory[j + 1];
						vHistory[j] = pBGrid;
						vHistory[j + 1] = pFGrid;
					}
					else
						break;
				}
			}
		}
	}

	for (int i = 0; i < vHistory.size(); i++)
	{
		rGrid_C *pRGrid = vHistory[i];
		pRGrid->m_nH = 0;
		pRGrid->m_nG = 0;
		pRGrid->m_nCost = 0;
		pRGrid->m_nBend = 0;
		pRGrid->m_pFrom = NULL;
		pRGrid->isRouted = false;
		pRGrid->isPath = false;
		pRGrid->isTarget = false;
	}

	/*	
	for( int i=0; i<vResult.size(); i++ )
	{
		int nX, nY, nZ;
		gGrid_C* pTmpGrid = vResult[i];
		pTmpGrid->getPosition( nX, nY, nZ );
		rGrid_C* pRGrid = getRGrid( nX, nY, nZ );
		pRGrid->m_nH = 0;
		pRGrid->m_nG = 0;
		pRGrid->m_nCost = 0;
		pRGrid->m_nBend = 0;
		pRGrid->m_pFrom = NULL;
		pRGrid->isRouted = false;
		pRGrid->isPath = false;
		pRGrid->isTarget = false;
	}
	*/
	for (int i = 0; i < vTerminal.size(); i++)
	{
		int nX, nY, nZ;
		gGrid_C *pTmpGrid = vTerminal[i];
		pTmpGrid->getPosition(nX, nY, nZ);
		rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
		pRGrid->m_nH = 0;
		pRGrid->m_nG = 0;
		pRGrid->m_nCost = 0;
		pRGrid->m_nBend = 0;
		pRGrid->m_pFrom = NULL;
		pRGrid->isRouted = false;
		pRGrid->isPath = false;
		pRGrid->isTarget = false;
	}
	if (nNumTermanals != 0)
	{
		cout << "Some net out of constraint" << endl;
		vResult.clear();
	}
	return vResult;
}

vector<gGrid_C *> router_C::routeNet_length_constraint(net_C *pNet, const int nConstraint)
{
	setRoutingConstraint(pNet);
	int &nLConstraint = m_nConstraintLayerId;
	//cout<<nLConstraint<<endl;

	vector<gGrid_C *> vTerminal;
	vector<pin_C *> vPin = pNet->getPin();
	//vector< wire_C* > vWire;
	vector<gGrid_C *> vResult;
	for (int i = 0; i < vPin.size(); i++)
	{
		pin_C *pPin = vPin[i];
		int nX, nY, nZ;
		instance_C *pInst = (instance_C *)pPin->getCell();
		nX = pInst->getPlacedX();
		nY = pInst->getPlacedY();
		// project the z to the constraint layer
		nZ = pPin->getLayerId();
		if (nZ < nLConstraint)
		{
			for (int l = nZ; l <= nLConstraint; l++)
			{
				gGrid_C *pGrid = getGrid(m_pDesign, nX, nY, l);
				vResult.push_back(pGrid);
			}
			nZ = nLConstraint;
		}

		gGrid_C *pTerminal = getGrid(m_pDesign, nX, nY, nZ);
		rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
		if (!pRGrid->isTarget) // if pRGrid is already put in vTerminal
		{
			pRGrid->isTarget = true;
			vTerminal.push_back(pTerminal);
		}
	}

	// check same position of terminal
	//cout<<"Check terminal"<<endl;
	/*
	for( int i=0; i<vTerminal.size(); i++ )
	{
		
		for( int j=i+1; j<vTerminal.size(); j++ )
		{
			if( vTerminal[i] == vTerminal[j] )
			{
				vTerminal.erase( vTerminal.begin() + j );
				j--;
			}
		}
		

	}
	*/
	//cout<<"Num of terminals: "<<vTerminal.size()<<endl;
	if (vTerminal.size() == 1)
	{
		if (vTerminal[0]->getRemand() <= 0)
		{

			int nX, nY, nZ;
			vTerminal[0]->getPosition(nX, nY, nZ);
			rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
			pRGrid->isTarget = false;
			vResult.clear();
			return vResult;
		}
		int nX, nY, nZ;
		vTerminal[0]->getPosition(nX, nY, nZ);
		rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
		pRGrid->isTarget = false;

		vResult.push_back(vTerminal[0]);
		return vResult;
	}
	else
	{
		bool bFindOverflow = false;
		for (int i = 0; i < vTerminal.size(); i++)
		{
			if (vTerminal[i]->getRemand() <= 0)
			{
				bFindOverflow = true;
				break;
			}
		}

		if (bFindOverflow)
		{
			for (int i = 0; i < vTerminal.size(); i++)
			{
				int nX, nY, nZ;
				vTerminal[i]->getPosition(nX, nY, nZ);
				rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
				pRGrid->isTarget = false;
			}
			if (vResult.size() != 0)
				vResult.clear();

			return vResult;
		}
	}

	vector<rGrid_C *> vRoutedGrid;
	//vector< rGrid_C* > vPriorityQueue;
	vector<rGrid_C *> vHistory;
	vector<rGrid_C *> vNotInPath;
	// find first terminal
	gGrid_C *pFirst = NULL;

	int nMinX = m_nTX;
	for (int i = 0; i < vTerminal.size(); i++)
	{
		gGrid_C *pGrid = vTerminal[i];
		int nX, nY, nZ;
		pGrid->getPosition(nX, nY, nZ);
		if (nX <= nMinX)
			nMinX = nX;
	}

	int nMinY = m_nTY;
	for (int i = 0; i < vTerminal.size(); i++)
	{
		gGrid_C *pGrid = vTerminal[i];
		int nX, nY, nZ;
		pGrid->getPosition(nX, nY, nZ);
		if (nX == nMinX && nY <= nMinY)
		{
			nMinY = nY;
			pFirst = vTerminal[i];
		}
	}

	//
	//pFirst = vTerminal[0];

	//chech the terminal is available to route the net
	int nX, nY, nZ;
	pFirst->getPosition(nX, nY, nZ);
	rGrid_C *pCurRGrid = getRGrid(nX, nY, nZ);
	gGrid_C *pGGrid = pFirst;
	if (pGGrid->getRemand() <= 0)
	{
		pCurRGrid->isTarget = false;
		// added at 0705 22:00
		for (int i = 0; i < vTerminal.size(); i++)
		{
			vTerminal[i]->getPosition(nX, nY, nZ);
			pCurRGrid = getRGrid(nX, nY, nZ);
			pCurRGrid->isTarget = false;
		}
		// end added at 0705 22:00
		vResult.clear();
		return vResult;
	}

	vHistory.push_back(pCurRGrid);
	pCurRGrid->isPath = true;
	for (int i = 0; i < vTerminal.size(); i++)
	{
		if (vTerminal[i] == pGGrid)
		{
			vTerminal.erase(vTerminal.begin() + i);
			break;
		}
	}
	int nNumTermanals = vTerminal.size();
	//nNumTermanals--; // because the first grid is a termianl
	vector<layer_C *> vLayer = m_pDesign->getLayer();
	//cout<<"Start routing"<<endl;
	vector<rGrid_C *> vHaveAddTarget;
	int nHistoryIndex = 0;
	while (nNumTermanals > 0)
	{
		//cout<<nNumTermanals<<endl;
		//cout<<pCurRGrid->m_nX<<" "<<pCurRGrid->m_nY<<" "<<pCurRGrid->m_nZ<<endl;
		// A* propogation
		if (vHistory.size() == 0)
		{
			vResult.clear();
			break;
		}
		//cout<<vHistory.size()<<endl;

		vector<rGrid_C *> vToPropogate;
		int nMinCost = 0;
		/*
		bool bPick = false;
		
		for( int i=0; i<vHistory.size(); i++ )
		{
			if( !vHistory[i]->isRouted )	
			{
				nMinCost = vHistory[i]->m_nCost;
				bPick = true;
				break;
			}
		}
		if( !bPick )
		{
			vResult.clear();
			//cout<<"Can't find result"<<endl;
			break;
		}
		*/
		if (nHistoryIndex == vHistory.size())
			break;
		else
		{
			nMinCost = vHistory[nHistoryIndex]->m_nCost;
			vToPropogate.push_back(vHistory[nHistoryIndex]);
			//nHistoryIndex++;
		}
		/*
		cout<<nMinCost<<" "<<vHistory.size()<<endl;
		for( int i=0; i<vHistory.size(); i++ )
		{
			if( !vHistory[i]->isRouted )	
			{
				cout << vHistory[i]->m_nCost <<" ";
			}
		}
		//cout<<endl;
		*/
		/*
		for( int i=0; i<vHistory.size(); i++ )
		{
			if( !vHistory[i]->isRouted )	
			{
				if( vHistory[i]->m_nCost == nMinCost )
				{
					vToPropogate.push_back( vHistory[i] );
					break;
				}
				//else
				//	break;
			}
		}
		*/
		/*	
		//----------------------------------------------------------------------------choubegin TODO:
		vector<rGrid_C *> samecost;
		int vsum = INT_MAX;
		rGrid_C *temprG;
		gGrid_C *tempgG;
		for (int i = nHistoryIndex; i < vHistory.size(); i++)
		{
			if (!vHistory[i]->isRouted)
			{
				if (vHistory[i]->m_nCost == nMinCost)
				{
					samecost.push_back(vHistory[i]);
				}
				else
					break;
			}
		}
		//cout << "iter--------" << endl;
		for (int x = 0; x < samecost.size(); x++)
		{
			int az = samecost[x]->m_nZ;
			layer_C *pLayer = vLayer[az - m_nOffsetZ];
			//if (pNet->getName() == "N2")
			//	cout << "same : " << samecost[x]->m_nX << " " << samecost[x]->m_nY << " " << samecost[x]->m_nZ << endl;
			tempgG = m_pDesign->getGrid(samecost[x]->m_nX, samecost[x]->m_nY, az);
			if (vsum > nEularDistance_sum(vTerminal, tempgG, pLayer))
			{
				vsum = nEularDistance_sum(vTerminal, tempgG, pLayer);
				temprG = samecost[x];
			}
		}
		for( int i=nHistoryIndex; i<vHistory.size(); i++ )
		{
			if( temprG == vHistory[i] )
			{
				vHistory[i] = vHistory[ nHistoryIndex ];
				vHistory[ nHistoryIndex ] = temprG;
				break;
			}
		}
		nHistoryIndex++;
		vToPropogate.clear();
		vToPropogate.push_back(temprG);
		samecost.clear();
		//------------------------------------------------------------------------------chouend 
 		*/
		nHistoryIndex++;
		/*	
		if (pNet->getName() == "N1482")
		{
			cout<<endl;
			cout << "Routed: ";
			for (int i = 0; i < vHistory.size(); i++)
			{
				if( vHistory[i]->isRouted )
				cout << "(" << vHistory[i]->m_nX << " " << vHistory[i]->m_nY << " " << vHistory[i]->m_nZ << " " << vHistory[i]->m_nCost << " )";
			}
			cout << endl;
			cout<< "His: ";
			for (int i = 0; i < vHistory.size(); i++)
			{
				if( !vHistory[i]->isRouted )
				cout << "(" << vHistory[i]->m_nX << " " << vHistory[i]->m_nY << " " << vHistory[i]->m_nZ << " " << vHistory[i]->m_nG <<" " << vHistory[i]->m_nH <<" " << vHistory[i]->m_nCost << " )";
			}
			cout<<endl;
		}
		*/
		vector<rGrid_C *> vAddGrid;

		for (int i = 0; i < vToPropogate.size(); i++)
		{
			pCurRGrid = vToPropogate[i];
			pCurRGrid->isRouted = true;
			int nCurDistance = pCurRGrid->m_nG;
			int nNewDistance = nCurDistance + 1;
			if (nNewDistance > nConstraint)
				continue;

			int nX, nY, nZ;
			nX = pCurRGrid->m_nX;
			nY = pCurRGrid->m_nY;
			nZ = pCurRGrid->m_nZ;
			//cout<<"P: "<<nX<<" "<<nY<<" "<<nZ<<endl;
			pGGrid = getGrid(m_pDesign, nX, nY, nZ);
			char cDir = vLayer[nZ - m_nOffsetZ]->getDir();
			//cout<<cDir<<endl;
			int nDX, nDY, nDZ;
			if (cDir == 'H')
			{
				nDX = 0;
				nDY = 1;
				nDZ = 0;
			}
			else if (cDir == 'V')
			{
				nDX = 1;
				nDY = 0;
				nDZ = 0;
			}
			else
				cout << "Unknown routing direction" << endl;
			// go in the same layer;
			gGrid_C *pNextGrid = graphTravel(m_pDesign, pGGrid, nDX, nDY, nDZ);
			if (pNextGrid != NULL && pNextGrid->getRemand() > 0)
			{
				rGrid_C *pNextRGrid = getRGrid(nX + nDX, nY + nDY, nZ + nDZ);
				if (!pNextRGrid->isRouted)
				{
					//pNextRGrid->isRouted = true;
					layer_C *pLayer = vLayer[nZ + nDZ - m_nOffsetZ];
					pNextRGrid->m_nG = nNewDistance;
					int nSumH = 0;
					//for( int t=0; t<vTerminal.size(); t++ )
					//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
					nSumH = nEularDistance(vTerminal, pNextGrid, pLayer);
					pNextRGrid->m_nH = nSumH;
					pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
					pNextRGrid->m_pFrom = pCurRGrid;
					bool bInsert = false;
					//for( int j=0; j<vHistory.size(); j++ )
					for (int j = nHistoryIndex; j < vHistory.size(); j++)
					{
						if (pNextRGrid->m_nCost <= vHistory[j]->m_nCost)
						{
							vHistory.insert(vHistory.begin() + j, pNextRGrid);
							bInsert = true;
							break;
						}
					}
					if (!bInsert)
					{
						vHistory.push_back(pNextRGrid);
					}
					vAddGrid.push_back(pNextRGrid);
				}
			}
			//cout<<"test"<<endl;
			pNextGrid = graphTravel(m_pDesign, pGGrid, -nDX, -nDY, -nDZ);
			if (pNextGrid != NULL && pNextGrid->getRemand() > 0)
			{
				rGrid_C *pNextRGrid = getRGrid(nX - nDX, nY - nDY, nZ - nDZ);
				if (!pNextRGrid->isRouted)
				{
					//pNextRGrid->isRouted = true;
					layer_C *pLayer = vLayer[nZ - nDZ - m_nOffsetZ];
					pNextRGrid->m_nG = nNewDistance;
					int nSumH = 0;
					//for( int t=0; t<vTerminal.size(); t++ )
					//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
					nSumH = nEularDistance(vTerminal, pNextGrid, pLayer);
					pNextRGrid->m_nH = nSumH;
					pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
					pNextRGrid->m_pFrom = pCurRGrid;
					bool bInsert = false;
					//for( int j=0; j<vHistory.size(); j++ )
					for (int j = nHistoryIndex; j < vHistory.size(); j++)
					{
						if (pNextRGrid->m_nCost <= vHistory[j]->m_nCost)
						{
							vHistory.insert(vHistory.begin() + j, pNextRGrid);
							bInsert = true;
							break;
						}
					}
					if (!bInsert)
					{
						vHistory.push_back(pNextRGrid);
					}
					vAddGrid.push_back(pNextRGrid);
				}
			}

			// go to different layer
			//cout<<"different layer"<<endl;
			pNextGrid = graphTravel(m_pDesign, pGGrid, 0, 0, 1);
			if (pNextGrid != NULL && pNextGrid->getRemand() > 0)
			{
				rGrid_C *pNextRGrid = getRGrid(nX, nY, nZ + 1);
				if (!pNextRGrid->isRouted)
				{
					//pNextRGrid->isRouted = true;
					layer_C *pLayer = vLayer[nZ + 1 - m_nOffsetZ];
					pNextRGrid->m_nG = nNewDistance;
					int nSumH = 0;
					//for( int t=0; t<vTerminal.size(); t++ )
					//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
					nSumH = nEularDistance(vTerminal, pNextGrid, pLayer);
					pNextRGrid->m_nH = nSumH;
					pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
					pNextRGrid->m_pFrom = pCurRGrid;
					bool bInsert = false;
					//for( int j=0; j<vHistory.size(); j++ )
					for (int j = nHistoryIndex; j < vHistory.size(); j++)
					{
						if (pNextRGrid->m_nCost <= vHistory[j]->m_nCost)
						{
							vHistory.insert(vHistory.begin() + j, pNextRGrid);
							bInsert = true;
							break;
						}
					}
					if (!bInsert)
					{
						vHistory.push_back(pNextRGrid);
					}
					vAddGrid.push_back(pNextRGrid);
				}
			}

			if (nZ - 1 >= nLConstraint)
			{
				pNextGrid = graphTravel(m_pDesign, pGGrid, 0, 0, -1);
				if (pNextGrid != NULL && pNextGrid->getRemand() > 0)
				{
					rGrid_C *pNextRGrid = getRGrid(nX, nY, nZ - 1);
					if (!pNextRGrid->isRouted)
					{
						//pNextRGrid->isRouted = true;
						layer_C *pLayer = vLayer[nZ - 1 - m_nOffsetZ];
						pNextRGrid->m_nG = nNewDistance;
						int nSumH = 0;
						//for( int t=0; t<vTerminal.size(); t++ )
						//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
						nSumH = nEularDistance(vTerminal, pNextGrid, pLayer);
						pNextRGrid->m_nH = nSumH;
						pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
						pNextRGrid->m_pFrom = pCurRGrid;
						bool bInsert = false;
						//for( int j=0; j<vHistory.size(); j++ )
						for (int j = nHistoryIndex; j < vHistory.size(); j++)
						{
							if (pNextRGrid->m_nCost <= vHistory[j]->m_nCost)
							{
								vHistory.insert(vHistory.begin() + j, pNextRGrid);
								bInsert = true;
								break;
							}
						}
						if (!bInsert)
						{
							vHistory.push_back(pNextRGrid);
						}
						vAddGrid.push_back(pNextRGrid);
					}
				}
			}
		}
		//cout<<"Check target"<<endl;
		// if there is a target in addgrid;
		//vector< rGrid_C* > vHaveAddTarget;
		bool bFindTarget = false;
		for (int i = 0; i < vAddGrid.size(); i++)
		{
			if (vAddGrid[i]->isTarget)
			{
				//if( pNet->getName() == "N1482")
				//	cout<<"Target is: "<<vAddGrid[i]->m_nX<<" "<<vAddGrid[i]->m_nY<<" "<<vAddGrid[i]->m_nZ<<" "<<endl;
				bFindTarget = true;
				bool bHaveSearch = false;
				for (int j = 0; j < vHaveAddTarget.size(); j++)
				{
					if (vHaveAddTarget[j] == vAddGrid[i])
					{
						bHaveSearch = true;
						break;
					}
				}
				if (bHaveSearch)
					continue;

				vHaveAddTarget.push_back(vAddGrid[i]);
				//cout<<"Back trace"<<endl;
				// find the path
				vector<gGrid_C *> vReversePath;

				int nPath = vAddGrid[i]->m_nG;
				//if( pNet->getName() == "N1482")
				//	cout<<"Path cost: "<<nPath<<endl;
				rGrid_C *pRPath = vAddGrid[i];
				pRPath->isPath = true;
				gGrid_C *pPathGrid = getGrid(m_pDesign, pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ);
				rGrid_C *pPreGrid = pRPath->m_pFrom;
				/*
				//  added at 07/20 20:30
				layer_C* pTmpLayer = m_pDesign->getLayer()[ pRPath->m_nZ - m_nOffsetZ ];
				int nPDX;
				int nPDY;
				if( pTmpLayer->getDir() == 'H' )
				{
					nPDX = 0; nPDY = 1;
				}
				else
				{
					nPDX = 1; nPDY = 0;
				}
				
				rGrid_C* pRGrid1 = getRGrid( pRPath->m_nX + nPDX, pRPath->m_nY + nPDY, pRPath->m_nZ );
				rGrid_C* pRGrid2 = getRGrid( pRPath->m_nX - nPDX, pRPath->m_nY - nPDY, pRPath->m_nZ );	
				rGrid_C* pRGrid3 = getRGrid( pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ + 1 );
				rGrid_C* pRGrid4 = getRGrid( pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ - 1 );
				vector< rGrid_C* > vTmpRGrid;
				//for( int rg=0; rg<4; rg++ )
				//{
					if( pRGrid1 != NULL && pRGrid1->isRouted )
						vTmpRGrid.push_back( pRGrid1 );
					if( pRGrid2 != NULL && pRGrid2->isRouted )
						vTmpRGrid.push_back( pRGrid2 );
					if( pRGrid3 != NULL && pRGrid3->isRouted )
						vTmpRGrid.push_back( pRGrid3 );
					if( pRGrid4 != NULL && pRGrid4->isRouted )
						vTmpRGrid.push_back( pRGrid4 );
				//}
				int nTmpPath = nPath;
				rGrid_C* pMinRGrid = vAddGrid[i]->m_pFrom;
				for( int rg=0; rg<vTmpRGrid.size(); rg++ )
				{
					if( nTmpPath > vTmpRGrid[ rg ]->m_nG )
					{
						nTmpPath = vTmpRGrid[ rg ]->m_nG;
						pMinRGrid = vTmpRGrid[ rg ];
					}	
				}
				vTmpRGrid.clear();
				vAddGrid[i]->m_pFrom = pMinRGrid;
				int nAddPath = 1;
				rGrid_C* pPreGrid = vAddGrid[i]->m_pFrom;
				//end added at 07/04 20:30
				*/
				//gGrid_C* pPathGrid = getGrid( m_pDesign, pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ );
				//vResult.push_back( pPathGrid );
				vReversePath.push_back(pPathGrid);
				while (!pPreGrid->isPath)
				{
					pRPath = pPreGrid;
					pRPath->isPath = true;
					pPathGrid = getGrid(m_pDesign, pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ);
					//vResult.push_back( pPathGrid );
					vReversePath.push_back(pPathGrid);
					/*
					// added at 07/04 20:30	
					pTmpLayer = m_pDesign->getLayer()[ pRPath->m_nZ - m_nOffsetZ ];
					
					if( pTmpLayer->getDir() == 'H' )
					{
						nPDX = 0; nPDY = 1;
					}
					else
					{
						nPDX = 1; nPDY = 0;
					}
					
					pRGrid1 = getRGrid( pRPath->m_nX + nPDX, pRPath->m_nY + nPDY, pRPath->m_nZ );
					pRGrid2 = getRGrid( pRPath->m_nX - nPDX, pRPath->m_nY - nPDY, pRPath->m_nZ );	
					pRGrid3 = getRGrid( pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ + 1 );
					pRGrid4 = getRGrid( pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ - 1 );
					//vector< rGrid_C* > vTmpRGrid;
					//for( int rg=0; rg<4; rg++ )
					//{
						if( pRGrid1 != NULL && pRGrid1->isRouted )
							vTmpRGrid.push_back( pRGrid1 );
						if( pRGrid2 != NULL && pRGrid2->isRouted )
							vTmpRGrid.push_back( pRGrid2 );
						if( pRGrid3 != NULL && pRGrid3->isRouted )
							vTmpRGrid.push_back( pRGrid3 );
						if( pRGrid4 != NULL && pRGrid4->isRouted )
							vTmpRGrid.push_back( pRGrid4 );
					//}
					int nTmpPath = nPath;
					rGrid_C* pMinRGrid = pRPath->m_pFrom;
					for( int rg=0; rg<vTmpRGrid.size(); rg++ )
					{
						if( nTmpPath > vTmpRGrid[ rg ]->m_nG )
						{
							nTmpPath = vTmpRGrid[ rg ]->m_nG;
							pMinRGrid = vTmpRGrid[ rg ];
						}	
					}
					vTmpRGrid.clear();
					pRPath->m_pFrom = pMinRGrid;
					nAddPath++;
					// end added at 07/04 20:30
					*/
					pPreGrid = pRPath->m_pFrom;
				}
				/*
				// added at 07/04 20:30
				nPath = pPreGrid->m_nG + nAddPath;
				// end added at 07/04 20:30
				*/

				pPathGrid = getGrid(m_pDesign, pPreGrid->m_nX, pPreGrid->m_nY, pPreGrid->m_nZ);
				//vResult.push_back( pPathGrid );
				vReversePath.push_back(pPathGrid);
				for (int g = vReversePath.size() - 1; g >= 0; g--)
				{
					vResult.push_back(vReversePath[g]);
				}
				/*
				if( pNet->getName() == "N1482" )
				{
					for( int p=0; p<vReversePath.size(); p++ )
					{
						int pX, pY, pZ;
						vReversePath[p]->getPosition( pX, pY, pZ );
						cout<<"("<<pX<<","<<pY<<","<<pZ<<") ";
					}
					cout<<endl;
				}
				*/
				// find the terminal and remove
				for (int t = 0; t < vTerminal.size(); t++)
				{
					int nTX, nTY, nTZ;
					vTerminal[t]->getPosition(nTX, nTY, nTZ);
					if (vAddGrid[i]->m_nX == nTX && vAddGrid[i]->m_nY == nTY && vAddGrid[i]->m_nZ == nTZ)
					{
						vTerminal.erase(vTerminal.begin() + t);
						nNumTermanals--;
						break;
					}
				}
				// update the nG in all the searched node ( without path )
				for (int h = 0; h < vHistory.size(); h++)
				{
					rGrid_C *pTmpGrid = vHistory[h];
					if (!pTmpGrid->isPath)
						calDistanceFromPath(pTmpGrid, nPath);
					else
						pTmpGrid->m_nG = nPath;
				}

				// update the nH in all the node in queue
				for (int h = nHistoryIndex; h < vHistory.size(); h++)
				{
					rGrid_C *pTmpGrid = vHistory[h];
					if (!pTmpGrid->isRouted)
					{
						int nH = 0;
						gGrid_C *pTmpGGrid = getGrid(m_pDesign, pTmpGrid->m_nX, pTmpGrid->m_nY, pTmpGrid->m_nZ);
						int nX, nY, nZ;
						pTmpGGrid->getPosition(nX, nY, nZ);
						layer_C *pLayer = vLayer[nZ - m_nOffsetZ];
						//for( int t=0; t<vTerminal.size(); t++ )
						//{
						//	nH = nH + nEularDistance( vTerminal[t], pTmpGGrid, pLayer );
						//}
						nH = nEularDistance(vTerminal, pTmpGGrid, pLayer);
						pTmpGrid->m_nH = nH;
						pTmpGrid->m_nCost = nH + pTmpGrid->m_nG;
					}
				}
				//cout<<endl;
			}
		}

		if (bFindTarget)
		{
			for (int i = nHistoryIndex; i < vHistory.size(); i++)
			{
				for (int j = i - 1; j >= 0; j--)
				{
					if (vHistory[j]->m_nCost > vHistory[j + 1]->m_nCost)
					{
						rGrid_C *pFGrid = vHistory[j];
						rGrid_C *pBGrid = vHistory[j + 1];
						vHistory[j] = pBGrid;
						vHistory[j + 1] = pFGrid;
					}
					else
						break;
				}
			}
		}
	}

	for (int i = 0; i < vHistory.size(); i++)
	{
		rGrid_C *pRGrid = vHistory[i];
		pRGrid->m_nH = 0;
		pRGrid->m_nG = 0;
		pRGrid->m_nCost = 0;
		pRGrid->m_nBend = 0;
		pRGrid->m_pFrom = NULL;
		pRGrid->isRouted = false;
		pRGrid->isPath = false;
		pRGrid->isTarget = false;
	}

	/*	
	for( int i=0; i<vResult.size(); i++ )
	{
		int nX, nY, nZ;
		gGrid_C* pTmpGrid = vResult[i];
		pTmpGrid->getPosition( nX, nY, nZ );
		rGrid_C* pRGrid = getRGrid( nX, nY, nZ );
		pRGrid->m_nH = 0;
		pRGrid->m_nG = 0;
		pRGrid->m_nCost = 0;
		pRGrid->m_nBend = 0;
		pRGrid->m_pFrom = NULL;
		pRGrid->isRouted = false;
		pRGrid->isPath = false;
		pRGrid->isTarget = false;
	}
	*/
	for (int i = 0; i < vTerminal.size(); i++)
	{
		int nX, nY, nZ;
		gGrid_C *pTmpGrid = vTerminal[i];
		pTmpGrid->getPosition(nX, nY, nZ);
		rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
		pRGrid->m_nH = 0;
		pRGrid->m_nG = 0;
		pRGrid->m_nCost = 0;
		pRGrid->m_nBend = 0;
		pRGrid->m_pFrom = NULL;
		pRGrid->isRouted = false;
		pRGrid->isPath = false;
		pRGrid->isTarget = false;
	}
	if (nNumTermanals != 0)
		vResult.clear();

	return vResult;
}

vector<gGrid_C *> router_C::routeNet(net_C *pNet)
{
	setRoutingConstraint(pNet);
	int nLConstraint = m_nConstraintLayerId;
	//cout<<nLConstraint<<endl;

	vector<gGrid_C *> vTerminal;
	vector<pin_C *> vPin = pNet->getPin();
	//vector< wire_C* > vWire;
	vector<gGrid_C *> vResult;
	for (int i = 0; i < vPin.size(); i++)
	{
		pin_C *pPin = vPin[i];
		int nX, nY, nZ;
		instance_C *pInst = (instance_C *)pPin->getCell();
		nX = pInst->getPlacedX();
		nY = pInst->getPlacedY();
		// project the z to the constraint layer
		nZ = pPin->getLayerId();
		if (nZ < nLConstraint)
		{
			for (int l = nZ; l <= nLConstraint; l++)
			{
				gGrid_C *pGrid = getGrid(m_pDesign, nX, nY, l);
				vResult.push_back(pGrid);
			}
			nZ = nLConstraint;
		}

		gGrid_C *pTerminal = getGrid(m_pDesign, nX, nY, nZ);
		rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
		pRGrid->isTarget = true;
		vTerminal.push_back(pTerminal);
	}

	// check same position of terminal
	//cout<<"Check terminal"<<endl;
	for (int i = 0; i < vTerminal.size(); i++)
	{
		for (int j = i + 1; j < vTerminal.size(); j++)
		{
			if (vTerminal[i] == vTerminal[j])
			{
				vTerminal.erase(vTerminal.begin() + j);
				j--;
			}
		}
	}
	//cout<<"Num of terminals: "<<vTerminal.size()<<endl;
	if (vTerminal.size() == 1)
	{
		if (vTerminal[0]->getRemand() <= 0)
		{

			int nX, nY, nZ;
			vTerminal[0]->getPosition(nX, nY, nZ);
			rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
			pRGrid->isTarget = false;
			return vResult;
		}
		int nX, nY, nZ;
		vTerminal[0]->getPosition(nX, nY, nZ);
		rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
		pRGrid->isTarget = false;

		vResult.push_back(vTerminal[0]);
		return vResult;
	}
	else
	{
		bool bFindOverflow = false;
		for (int i = 0; i < vTerminal.size(); i++)
		{
			if (vTerminal[i]->getRemand() <= 0)
			{
				bFindOverflow = true;
				break;
			}
		}

		if (bFindOverflow)
		{
			for (int i = 0; i < vTerminal.size(); i++)
			{
				int nX, nY, nZ;
				vTerminal[i]->getPosition(nX, nY, nZ);
				rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
				pRGrid->isTarget = false;
			}
			if (vResult.size() != 0)
				vResult.clear();

			return vResult;
		}
	}

	vector<rGrid_C *> vRoutedGrid;
	//vector< rGrid_C* > vPriorityQueue;
	vector<rGrid_C *> vHistory;
	vector<rGrid_C *> vNotInPath;
	// find first terminal
	gGrid_C *pFirst = NULL;

	int nMinX = m_nTX;
	for (int i = 0; i < vTerminal.size(); i++)
	{
		gGrid_C *pGrid = vTerminal[i];
		int nX, nY, nZ;
		pGrid->getPosition(nX, nY, nZ);
		if (nX <= nMinX)
			nMinX = nX;
	}

	int nMinY = m_nTY;
	for (int i = 0; i < vTerminal.size(); i++)
	{
		gGrid_C *pGrid = vTerminal[i];
		int nX, nY, nZ;
		pGrid->getPosition(nX, nY, nZ);
		if (nX == nMinX && nY <= nMinY)
		{
			nMinY = nY;
			pFirst = vTerminal[i];
		}
	}

	//
	//pFirst = vTerminal[0];

	//cout<<"Here"<<endl;
	int nX, nY, nZ;
	pFirst->getPosition(nX, nY, nZ);
	rGrid_C *pCurRGrid = getRGrid(nX, nY, nZ);
	gGrid_C *pGGrid = pFirst;
	if (pGGrid->getRemand() <= 0)
	{
		pCurRGrid->isTarget = false;
		// added at 0705 22:00
		for (int i = 0; i < vTerminal.size(); i++)
		{
			vTerminal[i]->getPosition(nX, nY, nZ);
			pCurRGrid = getRGrid(nX, nY, nZ);
			pCurRGrid->isTarget = false;
		}
		// end added at 0705 22:00
		vResult.clear();
		return vResult;
	}

	vHistory.push_back(pCurRGrid);
	pCurRGrid->isPath = true;
	for (int i = 0; i < vTerminal.size(); i++)
	{
		if (vTerminal[i] == pGGrid)
		{
			vTerminal.erase(vTerminal.begin() + i);
			break;
		}
	}
	int nNumTermanals = vTerminal.size();
	//nNumTermanals--; // because the first grid is a termianl
	vector<layer_C *> vLayer = m_pDesign->getLayer();
	//cout<<"Start routing"<<endl;
	vector<rGrid_C *> vHaveAddTarget;
	while (nNumTermanals > 0)
	{
		//cout<<nNumTermanals<<endl;
		//cout<<pCurRGrid->m_nX<<" "<<pCurRGrid->m_nY<<" "<<pCurRGrid->m_nZ<<endl;
		// A* propogation
		if (vHistory.size() == 0)
		{
			vResult.clear();
			break;
		}
		//cout<<vHistory.size()<<endl;

		vector<rGrid_C *> vToPropogate;
		int nMinCost = 0;
		bool bPick = false;
		/*	
		for( int i=0; i<vHistory.size(); i++ )
		{
			if( vHistory[i]->isRouted )
			{
				vHistory.erase( vHistory.begin() + i );
				i--;
			}
		}
		*/
		/*
		for( int i=0; i<vHistory.size(); i++ )
		{
			if( !vHistory[i]->isRouted )
			{
				cout<<vHistory[i]->m_nCost<<" ";
			}
		}
		cout<<endl;
		*/
		for (int i = 0; i < vHistory.size(); i++)
		{
			if (!vHistory[i]->isRouted)
			{
				nMinCost = vHistory[i]->m_nCost;
				bPick = true;
				break;
			}
		}
		if (!bPick)
		{
			vResult.clear();
			//cout<<"Can't find result"<<endl;
			break;
		}
		/*
		cout<<nMinCost<<" "<<vHistory.size()<<endl;
		for( int i=0; i<vHistory.size(); i++ )
		{
			if( !vHistory[i]->isRouted )	
			{
				cout << vHistory[i]->m_nCost <<" ";
			}
		}
		//cout<<endl;
		*/
		for (int i = 0; i < vHistory.size(); i++)
		{
			if (!vHistory[i]->isRouted)
			{
				if (vHistory[i]->m_nCost == nMinCost)
				{
					vToPropogate.push_back(vHistory[i]);
				}
				//else
				//	break;
			}
		}

		vector<rGrid_C *> vAddGrid;

		for (int i = 0; i < vToPropogate.size(); i++)
		{
			pCurRGrid = vToPropogate[i];
			pCurRGrid->isRouted = true;
			int nCurDistance = pCurRGrid->m_nG;
			int nNewDistance = nCurDistance + 1;
			int nX, nY, nZ;
			nX = pCurRGrid->m_nX;
			nY = pCurRGrid->m_nY;
			nZ = pCurRGrid->m_nZ;
			//cout<<"P: "<<nX<<" "<<nY<<" "<<nZ<<endl;
			pGGrid = getGrid(m_pDesign, nX, nY, nZ);
			char cDir = vLayer[nZ - m_nOffsetZ]->getDir();
			//cout<<cDir<<endl;
			int nDX, nDY, nDZ;
			if (cDir == 'H')
			{
				nDX = 0;
				nDY = 1;
				nDZ = 0;
			}
			else if (cDir == 'V')
			{
				nDX = 1;
				nDY = 0;
				nDZ = 0;
			}
			else
				cout << "Unknown routing direction" << endl;
			// go in the same layer;
			gGrid_C *pNextGrid = graphTravel(m_pDesign, pGGrid, nDX, nDY, nDZ);
			if (pNextGrid != NULL && pNextGrid->getRemand() > 0)
			{
				rGrid_C *pNextRGrid = getRGrid(nX + nDX, nY + nDY, nZ + nDZ);
				if (!pNextRGrid->isRouted)
				{
					//pNextRGrid->isRouted = true;
					layer_C *pLayer = vLayer[nZ + nDZ - m_nOffsetZ];
					pNextRGrid->m_nG = nNewDistance;
					int nSumH = 0;
					//for( int t=0; t<vTerminal.size(); t++ )
					//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
					nSumH = nEularDistance(vTerminal, pNextGrid, pLayer);
					pNextRGrid->m_nH = nSumH;
					pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
					pNextRGrid->m_pFrom = pCurRGrid;
					bool bInsert = false;
					for (int j = 0; j < vHistory.size(); j++)
					{
						if (pNextRGrid->m_nCost <= vHistory[j]->m_nCost)
						{
							vHistory.insert(vHistory.begin() + j, pNextRGrid);
							bInsert = true;
							break;
						}
					}
					if (!bInsert)
					{
						vHistory.push_back(pNextRGrid);
					}
					vAddGrid.push_back(pNextRGrid);
				}
			}
			//cout<<"test"<<endl;
			pNextGrid = graphTravel(m_pDesign, pGGrid, -nDX, -nDY, -nDZ);
			if (pNextGrid != NULL && pNextGrid->getRemand() > 0)
			{
				rGrid_C *pNextRGrid = getRGrid(nX - nDX, nY - nDY, nZ - nDZ);
				if (!pNextRGrid->isRouted)
				{
					//pNextRGrid->isRouted = true;
					layer_C *pLayer = vLayer[nZ - nDZ - m_nOffsetZ];
					pNextRGrid->m_nG = nNewDistance;
					int nSumH = 0;
					//for( int t=0; t<vTerminal.size(); t++ )
					//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
					nSumH = nEularDistance(vTerminal, pNextGrid, pLayer);
					pNextRGrid->m_nH = nSumH;
					pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
					pNextRGrid->m_pFrom = pCurRGrid;
					bool bInsert = false;
					for (int j = 0; j < vHistory.size(); j++)
					{
						if (pNextRGrid->m_nCost <= vHistory[j]->m_nCost)
						{
							vHistory.insert(vHistory.begin() + j, pNextRGrid);
							bInsert = true;
							break;
						}
					}
					if (!bInsert)
					{
						vHistory.push_back(pNextRGrid);
					}
					vAddGrid.push_back(pNextRGrid);
				}
			}

			// go to different layer
			//cout<<"different layer"<<endl;
			pNextGrid = graphTravel(m_pDesign, pGGrid, 0, 0, 1);
			if (pNextGrid != NULL && pNextGrid->getRemand() > 0)
			{
				rGrid_C *pNextRGrid = getRGrid(nX, nY, nZ + 1);
				if (!pNextRGrid->isRouted)
				{
					//pNextRGrid->isRouted = true;
					layer_C *pLayer = vLayer[nZ + 1 - m_nOffsetZ];
					pNextRGrid->m_nG = nNewDistance;
					int nSumH = 0;
					//for( int t=0; t<vTerminal.size(); t++ )
					//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
					nSumH = nEularDistance(vTerminal, pNextGrid, pLayer);
					pNextRGrid->m_nH = nSumH;
					pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
					pNextRGrid->m_pFrom = pCurRGrid;
					bool bInsert = false;
					for (int j = 0; j < vHistory.size(); j++)
					{
						if (pNextRGrid->m_nCost <= vHistory[j]->m_nCost)
						{
							vHistory.insert(vHistory.begin() + j, pNextRGrid);
							bInsert = true;
							break;
						}
					}
					if (!bInsert)
					{
						vHistory.push_back(pNextRGrid);
					}
					vAddGrid.push_back(pNextRGrid);
				}
			}

			if (nZ - 1 >= nLConstraint)
			{
				pNextGrid = graphTravel(m_pDesign, pGGrid, 0, 0, -1);
				if (pNextGrid != NULL && pNextGrid->getRemand() > 0)
				{
					rGrid_C *pNextRGrid = getRGrid(nX, nY, nZ - 1);
					if (!pNextRGrid->isRouted)
					{
						//pNextRGrid->isRouted = true;
						layer_C *pLayer = vLayer[nZ - 1 - m_nOffsetZ];
						pNextRGrid->m_nG = nNewDistance;
						int nSumH = 0;
						//for( int t=0; t<vTerminal.size(); t++ )
						//	nSumH = nSumH + nEularDistance( vTerminal[t], pNextGrid, pLayer );
						nSumH = nEularDistance(vTerminal, pNextGrid, pLayer);
						pNextRGrid->m_nH = nSumH;
						pNextRGrid->m_nCost = pNextRGrid->m_nG + nSumH;
						pNextRGrid->m_pFrom = pCurRGrid;
						bool bInsert = false;
						for (int j = 0; j < vHistory.size(); j++)
						{
							if (pNextRGrid->m_nCost <= vHistory[j]->m_nCost)
							{
								vHistory.insert(vHistory.begin() + j, pNextRGrid);
								bInsert = true;
								break;
							}
						}
						if (!bInsert)
						{
							vHistory.push_back(pNextRGrid);
						}
						vAddGrid.push_back(pNextRGrid);
					}
				}
			}
		}
		//cout<<"Check target"<<endl;
		// if there is a target in addgrid;
		//vector< rGrid_C* > vHaveAddTarget;
		for (int i = 0; i < vAddGrid.size(); i++)
		{
			if (vAddGrid[i]->isTarget)
			{
				bool bHaveSearch = false;
				for (int j = 0; j < vHaveAddTarget.size(); j++)
				{
					if (vHaveAddTarget[j] == vAddGrid[i])
					{
						bHaveSearch = true;
						break;
					}
				}
				if (bHaveSearch)
					continue;

				vHaveAddTarget.push_back(vAddGrid[i]);
				//cout<<"Back trace"<<endl;
				// find the path
				vector<gGrid_C *> vReversePath;

				int nPath = vAddGrid[i]->m_nG;
				rGrid_C *pRPath = vAddGrid[i];
				pRPath->isPath = true;
				gGrid_C *pPathGrid = getGrid(m_pDesign, pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ);
				rGrid_C *pPreGrid = pRPath->m_pFrom;
				/*
				//  added at 07/20 20:30
				layer_C* pTmpLayer = m_pDesign->getLayer()[ pRPath->m_nZ - m_nOffsetZ ];
				int nPDX;
				int nPDY;
				if( pTmpLayer->getDir() == 'H' )
				{
					nPDX = 0; nPDY = 1;
				}
				else
				{
					nPDX = 1; nPDY = 0;
				}
				
				rGrid_C* pRGrid1 = getRGrid( pRPath->m_nX + nPDX, pRPath->m_nY + nPDY, pRPath->m_nZ );
				rGrid_C* pRGrid2 = getRGrid( pRPath->m_nX - nPDX, pRPath->m_nY - nPDY, pRPath->m_nZ );	
				rGrid_C* pRGrid3 = getRGrid( pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ + 1 );
				rGrid_C* pRGrid4 = getRGrid( pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ - 1 );
				vector< rGrid_C* > vTmpRGrid;
				//for( int rg=0; rg<4; rg++ )
				//{
					if( pRGrid1 != NULL && pRGrid1->isRouted )
						vTmpRGrid.push_back( pRGrid1 );
					if( pRGrid2 != NULL && pRGrid2->isRouted )
						vTmpRGrid.push_back( pRGrid2 );
					if( pRGrid3 != NULL && pRGrid3->isRouted )
						vTmpRGrid.push_back( pRGrid3 );
					if( pRGrid4 != NULL && pRGrid4->isRouted )
						vTmpRGrid.push_back( pRGrid4 );
				//}
				int nTmpPath = nPath;
				rGrid_C* pMinRGrid = vAddGrid[i]->m_pFrom;
				for( int rg=0; rg<vTmpRGrid.size(); rg++ )
				{
					if( nTmpPath > vTmpRGrid[ rg ]->m_nG )
					{
						nTmpPath = vTmpRGrid[ rg ]->m_nG;
						pMinRGrid = vTmpRGrid[ rg ];
					}	
				}
				vTmpRGrid.clear();
				vAddGrid[i]->m_pFrom = pMinRGrid;
				int nAddPath = 1;
				rGrid_C* pPreGrid = vAddGrid[i]->m_pFrom;
				//end added at 07/04 20:30
				*/
				//gGrid_C* pPathGrid = getGrid( m_pDesign, pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ );
				//vResult.push_back( pPathGrid );
				vReversePath.push_back(pPathGrid);
				while (!pPreGrid->isPath)
				{
					pRPath = pPreGrid;
					pRPath->isPath = true;
					pPathGrid = getGrid(m_pDesign, pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ);
					//vResult.push_back( pPathGrid );
					vReversePath.push_back(pPathGrid);
					/*
					// added at 07/04 20:30	
					pTmpLayer = m_pDesign->getLayer()[ pRPath->m_nZ - m_nOffsetZ ];
					
					if( pTmpLayer->getDir() == 'H' )
					{
						nPDX = 0; nPDY = 1;
					}
					else
					{
						nPDX = 1; nPDY = 0;
					}
					
					pRGrid1 = getRGrid( pRPath->m_nX + nPDX, pRPath->m_nY + nPDY, pRPath->m_nZ );
					pRGrid2 = getRGrid( pRPath->m_nX - nPDX, pRPath->m_nY - nPDY, pRPath->m_nZ );	
					pRGrid3 = getRGrid( pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ + 1 );
					pRGrid4 = getRGrid( pRPath->m_nX, pRPath->m_nY, pRPath->m_nZ - 1 );
					//vector< rGrid_C* > vTmpRGrid;
					//for( int rg=0; rg<4; rg++ )
					//{
						if( pRGrid1 != NULL && pRGrid1->isRouted )
							vTmpRGrid.push_back( pRGrid1 );
						if( pRGrid2 != NULL && pRGrid2->isRouted )
							vTmpRGrid.push_back( pRGrid2 );
						if( pRGrid3 != NULL && pRGrid3->isRouted )
							vTmpRGrid.push_back( pRGrid3 );
						if( pRGrid4 != NULL && pRGrid4->isRouted )
							vTmpRGrid.push_back( pRGrid4 );
					//}
					int nTmpPath = nPath;
					rGrid_C* pMinRGrid = pRPath->m_pFrom;
					for( int rg=0; rg<vTmpRGrid.size(); rg++ )
					{
						if( nTmpPath > vTmpRGrid[ rg ]->m_nG )
						{
							nTmpPath = vTmpRGrid[ rg ]->m_nG;
							pMinRGrid = vTmpRGrid[ rg ];
						}	
					}
					vTmpRGrid.clear();
					pRPath->m_pFrom = pMinRGrid;
					nAddPath++;
					// end added at 07/04 20:30
					*/
					pPreGrid = pRPath->m_pFrom;
				}
				/*
				// added at 07/04 20:30
				nPath = pPreGrid->m_nG + nAddPath;
				// end added at 07/04 20:30
				*/

				pPathGrid = getGrid(m_pDesign, pPreGrid->m_nX, pPreGrid->m_nY, pPreGrid->m_nZ);
				//vResult.push_back( pPathGrid );
				vReversePath.push_back(pPathGrid);
				for (int g = vReversePath.size() - 1; g >= 0; g--)
				{
					vResult.push_back(vReversePath[g]);
				}

				//for( int p=0; p<vReversePath.size(); p++ )
				//{
				//	int pX, pY, pZ;
				//	vReversePath[p]->getPosition( pX, pY, pZ );
				//	cout<<"("<<pX<<","<<pY<<","<<pZ<<") ";
				//}
				//cout<<endl;

				// find the terminal and remove
				for (int t = 0; t < vTerminal.size(); t++)
				{
					int nX, nY, nZ;
					vTerminal[t]->getPosition(nX, nY, nZ);
					if (vAddGrid[i]->m_nX == nX && vAddGrid[i]->m_nY == nY && vAddGrid[i]->m_nZ == nZ)
					{
						vTerminal.erase(vTerminal.begin() + t);
						nNumTermanals--;
						break;
					}
				}
				// update the nG in all the searched node ( without path )
				for (int h = 0; h < vHistory.size(); h++)
				{
					rGrid_C *pTmpGrid = vHistory[h];
					if (!pTmpGrid->isPath)
						calDistanceFromPath(pTmpGrid, nPath);
				}

				// update the nH in all the node in queue
				for (int h = 0; h < vHistory.size(); h++)
				{
					rGrid_C *pTmpGrid = vHistory[h];
					if (!pTmpGrid->isRouted)
					{
						int nH = 0;
						gGrid_C *pTmpGGrid = getGrid(m_pDesign, pTmpGrid->m_nX, pTmpGrid->m_nY, pTmpGrid->m_nZ);
						int nX, nY, nZ;
						pTmpGGrid->getPosition(nX, nY, nZ);
						layer_C *pLayer = vLayer[nZ - m_nOffsetZ];
						//for( int t=0; t<vTerminal.size(); t++ )
						//{
						//	nH = nH + nEularDistance( vTerminal[t], pTmpGGrid, pLayer );
						//}
						nH = nEularDistance(vTerminal, pTmpGGrid, pLayer);
						pTmpGrid->m_nH = nH;
						pTmpGrid->m_nCost = nH + pTmpGrid->m_nG;
					}
				}
				//cout<<endl;
			}
		}
	}

	for (int i = 0; i < vHistory.size(); i++)
	{
		rGrid_C *pRGrid = vHistory[i];
		pRGrid->m_nH = 0;
		pRGrid->m_nG = 0;
		pRGrid->m_nCost = 0;
		pRGrid->m_nBend = 0;
		pRGrid->m_pFrom = NULL;
		pRGrid->isRouted = false;
		pRGrid->isPath = false;
		pRGrid->isTarget = false;
	}

	for (int i = 0; i < vResult.size(); i++)
	{
		int nX, nY, nZ;
		gGrid_C *pTmpGrid = vResult[i];
		pTmpGrid->getPosition(nX, nY, nZ);
		rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
		pRGrid->m_nH = 0;
		pRGrid->m_nG = 0;
		pRGrid->m_nCost = 0;
		pRGrid->m_nBend = 0;
		pRGrid->m_pFrom = NULL;
		pRGrid->isRouted = false;
		pRGrid->isPath = false;
		pRGrid->isTarget = false;
	}

	for (int i = 0; i < vTerminal.size(); i++)
	{
		int nX, nY, nZ;
		gGrid_C *pTmpGrid = vTerminal[i];
		pTmpGrid->getPosition(nX, nY, nZ);
		rGrid_C *pRGrid = getRGrid(nX, nY, nZ);
		pRGrid->m_nH = 0;
		pRGrid->m_nG = 0;
		pRGrid->m_nCost = 0;
		pRGrid->m_nBend = 0;
		pRGrid->m_pFrom = NULL;
		pRGrid->isRouted = false;
		pRGrid->isPath = false;
		pRGrid->isTarget = false;
	}
	if (nNumTermanals != 0)
		vResult.clear();

	return vResult;
}

bool router_C::checkPseudoPin()
{
	bool bNonZero = false;
	int nTotalInst = 0;
	int nPosCount = 0;
	int nNegCount = 0;
	for( int z=0; z<m_vGraph.size(); z++ )
	{
		for( int y=0; y<m_vGraph[z].size(); y++ )
		{
			for( int x=0; x<m_vGraph[z][y].size(); x++ )
			{
				gGrid_C* pGrid = getGrid( m_pDesign, x+1, y+1, z+1 );
				if( pGrid->getPinDemand() != 0 )
				{
					bNonZero = true;
					if( pGrid->getPinDemand() < 0 )
						nNegCount ++ ;
					else
						nPosCount++;
				}
			
				nTotalInst = nTotalInst + pGrid->getInstance().size();
			}
		}
	}
	if( bNonZero )
		cout << "There exit non zero pseudo pin, P: " << nPosCount << " N: " << nNegCount <<endl;
	else
		cout << "Pass the pseudo pin check"<<endl;
	cout << "Total Instance " << nTotalInst << endl;
}

bool router_C::saveNet(net_C *pNet, vector<vector<gGrid_C *>> &vTmpPath)
{
	wire_C *pSegment = NULL;
	int nDX, nDY, nDZ;
	/*
	if( vPath.size() == 0 )
	{
		cout<<"No path to save"<<endl;
		return false;
	}
	*/
	pNet->m_vPath = vTmpPath;
	vector<gGrid_C *> vPath;
	for (int i = 0; i < vTmpPath.size(); i++)
	{
		vector<gGrid_C *> vSubPath = vTmpPath[i];
		for (int j = 0; j < vSubPath.size(); j++)
		{
			vPath.push_back(vSubPath[j]);
		}
	}

	for (int i = 0; i < vPath.size(); i++)
	{
		gGrid_C *pGrid = vPath[i];
		int nPX, nPY, nPZ;
		pGrid->getPosition( nPX, nPY, nPZ );
		//if( pNet->getName() == "N13" )
		//	cout << nPX << " " << nPY << " " << nPZ << endl;
		if (pSegment == NULL)
		{
			pSegment = new wire_C;
			pSegment->setNet(pNet);
			pSegment->setGrid1(pGrid);
			gGrid_C *pNGrid = NULL;
			if (i + 1 < vPath.size())
			{
				pNGrid = vPath[i + 1];
				int nX1, nX2, nY1, nY2, nZ1, nZ2;
				pGrid->getPosition(nX1, nY1, nZ1);
				pNGrid->getPosition(nX2, nY2, nZ2);
				nDX = nX2 - nX1;
				nDY = nY2 - nY1;
				nDZ = nZ2 - nZ1;
				if (abs(nDX) + abs(nDY) + abs(nDZ) != 1)
				{
					delete pSegment;
					pSegment = NULL;
					continue;
				}
				vector<layer_C *> vLayer = m_pDesign->getLayer();
				int nLayer = nZ1;
				char cDir = vLayer[nLayer - m_nOffsetZ]->getDir();
				if (abs(nDX) == 1 && cDir != 'V')
				{
					delete pSegment;
					pSegment = NULL;
					continue;
				}
				else if (abs(nDY) == 1 && cDir != 'H')
				{
					delete pSegment;
					pSegment = NULL;
					continue;
				}
				//if( pNet->getName() == "N13" )
				//	cout<<nX1<<" "<<nY1<<" "<<nZ1<<"       ";
			}
			else
			{
				//cout<<"Error"<<endl;
				break;
			}
		}
		else
		{
			gGrid_C *pNGrid = NULL;
			if (i + 1 < vPath.size())
			{
				pNGrid = vPath[i + 1];
				int nX1, nX2, nY1, nY2, nZ1, nZ2, nTmpX, nTmpY, nTmpZ;
				pGrid->getPosition(nX1, nY1, nZ1);
				pNGrid->getPosition(nX2, nY2, nZ2);
				nTmpX = nX2 - nX1;
				nTmpY = nY2 - nY1;
				nTmpZ = nZ2 - nZ1;
				if (nTmpX == nDX && nTmpY == nDY && nTmpZ == nDZ)
				{
					continue;
				}
				else
				{
					//if( pNet->getName() == "N13" )
					//	cout<<nX1<<" "<<nY1<<" "<<nZ1<<" !"<<endl;
					pSegment->setGrid2(pGrid);
					pNet->addWire(pSegment);
					pSegment = NULL;
					nDX = 0;
					nDY = 0;
					nDZ = 0;
					i--;
				}
			}
			else
				break;
		}
	}
	pSegment->setGrid2(vPath.back());
	pNet->addWire(pSegment);
	pSegment = NULL;
	pNet->setLength(calWireLength(pNet));
	//cout<<"Save success"<<endl;
	return true;
}

bool router_C::saveNet(net_C *pNet, vector<gGrid_C *> &vPath)
{
	wire_C *pSegment = NULL;
	int nDX, nDY, nDZ;
	if (vPath.size() == 0)
	{
		cout << "No path to save" << endl;
		return false;
	}

	for (int i = 0; i < vPath.size(); i++)
	{
		gGrid_C *pGrid = vPath[i];
		if (pSegment == NULL)
		{
			pSegment = new wire_C;
			pSegment->setNet(pNet);
			pSegment->setGrid1(pGrid);
			gGrid_C *pNGrid = NULL;
			if (i + 1 < vPath.size())
			{
				pNGrid = vPath[i + 1];
				int nX1, nX2, nY1, nY2, nZ1, nZ2;
				pGrid->getPosition(nX1, nY1, nZ1);
				pNGrid->getPosition(nX2, nY2, nZ2);
				nDX = nX2 - nX1;
				nDY = nY2 - nY1;
				nDZ = nZ2 - nZ1;
				if (abs(nDX) + abs(nDY) + abs(nDZ) != 1)
				{
					delete pSegment;
					pSegment = NULL;
					continue;
				}
				vector<layer_C *> vLayer = m_pDesign->getLayer();
				int nLayer = nZ1;
				char cDir = vLayer[nLayer - m_nOffsetZ]->getDir();
				if (abs(nDX) == 1 && cDir != 'V')
				{
					delete pSegment;
					pSegment = NULL;
					continue;
				}
				else if (abs(nDY) == 1 && cDir != 'H')
				{
					delete pSegment;
					pSegment = NULL;
					continue;
				}
				//cout<<nX1<<" "<<nY1<<" "<<nZ1<<"       ";
			}
			else
			{
				//cout<<"Error"<<endl;
				break;
			}
		}
		else
		{
			gGrid_C *pNGrid = NULL;
			if (i + 1 < vPath.size())
			{
				pNGrid = vPath[i + 1];
				int nX1, nX2, nY1, nY2, nZ1, nZ2, nTmpX, nTmpY, nTmpZ;
				pGrid->getPosition(nX1, nY1, nZ1);
				pNGrid->getPosition(nX2, nY2, nZ2);
				nTmpX = nX2 - nX1;
				nTmpY = nY2 - nY1;
				nTmpZ = nZ2 - nZ1;
				if (nTmpX == nDX && nTmpY == nDY && nTmpZ == nDZ)
				{
					continue;
				}
				else
				{
					//cout<<nX1<<" "<<nY1<<" "<<nZ1<<" !"<<endl;
					pSegment->setGrid2(pGrid);
					pNet->addWire(pSegment);
					pSegment = NULL;
					nDX = 0;
					nDY = 0;
					nDZ = 0;
					i--;
				}
			}
			else
				break;
		}
	}
	pSegment->setGrid2(vPath.back());
	pNet->addWire(pSegment);
	pSegment = NULL;
	pNet->setLength(calWireLength(pNet));
	//cout<<"Save success"<<endl;
	return true;
}

int router_C::calDistanceFromPath(rGrid_C *pCurGrid, int nPath)
{
	rGrid_C *pPreGrid = pCurGrid->m_pFrom;
	/*
	// added at 07/04 20:30		
	int nPDX, nPDY;
	if( pCurGrid->m_cDir == 'H' )
	{
		nPDX = 0; nPDY = 1;
	}
	else
	{
		nPDX = 1; nPDY = 0;
	}
	
	rGrid_C* pRGrid1 = getRGrid( pCurGrid->m_nX + nPDX, pCurGrid->m_nY + nPDY, pCurGrid->m_nZ );
	rGrid_C* pRGrid2 = getRGrid( pCurGrid->m_nX - nPDX, pCurGrid->m_nY - nPDY, pCurGrid->m_nZ );	
	rGrid_C* pRGrid3 = getRGrid( pCurGrid->m_nX, pCurGrid->m_nY, pCurGrid->m_nZ + 1 );
	rGrid_C* pRGrid4 = getRGrid( pCurGrid->m_nX, pCurGrid->m_nY, pCurGrid->m_nZ - 1 );
	vector< rGrid_C* > vTmpRGrid;
	//for( int rg=0; rg<4; rg++ )
	//{
		if( pRGrid1 != NULL && pRGrid1->isRouted )
			vTmpRGrid.push_back( pRGrid1 );
		if( pRGrid2 != NULL && pRGrid2->isRouted )
			vTmpRGrid.push_back( pRGrid2 );
		if( pRGrid3 != NULL && pRGrid3->isRouted )
			vTmpRGrid.push_back( pRGrid3 );
		if( pRGrid4 != NULL && pRGrid4->isRouted )
			vTmpRGrid.push_back( pRGrid4 );
	//}
	int nTmpPath = pPreGrid->m_nG;
	//vector< rGrid_C* > vTmpRGrid;
	//rGrid_C* pMinRGrid = pRPath->m_pFrom;
	for( int rg=0; rg<vTmpRGrid.size(); rg++ )
	{
		if( nTmpPath > vTmpRGrid[ rg ]->m_nG )
		{
			nTmpPath = vTmpRGrid[ rg ]->m_nG;
			pPreGrid = vTmpRGrid[ rg ];
		}	
	}
	vTmpRGrid.clear();
	pCurGrid->m_pFrom = pPreGrid;
	// end added at 07/04 20:30
	*/
	if (pPreGrid->isPath)
	{
		int nG = nPath;
		nG++;
		pCurGrid->m_nG = nG;
		return nG;
	}
	else
	{
		int nG = calDistanceFromPath(pPreGrid, nPath);
		nG++;
		pCurGrid->m_nG = nG;
		return nG;
	}
}

inline int nEularDistance(gGrid_C *pGrid1, gGrid_C *pGrid2, layer_C *pCurLayer)
{
	int nX1, nX2, nY1, nY2, nZ1, nZ2;
	pGrid1->getPosition(nX1, nY1, nZ1);
	pGrid2->getPosition(nX2, nY2, nZ2);
	int nDis = abs(nX1 - nX2) + abs(nY1 - nY2) + abs(nZ1 - nZ2);
	if (abs(nZ1 - nZ2) == 0)
	{
		if (abs(nY1 - nY2) != 0 && pCurLayer->getDir() == 'V')
			nDis = nDis + 2;
		else if (abs(nX1 - nX2) != 0 && pCurLayer->getDir() == 'H')
			nDis = nDis + 2;
	}
	return nDis;
}

// added at 0704 23:30
inline int nEularDistance_sum(vector<gGrid_C *> vTarget, gGrid_C *pGrid2, layer_C *pCurLayer)
{
	int nDis = 0;
	for (int i = 0; i < vTarget.size(); i++)
	{
		nDis = nDis + nEularDistance(vTarget[i], pGrid2, pCurLayer);
	}
	return nDis;
}

inline int nEularDistance_min(vector<gGrid_C *> vTarget, gGrid_C *pGrid2, layer_C *pCurLayer)
{
	int nDis = INT_MAX;
	for (int i = 0; i < vTarget.size(); i++)
	{
		nDis = min(nDis, nEularDistance(vTarget[i], pGrid2, pCurLayer));
	}
	return nDis;
}
// end added at 0704 23:30

// added at 0706 21:00
inline int nEularDistance_FaS(vector<gGrid_C *> vTarget, gGrid_C *pGrid2, layer_C *pCurLayer, rGrid_C *pRGrid)
{
	int nFirstMin = INT_MAX;
	int nSecondMin = INT_MAX;
	int nDis = INT_MAX;
	for (int i = 0; i < vTarget.size(); i++)
	{
		nDis = nEularDistance(vTarget[i], pGrid2, pCurLayer);
		if (nFirstMin > nDis)
		{
			if (nFirstMin == INT_MAX)
			{
				nFirstMin = nDis;
			}
			else
			{
				nSecondMin = nFirstMin;
				nFirstMin = nDis;
			}
		}
	}
	pRGrid->m_nH = nFirstMin;
	pRGrid->m_nMH = nSecondMin;
	return nFirstMin;
}
// end added at 0706 21:00

inline int nEularDistance(vector<gGrid_C *> vTarget, gGrid_C *pGrid2, layer_C *pCurLayer)
{

	//version 1

	int nDis = INT_MAX;
	for (int i = 0; i < vTarget.size(); i++)
	{
		nDis = min(nDis, nEularDistance(vTarget[i], pGrid2, pCurLayer));
	}

	//version 1.5
	/*	
	int nDis = 0;
	for( int i=0; i<vTarget.size(); i++ )
	{
		nDis = nDis + nEularDistance( vTarget[i], pGrid2, pCurLayer );	
	}	
	*/

	//version 2
	/*	
	int nDis = 0;
	vTarget.push_back( pGrid2 );

	unsigned int *nFX = new unsigned int[ vTarget.size() ];
	unsigned int *nFY = new unsigned int[ vTarget.size() ];
	int nX1 = 0;
	int nX2 = INT_MAX;
	int nY1 = 0;
	int nY2 = INT_MAX;
	int nZ1 = 0;
	int nZ2 = INT_MAX;
	int nX, nY, nZ;
	for( int p=0; p<vTarget.size(); p++ )
	{
		vTarget[p]->getPosition( nX, nY, nZ );
		nFX[p] = nX;
		nFY[p] = nY;
		if( nX1 > nX );
			nX1 = nX;
		if( nX2 < nX );
			nX2 = nX;
		if( nY1 > nY );
			nY1 = nY;
		if( nY2 < nY );
			nY2 = nY;
		if( nZ1 > nZ );
			nZ1 = nZ;
		if( nZ2 < nZ );
			nZ2 = nZ;
	}
	nDis = flute_wl( vTarget.size(), nFX, nFY, ACCURACY );
	
	nDis = nDis + vTarget.size();
	if( abs( nZ1 - nZ2 ) == 0 )
	{
		//if( abs( nY1 - nY2 ) != 0 && pCurLayer->getDir() == 'H' )
		//	nDis = nDis + 2;
		//else if( abs( nX1 - nX2 ) != 0 && pCurLayer->getDir() == 'V' )
		//	nDis = nDis + 2;
	}
	else
		nDis = nDis + abs( nZ1 - nZ2 ) - 1;
	
	nDis = nDis + vTarget.size();
	delete []nFX;
	delete []nFY;
	*/
	return nDis;
}

bool router_C::setRoutingConstraint(net_C *pNet)
{
	string strLayer = pNet->getConstraint();
	if (strLayer == "NoCstr")
	{
		m_nConstraintLayerId = m_nOffsetZ;
		return true;
	}

	vector<layer_C *> vLayer = m_pDesign->getLayer();
	for (int i = 0; i < vLayer.size(); i++)
	{
		if (strLayer == vLayer[i]->getName())
		{
			m_nConstraintLayerId = vLayer[i]->getId();
			break;
		}
	}
	return true;
}

bool router_C::createForcedModel()
{
	vector<instance_C *> vInst = m_pDesign->getInstance();
	for (int i = 0; i < vInst.size(); i++)
	{
		instance_C *pInst = vInst[i];
		forced_C cF;
		cF.m_pInstance = pInst;
		if (pInst->isMovable())
		{
			cF.m_bLockInX = false;
			cF.m_bLockInY = false;
		}
		else
		{
			cF.m_bLockInX = true;
			cF.m_bLockInY = true;
		}
		m_vForced.push_back(cF);
	}
	return true;
}

bool router_C::createNetForcedModel()
{
	vector<networkForced_C> &vNF = m_vNetworkForced;
	vector<net_C *> vNet = m_pDesign->getNet();
	vector<instance_C *> vInst = m_pDesign->getInstance();


	for (int i = 0; i < vNF.size(); i++)
	{
		set< int > sRecordInst;
		set< int > sRecordNet;
		
		forced_net_C cNFModel;
		networkForced_C *pNF = &vNF[i];
		cNFModel.m_pNet = pNF->m_pNet;
		vector<int> vNetId;
		vector<int> vInstId;
		vector<forced_C *> vF = pNF->m_vForced;
		vector< vector< networkForced_C* > > m_vNetStore;
		bool bHasFixCell = false;
		for (int f = 0; f < vF.size(); f++)
		{
			instance_C *pInst = vF[f]->m_pInstance;
			if( !pInst->isMovable() )
				bHasFixCell = true;
			int nInstId = pInst->getId();
			if (sRecordInst.count( nInstId ) == 1 )
				continue;
			
			vector<networkForced_C *> vTmpNF = vF[f]->m_vNetwork;
			m_vNetStore.push_back( vTmpNF );
			
			cNFModel.addInstance(pInst);
			vInstId.push_back(nInstId);
			sRecordInst.insert( nInstId );
			//cout<<nInstId<<" ";
		}
		cNFModel.m_bHasFixedInst = bHasFixCell;
		cNFModel.m_bLock = false;
		//cout<<endl;
		for( int f=0; f<m_vNetStore.size(); f++ )
		{
			vector< networkForced_C* > &vTmpNF = m_vNetStore[f];
			for (int n = 0; n < vTmpNF.size(); n++)
			{
				networkForced_C *pTmpNF = vTmpNF[n];
				int nNetId = pTmpNF->m_pNet->getId();
				if( sRecordNet.count( nNetId ) == 1 || pTmpNF->m_pNet == pNF->m_pNet )
					continue;
				
				//cout<<pTmpNF->m_pNet->getName()<<" ";
				vector< forced_C* > &vTmpF = pTmpNF->m_vForced;
				bool bFind = false;
				
				if( vTmpF.size() <= vInstId.size() )
				{
					for( int k=0; k<vTmpF.size(); k++ )
					{
						//cout<<vTmpF[k]->m_pInstance->getId()<<" ";
						if( sRecordInst.count( vTmpF[k]->m_pInstance->getId() ) == 0 )
						{
							bFind = true;
							break;
						}
					}
					//cout<<endl;
					if( bFind )
						cNFModel.m_vInterNet.push_back( pTmpNF->m_pNet );
					else
						cNFModel.m_vIntraNet.push_back( pTmpNF->m_pNet );
				}
				else
				{
					cNFModel.m_vInterNet.push_back( pTmpNF->m_pNet );
				}
				
				vNetId.push_back( nNetId );
				sRecordNet.insert( nNetId );
			}
		}	
		//cout<<endl;
		cNFModel.m_sInstId = sRecordInst;
		m_vNetForced.push_back(cNFModel);
	}
	/*
	for( int i=0; i<m_vNetForced.size(); i++ )
	{
		forced_net_C &cNF = m_vNetForced[i];
		cout << cNF.m_pNet->getName() << endl;
		cout << cNF.m_vCluster.size() << " " << cNF.m_vInterNet.size() << " " << cNF.m_vIntraNet.size() << endl;
	}
	*/
	return true;
}

bool router_C::createForcedNetwork()
{
	vector<net_C *> vNet = m_pDesign->getNet();
	for (int i = 0; i < vNet.size(); i++)
	{
		net_C *pNet = vNet[i];
		networkForced_C cNF;
		cNF.m_pNet = pNet;
		m_vNetworkForced.push_back(cNF);
		m_vNetworkForced[i].m_cLB.m_pNetwork = &m_vNetworkForced[i];
		m_vNetworkForced[i].m_cRB.m_pNetwork = &m_vNetworkForced[i];
		m_vNetworkForced[i].m_cTB.m_pNetwork = &m_vNetworkForced[i];
		m_vNetworkForced[i].m_cDB.m_pNetwork = &m_vNetworkForced[i];
	}
	return true;
}

bool router_C::linkForcedModel()
{
	// link model to network
	for (int i = 0; i < m_vNetworkForced.size(); i++)
	{
		networkForced_C &cNF = m_vNetworkForced[i];
		net_C *pNet = cNF.m_pNet;
		vector<pin_C *> vPin = pNet->getPin();
		vector<instance_C *> vInst;
		for (int j = 0; j < vPin.size(); j++)
		{
			vInst.push_back((instance_C *)vPin[j]->getCell());
		}

		for (int j = 0; j < vInst.size(); j++)
		{
			instance_C *pInst = vInst[j];
			if (pInst == m_vForced[pInst->getId()].m_pInstance)
			{
				cNF.m_vForced.push_back(&m_vForced[pInst->getId()]);
				m_vForced[pInst->getId()].m_vNetwork.push_back(&cNF);
				cNF.m_mPLF[&m_vForced[pInst->getId()]] = 0;
				cNF.m_mPRF[&m_vForced[pInst->getId()]] = 0;
				cNF.m_mPDF[&m_vForced[pInst->getId()]] = 0;
				cNF.m_mPTF[&m_vForced[pInst->getId()]] = 0;
			}
			else
			{
				for (int k = 0; k < m_vForced.size(); k++)
				{
					if (m_vForced[k].m_pInstance == pInst)
					{
						cNF.m_vForced.push_back(&m_vForced[k]);
						m_vForced[k].m_vNetwork.push_back(&cNF);
						break;
					}
				}
			}
		}
	}
	//createNetForcedModel();
	return true;
}

bool router_C::freeBoundry( instance_C *pInst )
{
	forced_C *pF;
	if (pInst == m_vForced[pInst->getId()].m_pInstance)
	{
		pF = &m_vForced[pInst->getId()];
	}
	else
	{
		for (int i = 0; i < m_vForced.size(); i++)
		{
			if (m_vForced[i].m_pInstance == pInst)
			{
				pF = &m_vForced[i];
				break;
			}
		}
	}

	vector<networkForced_C *> vNetwork = pF->m_vNetwork;
	
	for (int i = 0; i < vNetwork.size(); i++)
	{
		vNetwork[i]->m_cLB.m_bLock = false;
		vNetwork[i]->m_cRB.m_bLock = false;
		vNetwork[i]->m_cTB.m_bLock = false;
		vNetwork[i]->m_cDB.m_bLock = false;
	}

	return true;
}


bool router_C::updateForcedModel_ver2( instance_C *pInst )
{
	forced_C *pF;
	if (pInst == m_vForced[pInst->getId()].m_pInstance)
	{
		pF = &m_vForced[pInst->getId()];
	}
	else
	{
		for (int i = 0; i < m_vForced.size(); i++)
		{
			if (m_vForced[i].m_pInstance == pInst)
			{
				pF = &m_vForced[i];
				break;
			}
		}
	}
	set<instance_C *> sInst;
	vector<forced_C *> vForced;
	sInst.insert(pInst);
	vForced.push_back(pF);

	vector<networkForced_C *> vNetwork = pF->m_vNetwork;
	for (int i = 0; i < vNetwork.size(); i++)
	{
		vector<forced_C *> vF = vNetwork[i]->m_vForced;
		for (int j = 0; j < vF.size(); j++)
		{
			forced_C *pTmpF = vF[j];
			instance_C *pTmpInst = pTmpF->m_pInstance;
			if (sInst.count(pTmpInst) == 0)
			{
				sInst.insert(pTmpInst);
				vForced.push_back(pTmpF);
			}
		}
	}

	// logger begin 0812
	for (int i = 0; i < vForced.size(); i++)
	{
		vForced[i]->m_nT = 0;
		vForced[i]->m_nD = 0;
		vForced[i]->m_nR = 0;
		vForced[i]->m_nL = 0;
		vForced[i]->m_nCX = 0;
		vForced[i]->m_nCY = 0;
	}
	// logger end

	for (int i = 0; i < vNetwork.size(); i++)
	{
		calForcedNetwork_ver3(*vNetwork[i]);
	}

	for (int i = 0; i < vForced.size(); i++)
	{
		calForcedModel_ver4(*vForced[i]);
	}

	for (int i = 0; i < vNetwork.size(); i++)
	{
		/*
		calBoundryModel_ver2( &vNetwork[i]->m_cLB );
		calBoundryModel_ver2( &vNetwork[i]->m_cRB );
		calBoundryModel_ver2( &vNetwork[i]->m_cDB );
		calBoundryModel_ver2( &vNetwork[i]->m_cTB );
		*/
		calBoundryModel( &vNetwork[i]->m_cLB );
		calBoundryModel( &vNetwork[i]->m_cRB );
		calBoundryModel( &vNetwork[i]->m_cDB );
		calBoundryModel( &vNetwork[i]->m_cTB );
	}

	return true;
}

bool router_C::updateForcedModel(instance_C *pInst)
{
	forced_C *pF;
	if (pInst == m_vForced[pInst->getId()].m_pInstance)
	{
		pF = &m_vForced[pInst->getId()];
	}
	else
	{
		for (int i = 0; i < m_vForced.size(); i++)
		{
			if (m_vForced[i].m_pInstance == pInst)
			{
				pF = &m_vForced[i];
				break;
			}
		}
	}
	set<instance_C *> sInst;
	vector<forced_C *> vForced;
	sInst.insert(pInst);
	vForced.push_back(pF);

	vector<networkForced_C *> vNetwork = pF->m_vNetwork;
	for (int i = 0; i < vNetwork.size(); i++)
	{
		vector<forced_C *> vF = vNetwork[i]->m_vForced;
		for (int j = 0; j < vF.size(); j++)
		{
			forced_C *pTmpF = vF[j];
			instance_C *pTmpInst = pTmpF->m_pInstance;
			if (sInst.count(pTmpInst) == 0)
			{
				sInst.insert(pTmpInst);
				vForced.push_back(pTmpF);
			}
		}
	}

	// logger begin 0812
	for (int i = 0; i < vForced.size(); i++)
	{
		vForced[i]->m_nT = 0;
		vForced[i]->m_nD = 0;
		vForced[i]->m_nR = 0;
		vForced[i]->m_nL = 0;
	}
	// logger end

	for (int i = 0; i < vNetwork.size(); i++)
	{
		calForcedNetwork_ver2(*vNetwork[i]);
	}

	for (int i = 0; i < vForced.size(); i++)
	{
		calForcedModel_ver3(*vForced[i]);
	}
	return true;
}

bool router_C::freeForcedModel(instance_C *pInst)
{
	forced_C *pF;
	if (pInst == m_vForced[pInst->getId()].m_pInstance)
	{
		pF = &m_vForced[pInst->getId()];
	}
	else
	{
		for (int i = 0; i < m_vForced.size(); i++)
		{
			if (m_vForced[i].m_pInstance == pInst)
			{
				pF = &m_vForced[i];
				break;
			}
		}
	}
	set<instance_C *> sInst;
	vector<forced_C *> vForced;
	sInst.insert(pInst);
	//vForced.push_back( pF );

	vector<networkForced_C *> &vNetwork = pF->m_vNetwork;
	for (int i = 0; i < vNetwork.size(); i++)
	{
		vector<forced_C *> vF = vNetwork[i]->m_vForced;
		for (int j = 0; j < vF.size(); j++)
		{
			forced_C *pTmpF = vF[j];
			instance_C *pTmpInst = pTmpF->m_pInstance;
			if (sInst.count(pTmpInst) == 0)
			{
				sInst.insert(pTmpInst);
				vForced.push_back(pTmpF);
			}
		}
	}

	for (int i = 0; i < vForced.size(); i++)
	{
		freeInstance(vForced[i]->m_pInstance);
	}

	return true;
}

bool router_C::updateForcedNetwork(networkForced_C &cNF)
{

}

bool router_C::calForcedModel( forced_C &cF, vector< net_C* > &vNet, int &nTF, int &nDF, int &nRF, int &nLF )
{
	int nX = cF.m_pInstance->getPlacedX();
	int nY = cF.m_pInstance->getPlacedY();

	// begin logger 0812
	//int nTF = cF.m_nT;
	//int nDF = cF.m_nD;
	//int nRF = cF.m_nR;
	//int nLF = cF.m_nL;
	// end logger 0812
	set< net_C* > sNet;
	for( int i=0; i<vNet.size(); i++ )
		sNet.insert( vNet[i] );


	for (int i = 0; i < cF.m_vNetwork.size(); i++)
	{
		networkForced_C *cNF = cF.m_vNetwork[i];
		if( sNet.count( cNF->m_pNet ) == 1 )
			continue;

		bool bNoForcedInX = false;
		bool bNoForcedInY = false;

		nTF = nTF + cNF->m_mPTF.find( &cF )->second;
		nDF = nDF + cNF->m_mPDF.find( &cF )->second;
		nRF = nRF + cNF->m_mPRF.find( &cF )->second;
		nLF = nLF + cNF->m_mPLF.find( &cF )->second;
		
		// second version
		
		if (cNF->m_nMaxY == cNF->m_nMinY && cNF->m_nMaxX == cNF->m_nMinX )
		{
			nRF--;
			nLF--;
		}
		else if (cNF->m_nMaxY == cNF->m_nMinY)
		{
			//nRF--;
			//nLF--;
			//nRF++;
			//nLF++;
		}
		//else 
		if (nY == cNF->m_nMaxY)
		{
			nRF--;
			if (cNF->m_nMaxYCount == 1)
				nLF = nLF + 1;
			if (cNF->m_nMaxXCount >= 1)
				nLF++;
		}
		//else 
		if (nY == cNF->m_nMinY)
		{
			nLF--;
			if (cNF->m_nMinYCount == 1)
				nRF = nRF + 1;
			if (cNF->m_nMinYCount >= 1)
				nRF++;
		}

		if (cNF->m_nMaxX == cNF->m_nMinX && cNF->m_nMaxY == cNF->m_nMinY )
		{
			nTF--;
			nDF--;
		}
		else if (cNF->m_nMaxX == cNF->m_nMinX)
		{
			//nTF--;
			//nDF--;
			///nTF++;
			//nDF++;
		}
		//else 
		if (nX == cNF->m_nMaxX)
		{
			nTF--;
			if (cNF->m_nMaxXCount == 1)
				nDF = nDF + 1;
			if (cNF->m_nMaxXCount >= 1)
				nDF++;
		}
		//else 
		if (nX == cNF->m_nMinX)
		{
			nDF--;
			if (cNF->m_nMinXCount == 1)
				nTF = nTF + 1;
			if (cNF->m_nMinXCount >= 1)
				nTF++;
		}
	}
	
	if (nTF < 0)
		nTF = 0;
	if (nDF < 0)
		nDF = 0;
	if (nRF < 0)
		nRF = 0;
	if (nLF < 0)
		nLF = 0;
	
	return true;
}

bool router_C::calForcedModel_ver4(forced_C &cF, set< net_C* > &sNet, char cBound )
{
	int nX = cF.m_pInstance->getPlacedX();
	int nY = cF.m_pInstance->getPlacedY();

	int nTF = 0;
	int nDF = 0;
	int nRF = 0;
	int nLF = 0;
	int nCX = 0;
	int nCY = 0;

	//cout << "Forced: " << nTF << " " << nDF << " " << nRF << " " << nLF << " " << nCF << endl;
	// added at 0712 22:00
	int nBX1, nBX2;
	int nBY1, nBY2;
	nBX1 = cF.m_pInstance->getPlacedX();
	nBY1 = cF.m_pInstance->getPlacedY();
	nBX2 = nBX1;
	nBY2 = nBY1;

	vector<int> vMinX;
	vector<int> vMaxX;

	vector<int> vMinY;
	vector<int> vMaxY;
	//cout << "cal bound"<<endl;
	vector<networkForced_C *> &vNetWork = cF.m_vNetwork;
	vector<vector<instance_C *>> vBoundingBox;
	for (int i = 0; i < vNetWork.size(); i++)
	{
		networkForced_C *pNF = vNetWork[i];
		if( sNet.count( pNF->m_pNet ) != 0 )
			continue;
		vMinX.push_back( pNF->m_mMinX.find( &cF )->second );
		vMaxX.push_back( pNF->m_mMaxX.find( &cF )->second );
		vMinY.push_back( pNF->m_mMinY.find( &cF )->second );
		vMaxY.push_back( pNF->m_mMaxY.find( &cF )->second );
		if( pNF->m_mMinX.find( &cF )->second > nX  )
		{
			nTF++;
		}
		if( pNF->m_mMaxX.find( &cF )->second < nX  )
		{
			nDF++;
		}
		if( pNF->m_mMinY.find( &cF )->second > nY  )
		{
			nRF++;
		}
		if( pNF->m_mMaxY.find( &cF )->second < nY  )
		{
			nLF++;
		}
		if( pNF->m_nMinX == pNF->m_nMaxX )
		{
			nCX++;
		}
		if( pNF->m_nMinY == pNF->m_nMaxY )
		{
			nCY++;
		}
		
	}
	
	cF.m_nT = nTF;
	cF.m_nD = nDF;
	cF.m_nR = nRF;
	cF.m_nL = nLF;
	cF.m_nCX = nCX;
	cF.m_nCY = nCY;
	/*
	cout << cF.m_pInstance->getName()<<endl;
	for( int i=0; i<vMinX.size(); i++ )
	{
		cout << vMinX[i] << " " << vMinY[i] << " " << vMaxX[i] << " " << vMaxY[i] << endl;
	}
	cout << "Forced: " << nTF << " " << nDF << " " << nRF << " " << nLF << " " << nCF << endl;
	*/
	//cout << "end bound"<<endl;
	int nGain = 0;
	int nXGain = 0;
	int nYGain = 0;
	bool bBalanceInX = false;
	bool bBalanceInY = false;

	int nDX = 0;
	int nDY = 0;
	nDX = -1;
	nDY = 0;
	int nBest;
	int nTmp;
	nTmp = cF.m_pInstance->getPlacedX();

	nBest = boundingBoxCost(nTmp, vMinX, vMaxX);
	gGrid_C *pBestGrid = NULL;
	gGrid_C *pGrid = getGrid(m_pDesign, cF.m_pInstance->getPlacedX(), cF.m_pInstance->getPlacedY(), m_nDZ);
	while (graphTravel(m_pDesign, pGrid, nDX, nDY, 0) != NULL)
	{
		pGrid = graphTravel(m_pDesign, pGrid, nDX, nDY, 0);
		int nTmpX, nTmpY, nTmpZ;
		pGrid->getPosition(nTmpX, nTmpY, nTmpZ);
		nTmp = nTmpX;

		int nTmpBest = boundingBoxCost(nTmp, vMinX, vMaxX);

		if (nTmpBest <= nBest) // change from <
		{
			pBestGrid = pGrid;
			nBX1 = nBX1 + nDX;
		}
		else
			break;
	}
	
	nDX = 1;
	nDY = 0;
	nTmp = cF.m_pInstance->getPlacedX();

	nBest = boundingBoxCost(nTmp, vMinX, vMaxX);
	pBestGrid = NULL;
	pGrid = getGrid(m_pDesign, cF.m_pInstance->getPlacedX(), cF.m_pInstance->getPlacedY(), m_nDZ);
	while (graphTravel(m_pDesign, pGrid, nDX, nDY, 0) != NULL)
	{
		pGrid = graphTravel(m_pDesign, pGrid, nDX, nDY, 0);
		int nTmpX, nTmpY, nTmpZ;
		pGrid->getPosition(nTmpX, nTmpY, nTmpZ);
		nTmp = nTmpX;

		int nTmpBest = boundingBoxCost(nTmp, vMinX, vMaxX);

		if (nTmpBest <= nBest) // change from <
		{
			pBestGrid = pGrid;
			nBX2 = nBX2 + nDX;
		}
		else
			break;
	}
	
	nDY = -1;
	nDX = 0;
	nTmp = cF.m_pInstance->getPlacedY();

	nBest = boundingBoxCost(nTmp, vMinY, vMaxY);
	pBestGrid = NULL;
	pGrid = getGrid(m_pDesign, cF.m_pInstance->getPlacedX(), cF.m_pInstance->getPlacedY(), m_nDZ);
	while (graphTravel(m_pDesign, pGrid, nDX, nDY, 0) != NULL)
	{
		pGrid = graphTravel(m_pDesign, pGrid, nDX, nDY, 0);
		int nTmpX, nTmpY, nTmpZ;
		pGrid->getPosition(nTmpX, nTmpY, nTmpZ);
		nTmp = nTmpY;

		int nTmpBest = boundingBoxCost(nTmp, vMinY, vMaxY);

		if (nTmpBest <= nBest) // change from <
		{
			pBestGrid = pGrid;
			nBY1 = nBY1 + nDY;
		}
		else
			break;
	}

	nDY = 1;
	nDX = 0;
	nTmp = cF.m_pInstance->getPlacedY();

	nBest = boundingBoxCost(nTmp, vMinY, vMaxY);
	pBestGrid = NULL;
	pGrid = getGrid(m_pDesign, cF.m_pInstance->getPlacedX(), cF.m_pInstance->getPlacedY(), m_nDZ);
	while (graphTravel(m_pDesign, pGrid, nDX, nDY, 0) != NULL)
	{
		pGrid = graphTravel(m_pDesign, pGrid, nDX, nDY, 0);
		int nTmpX, nTmpY, nTmpZ;
		pGrid->getPosition(nTmpX, nTmpY, nTmpZ);
		nTmp = nTmpY;

		int nTmpBest = boundingBoxCost(nTmp, vMinY, vMaxY);

		if (nTmpBest <= nBest) // change from <
		{
			pBestGrid = pGrid;
			nBY2 = nBY2 + nDY;
		}
		else
			break;
	}
	
	//nGain = nXGain + nYGain;
	cF.m_nGain = nGain;
	if( nBX1 > nBX2 )
	{
		int nTmpX = nBX1;
		nBX1 = nBX2;
		nBX2 = nTmpX;
	}
	
	if( nBY1 > nBY2 )
	{
		int nTmpY = nBY1;
		nBY1 = nBY2;
		nBY2 = nTmpY;
	}
	//cout << nBX1 << " " << nBY1 << " " << nBX2 << " " << nBY2 << endl;
	int nMoveStep = 5;
	if( cBound == 'L' )
	{
		if( nBY1 <= nY && nY + 1 <= m_vGraph[0].size() )
		{
			nBY1 = nY + 1;
		}

		if( nBY2 <= nBY1 && nY + 1 <=m_vGraph[0].size() )
		{
			nBY2 = nY;
			for( int i=0; i<nMoveStep; i++ )
			{
				if( nBY2 + 1 <= m_vGraph[0].size() )
					nBY2++;
			}
		}
	}
	else if( cBound == 'R' )
	{
		if( nBY2 >= nY && nY - 1 > 0  )
		{
			nBY2 = nY - 1;
		}

		if( nBY1 >= nBY2 && nY - 1 > 0 )
		{
			nBY1 = nY;
			for( int i=0; i<nMoveStep; i++ )
			{
				if( nBY1 - 1 > 0 )
					nBY1--;
			}
		}
	}
	else if( cBound == 'D' )
	{
		if( nBX1 <= nX && nX + 1 <= m_vGraph[0][0].size() )
		{
			nBX1 = nX + 1;
		}

		if( nBX2 <= nBX1 && nX + 1 <=m_vGraph[0][0].size() )
		{
			nBX2 = nX;
			for( int i=0; i<nMoveStep; i++ )
			{
				if( nBX2 + 1 <= m_vGraph[0][0].size() )
					nBX2++;
			}
		}
	}
	else if( cBound == 'T' )
	{
		if( nBX2 >= nX && nX - 1 > 0 )
		{
			nBX2 = nX - 1;
		}

		if( nBX1 >= nBX2 && nX - 1 > 0  )
		{
			nBX1 = nX ;
			for( int i=0; i<nMoveStep; i++ )
			{
				if( nBX1 - 1 > 0 )
					nBX1--;
			}
		}
	}
	
	cF.m_nBoundX1 = nBX1;
	cF.m_nBoundX2 = nBX2;
	cF.m_nBoundY1 = nBY1;
	cF.m_nBoundY2 = nBY2;
	cF.m_vMinX = vMinX;
	cF.m_vMaxX = vMaxX;

	cF.m_vMinY = vMinY;
	cF.m_vMaxY = vMaxY;
	
	return true;
}

bool router_C::calForcedModel_ver4(forced_C &cF, set< net_C* > &sNet )
{
	int nX = cF.m_pInstance->getPlacedX();
	int nY = cF.m_pInstance->getPlacedY();

	int nTF = 0;
	int nDF = 0;
	int nRF = 0;
	int nLF = 0;
	int nCX = 0;
	int nCY = 0;

	//cout << "Forced: " << nTF << " " << nDF << " " << nRF << " " << nLF << " " << nCF << endl;
	// added at 0712 22:00
	int nBX1, nBX2;
	int nBY1, nBY2;
	nBX1 = cF.m_pInstance->getPlacedX();
	nBY1 = cF.m_pInstance->getPlacedY();
	nBX2 = nBX1;
	nBY2 = nBY1;

	vector<int> vMinX;
	vector<int> vMaxX;

	vector<int> vMinY;
	vector<int> vMaxY;
	//cout << "cal bound"<<endl;
	vector<networkForced_C *> &vNetWork = cF.m_vNetwork;
	vector<vector<instance_C *>> vBoundingBox;
	for (int i = 0; i < vNetWork.size(); i++)
	{
		networkForced_C *pNF = vNetWork[i];
		if( sNet.count( pNF->m_pNet ) != 0 )
			continue;
		vMinX.push_back( pNF->m_mMinX.find( &cF )->second );
		vMaxX.push_back( pNF->m_mMaxX.find( &cF )->second );
		vMinY.push_back( pNF->m_mMinY.find( &cF )->second );
		vMaxY.push_back( pNF->m_mMaxY.find( &cF )->second );
		if( pNF->m_mMinX.find( &cF )->second > nX  )
		{
			nTF++;
		}
		if( pNF->m_mMaxX.find( &cF )->second < nX  )
		{
			nDF++;
		}
		if( pNF->m_mMinY.find( &cF )->second > nY  )
		{
			nRF++;
		}
		if( pNF->m_mMaxY.find( &cF )->second < nY  )
		{
			nLF++;
		}
		if( pNF->m_nMinX == pNF->m_nMaxX )
		{
			nCX++;
		}
		if( pNF->m_nMinY == pNF->m_nMaxY )
		{
			nCY++;
		}
		
	}
	
	cF.m_nT = nTF;
	cF.m_nD = nDF;
	cF.m_nR = nRF;
	cF.m_nL = nLF;
	cF.m_nCX = nCX;
	cF.m_nCY = nCY;
	/*
	cout << cF.m_pInstance->getName()<<endl;
	for( int i=0; i<vMinX.size(); i++ )
	{
		cout << vMinX[i] << " " << vMinY[i] << " " << vMaxX[i] << " " << vMaxY[i] << endl;
	}
	cout << "Forced: " << nTF << " " << nDF << " " << nRF << " " << nLF << " " << nCF << endl;
	*/
	//cout << "end bound"<<endl;
	int nGain = 0;
	int nXGain = 0;
	int nYGain = 0;
	bool bBalanceInX = false;
	bool bBalanceInY = false;

	int nDX = 0;
	int nDY = 0;
	nDX = -1;
	nDY = 0;
	int nBest;
	int nTmp;
	nTmp = cF.m_pInstance->getPlacedX();

	nBest = boundingBoxCost(nTmp, vMinX, vMaxX);
	gGrid_C *pBestGrid = NULL;
	gGrid_C *pGrid = getGrid(m_pDesign, cF.m_pInstance->getPlacedX(), cF.m_pInstance->getPlacedY(), m_nDZ);
	while (graphTravel(m_pDesign, pGrid, nDX, nDY, 0) != NULL)
	{
		pGrid = graphTravel(m_pDesign, pGrid, nDX, nDY, 0);
		int nTmpX, nTmpY, nTmpZ;
		pGrid->getPosition(nTmpX, nTmpY, nTmpZ);
		nTmp = nTmpX;

		int nTmpBest = boundingBoxCost(nTmp, vMinX, vMaxX);

		if (nTmpBest <= nBest) // change from <
		{
			pBestGrid = pGrid;
			nBX1 = nBX1 + nDX;
		}
		else
			break;
	}
	
	nDX = 1;
	nDY = 0;
	nTmp = cF.m_pInstance->getPlacedX();

	nBest = boundingBoxCost(nTmp, vMinX, vMaxX);
	pBestGrid = NULL;
	pGrid = getGrid(m_pDesign, cF.m_pInstance->getPlacedX(), cF.m_pInstance->getPlacedY(), m_nDZ);
	while (graphTravel(m_pDesign, pGrid, nDX, nDY, 0) != NULL)
	{
		pGrid = graphTravel(m_pDesign, pGrid, nDX, nDY, 0);
		int nTmpX, nTmpY, nTmpZ;
		pGrid->getPosition(nTmpX, nTmpY, nTmpZ);
		nTmp = nTmpX;

		int nTmpBest = boundingBoxCost(nTmp, vMinX, vMaxX);

		if (nTmpBest <= nBest) // change from <
		{
			pBestGrid = pGrid;
			nBX2 = nBX2 + nDX;
		}
		else
			break;
	}
	
	nDY = -1;
	nDX = 0;
	nTmp = cF.m_pInstance->getPlacedY();

	nBest = boundingBoxCost(nTmp, vMinY, vMaxY);
	pBestGrid = NULL;
	pGrid = getGrid(m_pDesign, cF.m_pInstance->getPlacedX(), cF.m_pInstance->getPlacedY(), m_nDZ);
	while (graphTravel(m_pDesign, pGrid, nDX, nDY, 0) != NULL)
	{
		pGrid = graphTravel(m_pDesign, pGrid, nDX, nDY, 0);
		int nTmpX, nTmpY, nTmpZ;
		pGrid->getPosition(nTmpX, nTmpY, nTmpZ);
		nTmp = nTmpY;

		int nTmpBest = boundingBoxCost(nTmp, vMinY, vMaxY);

		if (nTmpBest <= nBest) // change from <
		{
			pBestGrid = pGrid;
			nBY1 = nBY1 + nDY;
		}
		else
			break;
	}

	nDY = 1;
	nDX = 0;
	nTmp = cF.m_pInstance->getPlacedY();

	nBest = boundingBoxCost(nTmp, vMinY, vMaxY);
	pBestGrid = NULL;
	pGrid = getGrid(m_pDesign, cF.m_pInstance->getPlacedX(), cF.m_pInstance->getPlacedY(), m_nDZ);
	while (graphTravel(m_pDesign, pGrid, nDX, nDY, 0) != NULL)
	{
		pGrid = graphTravel(m_pDesign, pGrid, nDX, nDY, 0);
		int nTmpX, nTmpY, nTmpZ;
		pGrid->getPosition(nTmpX, nTmpY, nTmpZ);
		nTmp = nTmpY;

		int nTmpBest = boundingBoxCost(nTmp, vMinY, vMaxY);

		if (nTmpBest <= nBest) // change from <
		{
			pBestGrid = pGrid;
			nBY2 = nBY2 + nDY;
		}
		else
			break;
	}
	
	//nGain = nXGain + nYGain;
	cF.m_nGain = nGain;
	if( nBX1 > nBX2 )
	{
		int nTmpX = nBX1;
		nBX1 = nBX2;
		nBX2 = nTmpX;
	}
	
	if( nBY1 > nBY2 )
	{
		int nTmpY = nBY1;
		nBY1 = nBY2;
		nBY2 = nTmpY;
	}
	//cout << nBX1 << " " << nBY1 << " " << nBX2 << " " << nBY2 << endl;
	cF.m_nBoundX1 = nBX1;
	cF.m_nBoundX2 = nBX2;
	cF.m_nBoundY1 = nBY1;
	cF.m_nBoundY2 = nBY2;
	cF.m_vMinX = vMinX;
	cF.m_vMaxX = vMaxX;

	cF.m_vMinY = vMinY;
	cF.m_vMaxY = vMaxY;
	
	return true;
}



bool router_C::calForcedModel(forced_C &cF, set< net_C* > &sNet )
{
	int nX = cF.m_pInstance->getPlacedX();
	int nY = cF.m_pInstance->getPlacedY();

	int nTF = 0;
	int nDF = 0;
	int nRF = 0;
	int nLF = 0;

	int nCenterX = ( m_nDX + m_nTX )/2;
	int nCenterY = ( m_nDY + m_nTY )/2;
	/*	
	if( nX < nCenterX )
		nDF++;
	else if( nX > nCenterX )
		nTF++;

	if( nY < nCenterY )
		nLF++;
	else if( nY > nCenterY )
		nRF++;
	*/
	// begin logger 0812
	//int nTF = cF.m_nT;
	//int nDF = cF.m_nD;
	//int nRF = cF.m_nR;
	//int nLF = cF.m_nL;
	// end logger 0812
	/*
	for( int i = 0; i < cF.m_vNetwork.size(); i++ )
	{
		networkForced_C *cNF = cF.m_vNetwork[i];
		nTF = nTF + cNF->m_mPTF.find( &cF )->second;
		nDF = nDF + cNF->m_mPDF.find( &cF )->second;
		nRF = nRF + cNF->m_mPRF.find( &cF )->second;
		nLF = nLF + cNF->m_mPLF.find( &cF )->second;
	}
	*/
	for (int i = 0; i < cF.m_vNetwork.size(); i++)
	{
		networkForced_C *cNF = cF.m_vNetwork[i];
		if( sNet.count( cNF->m_pNet ) != 0 )
			continue;

		bool bNoForcedInX = false;
		bool bNoForcedInY = false;

		nTF = nTF + cNF->m_mPTF.find( &cF )->second;
		nDF = nDF + cNF->m_mPDF.find( &cF )->second;
		nRF = nRF + cNF->m_mPRF.find( &cF )->second;
		nLF = nLF + cNF->m_mPLF.find( &cF )->second;
		
		// second version
		if (cNF->m_nMaxY == cNF->m_nMinY && cNF->m_nMaxX == cNF->m_nMinX )
		{
			nRF--;
			nLF--;
			//nRF++;
			//nLF++;
		}
		else if (cNF->m_nMaxY == cNF->m_nMinY)
		{
			//nRF--;
			//nLF--;
			//nRF++;
			//nLF++;
		}
		//else 
		if (nY == cNF->m_nMaxY)
		{
			nRF--;
			if (cNF->m_nMaxYCount == 1)
				nLF = nLF + 1;
			if (cNF->m_nMaxXCount >= 1)
				nLF++;
		}
		//else 
		if (nY == cNF->m_nMinY)
		{
			nLF--;
			if (cNF->m_nMinYCount == 1)
				nRF = nRF + 1;
			if (cNF->m_nMinYCount >= 1)
				nRF++;
		}

		if (cNF->m_nMaxX == cNF->m_nMinX && cNF->m_nMaxY == cNF->m_nMinY )
		{
			nTF--;
			nDF--;
			//nTF++;
			//nDF++;
		}
		else if (cNF->m_nMaxX == cNF->m_nMinX)
		{
			//nTF--;
			//nDF--;
			//nTF++;
			//nDF++;
		}
		//else 
		if (nX == cNF->m_nMaxX)
		{
			nTF--;
			if (cNF->m_nMaxXCount == 1)
				nDF = nDF + 1;
			if (cNF->m_nMaxXCount >= 1)
				nDF++;
		}
		//else 
		if (nX == cNF->m_nMinX)
		{
			nDF--;
			if (cNF->m_nMinXCount == 1)
				nTF = nTF + 1;
			if (cNF->m_nMinXCount >= 1)
				nTF++;
		}
	}
	
	if (nTF < 0)
		nTF = 0;
	if (nDF < 0)
		nDF = 0;
	if (nRF < 0)
		nRF = 0;
	if (nLF < 0)
		nLF = 0;
	
	cF.m_nT = nTF;
	cF.m_nD = nDF;
	cF.m_nR = nRF;
	cF.m_nL = nLF;

	// added at 0712 22:00
	int nBX1, nBX2;
	int nBY1, nBY2;
	nBX1 = cF.m_pInstance->getPlacedX();
	nBY1 = cF.m_pInstance->getPlacedY();
	nBX2 = nBX1;
	nBY2 = nBY1;

	vector<int> vMinX;
	vector<int> vMaxX;

	vector<int> vMinY;
	vector<int> vMaxY;

	vector<networkForced_C *> &vNetWork = cF.m_vNetwork;
	vector<vector<instance_C *>> vBoundingBox;
	/*
	for (int i = 0; i < vNetWork.size(); i++)
	{
		vector<instance_C *> vInst;
		networkForced_C *pNF = vNetWork[i];
		vector<forced_C *> vF = pNF->m_vForced;
		for (int j = 0; j < vF.size(); j++)
		{
			instance_C *pTmpInst = vF[j]->m_pInstance;
			if (pTmpInst != cF.m_pInstance)
				vInst.push_back(pTmpInst);
		}
		vBoundingBox.push_back(vInst);
	}
	*/
	for (int i = 0; i < vNetWork.size(); i++)
	{
		networkForced_C *pNF = vNetWork[i];
		if( sNet.count( pNF->m_pNet ) != 0 )
			continue;
		/*	
		if( pNF->m_mMinX.find( &cF )->second == nX && nX == pNF->m_mMaxX.find( &cF )->second 
		 && pNF->m_mMinY.find( &cF )->second == nY && nY == pNF->m_mMaxY.find( &cF )->second )
		 {
		 	continue;
		 }
		*/
		if( pNF->m_mMinX.find( &cF )->second == nX && nX == pNF->m_mMaxX.find( &cF )->second )
		{
			vMinX.push_back( pNF->m_nMinX );
			vMaxX.push_back( pNF->m_nMaxX );	
		}
		else
		{
			vMinX.push_back( pNF->m_mMinX.find( &cF )->second );
			vMaxX.push_back( pNF->m_mMaxX.find( &cF )->second );
		}
		if( pNF->m_mMinY.find( &cF )->second == nY && nY == pNF->m_mMaxY.find( &cF )->second )
		{
			vMinY.push_back( pNF->m_nMinY );
			vMaxY.push_back( pNF->m_nMaxY );
		}
		else
		{
			vMinY.push_back( pNF->m_mMinY.find( &cF )->second );
			vMaxY.push_back( pNF->m_mMaxY.find( &cF )->second );
		}
	}

	int nGain = 0;
	int nXGain = 0;
	int nYGain = 0;
	
	int nDX = 0;
	int nDY = 0;
	if (!cF.m_bLockInX)
	{
		nDX = cF.m_nT - cF.m_nD;
	}

	if (nDX != 0)
	{
		nDX = nDX / abs(nDX);
		nDY = 0;
		//vector<int> vMin;
		//vector<int> vMax;
		/*
		for (int i = 0; i < vBoundingBox.size(); i++)
		{
			vector<instance_C *> &vInst = vBoundingBox[i];
			if( vInst.size() == 0 )
				continue;

			int nMax;
			int nMin;
			nMax = m_nDX;
			nMin = m_nTX;

			for (int j = 0; j < vInst.size(); j++)
			{
				instance_C *pTmpInst = vInst[j];
				nMax = max(pTmpInst->getPlacedX(), nMax);
				nMin = min(pTmpInst->getPlacedX(), nMin);
			}
			vMinX.push_back(nMin);
			vMaxX.push_back(nMax);
		}
		*/
		// moving instance
		int nBest;
		int nTmp;
		nTmp = cF.m_pInstance->getPlacedX();

		nBest = boundingBoxCost(nTmp, vMinX, vMaxX);
		gGrid_C *pBestGrid = NULL;
		gGrid_C *pGrid = getGrid(m_pDesign, cF.m_pInstance->getPlacedX(), cF.m_pInstance->getPlacedY(), m_nDZ);
		while (graphTravel(m_pDesign, pGrid, nDX, nDY, 0) != NULL)
		{
			pGrid = graphTravel(m_pDesign, pGrid, nDX, nDY, 0);
			// check placeable
			//if( !isPlaceable( pGrid, pInst ) )
			//	continue;
			//
			int nTmpX, nTmpY, nTmpZ;
			pGrid->getPosition(nTmpX, nTmpY, nTmpZ);
			nTmp = nTmpX;

			int nTmpBest = boundingBoxCost(nTmp, vMinX, vMaxX);
			// change at 0706 19:30
			//if( nTmpBest <= nBest ) // change from <
			//{
			//	nBest = nTmpBest;
			//	pBestGrid = pGrid;
			//	vBestGrid.push_back( pBestGrid );
			//}
			//else
			//	break;

			if (nTmpBest <= nBest) // change from <
			{
				if( nBest - nTmpBest > nXGain )
					nXGain = nBest - nTmpBest;
				pBestGrid = pGrid;
				nBX2 = nBX2 + nDX;
			}
			else
				break;

			// end change
			//cout<<nBest<<endl;
		}
	}

	if (!cF.m_bLockInY)
		nDY = cF.m_nR - cF.m_nL;

	if (nDY != 0)
	{
		nDY = nDY / abs(nDY);
		nDX = 0;
		// collecting all the correlated network
		//vector<int> vMin;
		//vector<int> vMax;
		/*
		for (int i = 0; i < vBoundingBox.size(); i++)
		{
			vector<instance_C *> &vInst = vBoundingBox[i];
			if( vInst.size() == 0 )
				continue;
			int nMax;
			int nMin;
			nMax = m_nDY;
			nMin = m_nTY;

			for (int j = 0; j < vInst.size(); j++)
			{
				instance_C *pTmpInst = vInst[j];
				nMax = max(pTmpInst->getPlacedY(), nMax);
				nMin = min(pTmpInst->getPlacedY(), nMin);
			}
			vMinY.push_back(nMin);
			vMaxY.push_back(nMax);
		}
		*/
		// moving instance
		int nBest;
		int nTmp;
		nTmp = cF.m_pInstance->getPlacedY();

		nBest = boundingBoxCost(nTmp, vMinY, vMaxY);
		gGrid_C *pBestGrid = NULL;
		gGrid_C *pGrid = getGrid(m_pDesign, cF.m_pInstance->getPlacedX(), cF.m_pInstance->getPlacedY(), m_nDZ);
		while (graphTravel(m_pDesign, pGrid, nDX, nDY, 0) != NULL)
		{
			pGrid = graphTravel(m_pDesign, pGrid, nDX, nDY, 0);
			// check placeable
			//if( !isPlaceable( pGrid, pInst ) )
			//	continue;
			//
			int nTmpX, nTmpY, nTmpZ;
			pGrid->getPosition(nTmpX, nTmpY, nTmpZ);
			nTmp = nTmpY;

			int nTmpBest = boundingBoxCost(nTmp, vMinY, vMaxY);
			// change at 0706 19:30
			//if( nTmpBest <= nBest ) // change from <
			//{
			//	nBest = nTmpBest;
			//	pBestGrid = pGrid;
			//	vBestGrid.push_back( pBestGrid );
			//}
			//else
			//	break;

			if (nTmpBest <= nBest) // change from <
			{
				if( nBest - nTmpBest > nYGain )
					nYGain = nBest - nTmpBest;
				pBestGrid = pGrid;
				nBY2 = nBY2 + nDY;
			}
			else
				break;

			// end change
			//cout<<nBest<<endl;
		}
	}
	nGain = nXGain + nYGain;
	cF.m_nGain = nGain;
	/*	
	if( !cF.m_bLockInX && !cF.m_bLockInY && cF.m_nR - cF.m_nL == 0 && cF.m_nT - cF.m_nD == 0 )
	{
		nDX = 1;
		nDY = 0;
		//vector<int> vMin;
		//vector<int> vMax;
		for( int i=0; i<vBoundingBox.size(); i++ )
		{
			vector< instance_C* > &vInst = vBoundingBox[i];
			int nMax;
			int nMin;
			nMax = m_nDX;
			nMin = m_nTX;


			for( int j=0; j<vInst.size(); j++ )
			{
				instance_C* pTmpInst = vInst[j];
				nMax = max( pTmpInst->getPlacedX(), nMax );
				nMin = min( pTmpInst->getPlacedX(), nMin );
			}
			vMinX.push_back( nMin );
			vMaxX.push_back( nMax );
		}
		// moving instance
		int nBest;
		int nTmp;
		nTmp = cF.m_pInstance->getPlacedX();

		nBest = boundingBoxCost( nTmp, vMinX, vMaxX );
		gGrid_C* pBestGrid = NULL;
		gGrid_C* pGrid = getGrid( m_pDesign, cF.m_pInstance->getPlacedX(), cF.m_pInstance->getPlacedY(), m_nDZ);
		while( graphTravel( m_pDesign, pGrid, nDX, nDY, 0 ) != NULL )
		{
			pGrid = graphTravel( m_pDesign, pGrid, nDX, nDY, 0 );
			// check placeable
			//if( !isPlaceable( pGrid, pInst ) )
			//	continue;
			// 
			int nTmpX, nTmpY, nTmpZ;
			pGrid->getPosition( nTmpX, nTmpY, nTmpZ );
			nTmp = nTmpX;
			
			int nTmpBest = boundingBoxCost( nTmp, vMinX, vMaxX );
			// change at 0706 19:30
			//if( nTmpBest <= nBest ) // change from <
			//{
			//	nBest = nTmpBest;
			//	pBestGrid = pGrid;
			//	vBestGrid.push_back( pBestGrid );
			//}
			//else
			//	break;
			
			if( nTmpBest <= nBest ) // change from <
			{
				pBestGrid = pGrid;
				nBX2 = nBX2 + nDX;	
			}
			else
				break;
			
			// end change
			//cout<<nBest<<endl;
		}
		nDX = -1;
		nTmp = cF.m_pInstance->getPlacedX();

		nBest = boundingBoxCost( nTmp, vMinX, vMaxX );
		pBestGrid = NULL;
		pGrid = getGrid( m_pDesign, cF.m_pInstance->getPlacedX(), cF.m_pInstance->getPlacedY(), m_nDZ);
		while( graphTravel( m_pDesign, pGrid, nDX, nDY, 0 ) != NULL )
		{
			pGrid = graphTravel( m_pDesign, pGrid, nDX, nDY, 0 );
			// check placeable
			//if( !isPlaceable( pGrid, pInst ) )
			//	continue;
			// 
			int nTmpX, nTmpY, nTmpZ;
			pGrid->getPosition( nTmpX, nTmpY, nTmpZ );
			nTmp = nTmpX;
			
			int nTmpBest = boundingBoxCost( nTmp, vMinX, vMaxX );
			// change at 0706 19:30
			//if( nTmpBest <= nBest ) // change from <
			//{
			//	nBest = nTmpBest;
			//	pBestGrid = pGrid;
			//	vBestGrid.push_back( pBestGrid );
			//}
			//else
			//	break;
			
			if( nTmpBest <= nBest ) // change from <
			{
				pBestGrid = pGrid;
				nBX1 = nBX1 + nDX;	
			}
			else
				break;
			
			// end change
			//cout<<nBest<<endl;
		}
	
		nDX = 0;
		nDY = 1;
		for( int i=0; i<vBoundingBox.size(); i++ )
		{
			vector< instance_C* > &vInst = vBoundingBox[i];
			int nMax;
			int nMin;
			nMax = m_nDY;
			nMin = m_nTY;


			for( int j=0; j<vInst.size(); j++ )
			{
				instance_C* pTmpInst = vInst[j];
				nMax = max( pTmpInst->getPlacedY(), nMax );
				nMin = min( pTmpInst->getPlacedY(), nMin );
			}
			vMinY.push_back( nMin );
			vMaxY.push_back( nMax );
		}
		// moving instance
		nTmp = cF.m_pInstance->getPlacedY();

		nBest = boundingBoxCost( nTmp, vMinY, vMaxY );
		pBestGrid = NULL;
		pGrid = getGrid( m_pDesign, cF.m_pInstance->getPlacedX(), cF.m_pInstance->getPlacedY(), m_nDZ);
		while( graphTravel( m_pDesign, pGrid, nDX, nDY, 0 ) != NULL )
		{
			pGrid = graphTravel( m_pDesign, pGrid, nDX, nDY, 0 );
			// check placeable
			//if( !isPlaceable( pGrid, pInst ) )
			//	continue;
			// 
			int nTmpX, nTmpY, nTmpZ;
			pGrid->getPosition( nTmpX, nTmpY, nTmpZ );
			nTmp = nTmpY;
			
			int nTmpBest = boundingBoxCost( nTmp, vMinY, vMaxY );
			// change at 0706 19:30
			//if( nTmpBest <= nBest ) // change from <
			//{
			//	nBest = nTmpBest;
			//	pBestGrid = pGrid;
			//	vBestGrid.push_back( pBestGrid );
			//}
			//else
			//	break;
			
			if( nTmpBest <= nBest ) // change from <
			{
				pBestGrid = pGrid;
				nBY2 = nBY2 + nDY;
			}
			else
				break;
			
			// end change
			//cout<<nBest<<endl;
		}
		
		nDY = -1;
		nTmp = cF.m_pInstance->getPlacedY();

		nBest = boundingBoxCost( nTmp, vMinY, vMaxY );
		pBestGrid = NULL;
		pGrid = getGrid( m_pDesign, cF.m_pInstance->getPlacedX(), cF.m_pInstance->getPlacedY(), m_nDZ);
		while( graphTravel( m_pDesign, pGrid, nDX, nDY, 0 ) != NULL )
		{
			pGrid = graphTravel( m_pDesign, pGrid, nDX, nDY, 0 );
			// check placeable
			//if( !isPlaceable( pGrid, pInst ) )
			//	continue;
			// 
			int nTmpX, nTmpY, nTmpZ;
			pGrid->getPosition( nTmpX, nTmpY, nTmpZ );
			nTmp = nTmpY;
			
			int nTmpBest = boundingBoxCost( nTmp, vMinY, vMaxY );
			// change at 0706 19:30
			//if( nTmpBest <= nBest ) // change from <
			//{
			//	nBest = nTmpBest;
			//	pBestGrid = pGrid;
			//	vBestGrid.push_back( pBestGrid );
			//}
			//else
			//	break;
			
			if( nTmpBest <= nBest ) // change from <
			{
				pBestGrid = pGrid;
				nBY1 = nBY1 + nDY;
			}
			else
				break;
			
			// end change
			//cout<<nBest<<endl;
		}
	}
	*/
	if( nBX1 > nBX2 )
	{
		int nTmpX = nBX1;
		nBX1 = nBX2;
		nBX2 = nTmpX;
	}
	
	if( nBY1 > nBY2 )
	{
		int nTmpY = nBY1;
		nBY1 = nBY2;
		nBY2 = nTmpY;
	}
	/*
	//if( nBY1 - nBY2 != 0 && nBX1 - nBX2 == 0 )
	//{
		int nDDY = abs( nBY1 - nBY2 )/2;
		if( nBX1 - nDDY >= m_nDX )
			nBX1 = nBX1 - nDDY;
		if( nBX2 + nDDY <= m_nTX )
			nBX2 = nBX2 + nDDY;
	//}

	//if( nBX1 - nBX2 != 0 && nBY1 - nBY2 == 0 )
	//{
		int nDDX = abs( nBX1 - nBX2 )/2;
		if( nBY1 - nDDX >= m_nDY )
			nBY1 = nBY1 - nDDX;
		if( nBY2 + nDDX <= m_nTY )
			nBY2 = nBY2 + nDDX;
	//}
	*/
	/*
	else if( nBX1 - nBX2 != 0 && nBY1 - nBY2 != 0 )
	{
		int nDDY = abs( nBY1 - nBY2 );
		if( nBX1 - nDDY >= m_nDX )
			nBX1 = nBX1 - nDDY;
		if( nBX2 + nDDY <= m_nTX )
			nBX2 = nBX2 + nDDY;
		
		int nDDX = abs( nBX1 - nBX2 );
		if( nBY1 - nDDX >= m_nDY )
			nBY1 = nBY1 - nDDX;
		if( nBY2 + nDDX <= m_nTY )
			nBY2 = nBY2 + nDDX;
		
	}
	*/
	//if (nBX1 < nBX2)
	//{
		cF.m_nBoundX1 = nBX1;
		cF.m_nBoundX2 = nBX2;
	//}
	//else
	//{
	//	cF.m_nBoundX1 = nBX2;
	//	cF.m_nBoundX2 = nBX1;
	//}

	//if (nBY1 < nBY2)
	//{
		cF.m_nBoundY1 = nBY1;
		cF.m_nBoundY2 = nBY2;
	//}
	//else
	//{
	//	cF.m_nBoundY1 = nBY2;
	//	cF.m_nBoundY2 = nBY1;
	//}
	
	//cF.m_vMinX.clear();
	//cF.m_vMaxX.clear();

	//cF.m_vMinY.clear();
	//cF.m_vMaxY.clear();

	cF.m_vMinX = vMinX;
	cF.m_vMaxX = vMaxX;

	cF.m_vMinY = vMinY;
	cF.m_vMaxY = vMaxY;

	//if( cF.m_pInstance->getName() == "C1200" )
	//	cout<<"Range: "<<cF.m_nBoundX1<<" "<<cF.m_nBoundY1<<" "<<cF.m_nBoundX2<<" "<<cF.m_nBoundY2<<endl;
	//cout << cF.m_pInstance->getName()  << " " << abs( nDF - nTF ) + abs( nRF - nLF ) << endl;
	return true;
}

bool router_C::calForcedModel_ver4(forced_C &cF)
{
	int nX = cF.m_pInstance->getPlacedX();
	int nY = cF.m_pInstance->getPlacedY();

	int nTF = 0;
	int nDF = 0;
	int nRF = 0;
	int nLF = 0;
	int nCX = 0;
	int nCY = 0;

	//cout << "Forced: " << nTF << " " << nDF << " " << nRF << " " << nLF << " " << nCF << endl;
	// added at 0712 22:00
	int nBX1, nBX2;
	int nBY1, nBY2;
	nBX1 = cF.m_pInstance->getPlacedX();
	nBY1 = cF.m_pInstance->getPlacedY();
	nBX2 = nBX1;
	nBY2 = nBY1;

	vector<int> vMinX;
	vector<int> vMaxX;

	vector<int> vMinY;
	vector<int> vMaxY;
	//cout << "cal bound"<<endl;
	vector<networkForced_C *> &vNetWork = cF.m_vNetwork;
	vector<vector<instance_C *>> vBoundingBox;
	for (int i = 0; i < vNetWork.size(); i++)
	{
		networkForced_C *pNF = vNetWork[i];
		vMinX.push_back( pNF->m_mMinX.find( &cF )->second );
		vMaxX.push_back( pNF->m_mMaxX.find( &cF )->second );
		vMinY.push_back( pNF->m_mMinY.find( &cF )->second );
		vMaxY.push_back( pNF->m_mMaxY.find( &cF )->second );
		if( pNF->m_mMinX.find( &cF )->second > nX  )
		{
			nTF++;
		}
		if( pNF->m_mMaxX.find( &cF )->second < nX  )
		{
			nDF++;
		}
		if( pNF->m_mMinY.find( &cF )->second > nY  )
		{
			nRF++;
		}
		if( pNF->m_mMaxY.find( &cF )->second < nY  )
		{
			nLF++;
		}
		if( pNF->m_nMinX == pNF->m_nMaxX )
		{
			nCX++;
		}
		if( pNF->m_nMinY == pNF->m_nMaxY )
		{
			nCY++;
		}
		
	}
	
	cF.m_nT = nTF;
	cF.m_nD = nDF;
	cF.m_nR = nRF;
	cF.m_nL = nLF;
	cF.m_nCX = nCX;
	cF.m_nCY = nCY;
	/*
	cout << cF.m_pInstance->getName()<<endl;
	for( int i=0; i<vMinX.size(); i++ )
	{
		cout << vMinX[i] << " " << vMinY[i] << " " << vMaxX[i] << " " << vMaxY[i] << endl;
	}
	cout << "Forced: " << nTF << " " << nDF << " " << nRF << " " << nLF << " " << nCF << endl;
	*/
	//cout << "end bound"<<endl;
	int nGain = 0;
	int nXGain = 0;
	int nYGain = 0;
	bool bBalanceInX = false;
	bool bBalanceInY = false;

	int nDX = 0;
	int nDY = 0;
	nDX = -1;
	nDY = 0;
	int nBest;
	int nTmp;
	nTmp = cF.m_pInstance->getPlacedX();

	nBest = boundingBoxCost(nTmp, vMinX, vMaxX);
	gGrid_C *pBestGrid = NULL;
	gGrid_C *pGrid = getGrid(m_pDesign, cF.m_pInstance->getPlacedX(), cF.m_pInstance->getPlacedY(), m_nDZ);
	while (graphTravel(m_pDesign, pGrid, nDX, nDY, 0) != NULL)
	{
		pGrid = graphTravel(m_pDesign, pGrid, nDX, nDY, 0);
		int nTmpX, nTmpY, nTmpZ;
		pGrid->getPosition(nTmpX, nTmpY, nTmpZ);
		nTmp = nTmpX;

		int nTmpBest = boundingBoxCost(nTmp, vMinX, vMaxX);

		if (nTmpBest <= nBest) // change from <
		{
			pBestGrid = pGrid;
			nBX1 = nBX1 + nDX;
		}
		else
			break;
	}
	
	nDX = 1;
	nDY = 0;
	nTmp = cF.m_pInstance->getPlacedX();

	nBest = boundingBoxCost(nTmp, vMinX, vMaxX);
	pBestGrid = NULL;
	pGrid = getGrid(m_pDesign, cF.m_pInstance->getPlacedX(), cF.m_pInstance->getPlacedY(), m_nDZ);
	while (graphTravel(m_pDesign, pGrid, nDX, nDY, 0) != NULL)
	{
		pGrid = graphTravel(m_pDesign, pGrid, nDX, nDY, 0);
		int nTmpX, nTmpY, nTmpZ;
		pGrid->getPosition(nTmpX, nTmpY, nTmpZ);
		nTmp = nTmpX;

		int nTmpBest = boundingBoxCost(nTmp, vMinX, vMaxX);

		if (nTmpBest <= nBest) // change from <
		{
			pBestGrid = pGrid;
			nBX2 = nBX2 + nDX;
		}
		else
			break;
	}
	
	nDY = -1;
	nDX = 0;
	nTmp = cF.m_pInstance->getPlacedY();

	nBest = boundingBoxCost(nTmp, vMinY, vMaxY);
	pBestGrid = NULL;
	pGrid = getGrid(m_pDesign, cF.m_pInstance->getPlacedX(), cF.m_pInstance->getPlacedY(), m_nDZ);
	while (graphTravel(m_pDesign, pGrid, nDX, nDY, 0) != NULL)
	{
		pGrid = graphTravel(m_pDesign, pGrid, nDX, nDY, 0);
		int nTmpX, nTmpY, nTmpZ;
		pGrid->getPosition(nTmpX, nTmpY, nTmpZ);
		nTmp = nTmpY;

		int nTmpBest = boundingBoxCost(nTmp, vMinY, vMaxY);

		if (nTmpBest <= nBest) // change from <
		{
			pBestGrid = pGrid;
			nBY1 = nBY1 + nDY;
		}
		else
			break;
	}

	nDY = 1;
	nDX = 0;
	nTmp = cF.m_pInstance->getPlacedY();

	nBest = boundingBoxCost(nTmp, vMinY, vMaxY);
	pBestGrid = NULL;
	pGrid = getGrid(m_pDesign, cF.m_pInstance->getPlacedX(), cF.m_pInstance->getPlacedY(), m_nDZ);
	while (graphTravel(m_pDesign, pGrid, nDX, nDY, 0) != NULL)
	{
		pGrid = graphTravel(m_pDesign, pGrid, nDX, nDY, 0);
		int nTmpX, nTmpY, nTmpZ;
		pGrid->getPosition(nTmpX, nTmpY, nTmpZ);
		nTmp = nTmpY;

		int nTmpBest = boundingBoxCost(nTmp, vMinY, vMaxY);

		if (nTmpBest <= nBest) // change from <
		{
			pBestGrid = pGrid;
			nBY2 = nBY2 + nDY;
		}
		else
			break;
	}
	
	//nGain = nXGain + nYGain;
	cF.m_nGain = nGain;
	if( nBX1 > nBX2 )
	{
		int nTmpX = nBX1;
		nBX1 = nBX2;
		nBX2 = nTmpX;
	}
	
	if( nBY1 > nBY2 )
	{
		int nTmpY = nBY1;
		nBY1 = nBY2;
		nBY2 = nTmpY;
	}
	//cout << nBX1 << " " << nBY1 << " " << nBX2 << " " << nBY2 << endl;
	cF.m_nBoundX1 = nBX1;
	cF.m_nBoundX2 = nBX2;
	cF.m_nBoundY1 = nBY1;
	cF.m_nBoundY2 = nBY2;
	cF.m_vMinX = vMinX;
	cF.m_vMaxX = vMaxX;

	cF.m_vMinY = vMinY;
	cF.m_vMaxY = vMaxY;
	
	return true;
}


bool router_C::calForcedModel_ver3(forced_C &cF)
{
	int nX = cF.m_pInstance->getPlacedX();
	int nY = cF.m_pInstance->getPlacedY();

	int nTF = 0;
	int nDF = 0;
	int nRF = 0;
	int nLF = 0;
	int nCF = 0;

	for (int i = 0; i < cF.m_vNetwork.size(); i++)
	{
		networkForced_C *cNF = cF.m_vNetwork[i];
		bool bNoForcedInX = false;
		bool bNoForcedInY = false;

		int nCenterX = cNF->m_nCenterX;
		int nCenterY = cNF->m_nCenterY;
		int nHalfX = cNF->m_nXHalf;
		int nHalfY = cNF->m_nYHalf;
		//cout << cNF->m_pNet->getName() << " " << nCenterX << " " << nHalfX << " " << nCenterY << " " << nHalfY << endl;
		if( nCenterX > nX )
		{
			nTF++;
		}
		else if( nCenterX < nX )
		{
			nDF++;
		}
		else if( nCenterX == nX && nHalfX == 1 )
		{
			nTF++;
		}
		else
		{
			bNoForcedInX = true;
		}	
		
		if( nCenterY > nY )
		{
			nRF++;
		}
		else if( nCenterY < nY )
		{
			nLF++;
		}
		else if( nCenterY == nY && nHalfY == 1 )
		{
			nRF++;
		}
		else
		{
			bNoForcedInY = true;
		}	

		if( bNoForcedInX && bNoForcedInY )
			nCF++;

	}
	

	//cout << "Forced: " << nTF << " " << nDF << " " << nRF << " " << nLF << " " << nCF << endl;
	// added at 0712 22:00
	int nBX1, nBX2;
	int nBY1, nBY2;
	nBX1 = cF.m_pInstance->getPlacedX();
	nBY1 = cF.m_pInstance->getPlacedY();
	nBX2 = nBX1;
	nBY2 = nBY1;

	vector<int> vMinX;
	vector<int> vMaxX;

	vector<int> vMinY;
	vector<int> vMaxY;
	//cout << "cal bound"<<endl;
	vector<networkForced_C *> &vNetWork = cF.m_vNetwork;
	vector<vector<instance_C *>> vBoundingBox;
	for (int i = 0; i < vNetWork.size(); i++)
	{
		networkForced_C *pNF = vNetWork[i];
		vMinX.push_back( pNF->m_mMinX.find( &cF )->second );
		vMaxX.push_back( pNF->m_mMaxX.find( &cF )->second );
		vMinY.push_back( pNF->m_mMinY.find( &cF )->second );
		vMaxY.push_back( pNF->m_mMaxY.find( &cF )->second );
		if( pNF->m_mMinX.find( &cF )->second > nX  )
		{
			nTF++;
		}
		if( pNF->m_mMaxX.find( &cF )->second < nX  )
		{
			nDF++;
		}
		if( pNF->m_mMinY.find( &cF )->second > nY  )
		{
			nRF++;
		}
		if( pNF->m_mMaxY.find( &cF )->second < nY  )
		{
			nLF++;
		}
		
	}
	
	cF.m_nT = nTF;
	cF.m_nD = nDF;
	cF.m_nR = nRF;
	cF.m_nL = nLF;
	cF.m_nC = nCF;
	/*
	cout << cF.m_pInstance->getName()<<endl;
	for( int i=0; i<vMinX.size(); i++ )
	{
		cout << vMinX[i] << " " << vMinY[i] << " " << vMaxX[i] << " " << vMaxY[i] << endl;
	}
	cout << "Forced: " << nTF << " " << nDF << " " << nRF << " " << nLF << " " << nCF << endl;
	*/
	//cout << "end bound"<<endl;
	int nGain = 0;
	int nXGain = 0;
	int nYGain = 0;
	bool bBalanceInX = false;
	bool bBalanceInY = false;

	int nDX = 0;
	int nDY = 0;
	if (!cF.m_bLockInX)
	{
		nDX = cF.m_nT - cF.m_nD;
	}

	if( abs( cF.m_nT - cF.m_nD ) - cF.m_nC != 0 && nDX != 0 )
	{
		nDX = nDX / abs(nDX);
		nDY = 0;
		//cout << "Move direction in X: "<< nDX << endl;
		// moving instance
		int nBest;
		int nTmp;
		nTmp = cF.m_pInstance->getPlacedX();

		nBest = boundingBoxCost(nTmp, vMinX, vMaxX);
		gGrid_C *pBestGrid = NULL;
		gGrid_C *pGrid = getGrid(m_pDesign, cF.m_pInstance->getPlacedX(), cF.m_pInstance->getPlacedY(), m_nDZ);
		while (graphTravel(m_pDesign, pGrid, nDX, nDY, 0) != NULL)
		{
			pGrid = graphTravel(m_pDesign, pGrid, nDX, nDY, 0);
			// check placeable
			//if( !isPlaceable( pGrid, pInst ) )
			//	continue;
			//
			int nTmpX, nTmpY, nTmpZ;
			pGrid->getPosition(nTmpX, nTmpY, nTmpZ);
			nTmp = nTmpX;

			int nTmpBest = boundingBoxCost(nTmp, vMinX, vMaxX);
			// change at 0706 19:30
			//if( nTmpBest <= nBest ) // change from <
			//{
			//	nBest = nTmpBest;
			//	pBestGrid = pGrid;
			//	vBestGrid.push_back( pBestGrid );
			//}
			//else
			//	break;

			if (nTmpBest <= nBest) // change from <
			{
	//			if( nBest - nTmpBest > nXGain )
	//				nXGain = nBest - nTmpBest;
				pBestGrid = pGrid;
				nBX2 = nBX2 + nDX;
			}
			else
				break;

			// end change
			//cout<<nBest<<endl;
		}
	}
	else
	{
		bBalanceInX = true;
	}

	if (!cF.m_bLockInY)
		nDY = cF.m_nR - cF.m_nL;

	if( abs( cF.m_nR - cF.m_nL ) - cF.m_nC != 0 && nDY != 0 )
	{
		nDY = nDY / abs(nDY);
		nDX = 0;
		//cout << "Move direction in y: "<< nDY << endl;
		// moving instance
		int nBest;
		int nTmp;
		nTmp = cF.m_pInstance->getPlacedY();

		nBest = boundingBoxCost(nTmp, vMinY, vMaxY);
		gGrid_C *pBestGrid = NULL;
		gGrid_C *pGrid = getGrid(m_pDesign, cF.m_pInstance->getPlacedX(), cF.m_pInstance->getPlacedY(), m_nDZ);
		while (graphTravel(m_pDesign, pGrid, nDX, nDY, 0) != NULL)
		{
			pGrid = graphTravel(m_pDesign, pGrid, nDX, nDY, 0);
			// check placeable
			//if( !isPlaceable( pGrid, pInst ) )
			//	continue;
			//
			int nTmpX, nTmpY, nTmpZ;
			pGrid->getPosition(nTmpX, nTmpY, nTmpZ);
			nTmp = nTmpY;

			int nTmpBest = boundingBoxCost(nTmp, vMinY, vMaxY);
			// change at 0706 19:30
			//if( nTmpBest <= nBest ) // change from <
			//{
			//	nBest = nTmpBest;
			//	pBestGrid = pGrid;
			//	vBestGrid.push_back( pBestGrid );
			//}
			//else
			//	break;

			if (nTmpBest <= nBest) // change from <
			{
	//			if( nBest - nTmpBest > nYGain )
	//				nYGain = nBest - nTmpBest;
				pBestGrid = pGrid;
				nBY2 = nBY2 + nDY;
			}
			else
				break;

			// end change
			//cout<<nBest<<endl;
		}
	}
	else
	{
		bBalanceInY = true;
	}

	if( bBalanceInX && bBalanceInY )
	{
		nDX = -1;
		nDY = 0;
		int nBest;
		int nTmp;
		nTmp = cF.m_pInstance->getPlacedX();

		nBest = boundingBoxCost(nTmp, vMinX, vMaxX);
		gGrid_C *pBestGrid = NULL;
		gGrid_C *pGrid = getGrid(m_pDesign, cF.m_pInstance->getPlacedX(), cF.m_pInstance->getPlacedY(), m_nDZ);
		while (graphTravel(m_pDesign, pGrid, nDX, nDY, 0) != NULL)
		{
			pGrid = graphTravel(m_pDesign, pGrid, nDX, nDY, 0);
			int nTmpX, nTmpY, nTmpZ;
			pGrid->getPosition(nTmpX, nTmpY, nTmpZ);
			nTmp = nTmpX;

			int nTmpBest = boundingBoxCost(nTmp, vMinX, vMaxX);

			if (nTmpBest <= nBest) // change from <
			{
				pBestGrid = pGrid;
				nBX1 = nBX1 + nDX;
			}
			else
				break;
		}
		
		nDX = 1;
		nDY = 0;
		nTmp = cF.m_pInstance->getPlacedX();

		nBest = boundingBoxCost(nTmp, vMinX, vMaxX);
		pBestGrid = NULL;
		pGrid = getGrid(m_pDesign, cF.m_pInstance->getPlacedX(), cF.m_pInstance->getPlacedY(), m_nDZ);
		while (graphTravel(m_pDesign, pGrid, nDX, nDY, 0) != NULL)
		{
			pGrid = graphTravel(m_pDesign, pGrid, nDX, nDY, 0);
			int nTmpX, nTmpY, nTmpZ;
			pGrid->getPosition(nTmpX, nTmpY, nTmpZ);
			nTmp = nTmpX;

			int nTmpBest = boundingBoxCost(nTmp, vMinX, vMaxX);

			if (nTmpBest <= nBest) // change from <
			{
				pBestGrid = pGrid;
				nBX2 = nBX2 + nDX;
			}
			else
				break;
		}
		
		nDY = -1;
		nDX = 0;
		nTmp = cF.m_pInstance->getPlacedY();

		nBest = boundingBoxCost(nTmp, vMinY, vMaxY);
		pBestGrid = NULL;
		pGrid = getGrid(m_pDesign, cF.m_pInstance->getPlacedX(), cF.m_pInstance->getPlacedY(), m_nDZ);
		while (graphTravel(m_pDesign, pGrid, nDX, nDY, 0) != NULL)
		{
			pGrid = graphTravel(m_pDesign, pGrid, nDX, nDY, 0);
			int nTmpX, nTmpY, nTmpZ;
			pGrid->getPosition(nTmpX, nTmpY, nTmpZ);
			nTmp = nTmpY;

			int nTmpBest = boundingBoxCost(nTmp, vMinY, vMaxY);

			if (nTmpBest <= nBest) // change from <
			{
				pBestGrid = pGrid;
				nBY1 = nBY1 + nDY;
			}
			else
				break;
		}
	
		nDY = 1;
		nDX = 0;
		nTmp = cF.m_pInstance->getPlacedY();

		nBest = boundingBoxCost(nTmp, vMinY, vMaxY);
		pBestGrid = NULL;
		pGrid = getGrid(m_pDesign, cF.m_pInstance->getPlacedX(), cF.m_pInstance->getPlacedY(), m_nDZ);
		while (graphTravel(m_pDesign, pGrid, nDX, nDY, 0) != NULL)
		{
			pGrid = graphTravel(m_pDesign, pGrid, nDX, nDY, 0);
			int nTmpX, nTmpY, nTmpZ;
			pGrid->getPosition(nTmpX, nTmpY, nTmpZ);
			nTmp = nTmpY;

			int nTmpBest = boundingBoxCost(nTmp, vMinY, vMaxY);

			if (nTmpBest <= nBest) // change from <
			{
				pBestGrid = pGrid;
				nBY2 = nBY2 + nDY;
			}
			else
				break;
		}
	
	}


	//nGain = nXGain + nYGain;
	cF.m_nGain = nGain;
	if( nBX1 > nBX2 )
	{
		int nTmpX = nBX1;
		nBX1 = nBX2;
		nBX2 = nTmpX;
	}
	
	if( nBY1 > nBY2 )
	{
		int nTmpY = nBY1;
		nBY1 = nBY2;
		nBY2 = nTmpY;
	}
	//cout << nBX1 << " " << nBY1 << " " << nBX2 << " " << nBY2 << endl;
	cF.m_nBoundX1 = nBX1;
	cF.m_nBoundX2 = nBX2;
	cF.m_nBoundY1 = nBY1;
	cF.m_nBoundY2 = nBY2;
	cF.m_vMinX = vMinX;
	cF.m_vMaxX = vMaxX;

	cF.m_vMinY = vMinY;
	cF.m_vMaxY = vMaxY;
	
	return true;
}


bool router_C::calForcedModel_ver2(forced_C &cF)
{
	int nX = cF.m_pInstance->getPlacedX();
	int nY = cF.m_pInstance->getPlacedY();

	vector<int> vMinX;
	vector<int> vMaxX;

	vector<int> vMinY;
	vector<int> vMaxY;

	vector<networkForced_C *> &vNetWork = cF.m_vNetwork;

	for( int i=0; i<vNetWork.size(); i++ )
	{
		networkForced_C* pNF = vNetWork[i];
		if( pNF->m_nMinXCount == 0 || pNF->m_nMinYCount == 0 || pNF->m_nMaxXCount == 0 || pNF->m_nMaxYCount == 0 )
			continue;
		
		//cout << pNF->m_pNet->getName() << endl;
		//cout << pNF->m_nMinX << " " << pNF->m_nMinY << endl;
		//if( pNF->m_nMinXCount == 1 && pNF->m_n2MinX != INT_MAX )
		if( pNF->m_nMinXCount == 1 )
			vMinX.push_back( pNF->m_n2MinX );
		else
			vMinX.push_back( pNF->m_nMinX );
		
		//if( pNF->m_nMaxXCount == 1 && pNF->m_n2MaxX != INT_MIN )
		if( pNF->m_nMaxXCount == 1 )
			vMaxX.push_back( pNF->m_n2MaxX );
		else
			vMaxX.push_back( pNF->m_nMaxX );
		

		//if( pNF->m_nMinYCount == 1 && pNF->m_n2MinY != INT_MAX )
		if( pNF->m_nMinYCount == 1 )
			vMinY.push_back( pNF->m_n2MinY );
		else
			vMinY.push_back( pNF->m_nMinY );
		
		//if( pNF->m_nMaxYCount == 1 && pNF->m_n2MaxY != INT_MIN )
		if( pNF->m_nMaxYCount == 1 )
			vMaxY.push_back( pNF->m_n2MaxY );
		else
			vMaxY.push_back( pNF->m_nMaxY );
	}

	//cout << &cF << endl;

	//cout << vMinX.size() << endl;
	cF.m_vMinX = vMinX;
	cF.m_vMaxX = vMaxX;

	//cout << vMinY.size() << endl;
	cF.m_vMinY = vMinY;
	cF.m_vMaxY = vMaxY;

	return true;
}


bool router_C::calForcedModel(forced_C &cF)
{
	int nX = cF.m_pInstance->getPlacedX();
	int nY = cF.m_pInstance->getPlacedY();

	int nTF = 0;
	int nDF = 0;
	int nRF = 0;
	int nLF = 0;

	int nCenterX = ( m_nDX + m_nTX )/2;
	int nCenterY = ( m_nDY + m_nTY )/2;
	/*	
	if( nX < nCenterX )
		nDF++;
	else if( nX > nCenterX )
		nTF++;

	if( nY < nCenterY )
		nLF++;
	else if( nY > nCenterY )
		nRF++;
	*/
	// begin logger 0812
	//int nTF = cF.m_nT;
	//int nDF = cF.m_nD;
	//int nRF = cF.m_nR;
	//int nLF = cF.m_nL;
	// end logger 0812
	/*
	for( int i = 0; i < cF.m_vNetwork.size(); i++ )
	{
		networkForced_C *cNF = cF.m_vNetwork[i];
		nTF = nTF + cNF->m_mPTF.find( &cF )->second;
		nDF = nDF + cNF->m_mPDF.find( &cF )->second;
		nRF = nRF + cNF->m_mPRF.find( &cF )->second;
		nLF = nLF + cNF->m_mPLF.find( &cF )->second;
	}
	*/
	for (int i = 0; i < cF.m_vNetwork.size(); i++)
	{
		networkForced_C *cNF = cF.m_vNetwork[i];
		bool bNoForcedInX = false;
		bool bNoForcedInY = false;

		nTF = nTF + cNF->m_mPTF.find( &cF )->second;
		nDF = nDF + cNF->m_mPDF.find( &cF )->second;
		nRF = nRF + cNF->m_mPRF.find( &cF )->second;
		nLF = nLF + cNF->m_mPLF.find( &cF )->second;
		
		// second version
		if (cNF->m_nMaxY == cNF->m_nMinY && cNF->m_nMaxX == cNF->m_nMinX )
		{
			nRF--;
			nLF--;
			//nRF++;
			//nLF++;
		}
		else if (cNF->m_nMaxY == cNF->m_nMinY)
		{
			//nRF--;
			//nLF--;
			//nRF++;
			//nLF++;
		}
		//else 
		if (nY == cNF->m_nMaxY)
		{
			nRF--;
			if (cNF->m_nMaxYCount == 1)
				nLF = nLF + 1;
			if (cNF->m_nMaxXCount >= 1)
				nLF++;
		}
		//else 
		if (nY == cNF->m_nMinY)
		{
			nLF--;
			if (cNF->m_nMinYCount == 1)
				nRF = nRF + 1;
			if (cNF->m_nMinYCount >= 1)
				nRF++;
		}

		if (cNF->m_nMaxX == cNF->m_nMinX && cNF->m_nMaxY == cNF->m_nMinY )
		{
			nTF--;
			nDF--;
			//nTF++;
			//nDF++;
		}
		else if (cNF->m_nMaxX == cNF->m_nMinX)
		{
			//nTF--;
			//nDF--;
			//nTF++;
			//nDF++;
		}
		//else 
		if (nX == cNF->m_nMaxX)
		{
			nTF--;
			if (cNF->m_nMaxXCount == 1)
				nDF = nDF + 1;
			if (cNF->m_nMaxXCount >= 1)
				nDF++;
		}
		//else 
		if (nX == cNF->m_nMinX)
		{
			nDF--;
			if (cNF->m_nMinXCount == 1)
				nTF = nTF + 1;
			if (cNF->m_nMinXCount >= 1)
				nTF++;
		}
	}
	
	if (nTF < 0)
		nTF = 0;
	if (nDF < 0)
		nDF = 0;
	if (nRF < 0)
		nRF = 0;
	if (nLF < 0)
		nLF = 0;
	
	cF.m_nT = nTF;
	cF.m_nD = nDF;
	cF.m_nR = nRF;
	cF.m_nL = nLF;

	// added at 0712 22:00
	int nBX1, nBX2;
	int nBY1, nBY2;
	nBX1 = cF.m_pInstance->getPlacedX();
	nBY1 = cF.m_pInstance->getPlacedY();
	nBX2 = nBX1;
	nBY2 = nBY1;

	vector<int> vMinX;
	vector<int> vMaxX;

	vector<int> vMinY;
	vector<int> vMaxY;

	vector<networkForced_C *> &vNetWork = cF.m_vNetwork;
	vector<vector<instance_C *>> vBoundingBox;
	/*
	for (int i = 0; i < vNetWork.size(); i++)
	{
		vector<instance_C *> vInst;
		networkForced_C *pNF = vNetWork[i];
		vector<forced_C *> vF = pNF->m_vForced;
		for (int j = 0; j < vF.size(); j++)
		{
			instance_C *pTmpInst = vF[j]->m_pInstance;
			if (pTmpInst != cF.m_pInstance)
				vInst.push_back(pTmpInst);
		}
		vBoundingBox.push_back(vInst);
	}
	*/
	for (int i = 0; i < vNetWork.size(); i++)
	{
		networkForced_C *pNF = vNetWork[i];
		/*	
		if( pNF->m_mMinX.find( &cF )->second == nX && nX == pNF->m_mMaxX.find( &cF )->second 
		 && pNF->m_mMinY.find( &cF )->second == nY && nY == pNF->m_mMaxY.find( &cF )->second )
		 {
		 	continue;
		 }
		*/
		if( pNF->m_mMinX.find( &cF )->second == nX && nX == pNF->m_mMaxX.find( &cF )->second )
		{
			vMinX.push_back( pNF->m_nMinX );
			vMaxX.push_back( pNF->m_nMaxX );	
		}
		else
		{
			vMinX.push_back( pNF->m_mMinX.find( &cF )->second );
			vMaxX.push_back( pNF->m_mMaxX.find( &cF )->second );
		}
		if( pNF->m_mMinY.find( &cF )->second == nY && nY == pNF->m_mMaxY.find( &cF )->second )
		{
			vMinY.push_back( pNF->m_nMinY );
			vMaxY.push_back( pNF->m_nMaxY );
		}
		else
		{
			vMinY.push_back( pNF->m_mMinY.find( &cF )->second );
			vMaxY.push_back( pNF->m_mMaxY.find( &cF )->second );
		}
	}

	int nGain = 0;
	int nXGain = 0;
	int nYGain = 0;
	
	int nDX = 0;
	int nDY = 0;
	if (!cF.m_bLockInX)
	{
		nDX = cF.m_nT - cF.m_nD;
	}

	if (nDX != 0)
	{
		nDX = nDX / abs(nDX);
		nDY = 0;
		//vector<int> vMin;
		//vector<int> vMax;
		/*
		for (int i = 0; i < vBoundingBox.size(); i++)
		{
			vector<instance_C *> &vInst = vBoundingBox[i];
			if( vInst.size() == 0 )
				continue;

			int nMax;
			int nMin;
			nMax = m_nDX;
			nMin = m_nTX;

			for (int j = 0; j < vInst.size(); j++)
			{
				instance_C *pTmpInst = vInst[j];
				nMax = max(pTmpInst->getPlacedX(), nMax);
				nMin = min(pTmpInst->getPlacedX(), nMin);
			}
			vMinX.push_back(nMin);
			vMaxX.push_back(nMax);
		}
		*/
		// moving instance
		int nBest;
		int nTmp;
		nTmp = cF.m_pInstance->getPlacedX();

		nBest = boundingBoxCost(nTmp, vMinX, vMaxX);
		gGrid_C *pBestGrid = NULL;
		gGrid_C *pGrid = getGrid(m_pDesign, cF.m_pInstance->getPlacedX(), cF.m_pInstance->getPlacedY(), m_nDZ);
		while (graphTravel(m_pDesign, pGrid, nDX, nDY, 0) != NULL)
		{
			pGrid = graphTravel(m_pDesign, pGrid, nDX, nDY, 0);
			// check placeable
			//if( !isPlaceable( pGrid, pInst ) )
			//	continue;
			//
			int nTmpX, nTmpY, nTmpZ;
			pGrid->getPosition(nTmpX, nTmpY, nTmpZ);
			nTmp = nTmpX;

			int nTmpBest = boundingBoxCost(nTmp, vMinX, vMaxX);
			// change at 0706 19:30
			//if( nTmpBest <= nBest ) // change from <
			//{
			//	nBest = nTmpBest;
			//	pBestGrid = pGrid;
			//	vBestGrid.push_back( pBestGrid );
			//}
			//else
			//	break;

			if (nTmpBest <= nBest) // change from <
			{
				if( nBest - nTmpBest > nXGain )
					nXGain = nBest - nTmpBest;
				pBestGrid = pGrid;
				nBX2 = nBX2 + nDX;
			}
			else
				break;

			// end change
			//cout<<nBest<<endl;
		}
	}

	if (!cF.m_bLockInY)
		nDY = cF.m_nR - cF.m_nL;

	if (nDY != 0)
	{
		nDY = nDY / abs(nDY);
		nDX = 0;
		// collecting all the correlated network
		//vector<int> vMin;
		//vector<int> vMax;
		/*
		for (int i = 0; i < vBoundingBox.size(); i++)
		{
			vector<instance_C *> &vInst = vBoundingBox[i];
			if( vInst.size() == 0 )
				continue;
			int nMax;
			int nMin;
			nMax = m_nDY;
			nMin = m_nTY;

			for (int j = 0; j < vInst.size(); j++)
			{
				instance_C *pTmpInst = vInst[j];
				nMax = max(pTmpInst->getPlacedY(), nMax);
				nMin = min(pTmpInst->getPlacedY(), nMin);
			}
			vMinY.push_back(nMin);
			vMaxY.push_back(nMax);
		}
		*/
		// moving instance
		int nBest;
		int nTmp;
		nTmp = cF.m_pInstance->getPlacedY();

		nBest = boundingBoxCost(nTmp, vMinY, vMaxY);
		gGrid_C *pBestGrid = NULL;
		gGrid_C *pGrid = getGrid(m_pDesign, cF.m_pInstance->getPlacedX(), cF.m_pInstance->getPlacedY(), m_nDZ);
		while (graphTravel(m_pDesign, pGrid, nDX, nDY, 0) != NULL)
		{
			pGrid = graphTravel(m_pDesign, pGrid, nDX, nDY, 0);
			// check placeable
			//if( !isPlaceable( pGrid, pInst ) )
			//	continue;
			//
			int nTmpX, nTmpY, nTmpZ;
			pGrid->getPosition(nTmpX, nTmpY, nTmpZ);
			nTmp = nTmpY;

			int nTmpBest = boundingBoxCost(nTmp, vMinY, vMaxY);
			// change at 0706 19:30
			//if( nTmpBest <= nBest ) // change from <
			//{
			//	nBest = nTmpBest;
			//	pBestGrid = pGrid;
			//	vBestGrid.push_back( pBestGrid );
			//}
			//else
			//	break;

			if (nTmpBest <= nBest) // change from <
			{
				if( nBest - nTmpBest > nYGain )
					nYGain = nBest - nTmpBest;
				pBestGrid = pGrid;
				nBY2 = nBY2 + nDY;
			}
			else
				break;

			// end change
			//cout<<nBest<<endl;
		}
	}
	nGain = nXGain + nYGain;
	cF.m_nGain = nGain;
	/*	
	if( !cF.m_bLockInX && !cF.m_bLockInY && cF.m_nR - cF.m_nL == 0 && cF.m_nT - cF.m_nD == 0 )
	{
		nDX = 1;
		nDY = 0;
		//vector<int> vMin;
		//vector<int> vMax;
		for( int i=0; i<vBoundingBox.size(); i++ )
		{
			vector< instance_C* > &vInst = vBoundingBox[i];
			int nMax;
			int nMin;
			nMax = m_nDX;
			nMin = m_nTX;


			for( int j=0; j<vInst.size(); j++ )
			{
				instance_C* pTmpInst = vInst[j];
				nMax = max( pTmpInst->getPlacedX(), nMax );
				nMin = min( pTmpInst->getPlacedX(), nMin );
			}
			vMinX.push_back( nMin );
			vMaxX.push_back( nMax );
		}
		// moving instance
		int nBest;
		int nTmp;
		nTmp = cF.m_pInstance->getPlacedX();

		nBest = boundingBoxCost( nTmp, vMinX, vMaxX );
		gGrid_C* pBestGrid = NULL;
		gGrid_C* pGrid = getGrid( m_pDesign, cF.m_pInstance->getPlacedX(), cF.m_pInstance->getPlacedY(), m_nDZ);
		while( graphTravel( m_pDesign, pGrid, nDX, nDY, 0 ) != NULL )
		{
			pGrid = graphTravel( m_pDesign, pGrid, nDX, nDY, 0 );
			// check placeable
			//if( !isPlaceable( pGrid, pInst ) )
			//	continue;
			// 
			int nTmpX, nTmpY, nTmpZ;
			pGrid->getPosition( nTmpX, nTmpY, nTmpZ );
			nTmp = nTmpX;
			
			int nTmpBest = boundingBoxCost( nTmp, vMinX, vMaxX );
			// change at 0706 19:30
			//if( nTmpBest <= nBest ) // change from <
			//{
			//	nBest = nTmpBest;
			//	pBestGrid = pGrid;
			//	vBestGrid.push_back( pBestGrid );
			//}
			//else
			//	break;
			
			if( nTmpBest <= nBest ) // change from <
			{
				pBestGrid = pGrid;
				nBX2 = nBX2 + nDX;	
			}
			else
				break;
			
			// end change
			//cout<<nBest<<endl;
		}
		nDX = -1;
		nTmp = cF.m_pInstance->getPlacedX();

		nBest = boundingBoxCost( nTmp, vMinX, vMaxX );
		pBestGrid = NULL;
		pGrid = getGrid( m_pDesign, cF.m_pInstance->getPlacedX(), cF.m_pInstance->getPlacedY(), m_nDZ);
		while( graphTravel( m_pDesign, pGrid, nDX, nDY, 0 ) != NULL )
		{
			pGrid = graphTravel( m_pDesign, pGrid, nDX, nDY, 0 );
			// check placeable
			//if( !isPlaceable( pGrid, pInst ) )
			//	continue;
			// 
			int nTmpX, nTmpY, nTmpZ;
			pGrid->getPosition( nTmpX, nTmpY, nTmpZ );
			nTmp = nTmpX;
			
			int nTmpBest = boundingBoxCost( nTmp, vMinX, vMaxX );
			// change at 0706 19:30
			//if( nTmpBest <= nBest ) // change from <
			//{
			//	nBest = nTmpBest;
			//	pBestGrid = pGrid;
			//	vBestGrid.push_back( pBestGrid );
			//}
			//else
			//	break;
			
			if( nTmpBest <= nBest ) // change from <
			{
				pBestGrid = pGrid;
				nBX1 = nBX1 + nDX;	
			}
			else
				break;
			
			// end change
			//cout<<nBest<<endl;
		}
	
		nDX = 0;
		nDY = 1;
		for( int i=0; i<vBoundingBox.size(); i++ )
		{
			vector< instance_C* > &vInst = vBoundingBox[i];
			int nMax;
			int nMin;
			nMax = m_nDY;
			nMin = m_nTY;


			for( int j=0; j<vInst.size(); j++ )
			{
				instance_C* pTmpInst = vInst[j];
				nMax = max( pTmpInst->getPlacedY(), nMax );
				nMin = min( pTmpInst->getPlacedY(), nMin );
			}
			vMinY.push_back( nMin );
			vMaxY.push_back( nMax );
		}
		// moving instance
		nTmp = cF.m_pInstance->getPlacedY();

		nBest = boundingBoxCost( nTmp, vMinY, vMaxY );
		pBestGrid = NULL;
		pGrid = getGrid( m_pDesign, cF.m_pInstance->getPlacedX(), cF.m_pInstance->getPlacedY(), m_nDZ);
		while( graphTravel( m_pDesign, pGrid, nDX, nDY, 0 ) != NULL )
		{
			pGrid = graphTravel( m_pDesign, pGrid, nDX, nDY, 0 );
			// check placeable
			//if( !isPlaceable( pGrid, pInst ) )
			//	continue;
			// 
			int nTmpX, nTmpY, nTmpZ;
			pGrid->getPosition( nTmpX, nTmpY, nTmpZ );
			nTmp = nTmpY;
			
			int nTmpBest = boundingBoxCost( nTmp, vMinY, vMaxY );
			// change at 0706 19:30
			//if( nTmpBest <= nBest ) // change from <
			//{
			//	nBest = nTmpBest;
			//	pBestGrid = pGrid;
			//	vBestGrid.push_back( pBestGrid );
			//}
			//else
			//	break;
			
			if( nTmpBest <= nBest ) // change from <
			{
				pBestGrid = pGrid;
				nBY2 = nBY2 + nDY;
			}
			else
				break;
			
			// end change
			//cout<<nBest<<endl;
		}
		
		nDY = -1;
		nTmp = cF.m_pInstance->getPlacedY();

		nBest = boundingBoxCost( nTmp, vMinY, vMaxY );
		pBestGrid = NULL;
		pGrid = getGrid( m_pDesign, cF.m_pInstance->getPlacedX(), cF.m_pInstance->getPlacedY(), m_nDZ);
		while( graphTravel( m_pDesign, pGrid, nDX, nDY, 0 ) != NULL )
		{
			pGrid = graphTravel( m_pDesign, pGrid, nDX, nDY, 0 );
			// check placeable
			//if( !isPlaceable( pGrid, pInst ) )
			//	continue;
			// 
			int nTmpX, nTmpY, nTmpZ;
			pGrid->getPosition( nTmpX, nTmpY, nTmpZ );
			nTmp = nTmpY;
			
			int nTmpBest = boundingBoxCost( nTmp, vMinY, vMaxY );
			// change at 0706 19:30
			//if( nTmpBest <= nBest ) // change from <
			//{
			//	nBest = nTmpBest;
			//	pBestGrid = pGrid;
			//	vBestGrid.push_back( pBestGrid );
			//}
			//else
			//	break;
			
			if( nTmpBest <= nBest ) // change from <
			{
				pBestGrid = pGrid;
				nBY1 = nBY1 + nDY;
			}
			else
				break;
			
			// end change
			//cout<<nBest<<endl;
		}
	}
	*/
	if( nBX1 > nBX2 )
	{
		int nTmpX = nBX1;
		nBX1 = nBX2;
		nBX2 = nTmpX;
	}
	
	if( nBY1 > nBY2 )
	{
		int nTmpY = nBY1;
		nBY1 = nBY2;
		nBY2 = nTmpY;
	}
	/*
	//if( nBY1 - nBY2 != 0 && nBX1 - nBX2 == 0 )
	//{
		int nDDY = abs( nBY1 - nBY2 )/2;
		if( nBX1 - nDDY >= m_nDX )
			nBX1 = nBX1 - nDDY;
		if( nBX2 + nDDY <= m_nTX )
			nBX2 = nBX2 + nDDY;
	//}

	//if( nBX1 - nBX2 != 0 && nBY1 - nBY2 == 0 )
	//{
		int nDDX = abs( nBX1 - nBX2 )/2;
		if( nBY1 - nDDX >= m_nDY )
			nBY1 = nBY1 - nDDX;
		if( nBY2 + nDDX <= m_nTY )
			nBY2 = nBY2 + nDDX;
	//}
	*/
	/*
	else if( nBX1 - nBX2 != 0 && nBY1 - nBY2 != 0 )
	{
		int nDDY = abs( nBY1 - nBY2 );
		if( nBX1 - nDDY >= m_nDX )
			nBX1 = nBX1 - nDDY;
		if( nBX2 + nDDY <= m_nTX )
			nBX2 = nBX2 + nDDY;
		
		int nDDX = abs( nBX1 - nBX2 );
		if( nBY1 - nDDX >= m_nDY )
			nBY1 = nBY1 - nDDX;
		if( nBY2 + nDDX <= m_nTY )
			nBY2 = nBY2 + nDDX;
		
	}
	*/
	//if (nBX1 < nBX2)
	//{
		cF.m_nBoundX1 = nBX1;
		cF.m_nBoundX2 = nBX2;
	//}
	//else
	//{
	//	cF.m_nBoundX1 = nBX2;
	//	cF.m_nBoundX2 = nBX1;
	//}

	//if (nBY1 < nBY2)
	//{
		cF.m_nBoundY1 = nBY1;
		cF.m_nBoundY2 = nBY2;
	//}
	//else
	//{
	//	cF.m_nBoundY1 = nBY2;
	//	cF.m_nBoundY2 = nBY1;
	//}
	
	//cF.m_vMinX.clear();
	//cF.m_vMaxX.clear();

	//cF.m_vMinY.clear();
	//cF.m_vMaxY.clear();

	cF.m_vMinX = vMinX;
	cF.m_vMaxX = vMaxX;

	cF.m_vMinY = vMinY;
	cF.m_vMaxY = vMaxY;

	//if( cF.m_pInstance->getName() == "C1200" )
	//	cout<<"Range: "<<cF.m_nBoundX1<<" "<<cF.m_nBoundY1<<" "<<cF.m_nBoundX2<<" "<<cF.m_nBoundY2<<endl;
	//cout << cF.m_pInstance->getName()  << " " << abs( nDF - nTF ) + abs( nRF - nLF ) << endl;
	return true;
}
/*
bool router_C::calNetForcedModel(forced_C &cF)
{
	forced_net_C *pPInst;
	vector<instance_C *> vInst = pPInst->getCluster();
	vector<forced_C> &vF = m_vForced;
	bool bFix = false;
	int nFT = 0;
	int nFD = 0;
	int nFR = 0;
	int nFL = 0;

	int nX = 0;
	int nY = 0;
	for (int i = 0; i < vInst.size(); i++)
	{
		instance_C *pInst = vInst[i];
		if (!pInst->isMovable())
			bFix = true;
		nX = nX + pInst->getPlacedX();
		nY = nY + pInst->getPlacedY();
		forced_C *pF = &m_vForced[pInst->getId()];
		nFT = nFT + pF->m_nT;
		nFD = nFT + pF->m_nD;
		nFR = nFR + pF->m_nR;
		nFL = nFL + pF->m_nL;
	}
	int nNumInst = vInst.size();

	nX = nX / nNumInst;
	nY = nY / nNumInst;

	if (!bFix)
	{
		pPInst->setPlaced(nX, nY);

		cF.m_nT = nFT;
		cF.m_nD = nFD;
		cF.m_nR = nFR;
		cF.m_nL = nFL;
		cF.m_bLockInX = false;
		cF.m_bLockInY = false;

		return true;
	}
	else
	{

		pPInst->setPlaced(nX, nY);

		cF.m_nT = 0;
		cF.m_nD = 0;
		cF.m_nR = 0;
		cF.m_nL = 0;
		cF.m_bLockInX = true;
		cF.m_bLockInY = true;

		return true;
	}
}
*/

bool router_C::calNetForcedModel( forced_net_C &cFN )
{
	if( cFN.m_bLock || cFN.m_bHasFixedInst )
	{
		cFN.m_nTF = 0;
		cFN.m_nDF = 0;
		cFN.m_nRF = 0;
		cFN.m_nLF = 0;
		return true;
	}
	int nTF = 0;
	int nDF = 0;
	int nRF = 0;
	int nLF = 0;

	vector< instance_C* > vInst = cFN.m_vCluster;
	vector< net_C* > vIgnoreNet = cFN.m_vIntraNet;
	vIgnoreNet.push_back( cFN.m_pNet );
	for( int i=0; i<vInst.size(); i++ )
	{
		calForcedModel( m_vForced[ vInst[i]->getId() ], vIgnoreNet, nTF, nDF, nRF, nLF );
	}

	if (nTF < 0)
		nTF = 0;
	if (nDF < 0)
		nDF = 0;
	if (nRF < 0)
		nRF = 0;
	if (nLF < 0)
		nLF = 0;
	
	cFN.m_nTF = nTF;
	cFN.m_nDF = nDF;
	cFN.m_nRF = nRF;
	cFN.m_nLF = nLF;

	cout<< nTF << " " << nDF << " " << nRF << " " << nLF << endl;

	return true;

}

net_C* router_C::pickNetToMove()
{
	int nPickId = -1;
	int nMaxForced = 0;
	for( int i=0; i<m_vNetForced.size(); i++ )
	{
		forced_net_C &cFN = m_vNetForced[i];
		if( cFN.m_bLock || cFN.m_bHasFixedInst )
			continue;

		int nXF = abs( cFN.m_nTF - cFN.m_nDF );
		int nYF = abs( cFN.m_nRF - cFN.m_nLF );
		if( nMaxForced < nXF + nYF )
		{
			nMaxForced = nXF + nYF;
			nPickId = i;
		}
	}

	if( nPickId == -1 )
		return NULL;
	else
		return m_vNetForced[ nPickId ].m_pNet;
}

bool router_C::calForcedNetwork(networkForced_C &cNF, set< instance_C* > &sUnplacedInst )
{

	// second version
	// put net on 2D graph
	// put target
	net_C* pNet = cNF.m_pNet;
	vector< instance_C* > vInst = pNet->getInst();
		
	//cerr <<"Here"<<endl;
	int nMaxX = INT_MIN;
	int nMinX = INT_MAX;
	int nMaxY = INT_MIN;
	int nMinY = INT_MAX;
	int n2MaxX = INT_MIN;
	int n2MinX = INT_MAX;
	int n2MaxY = INT_MIN;
	int n2MinY = INT_MAX;

	int nMinXCount = 0;
	int nMaxXCount = 0;
	int nMinYCount = 0;
	int nMaxYCount = 0;
	vector< int > vLF;
	vector< int > vRF;
	vector< int > vTF;
	vector< int > vDF;
	for (int i = 0; i < cNF.m_vForced.size(); i++)
	{
		forced_C *cF = cNF.m_vForced[i];
		instance_C *pInst = cF->m_pInstance;
		if( sUnplacedInst.count( pInst ) != 0 )
			continue;
		// logger begin 0812
		/*
		if (!cF->m_bLockInX && !cF->m_bLockInY)
		{
			if (cNF.m_pNet->m_vFL[i] != 0)
				cF->m_nL++;
			if (cNF.m_pNet->m_vFR[i] != 0)
				cF->m_nR++;
			if (cNF.m_pNet->m_vFT[i] != 0)
				cF->m_nT++;
			if (cNF.m_pNet->m_vFD[i] != 0)
				cF->m_nD++;
		}
		*/
		// logger end
		//cout<< m_vTargetGraph.size() <<endl ;
		int nX = pInst->getPlacedX();
		int nY = pInst->getPlacedY();
		
		if (nX < nMinX)
		{
			n2MinX = nMinX;
			nMinX = nX;
			nMinXCount = 1;
		}
		else if (nX == nMinX)
		{
			n2MinX = nMinX;
			nMinXCount++;
		}

		if (nX > nMaxX)
		{
			n2MaxX = nMaxX;
			nMaxX = nX;
			nMaxXCount = 1;
		}
		else if (nX == nMaxX)
		{	
			n2MaxX = nMaxX;
			nMaxXCount++;
		}

		if (nY < nMinY)
		{
			n2MinY = nMinY;
			nMinY = nY;
			nMinYCount = 1;
		}
		else if (nY == nMinY)
		{
			n2MinY = nMinY;
			nMinYCount++;
		}

		if (nY > nMaxY)
		{
			n2MaxY = nMaxY;
			nMaxY = nY;
			nMaxYCount = 1;
		}
		else if (nY == nMaxY)
		{
			n2MaxY = nMaxY;
			nMaxYCount++;
		}


	}

	cNF.m_nMinX = nMinX;
	cNF.m_nMaxX = nMaxX;
	cNF.m_nMinY = nMinY;
	cNF.m_nMaxY = nMaxY;
	if( n2MinX == INT_MAX )
		cNF.m_n2MinX = nMaxX;
	else
		cNF.m_n2MinX = n2MinX;
	if( n2MaxX == INT_MIN )
		cNF.m_n2MaxX = nMinX;
	else
		cNF.m_n2MaxX = n2MaxX;
	if( n2MinY == INT_MAX )
		cNF.m_n2MinY = nMaxY;
	else
		cNF.m_n2MinY = n2MinY;
	if( n2MaxY == INT_MIN )
		cNF.m_n2MaxY = nMinY;
	else
		cNF.m_n2MaxY = n2MaxY;
	
	cNF.m_nMinXCount = nMinXCount;
	cNF.m_nMaxXCount = nMaxXCount;
	cNF.m_nMinYCount = nMinYCount;
	cNF.m_nMaxYCount = nMaxYCount;
	//cout << cNF.m_pNet->getName() << ": " << nMinX << " " << nMinY << " " << nMaxX << " " << nMaxY <<endl;

	return true;
}

bool router_C::calBoundryModel_ver2( boundry_C* pB )
{
	networkForced_C* pNF = pB->m_pNetwork;
	/*
	int nNetF = 0;
	bool bIsMovable = true;
	if( pB->m_cType == 'L' )
		nNetF = pNF->m_nMinYCount - 1;
	else if( pB->m_cType == 'R' )
		nNetF = pNF->m_nMaxYCount - 1;
	else if( pB->m_cType == 'D' )
		nNetF = pNF->m_nMinXCount - 1;
	else if( pB->m_cType == 'T' )
		nNetF = pNF->m_nMaxXCount - 1;
	*/
	int nGain = 0;
	int nBase = 0;
	if( pB->m_cType == 'L' )
		nBase = pNF->m_nMinY;
	else if( pB->m_cType == 'R' )
		nBase = pNF->m_nMaxY;
	else if( pB->m_cType == 'D' )
		nBase = pNF->m_nMinX;
	else if( pB->m_cType == 'T' )
		nBase = pNF->m_nMaxX;

	vector< forced_C* > &vF = pB->m_vForced;
	vector< networkForced_C* > vTTmpNF;
	set< networkForced_C* > sTmpNF;
	set< forced_C* > sF;
	vector< int > vMinX;
	vector< int > vMaxX;
	vector< int > vMinY;
	vector< int > vMaxY;
	bool bIsMovable = true;
	int nBoundX1 = INT_MAX;
	int nBoundX2 = INT_MIN;
	int nBoundY1 = INT_MAX;
	int nBoundY2 = INT_MIN;
	for( int i=0; i<vF.size(); i++ )
	{
		sF.insert( vF[i] );
		if( nBoundX1 > vF[i]->m_pInstance->getPlacedX() )
			nBoundX1 = vF[i]->m_pInstance->getPlacedX();
		if( nBoundX2 < vF[i]->m_pInstance->getPlacedX() )
			nBoundX2 = vF[i]->m_pInstance->getPlacedX();
		if( nBoundY1 > vF[i]->m_pInstance->getPlacedY() )
			nBoundY1 = vF[i]->m_pInstance->getPlacedY();
		if( nBoundY2 < vF[i]->m_pInstance->getPlacedY() )
			nBoundY2 = vF[i]->m_pInstance->getPlacedY();
		
		vector< networkForced_C* > vTmpNF = vF[i]->m_vNetwork;
		for( int j=0; j<vTmpNF.size(); j++ )
		{
			networkForced_C* pNF = vTmpNF[j];
			if( sTmpNF.count( pNF ) == 0 )
			{
				sTmpNF.insert( pNF );
				vTTmpNF.push_back( pNF );
			}	
		}
		if( !vF[i]->m_pInstance->isMovable() )
			bIsMovable = false;
	}
	
	int nLF = 0;
	int nRF = 0;
	int nTF = 0;
	int nDF = 0;
	int nCX = 0;
	int nCY = 0;
	int nPosF = 0;
	int nNegF = 0;

	for( int i=0; i<vTTmpNF.size(); i++ )
	{
		vector< forced_C* > vTmpF = vTTmpNF[i]->m_vForced;
		int nMinX = INT_MAX;
		int nMaxX = INT_MIN;
		int nMinY = INT_MAX;
		int nMaxY = INT_MIN;
		for( int j=0; j<vTmpF.size(); j++ )
		{
			forced_C* pF = vTmpF[j];
			if( sF.count( pF ) == 0 )
			{
				if( pF->m_pInstance->getPlacedX() < nMinX )
					nMinX = pF->m_pInstance->getPlacedX();

				if( pF->m_pInstance->getPlacedX() > nMaxX )
					nMaxX = pF->m_pInstance->getPlacedX();
				
				if( pF->m_pInstance->getPlacedY() < nMinY )
					nMinY = pF->m_pInstance->getPlacedY();

				if( pF->m_pInstance->getPlacedY() > nMaxY )
					nMaxY = pF->m_pInstance->getPlacedY();
			}
		}
		if( nMinX == INT_MAX || nMaxX == INT_MIN )
		{
			vMinX.push_back( -1 );
			vMaxX.push_back( -1 );
		}
		else
		{
			vMinX.push_back( nMinX );
			vMaxX.push_back( nMaxX );
		}
		if( nMinY == INT_MAX || nMaxY == INT_MIN )
		{
			vMinY.push_back( -1 );
			vMaxY.push_back( -1 );
		}
		else
		{
			vMinY.push_back( nMinY );
			vMaxY.push_back( nMaxY );
		}
	}
	
	int nMinRangeX = m_nOffsetX;
	int nMaxRangeX = m_vGraph[0][0].size();
	int nMinRangeY = m_nOffsetY;
	int nMaxRangeY = m_vGraph[0].size();

	int nBestX = ( nBoundX1 + nBoundX2 )/2;
	int nBestY = ( nBoundY1 + nBoundY2 )/2;
	int nInitCostX = boundingBoxCost( nBestX, vMinX, vMaxX );   
	int nInitCostY = boundingBoxCost( nBestY, vMinY, vMaxY );
	int nBestCostX = nInitCostX;
	int nBestCostY = nInitCostY;
	for( int i=nBoundX1 - 1; i >= nMinRangeX; i-- )	
	{
		int nNewCost = boundingBoxCost( i, vMinX, vMaxX );   
		if( nNewCost <= nInitCostX )
		{
			if( nNewCost < nBestCostX )
				nBestCostX = nNewCost;
			nBoundX1--;
		}
		else
			break;
	}
	
	for( int i=nBoundX2 + 1; i <= nMaxRangeX; i++ )	
	{
		int nNewCost = boundingBoxCost( i, vMinX, vMaxX );   
		if( nNewCost <= nInitCostX )
		{	
			if( nNewCost < nBestCostX )
				nBestCostX = nNewCost;
			nBoundX2++;
		}
		else
			break;
	}
	
	for( int i=nBoundY1 - 1; i >= nMinRangeY; i-- )	
	{
		int nNewCost = boundingBoxCost( i, vMinY, vMaxY );   
		if( nNewCost <= nInitCostY )
		{
			if( nNewCost < nBestCostY )
				nBestCostY = nNewCost;
			nBoundY1--;
		}
		else
			break;
	}
	
	for( int i=nBoundY2 + 1; i <= nMaxRangeY; i++ )	
	{
		int nNewCost = boundingBoxCost( i, vMinY, vMaxY );   
		if( nNewCost <= nInitCostY )
		{
			if( nNewCost < nBestCostY )
				nBestCostY = nNewCost;
			nBoundY2++;
		}
		else
			break;
	}
	nGain = ( nInitCostX + nInitCostY ) - ( nBestCostX + nBestCostY );
	/*
	if( nBoundX1 == nBestX )
		nBoundX1++ ;
	if( nBoundX2 == nBestX )
		nBoundX2--;
	if( nBoundY1 == nBestY )
		nBoundY1++ ;
	if( nBoundY2 == nBestY )
		nBoundY2--;
	*/
	pB->m_nBoundX1 = nBoundX1;
	pB->m_nBoundX2 = nBoundX2;
	pB->m_nBoundY1 = nBoundY1;
	pB->m_nBoundY2 = nBoundY2;

	pB->m_nT = nTF;
	pB->m_nD = nDF;
	pB->m_nR = nRF;
	pB->m_nL = nLF;
	pB->m_nCX = nCX;
	pB->m_nCY = nCY;
	pB->m_nPosF = nPosF;
	//pB->m_nPosF = nCostF;
	pB->m_bMovable = bIsMovable;
	pB->m_vNetwork = vTTmpNF;
	pB->m_vMinX = vMinX;
	pB->m_vMaxX = vMaxX;
	pB->m_vMinY = vMinY;
	pB->m_vMaxY = vMaxY;
	pB->m_nGain = nGain;
	//pB->m_nMove = nMove;
	/*
	cout << pNF->m_pNet->getName() << " " << pB->m_cType << endl;
	for( int i=0; i<vMinX.size(); i++ )
	{
		cout << vMinX[i] << " " << vMinY[i] << " " << vMaxX[i] << " " << vMaxY[i] << endl;
	}
	*/
	//pB->m_nGain = nCostF;
	//cout << pNF->m_pNet->getName() << " " << pB->m_cType << " " << pB->m_nBoundX1 << " " << pB->m_nBoundY1 << " " << pB->m_nBoundX2 << " " << pB->m_nBoundY2 << endl;

	return true;
}



bool router_C::calBoundryModel( boundry_C* pB )
{
	networkForced_C* pNF = pB->m_pNetwork;
	/*
	int nNetF = 0;
	bool bIsMovable = true;
	if( pB->m_cType == 'L' )
		nNetF = pNF->m_nMinYCount - 1;
	else if( pB->m_cType == 'R' )
		nNetF = pNF->m_nMaxYCount - 1;
	else if( pB->m_cType == 'D' )
		nNetF = pNF->m_nMinXCount - 1;
	else if( pB->m_cType == 'T' )
		nNetF = pNF->m_nMaxXCount - 1;
	*/
	int nGain = 0;
	int nBase = 0;
	if( pB->m_cType == 'L' )
		nBase = pNF->m_nMinY;
	else if( pB->m_cType == 'R' )
		nBase = pNF->m_nMaxY;
	else if( pB->m_cType == 'D' )
		nBase = pNF->m_nMinX;
	else if( pB->m_cType == 'T' )
		nBase = pNF->m_nMaxX;

	vector< forced_C* > &vF = pB->m_vForced;
	vector< networkForced_C* > vTTmpNF;
	vector< int > vMinXCount;
	vector< int > vMaxXCount;
	vector< int > vMinYCount;
	vector< int > vMaxYCount;
	set< networkForced_C* > sTmpNF;

	vector< int > vMinX;
	vector< int > vMaxX;
	vector< int > vMinY;
	vector< int > vMaxY;
	vector< int > vMinBX;
	vector< int > vMaxBX;
	vector< int > vMinBY;
	vector< int > vMaxBY;
	bool bIsMovable = true;
	for( int i=0; i<vF.size(); i++ )
	{
		vector< networkForced_C* > vTmpNF = vF[i]->m_vNetwork;
		for( int j=0; j<vTmpNF.size(); j++ )
		{
			networkForced_C* pNF = vTmpNF[j];
			if( sTmpNF.count( pNF ) == 0 )
			{
				int nMinX = pNF->m_mMinX.find( vF[i] )->second;
				int nMaxX = pNF->m_mMaxX.find( vF[i] )->second;
				int nMinY = pNF->m_mMinY.find( vF[i] )->second;
				int nMaxY = pNF->m_mMaxY.find( vF[i] )->second;
				vMinX.push_back( nMinX );
				vMaxX.push_back( nMaxX );
				vMinY.push_back( nMinY );
				vMaxY.push_back( nMaxY );
				vMinBX.push_back( pNF->m_nMinX );
				vMaxBX.push_back( pNF->m_nMaxX );
				vMinBY.push_back( pNF->m_nMinY );
				vMaxBY.push_back( pNF->m_nMaxY );
				sTmpNF.insert( pNF );
				vTTmpNF.push_back( pNF );
				
				
				instance_C* pInst = vF[i]->m_pInstance;
				if( pInst->getPlacedX() == pNF->m_nMinX )
					vMinXCount.push_back( 1 ); 	
				else
					vMinXCount.push_back( 0 );
				
				if( pInst->getPlacedX() == pNF->m_nMaxX )
					vMaxXCount.push_back( 1 ); 	
				else
					vMaxXCount.push_back( 0 );
				
				if( pInst->getPlacedY() == pNF->m_nMinY )
					vMinYCount.push_back( 1 ); 	
				else
					vMinYCount.push_back( 0 );
				
				if( pInst->getPlacedY() == pNF->m_nMaxY )
					vMaxYCount.push_back( 1 ); 	
				else
					vMaxYCount.push_back( 0 );
			}
			else
			{
				for( int n=0; n<vTTmpNF.size(); n++ )
				{
					if( vTTmpNF[n] == pNF )
					{
						instance_C* pInst = vF[i]->m_pInstance;
						if( pInst->getPlacedX() == pNF->m_nMinX )
							vMinXCount[n]++; 	
						
						if( pInst->getPlacedX() == pNF->m_nMaxX )
							vMaxXCount[n]++; 	
						
						if( pInst->getPlacedY() == pNF->m_nMinY )
							vMinYCount[n]++; 	
						
						if( pInst->getPlacedY() == pNF->m_nMaxY )
							vMaxYCount[n]++; 	
						break;	
					}
				}
			}
		}
		if( !vF[i]->m_pInstance->isMovable() )
			bIsMovable = false;
	}
	
	int nLF = 0;
	int nRF = 0;
	int nTF = 0;
	int nDF = 0;
	int nCX = 0;
	int nCY = 0;
	int nPosF = 0;
	int nNegF = 0;
	vector< int > vTmpPMaxX = vMaxX;
	vector< int > vTmpPMinX = vMinX;
	vector< int > vTmpNMaxX = vMaxX;
	vector< int > vTmpNMinX = vMinX;
	vector< int > vTmpPMinY = vMinY;
	vector< int > vTmpPMaxY = vMaxY;
	vector< int > vTmpNMinY = vMinY;
	vector< int > vTmpNMaxY = vMaxY;


	int nPosCost;
	int nNegCost;
	if( pB->m_cType == 'T' )
	{
		for( int i=0; i<vMinBX.size(); i++ )
		{
			if( vMinBX[i] == nBase && vMaxBX[i] == nBase )
			{
				if( vTTmpNF[i] == pB->m_pNetwork )
				{
					vTmpPMaxX[i] = -1;
					vTmpPMinX[i] = -1;
					continue;
				}
				nCX++;
			}
			else
			{
				if( vMinBX[i] == nBase )
				{
					nTF++;	
				}

				if( vMaxBX[i] == nBase && vMaxXCount[i] == vTTmpNF[i]->m_nMaxXCount ) 
				{
					vTmpPMaxX[i] = vTTmpNF[i]->m_n2MaxX;
					nDF++;
				}
			}
		}
		nPosF = nDF - nTF - nCX;
		nPosCost = boundingBoxCost( nBase, vTmpPMinX, vTmpPMaxX );

		nDF = 0;
		nTF = 0;
		nCX = 0;
		
		for( int i=0; i<vMinBX.size(); i++ )
		{
			if( vMinBX[i] == nBase && vMaxBX[i] == nBase )
			{
				if( vTTmpNF[i] == pB->m_pNetwork )
				{
					vTmpNMaxX[i] = -1;
					vTmpNMinX[i] = -1;
					continue;
				}
				nCX++;
			}
			else
			{
				if( vMinBX[i] == nBase && vMinXCount[i] == vTTmpNF[i]->m_nMinXCount )
				{
					vTmpNMinX[i] = vTTmpNF[i]->m_n2MinX;
					nTF++;	
				}

				if( vMaxBX[i] == nBase && vMaxXCount[i] == vTTmpNF[i]->m_nMaxXCount ) 
				{
					nDF++;
				}
			}
		}
		nNegF = nTF - nDF - nCX;
		nNegCost = boundingBoxCost( nBase, vTmpNMinX, vTmpNMaxX );
	}
	else if( pB->m_cType == 'D' )
	{
		for( int i=0; i<vMinBX.size(); i++ )
		{
			if( vMinBX[i] == nBase && vMaxBX[i] == nBase )
			{
				if( vTTmpNF[i] == pB->m_pNetwork )
				{
					vTmpPMaxX[i] = -1;
					vTmpPMinX[i] = -1;
					continue;
				}
				nCX++;
			}
			else
			{
				if( vMinBX[i] == nBase && vMinXCount[i] == vTTmpNF[i]->m_nMinXCount )
				{
					nTF++;	
					vTmpPMinX[i] = vTTmpNF[i]->m_n2MinX;
				}

				if( vMaxBX[i] == nBase ) 
				{
					nDF++;
				}
			}
		}
		nPosF = nTF - nDF - nCX;
		nPosCost = boundingBoxCost( nBase, vTmpPMinX, vTmpPMaxX );

		int nTF = 0;
		int nDF = 0;
		int nCX = 0;
		
		for( int i=0; i<vMinBX.size(); i++ )
		{
			if( vMinBX[i] == nBase && vMaxBX[i] == nBase )
			{
				if( vTTmpNF[i] == pB->m_pNetwork )
				{
					vTmpNMaxX[i] = -1;
					vTmpNMinX[i] = -1;
					continue;
				}
				nCX++;
			}
			else
			{
				if( vMinBX[i] == nBase )
				{
					nTF++;	
				}

				if( vMaxBX[i] == nBase && vMaxXCount[i] == vTTmpNF[i]->m_nMaxXCount ) 
				{
					nDF++;
					vTmpNMaxX[i] = vTTmpNF[i]->m_n2MaxX;
				}
			}
		}
		nNegF = nDF - nTF - nCX;
		nNegCost = boundingBoxCost( nBase, vTmpNMinX, vTmpNMaxX );

	}
	else if( pB->m_cType == 'R' )
	{
		for( int i=0; i<vMinBY.size(); i++ )
		{
			if( vMinBY[i] == nBase && vMaxBY[i] == nBase )
			{
				if( vTTmpNF[i] == pB->m_pNetwork )
				{
					vTmpPMaxY[i] = -1;
					vTmpPMinY[i] = -1;
					continue;
				}
				nCY++;
			}
			else
			{
				if( vMinBY[i] == nBase )
				{
					nRF++;	
				}

				if( vMaxBY[i] == nBase && vMaxYCount[i] == vTTmpNF[i]->m_nMaxYCount ) 
				{
					nLF++;
					vTmpPMaxY[i] = vTTmpNF[i]->m_n2MaxY;
				}
			}
		}
		nPosF = nLF - nRF - nCY;
		nPosCost = boundingBoxCost( nBase, vMinY, vMaxY );
	
		int nRF = 0;
		int nLF = 0;
		int nCY = 0;

		for( int i=0; i<vMinBY.size(); i++ )
		{
			if( vMinBY[i] == nBase && vMaxBY[i] == nBase )
			{
				if( vTTmpNF[i] == pB->m_pNetwork )
				{
					vTmpNMaxY[i] = -1;
					vTmpNMinY[i] = -1;
					continue;
				}
				nCY++;
			}
			else
			{
				if( vMinBY[i] == nBase && vMinYCount[i] == vTTmpNF[i]->m_nMinYCount )
				{
					nRF++;	
					vTmpNMinY[i] = vTTmpNF[i]->m_n2MinY;
				}

				if( vMaxBY[i] == nBase ) 
				{
					nLF++;
				}
			}
		}
		nNegF = nRF - nLF - nCY;
		nNegCost = boundingBoxCost( nBase, vTmpNMinY, vTmpNMaxY );
	}
	else
	{
		for( int i=0; i<vMinBY.size(); i++ )
		{
			if( vMinBY[i] == nBase && vMaxBY[i] == nBase )
			{
				if( vTTmpNF[i] == pB->m_pNetwork )
				{
					vTmpPMaxY[i] = -1;
					vTmpPMinY[i] = -1;
					continue;
				}
				nCY++;
			}
			else
			{
				if( vMinBY[i] == nBase && vMinYCount[i] == vTTmpNF[i]->m_nMinYCount )
				{
					nRF++;	
					vTmpPMinY[i] = vTTmpNF[i]->m_n2MinY;
				}

				if( vMaxBY[i] == nBase ) 
				{
					nLF++;
				}
			}
		}
		nPosF = nRF - nLF - nCY;
		nPosCost = boundingBoxCost( nBase, vTmpPMinY, vTmpPMaxY );

		int nRF = 0;
		int nLF = 0;
		int nCY = 0;
		
		for( int i=0; i<vMinBY.size(); i++ )
		{
			if( vMinBY[i] == nBase && vMaxBY[i] == nBase )
			{
				if( vTTmpNF[i] == pB->m_pNetwork )
				{
					vTmpNMaxY[i] = -1;
					vTmpNMinY[i] = -1;
					continue;
				}
				nCY++;
			}
			else
			{
				if( vMinBY[i] == nBase )
				{
					nRF++;	
				}

				if( vMaxBY[i] == nBase && vMaxYCount[i] == vTTmpNF[i]->m_nMaxYCount ) 
				{
					nLF++;
					vTmpNMaxY[i] = vTTmpNF[i]->m_n2MaxY;
				}
			}
		}
		nNegF = nLF - nRF - nCY;
		nNegCost = boundingBoxCost( nBase, vTmpNMinY, vTmpNMaxY );
	
	}

	int nMove = 0; // 1: pos, 0: no, -1: neg;
	//if( nPosF < 0 )
	//	nPosF = 0;
	int nOrigCost = 0;
	int nCostF;
	if( nPosF > nNegF )
	{
		nMove = 1;
		nCostF = nPosF;
		nOrigCost = nPosCost;
	}
	else if( nPosF < nNegF )
	{
		nMove = -1;
		nCostF = nNegF;
		nOrigCost = nNegCost;
	}
	else
	{	
		nCostF = 0;
		nMove = 0;
	}
	pB->m_nBoundX1 = pNF->m_nMinX;
	pB->m_nBoundX2 = pNF->m_nMaxX;
	pB->m_nBoundY1 = pNF->m_nMinY;
	pB->m_nBoundY2 = pNF->m_nMaxY;

	int nBest = 0;
	int nStep = 0;
	int nDiff = 0;
	bool bDec = true;
	if( pB->m_cType == 'T' )
	{
		if( nMove == 1 )
		{
			int nTmpBase = nBase-1;
			nBest = nPosCost;
			for( int x = nBase-1; x >= 1; x-- )
			{
				int nTmpCost = boundingBoxCost( x, vTmpPMinX, vTmpPMaxX );
				if( nTmpCost < nPosCost )
					nStep++;
				//if( nTmpCost <= nPosCost )
				if( nTmpCost < nPosCost || bDec )
				{
					nTmpBase = x;
					if( nTmpCost <= nBest )
						nBest = nTmpCost;
					else
						bDec = false;
				}
				else
					break;
			}
			pB->m_nBoundX2 = nBase-1;
			pB->m_nBoundX1 = nTmpBase;
			vMinX = vTmpPMinX;
			vMaxX = vTmpPMaxX;
			nDiff = nBest - nPosCost;
		}
		else if( nMove == -1 )
		{
			int nTmpBase = nBase+1;
			nBest = nNegCost;
			for( int x = nBase+1; x <= m_vGraph[0][0].size(); x++ )
			{
				int nTmpCost = boundingBoxCost( x, vTmpNMinX, vTmpNMaxX );
				if( nTmpCost < nNegCost )
					nStep++;
				//if( nTmpCost <= nNegCost )
				if( nTmpCost < nNegCost || bDec )
				{
					nTmpBase = x;
					if( nTmpCost <= nBest )
						nBest = nTmpCost;
					else
						bDec = false;
				}
				else
					break;
			}
			pB->m_nBoundX1 = nBase+1;
			pB->m_nBoundX2 = nTmpBase;	
			vMinX = vTmpNMinX;
			vMaxX = vTmpNMaxX;
			nDiff = nBest - nNegCost;
		}
	}
	else if( pB->m_cType == 'D' )
	{
		if( nMove == 1 )
		{
			int nTmpBase = nBase+1;
			nBest = nPosCost;
			for( int x = nBase+1; x <= m_vGraph[0][0].size() ; x++ )
			{
				int nTmpCost = boundingBoxCost( x, vTmpPMinX, vTmpPMaxX );
				if( nTmpCost < nPosCost )
					nStep++;
				//if( nTmpCost <= nPosCost )
				if( nTmpCost < nPosCost || bDec )
				{
					nTmpBase = x;
					if( nTmpCost <= nBest )
						nBest = nTmpCost;
					else
						bDec = false;
				}
				else
					break;
			}
			pB->m_nBoundX2 = nTmpBase;
			pB->m_nBoundX1 = nBase+1;
			vMinX = vTmpPMinX;
			vMaxX = vTmpPMaxX;
			nDiff - nBest - nPosCost;
		}
		else if( nMove == -1 )
		{
			int nTmpBase = nBase-1;
			nBest = nNegCost;
			for( int x = nBase-1; x >= 1; x-- )
			{
				int nTmpCost = boundingBoxCost( x, vTmpNMinX, vTmpNMaxX );
				if( nTmpCost < nNegCost )
					nStep++;
				//if( nTmpCost <= nNegCost )
				if( nTmpCost < nNegCost || bDec )
				{
					nTmpBase = x;
					if( nTmpCost <= nBest )
						nBest = nTmpCost;
					else
						bDec = false;
				}
				else
					break;
			}
			pB->m_nBoundX1 = nTmpBase;
			pB->m_nBoundX2 = nBase-1;
			vMinX = vTmpNMinX;
			vMaxX = vTmpNMaxX;
			nDiff = nBest - nNegCost;
		}
	}
	else if( pB->m_cType == 'R' )
	{
		if( nMove == 1 )
		{
			int nTmpBase = nBase-1;
			nBest = nPosCost;
			for( int y = nBase-1; y >= 1; y-- )
			{
				int nTmpCost = boundingBoxCost( y, vTmpPMinY, vTmpPMaxY );
				if( nTmpCost < nPosCost )
					nStep++;
				//if( nTmpCost <= nPosCost )
				if( nTmpCost < nPosCost || bDec )
				{
					nTmpBase = y;
					if( nTmpCost <= nBest )
						nBest = nTmpCost;
					else
						bDec = false;
				}
				else
					break;
			}
			pB->m_nBoundY1 = nTmpBase;
			pB->m_nBoundY2 = nBase-1;
			vMinY = vTmpPMinY;
			vMaxY = vTmpPMaxY;
			nDiff = nBest - nPosCost;
		}
		else if( nMove == -1 )
		{
			int nTmpBase = nBase+1;
			nBest = nNegCost;
			for( int y = nBase+1; y <= m_vGraph[0].size(); y++ )
			{
				int nTmpCost = boundingBoxCost( y, vTmpNMinY, vTmpNMaxY );
				if( nTmpCost < nNegCost )
					nStep++;
				//if( nTmpCost <= nNegCost )
				if( nTmpCost < nNegCost || bDec )
				{
					nTmpBase = y;
					if( nTmpCost <= nBest )
						nBest = nTmpCost;
					else
						bDec = false;
				}
				else
					break;
			}
			pB->m_nBoundY2 = nTmpBase;
			pB->m_nBoundY1 = nBase+1;
			vMinY = vTmpNMinY;
			vMaxY = vTmpNMaxY;
			nDiff = nBest - nNegCost;
		}
	}
	else if( pB->m_cType == 'L' )
	{
		if( nMove == 1 )
		{
			int nTmpBase = nBase+1;
			nBest = nPosCost;
			for( int y = nBase+1; y <= m_vGraph[0].size() ; y++ )
			{
				int nTmpCost = boundingBoxCost( y, vTmpPMinY, vTmpPMaxY );
				if( nTmpCost < nPosCost )
					nStep++;
				//if( nTmpCost <= nPosCost )
				if( nTmpCost < nPosCost || bDec )
				{
					nTmpBase = y;
					if( nTmpCost <= nBest )
						nBest = nTmpCost;
					else
						bDec = false;
				}
				else
					break;
			}
			pB->m_nBoundY2 = nTmpBase;
			pB->m_nBoundY1 = nBase+1;
			vMinY = vTmpPMinY;
			vMaxY = vTmpPMaxY;
			nDiff = nBest - nPosCost;
		}
		else if( nMove == -1 )
		{
			int nTmpBase = nBase-1;
			nBest = nNegCost;
			for( int y = nBase-1; y >= 1  ; y-- )
			{
				int nTmpCost = boundingBoxCost( y, vTmpNMinY, vTmpNMaxY );
				if( nTmpCost < nNegCost )
					nStep++;
				//if( nTmpCost <= nNegCost )
				if( nTmpCost < nNegCost || bDec )
				{
					nTmpBase = y;
					if( nTmpCost <= nBest )
						nBest = nTmpCost;
					else
						bDec = false;
				}
				else
					break;
			}
			pB->m_nBoundY1 = nTmpBase;
			pB->m_nBoundY2 = nBase-1;
			vMinY = vTmpNMinY;
			vMaxY = vTmpNMaxY;
			nDiff = nBest - nNegCost;
		
		}
	}
	//nGain = nBest * nPosF;
	//nGain = nStep * nCostF;
	nGain = nCostF;
	//nGain = nStep;
	//nGain = -nDiff;
	//cout << pNF->m_pNet->getName() << endl;
	/*
	for( int i=0; i<vF.size(); i++ )
	{
		forced_C* pF = vF[i];
		//cout << pF->m_pInstance->getName() << " " << pF->m_nT << " " << pF->m_nD << " " << pF->m_nR << " " << pF->m_nL << " " << pF->m_nCX << " " << pF->m_nCY << endl;
		nLF = nLF + pF->m_nL;
		nRF = nRF + pF->m_nR;
		nTF = nTF + pF->m_nT;
		nDF = nDF + pF->m_nD;
		nCX = nCX + pF->m_nCX;	
		nCY = nCY + pF->m_nCY;	
		if( !pF->m_pInstance->isMovable() )
			bIsMovable = false;
	}

	if( pNF->m_nMinX == pNF->m_nMaxX )
		nCX = nCX - pNF->m_nMinXCount;
	
	if( pNF->m_nMinY == pNF->m_nMaxY )
		nCY = nCY - pNF->m_nMinYCount;

	if( pB->m_cType == 'L' && pNF->m_nMinY != pNF->m_nMaxY )
		nRF = nRF - nNetF;
	else if( pB->m_cType == 'R' && pNF->m_nMinY != pNF->m_nMaxY )
		nLF = nLF - nNetF;
	else if( pB->m_cType == 'D' && pNF->m_nMinX != pNF->m_nMaxX )
		nTF = nTF - nNetF;
	else if( pB->m_cType == 'T' && pNF->m_nMinX != pNF->m_nMaxX )
		nDF = nDF - nNetF;
	if( nLF < 0 )
	nLF = 0;
	if( nRF < 0 )
	nRF = 0;
	if( nTF < 0 )
	nTF = 0;
	if( nDF < 0 )
	nDF = 0;
	if( nCX < 0 )
	nCX = 0;
	if( nCY < 0 )
	nCY = 0;	
	*/
	pB->m_nT = nTF;
	pB->m_nD = nDF;
	pB->m_nR = nRF;
	pB->m_nL = nLF;
	pB->m_nCX = nCX;
	pB->m_nCY = nCY;
	pB->m_nPosF = nPosF;
	//pB->m_nPosF = nCostF;
	pB->m_bMovable = bIsMovable;
	pB->m_vNetwork = vTTmpNF;
	pB->m_vMinX = vMinX;
	pB->m_vMaxX = vMaxX;
	pB->m_vMinY = vMinY;
	pB->m_vMaxY = vMaxY;
	pB->m_nGain = nGain;
	pB->m_nMove = nMove;
	/*
	cout << pNF->m_pNet->getName() << " " << pB->m_cType << endl;
	for( int i=0; i<vMinX.size(); i++ )
	{
		cout << vMinX[i] << " " << vMinY[i] << " " << vMaxX[i] << " " << vMaxY[i] << endl;
	}
	*/
	//pB->m_nGain = nCostF;
	//cout << pNF->m_pNet->getName() << " " << pB->m_cType << " " << pB->m_nBoundX1 << " " << pB->m_nBoundY1 << " " << pB->m_nBoundX2 << " " << pB->m_nBoundY2 << endl;

	return true;
}


bool router_C::calForcedNetwork_ver3(networkForced_C &cNF)
{
	net_C* pNet = cNF.m_pNet;
	vector< instance_C* > vInst = pNet->getInst();

	int nTmpX = -1;
	int nTmpY = -1;
	
	int nMaxX = INT_MIN;
	int nMinX = INT_MAX;
	int nMaxY = INT_MIN;
	int nMinY = INT_MAX;
	int n2MaxX = INT_MIN;
	int n2MinX = INT_MAX;
	int n2MaxY = INT_MIN;
	int n2MinY = INT_MAX;
	int nMinXCount = 0;
	int nMaxXCount = 0;
	int nMinYCount = 0;
	int nMaxYCount = 0;
	
	boundry_C cLB; // left bound
	boundry_C cRB; // right bound
	boundry_C cTB; // top bound
	boundry_C cDB; // down bound
	set< forced_C* > sLB;
	set< forced_C* > sRB;
	set< forced_C* > sTB;
	set< forced_C* > sDB;

	cLB.m_pNetwork = &cNF;
	cLB.m_bLock = cNF.m_cLB.m_bLock;
	cLB.m_cType = 'L';
	cRB.m_pNetwork = &cNF;
	cRB.m_bLock = cNF.m_cRB.m_bLock;
	cRB.m_cType = 'R';
	cTB.m_pNetwork = &cNF;
	cTB.m_bLock = cNF.m_cTB.m_bLock;
	cTB.m_cType = 'T';
	cDB.m_pNetwork = &cNF;
	cDB.m_bLock = cNF.m_cDB.m_bLock;
	cDB.m_cType = 'D';

	vector< int > vLF;
	vector< int > vRF;
	vector< int > vTF;
	vector< int > vDF;

	bool bHasFixedInst = false;
	for (int i = 0; i < cNF.m_vForced.size(); i++)
	{
		forced_C *cF = cNF.m_vForced[i];
		instance_C *pInst = cF->m_pInstance;
		//cout<< m_vTargetGraph.size() <<endl ;
		int nX = pInst->getPlacedX();
		int nY = pInst->getPlacedY();
		
		if (nX < nMinX)
		{
			n2MinX = nMinX;
			nMinX = nX;
			nMinXCount = 1;
		}
		else if (nX == nMinX)
		{
//			n2MinX = nMinX;
			nMinXCount++;
		}
		else if( nX < n2MinX )
		{
			n2MinX = nX;
		}

		if (nX > nMaxX)
		{
			n2MaxX = nMaxX;
			nMaxX = nX;
			nMaxXCount = 1;
		}
		else if (nX == nMaxX)
		{	
//			n2MaxX = nMaxX;
			nMaxXCount++;
		}
		else if( nX > n2MaxX)
		{
			n2MaxX = nX;
		}

		if (nY < nMinY)
		{
			n2MinY = nMinY;
			nMinY = nY;
			nMinYCount = 1;
		}
		else if (nY == nMinY)
		{
//			n2MinY = nMinY;
			nMinYCount++;
		}
		else if( nY < n2MinY )
		{
			n2MinY = nY;
		}

		if (nY > nMaxY)
		{
			n2MaxY = nMaxY;
			nMaxY = nY;
			nMaxYCount = 1;
		}
		else if (nY == nMaxY)
		{
//			n2MaxY = nMaxY;
			nMaxYCount++;
		}
		else if( nY > n2MaxY )
		{
			n2MaxY = nY;
		}

	}
	
	if( n2MinX == INT_MAX )
		n2MinX = nMaxX;
	
	if( n2MaxX == INT_MIN )
		n2MaxX = nMinX;
	
	if( n2MinY == INT_MAX )
		n2MinY = nMaxY;
	
	if( n2MaxY == INT_MIN )
		n2MaxY = nMinY;


	unordered_map< forced_C*, int > mMinX;
	unordered_map< forced_C*, int > mMaxX;
	unordered_map< forced_C*, int > mMinY;
	unordered_map< forced_C*, int > mMaxY;
	
	for (int i = 0; i < cNF.m_vForced.size(); i++)
	{
		forced_C *pF = cNF.m_vForced[i];
		instance_C *pInst = pF->m_pInstance;
		
		//cout<< m_vTargetGraph.size() <<endl ;
		int nX = pInst->getPlacedX();
		int nY = pInst->getPlacedY();
		
		if( nMinXCount == 1 )
		{
			if( nX == nMinX )
				mMinX[pF] = n2MinX;	
			else
				mMinX[pF] = nMinX;
		}
		else
		{
			mMinX[ pF ] = nMinX;
		}

		if( nMaxXCount == 1 )
		{
			if( nX == nMaxX )
				mMaxX[pF] = n2MaxX;
			else
				mMaxX[pF] = nMaxX;
		}
		else
		{
			mMaxX[ pF ] = nMaxX;
		}
		
		if( nMinYCount == 1 )
		{
			if( nY == nMinY )
				mMinY[pF] = n2MinY;
			else
				mMinY[pF] = nMinY;
		}
		else
		{
			mMinY[pF] = nMinY;
		}
		
		if( nMaxYCount == 1 )
		{
			if( nY == nMaxY )
				mMaxY[pF] = n2MaxY;
			else
				mMaxY[pF] = nMaxY;
		}
		else
		{
			mMaxY[pF] = nMaxY;
		}

		if( nX == nMinX )
		{
			if( sDB.count( pF ) == 0 )
			{
				cDB.m_vForced.push_back( pF );
				sDB.insert( pF );
			}
		}

		if( nX == nMaxX )
		{
			if( sTB.count( pF ) == 0 )
			{
				cTB.m_vForced.push_back( pF );
				sTB.insert( pF );
			}
		}
		
		if( nY == nMinY )
		{
			if( sLB.count( pF ) == 0 )
			{
				cLB.m_vForced.push_back( pF );
				sLB.insert( pF );
			}
		}

		if( nY == nMaxY )
		{
			if( sRB.count( pF ) == 0 )
			{
				cRB.m_vForced.push_back( pF );
				sRB.insert( pF );
			}
		}

	}

	cNF.m_nMinX = nMinX;
	cNF.m_nMaxX = nMaxX;
	cNF.m_nMinY = nMinY;
	cNF.m_nMaxY = nMaxY;
	cNF.m_n2MinX = n2MinX;
	cNF.m_n2MaxX = n2MaxX;
	cNF.m_n2MinY = n2MinY;
	cNF.m_n2MaxY = n2MaxY;
	cNF.m_nMinXCount = nMinXCount;
	cNF.m_nMaxXCount = nMaxXCount;
	cNF.m_nMinYCount = nMinYCount;
	cNF.m_nMaxYCount = nMaxYCount;
	cNF.m_cLB = cLB;
	cNF.m_cRB = cRB;
	cNF.m_cTB = cTB;
	cNF.m_cDB = cDB;

	cNF.m_mMinX = mMinX;
	cNF.m_mMaxX = mMaxX;
	cNF.m_mMinY = mMinY;
	cNF.m_mMaxY = mMaxY;

	return true;
}


bool router_C::calForcedNetwork_ver2(networkForced_C &cNF)
{
	net_C* pNet = cNF.m_pNet;
	vector< instance_C* > vInst = pNet->getInst();

	int nCount = 0;
	int nTmpX = -1;
	int nTmpY = -1;
	
	int nTotalX = 0;
	int nTotalY = 0;
	int nHalfX = 0;
	int nHalfY = 0;
	int nMaxX = INT_MIN;
	int nMinX = INT_MAX;
	int nMaxY = INT_MIN;
	int nMinY = INT_MAX;
	int n2MaxX = INT_MIN;
	int n2MinX = INT_MAX;
	int n2MaxY = INT_MIN;
	int n2MinY = INT_MAX;
	int nMinXCount = 0;
	int nMaxXCount = 0;
	int nMinYCount = 0;
	int nMaxYCount = 0;
	vector< int > vLF;
	vector< int > vRF;
	vector< int > vTF;
	vector< int > vDF;

	bool bHasFixedInst = false;
	for (int i = 0; i < cNF.m_vForced.size(); i++)
	{
		forced_C *cF = cNF.m_vForced[i];
		instance_C *pInst = cF->m_pInstance;
		if( !pInst->isMovable() )
		{
			bHasFixedInst = true;
		}
		//cout<< m_vTargetGraph.size() <<endl ;
		int nX = pInst->getPlacedX();
		int nY = pInst->getPlacedY();
		
		nTotalX = nTotalX + nX;
		nTotalY = nTotalY + nY;

		if (nX < nMinX)
		{
			n2MinX = nMinX;
			nMinX = nX;
			nMinXCount = 1;
		}
		else if (nX == nMinX)
		{
			n2MinX = nMinX;
			nMinXCount++;
		}
		else if( nX < n2MinX )
		{
			n2MinX = nX;
		}

		if (nX > nMaxX)
		{
			n2MaxX = nMaxX;
			nMaxX = nX;
			nMaxXCount = 1;
		}
		else if (nX == nMaxX)
		{	
			n2MaxX = nMaxX;
			nMaxXCount++;
		}
		else if( nX > n2MaxX)
		{
			n2MaxX = nX;
		}

		if (nY < nMinY)
		{
			n2MinY = nMinY;
			nMinY = nY;
			nMinYCount = 1;
		}
		else if (nY == nMinY)
		{
			n2MinY = nMinY;
			nMinYCount++;
		}
		else if( nY < n2MinY )
		{
			n2MinY = nY;
		}

		if (nY > nMaxY)
		{
			n2MaxY = nMaxY;
			nMaxY = nY;
			nMaxYCount = 1;
		}
		else if (nY == nMaxY)
		{
			n2MaxY = nMaxY;
			nMaxYCount++;
		}
		else if( nY > n2MaxY )
		{
			n2MaxY = nY;
		}
		nCount++;

	}
	
	if( n2MinX == INT_MAX )
		n2MinX = nMaxX;
	
	if( n2MaxX == INT_MIN )
		n2MaxX = nMinX;
	
	if( n2MinY == INT_MAX )
		n2MinY = nMaxY;
	
	if( n2MaxY == INT_MIN )
		n2MaxY = nMinY;

	if( bHasFixedInst )
	{
		int nTotalX = 0;
		int nTotalY = 0;
		nCount = 0;
		for (int i = 0; i < cNF.m_vForced.size(); i++)
		{
			forced_C *cF = cNF.m_vForced[i];
			instance_C *pInst = cF->m_pInstance;
			if( !pInst->isMovable() )
			{
				int nX = pInst->getPlacedX();
				int nY = pInst->getPlacedY();
				
				nTotalX = nTotalX + nX;
				nTotalY = nTotalY + nY;
			}
			nCount++;
		}
	}

	if( nTotalX%nCount  == 0 )
	{
		nTotalX = nTotalX / nCount;
	}
	else
	{
		nTotalX = nTotalX / nCount;
		nHalfX = 1;
	}
	
	if( nTotalY%nCount  == 0 )
	{
		nTotalY = nTotalY / nCount;
	}
	else
	{
		nTotalY = nTotalY / nCount;
		nHalfY = 1;
	}

	unordered_map< forced_C*, int > mMinX;
	unordered_map< forced_C*, int > mMaxX;
	unordered_map< forced_C*, int > mMinY;
	unordered_map< forced_C*, int > mMaxY;
	
	for (int i = 0; i < cNF.m_vForced.size(); i++)
	{
		forced_C *pF = cNF.m_vForced[i];
		instance_C *pInst = pF->m_pInstance;
		
		//cout<< m_vTargetGraph.size() <<endl ;
		int nX = pInst->getPlacedX();
		int nY = pInst->getPlacedY();
		
		if( nMinXCount == 1 )
		{
			if( nX == nMinX )
				mMinX[pF] = n2MinX;	
			else
				mMinX[pF] = nMinX;
		}
		else
		{
			mMinX[ pF ] = nMinX;
		}

		if( nMaxXCount == 1 )
		{
			if( nX == nMaxX )
				mMaxX[pF] = n2MaxX;
			else
				mMaxX[pF] = nMaxX;
		}
		else
		{
			mMaxX[ pF ] = nMaxX;
		}
		
		if( nMinYCount == 1 )
		{
			if( nY == nMinY )
				mMinY[pF] = n2MinY;
			else
				mMinY[pF] = nMinY;
		}
		else
		{
			mMinY[pF] = nMinY;
		}
		
		if( nMaxYCount == 1 )
		{
			if( nY == nMaxY )
				mMaxY[pF] = n2MaxY;
			else
				mMaxY[pF] = nMaxY;
		}
		else
		{
			mMaxY[pF] = nMaxY;
		}

	}

	cNF.m_nCenterX = nTotalX;
	cNF.m_nCenterY = nTotalY;
	cNF.m_nXHalf = nHalfX;
	cNF.m_nYHalf = nHalfY;
	cNF.m_nMinX = nMinX;
	cNF.m_nMaxX = nMaxX;
	cNF.m_nMinY = nMinY;
	cNF.m_nMaxY = nMaxY;
	cNF.m_n2MinX = n2MinX;
	cNF.m_n2MaxX = n2MaxX;
	cNF.m_n2MinY = n2MinY;
	cNF.m_n2MaxY = n2MaxY;
	cNF.m_nMinXCount = nMinXCount;
	cNF.m_nMaxXCount = nMaxXCount;
	cNF.m_nMinYCount = nMinYCount;
	cNF.m_nMaxYCount = nMaxYCount;
	
	cNF.m_mMinX = mMinX;
	cNF.m_mMaxX = mMaxX;
	cNF.m_mMinY = mMinY;
	cNF.m_mMaxY = mMaxY;

	return true;
}

bool router_C::calForcedNetwork(networkForced_C &cNF)
{

	// second version
	// put net on 2D graph
	// put target
	net_C* pNet = cNF.m_pNet;
	vector< instance_C* > vInst = pNet->getInst();
	
	int nTmpX = -1;
	int nTmpY = -1;
	bool bInSameGrid = true;
	for( int i=0; i<vInst.size(); i++ )
	{
		int nX = vInst[i]->getPlacedX();
		int nY = vInst[i]->getPlacedY();
		nX = nX - m_nOffsetX;
		nY = nY - m_nOffsetY;
		m_vTargetGraph[nY][nX]++;
		if( i == 0 )
		{
			nTmpX = nX;
			nTmpY = nY;
			if( !vInst[i]->isMovable() )
			{
				bInSameGrid = false;
				//break;
			}
		}
		else
		{
			if( nTmpX != nX || nTmpY != nY || !vInst[i]->isMovable() )
			{
				bInSameGrid = false;
				//break;
			}
		}
	}

	if( bInSameGrid )
	{
		cNF.m_bSame = true;
	}
	else
		cNF.m_bSame = false;
	// put path
	vector< wire_C* > vWire = pNet->getWire();
	for( int i=0; i<vWire.size(); i++ )
	{
		gGrid_C* gGrid1 = NULL;
		gGrid_C* gGrid2 = NULL;
		vWire[i]->getGrid( gGrid1, gGrid2 );
		int nX1, nY1, nZ1, nX2, nY2, nZ2;
		gGrid1->getPosition( nX1, nY1, nZ1 );
		gGrid2->getPosition( nX2, nY2, nZ2 );
		if( nZ1 != nZ2 )
			continue;
		if( nX1 != nX2 )
		{
			int nDX = nX2 - nX1;
			nDX = nDX / abs( nDX );
			for( int nX = nX1; nX != nX2; nX = nX + nDX )
			{
				m_v2DGraph[nY1 - m_nOffsetY ][nX - m_nOffsetX ] = true;
			}
			m_v2DGraph[nY1 - m_nOffsetY ][nX2 - m_nOffsetX ] = true;

		}
		else if( nY1 != nY2 )
		{
			int nDY = nY2 - nY1;
			nDY = nDY / abs( nDY );
			for( int nY = nY1; nY != nY2; nY = nY + nDY )
			{
				m_v2DGraph[nY - m_nOffsetY ][nX1 - m_nOffsetX ] = true;
			}
			m_v2DGraph[nY2 - m_nOffsetY ][nX1 - m_nOffsetX ] = true;
		}
	}

	//cerr <<"Here"<<endl;
	int nMaxX = INT_MIN;
	int nMinX = INT_MAX;
	int nMaxY = INT_MIN;
	int nMinY = INT_MAX;
	int nMinXCount = 0;
	int nMaxXCount = 0;
	int nMinYCount = 0;
	int nMaxYCount = 0;
	vector< int > vLF;
	vector< int > vRF;
	vector< int > vTF;
	vector< int > vDF;
	for (int i = 0; i < cNF.m_vForced.size(); i++)
	{
		forced_C *cF = cNF.m_vForced[i];
		instance_C *pInst = cF->m_pInstance;
		// logger begin 0812
		/*
		if (!cF->m_bLockInX && !cF->m_bLockInY)
		{
			if (cNF.m_pNet->m_vFL[i] != 0)
				cF->m_nL++;
			if (cNF.m_pNet->m_vFR[i] != 0)
				cF->m_nR++;
			if (cNF.m_pNet->m_vFT[i] != 0)
				cF->m_nT++;
			if (cNF.m_pNet->m_vFD[i] != 0)
				cF->m_nD++;
		}
		*/
		// logger end
		//cout<< m_vTargetGraph.size() <<endl ;
		int nX = pInst->getPlacedX();
		int nY = pInst->getPlacedY();
		int nTmpMinX = nX;
		int nTmpMaxX = nX;
		int nTmpMinY = nY;
		int nTmpMaxY = nY;
		// logger begin 0823
		if ( cF->m_bLockInX || cF->m_bLockInY || m_vTargetGraph[nY - m_nOffsetY ][nX - m_nOffsetX ] > 1 )
		{
			vLF.push_back( 0 );
			vRF.push_back( 0 );
			vTF.push_back( 0 );
			vDF.push_back( 0 );
		}
		else
		{
			if( nX > m_nDX )
			{
				if( m_v2DGraph[nY - m_nOffsetY][nX - m_nOffsetX -1 ] )
				{
					for( int x=nX - 1; x >=m_nOffsetX; x-- )
					{
						if( m_v2DGraph[ nY - m_nOffsetY ][ x - m_nOffsetX ] )
							nTmpMinX = x;
						else
							break;
					}
					vDF.push_back( 1 );
				}
				else
					vDF.push_back( 0 );
			}
			else
				vDF.push_back( 0 );
				
			
			if( nX < m_nTX )
			{
				if( m_v2DGraph[nY - m_nOffsetY ][nX - m_nOffsetX +1] )
				{
					for( int x=nX + 1; x <= m_nTX; x++ )
					{
						if( m_v2DGraph[ nY - m_nOffsetY ][ x - m_nOffsetX ] )
							nTmpMaxX = x;
						else
							break;
					}	
					vTF.push_back( 1 );
				}
				else
					vTF.push_back( 0 );
			}
			else
				vTF.push_back( 0 );


			if( nY > m_nDY )
			{
				if( m_v2DGraph[nY - m_nOffsetY -1][ nX - m_nOffsetX ] )
				{
					for( int y=nY - 1; y >= m_nOffsetY; y-- )
					{
						if( m_v2DGraph[ y - m_nOffsetY ][ nX - m_nOffsetX ] )
							nTmpMinY = y;
						else
							break;
					}	
					vLF.push_back( 1 );
				}
				else
					vLF.push_back( 0 );
			}
			else
				vLF.push_back( 0 );

			
			if( nY < m_nTY )
			{
				if( m_v2DGraph[nY - m_nOffsetY +1][ nX - m_nOffsetX ] )
				{
					for( int y=nY + 1; y <= m_nTY; y++ )
					{
						if( m_v2DGraph[ y - m_nOffsetY ][ nX - m_nOffsetX ] )
							nTmpMaxY = y;
						else
							break;
					}	
					vRF.push_back( 1 );
				}
				else
					vRF.push_back( 0 );
			}
			else
				vRF.push_back( 0 );

		}
		cNF.m_mPLF[ cF ] = vLF.back();
		cNF.m_mPRF[ cF ] = vRF.back();
		cNF.m_mPTF[ cF ] = vTF.back();
		cNF.m_mPDF[ cF ] = vDF.back();
		/*
		if( vLF.back() == 1 && vRF.back() == 0 && vTF.back() == 0 && vDF.back() == 0 )
			nTmpMaxY = nTmpMinY;
		if( vLF.back() == 0 && vRF.back() == 1 && vTF.back() == 0 && vDF.back() == 0 )
			nTmpMinY = nTmpMaxY;

		if( vLF.back() == 0 && vRF.back() == 0 && vTF.back() == 0 && vDF.back() == 1 )
			nTmpMaxX = nTmpMinX;
		if( vLF.back() == 0 && vRF.back() == 0 && vTF.back() == 1 && vDF.back() == 0 )
			nTmpMinX = nTmpMaxX;
		*/
		if( vLF.back() == 1 && vRF.back() == 0 )
			nTmpMaxY = nTmpMinY;
		if( vLF.back() == 0 && vRF.back() == 1 )
			nTmpMinY = nTmpMaxY;

		if( vTF.back() == 0 && vDF.back() == 1 )
			nTmpMaxX = nTmpMinX;
		if( vTF.back() == 1 && vDF.back() == 0 )
			nTmpMinX = nTmpMaxX;

		cNF.m_mMinX[ cF ] = nTmpMinX;
		cNF.m_mMaxX[ cF ] = nTmpMaxX;
		cNF.m_mMinY[ cF ] = nTmpMinY;
		cNF.m_mMaxY[ cF ] = nTmpMaxY;
		//cerr << "logger end"<<endl;
		// logger end
		
		if (nX < nMinX)
		{
			nMinX = nX;
			nMinXCount = 1;
		}
		else if (nX == nMinX)
		{
			nMinXCount++;
		}

		if (nX > nMaxX)
		{
			nMaxX = nX;
			nMaxXCount = 1;
		}
		else if (nX == nMaxX)
		{

			nMaxXCount++;
		}

		if (nY < nMinY)
		{
			nMinY = nY;
			nMinYCount = 1;
		}
		else if (nY == nMinY)
		{
			nMinYCount++;
		}

		if (nY > nMaxY)
		{
			nMaxY = nY;
			nMaxYCount = 1;
		}
		else if (nY == nMaxY)
		{
			nMaxYCount++;
		}


	}

	// reset the data
	for( int i=0; i<vInst.size(); i++ )
	{
		int nX = vInst[i]->getPlacedX();
		int nY = vInst[i]->getPlacedY();
		nX = nX - m_nOffsetX;
		nY = nY - m_nOffsetY;
		m_vTargetGraph[nY][nX] = 0;
	}

	for( int i=0; i<vWire.size(); i++ )
	{
		gGrid_C* gGrid1 = NULL;
		gGrid_C* gGrid2 = NULL;
		vWire[i]->getGrid( gGrid1, gGrid2 );
		int nX1, nY1, nZ1, nX2, nY2, nZ2;
		gGrid1->getPosition( nX1, nY1, nZ1 );
		gGrid2->getPosition( nX2, nY2, nZ2 );
		if( nZ1 != nZ2 )
			continue;
		if( nX1 != nX2 )
		{
			int nDX = nX2 - nX1;
			nDX = nDX / abs( nDX );
			for( int nX = nX1; nX != nX2; nX = nX + nDX )
			{
				m_v2DGraph[nY1 - m_nOffsetY ][nX - m_nOffsetX ] = false;
			}
			m_v2DGraph[nY1 - m_nOffsetY ][nX2 - m_nOffsetX ] = false;

		}
		else if( nY1 != nY2 )
		{
			int nDY = nY2 - nY1;
			nDY = nDY / abs( nDY );
			for( int nY = nY1; nY != nY2; nY = nY + nDY )
			{
				m_v2DGraph[nY - m_nOffsetY ][nX1 - m_nOffsetX ] = false;
			}
			m_v2DGraph[nY2 - m_nOffsetY ][nX1 - m_nOffsetX] = false;
		}
	}

	cNF.m_nMinX = nMinX;
	cNF.m_nMaxX = nMaxX;
	cNF.m_nMinY = nMinY;
	cNF.m_nMaxY = nMaxY;
	cNF.m_nMinXCount = nMinXCount;
	cNF.m_nMaxXCount = nMaxXCount;
	cNF.m_nMinYCount = nMinYCount;
	cNF.m_nMaxYCount = nMaxYCount;

	return true;
}

instance_C *router_C::pickHasMovedInstanceToMove()
{
	//cout<<"Picking Instance..."<<endl;
	vector<int> vSumOfForced;
	vector<int> vMaxForced;
	int nMaxSForced = 0; // sum of forced;

	vector<forced_C *> vForced;
	for (int i = 0; i < m_vForced.size(); i++)
	{
		if (m_vForced[i].m_pInstance->hasBeenMoved())
			vForced.push_back(&m_vForced[i]);
	}

	for (int i = 0; i < vForced.size(); i++)
	{
		forced_C *pF = vForced[i];
		int nFX;
		if (pF->m_bLockInX)
			nFX = 0;
		else
			nFX = abs(pF->m_nR - pF->m_nL);
		int nFY;
		if (pF->m_bLockInY)
			nFY = 0;
		else
			nFY = abs(pF->m_nT - pF->m_nD);
		int nSumOfForced = nFX + nFY;
		int nMaxForced = max(nFX, nFY);
		nMaxSForced = max(nMaxSForced, nSumOfForced);
		vSumOfForced.push_back(nSumOfForced);
		vMaxForced.push_back(nMaxForced);
	}
	// change at 0711 21:00
	/*	
	vector< int > vForcedId;
	for( int i=0; i<vSumOfForced.size(); i++ )
	{
		if( vSumOfForced[i] == nMaxSForced )
			vForcedId.push_back(i);
	}

	int nPickMaxForced = 0;
	int nPickId = -1;
	for( int i=0; i<vForcedId.size(); i++ )
	{
		int nId = vForcedId[i];
		if( vMaxForced[nId] > nPickMaxForced )
		{
			nPickMaxForced = vMaxForced[nId];
			nPickId = nId;
		}
	}
	*/
	//cout<<"Max"<<nMaxSForced<<endl;
	int nPickId = -1;
	
	for (int i = 0; i < vSumOfForced.size(); i++)
	{
		if (vSumOfForced[i] == nMaxSForced && vSumOfForced[i] != 0)
		{
			nPickId = i;
			//cout<<" Forced: "<<vSumOfForced[i];
			break;
		}
	}
	
	/*
	int nMaxGain = 0;
	for( int i=0; i<vForced.size(); i++ )
	{
		if( !vForced[i]->m_bLockInX && !vForced[i]->m_bLockInY && vForced[i]->m_nGain > nMaxGain )
		{
			nMaxGain = vForced[i]->m_nGain;
			nPickId = i;
		}
	}
	*/
	// end change

	/*
	int nPickMaxForced = 0;
	int nPickId = -1;
	for( int i=0; i<vMaxForced.size(); i++ )
	{
		if( vMaxForced[i] > nPickMaxForced )
		{
			nPickMaxForced = vMaxForced[i];
			nPickId = i;
		}
	}
	*/
	//cout<<nPickMaxForced<<endl;
	if (nPickId < 0)
	{
		//cout<<"No instances are picked"<<endl;
		return NULL;
	}
	//cout<<"Pick instance: "<<m_vForced[nPickId].m_pInstance->getName()<<endl;
	//cout<<nPickId<<endl;
	return vForced[nPickId]->m_pInstance;
}

vector< instance_C* > router_C::pickInstanceToMove( int nNumInst )
{

}

vector< forced_C* > router_C::moveingCellCollection_ver2( int nForcedId )
{
	vector< forced_C* > vAvailMovedForced;
	vector< forced_C* > vCandidateForced;
	set< forced_C* > sForced;
	set< networkForced_C* > sNet;
	set< networkForced_C* > sCommandNet;
	
	forced_C* pTarget = &m_vForced[ nForcedId ];
	vector< networkForced_C* > &vTmpNet = pTarget->m_vNetwork;
	sForced.insert( pTarget );	
	// collecting the forced that connect to the target
	//cout << "Collect nets: ";
	vector< int > vLF;
	vector< int > vRF;
	vector< int > vTF;
	vector< int > vDF;

	for( int i=0; i<vTmpNet.size(); i++ )
	{
		//cout << vTmpNet[i]->m_pNet->getName() << " ";
		vector< forced_C* > &vTmpForced = vTmpNet[i]->m_vForced;
		int nLF = 0;
		int nRF = 0;
		int nTF = 0;
		int nDF = 0;
		bool bHasFix = false;
		for( int j=0; j<vTmpForced.size(); j++ )
		{
			nLF = nLF + vTmpForced[j]->m_nL;	
			nRF = nRF + vTmpForced[j]->m_nR;	
			nTF = nTF + vTmpForced[j]->m_nT;	
			nDF = nDF + vTmpForced[j]->m_nD;	
			if( !vTmpForced[j]->m_pInstance->isMovable() )
			{
				bHasFix = true;
				break;
			}
		}

		if( bHasFix || m_vMovedInstance.size() + vTmpForced.size() > m_pDesign->getMaxCellConstraint() )
		{
			vLF.push_back( 0 );
			vRF.push_back( 0 );
			vTF.push_back( 0 );
			vDF.push_back( 0 );
		
		}
		else
		{
			vLF.push_back( nLF );
			vRF.push_back( nRF );
			vTF.push_back( nTF );
			vDF.push_back( nDF );
		}
	}
	
	int nIndex = -1;
	int nMaxForced = abs( pTarget->m_nR - pTarget->m_nL ) + abs( pTarget->m_nT - pTarget->m_nD );
	double dMaxForced = 0;
	/*	
	for( int i=0; i<vTmpNet.size(); i++ )
	{
		int nTmpForced = abs( vLF[i] - vRF[i] ) + abs( vTF[i] - vDF[i] );
		if( nTmpForced > nMaxForced )
		{
			nMaxForced = nTmpForced;
			nIndex = i;
		}
	}
	*/
	
	for( int i=0; i<vTmpNet.size(); i++ )
	{
		double dTmpForced = abs( vLF[i] - vRF[i] ) + abs( vTF[i] - vDF[i] );
		dTmpForced = dTmpForced / vTmpNet[i]->m_vForced.size();
		if( dTmpForced > dMaxForced )
		{
			dMaxForced = dTmpForced;
			nIndex = i;
		}
	}
	
	if( nIndex != -1 )
	{
		cout << "Pick net: " << vTmpNet[ nIndex ]->m_pNet->getName() << endl;
		for( int i=0; i<vTmpNet[ nIndex ]->m_vForced.size(); i++ )
		{
			if( sForced.count( vTmpNet[ nIndex ]->m_vForced[i] ) == 0 )
			{
				vAvailMovedForced.push_back( vTmpNet[ nIndex ]->m_vForced[i] );
				sForced.insert( vTmpNet[ nIndex]->m_vForced[i] );
			}
		}
// graph checker
/*
		vector< forced_C* > vTmpF = vTmpNet[nIndex]->m_vForced;
		set< networkForced_C* > sTmpNF;
		sTmpNF.insert( vTmpNet[ nIndex] );
		bool bFirst = true;
		for( int i=0; i < vTmpF.size(); i++ )
		{
			vector< networkForced_C* > vTmpNF = vTmpF[i]->m_vNetwork;	
			for( int j=0; j<vTmpNF.size(); j++ )
			{
				if( sTmpNF.count( vTmpNF[j] ) == 0 )
				{
					if( bFirst )
					{
						matlab_graph( "Before", vTmpNF[ j ]->m_pNet->getName(), m_pDesign, false, 0 );		
						bFirst = false;
					}
					else
						matlab_graph( "Before", vTmpNF[ j ]->m_pNet->getName(), m_pDesign, false, 1 );		
					sTmpNF.insert( vTmpNF[ j ] );
					
				}
			}
		}
		matlab_graph( "Before", vTmpNet[nIndex]->m_pNet->getName(), m_pDesign, true, 1 );		
		getchar();	
*/
//
	}

	return vAvailMovedForced;
}

bool router_C::checkLength()
{
	int nGridLength = 0;
	for( int z=0; z<m_vGraph.size(); z++ )
	{
		for( int y=0; y<m_vGraph[z].size(); y++ )
		{
			for( int x=0; x<m_vGraph[z][y].size(); x++ )
			{
				gGrid_C* pGrid = m_vGraph[z][y][x];
				nGridLength = nGridLength + pGrid->getNet().size();
			}
		}
	}

	int nLength = 0;
	for( int i=0; i<m_vNetworkForced.size(); i++ )
	{
		nLength = nLength + m_vNetworkForced[i].m_pNet->getLength();
	}

	cout << "Grid Length: " << nGridLength << " Calculate Length: " << nLength << endl;
	return true;
}


vector< forced_C* > router_C::moveingCellCollection( int nForcedId )
{
	vector< forced_C* > vAvailMovedForced;
	vector< forced_C* > vCandidateForced;
	set< forced_C* > sForced;
	set< networkForced_C* > sNet;
	set< networkForced_C* > sCommandNet;
	
	forced_C* pTarget = &m_vForced[ nForcedId ];
	vector< networkForced_C* > &vTmpNet = pTarget->m_vNetwork;
	sForced.insert( pTarget );	
	// collecting the forced that connect to the target
	//cout << "Collect nets: ";
	for( int i=0; i<vTmpNet.size(); i++ )
	{
		//cout << vTmpNet[i]->m_pNet->getName() << " ";
		sNet.insert( vTmpNet[i] );
		vector< forced_C* > &vTmpForced = vTmpNet[i]->m_vForced;
		for( int j=0; j<vTmpForced.size(); j++ )
		{
			if( sForced.count( vTmpForced[j] ) == 0 )
			{
				sForced.insert( vTmpForced[j] );
				vCandidateForced.push_back( vTmpForced[j] );
			}
		}
	}
	//cout << endl;

	//cout << "Candidates: " << vCandidateForced.size() << endl;
	for( int i=0; i<vCandidateForced.size(); i++ )
	{
		if( !vCandidateForced[i]->m_pInstance->isMovable() )
		{
			continue;
		}
		//cout << vCandidateForced[i]->m_pInstance->getName() << " ";
		vector< networkForced_C* > vTmpNet = vCandidateForced[i]->m_vNetwork;
		int nT = 0;
		int nD = 0;
		int nR = 0;
		int nL = 0;
		vector< net_C* > vCommonNet;
		for( int j=0; j<vTmpNet.size(); j++ )
		{
			if( sNet.count( vTmpNet[j]) == 1 )
			{
				vCommonNet.push_back( vTmpNet[j]->m_pNet );
			}
		}
		
		calForcedModel( *vCandidateForced[i], vCommonNet, nT, nD, nR, nL );
		
		int nXF;
		int nYF;
		int nTXF;
		int nTYF;
		if( nT - nD < 0 )
			nYF = -1;
		else if( nT - nD > 0 )
			nYF = 1;
		else
			nYF = 0;
		
		if( nR - nL < 0 )
			nXF = -1;
		else if( nR - nL > 0 )
			nXF = 1;
		else
			nXF = 0;
		
		if( pTarget->m_nT - pTarget->m_nD < 0 )
			nTYF = -1;
		else if( pTarget->m_nT - pTarget->m_nD > 0 )
			nTYF = 1;
		else
			nTYF = 0;
		
		if( pTarget->m_nR - pTarget->m_nL < 0 )
			nTXF = -1;
		else if( pTarget->m_nR - pTarget->m_nL > 0 )
			nTXF = 1;
		else
			nTXF = 0;
		
		//if( abs( nT - nD ) + abs( nR - nL ) > 0 )
		if( nXF * nTXF > 0 && nYF * nTYF > 0 )
			vAvailMovedForced.push_back( vCandidateForced[i] );
		//cout <<  abs( nT - nD ) + abs( nR - nL ) << endl;
	}
	//cout << endl;

	return vAvailMovedForced;

}


vector< forced_C* > router_C::recalForced( int nForcedId )
{
	vector< forced_C* > vAvailMovedForced;
	vector< forced_C* > vCandidateForced;
	set< forced_C* > sForced;
	set< networkForced_C* > sNet;
	set< networkForced_C* > sCommandNet;
	
	forced_C* pTarget = &m_vForced[ nForcedId ];
	vector< networkForced_C* > &vTmpNet = pTarget->m_vNetwork;
	sForced.insert( pTarget );	
	// collecting the forced that connect to the target
	//cout << "Collect nets: ";
	for( int i=0; i<vTmpNet.size(); i++ )
	{
		//cout << vTmpNet[i]->m_pNet->getName() << " ";
		sNet.insert( vTmpNet[i] );
		vector< forced_C* > &vTmpForced = vTmpNet[i]->m_vForced;
		for( int j=0; j<vTmpForced.size(); j++ )
		{
			if( sForced.count( vTmpForced[j] ) == 0 )
			{
				sForced.insert( vTmpForced[j] );
				vCandidateForced.push_back( vTmpForced[j] );
			}
		}
	}
	//cout << endl;

	//cout << "Candidates: " << vCandidateForced.size() << endl;
	for( int i=0; i<vCandidateForced.size(); i++ )
	{
		if( !vCandidateForced[i]->m_pInstance->isMovable() )
		{
			continue;
		}
		//cout << vCandidateForced[i]->m_pInstance->getName() << " ";
		vector< networkForced_C* > vTmpNet = vCandidateForced[i]->m_vNetwork;
		int nT = 0;
		int nD = 0;
		int nR = 0;
		int nL = 0;
		vector< net_C* > vCommonNet;
		for( int j=0; j<vTmpNet.size(); j++ )
		{
			if( sNet.count( vTmpNet[j]) == 1 )
			{
				vCommonNet.push_back( vTmpNet[j]->m_pNet );
			}
		}
		
		calForcedModel( *vCandidateForced[i], vCommonNet, nT, nD, nR, nL );
		if( abs( nT - nD ) + abs( nR - nL ) > 0 )
			vAvailMovedForced.push_back( vCandidateForced[i] );
		//cout <<  abs( nT - nD ) + abs( nR - nL ) << endl;
	}
	//cout << endl;

	return vAvailMovedForced;

}

boundry_C* router_C::pickInstanceToMove_ver5()
{
	cout<<"Picking Instances..."<<endl;
	boundry_C* pBestB = NULL;
	int nMaxForced = 0;
	int nLength = 0;
	vector< instance_C* > vInst;

	//cout << "Cal Instance Forced" << endl;
	for( int i=0; i<m_vBoundry.size(); i++ )
	{
		boundry_C* pTmpB = m_vBoundry[i];
		if( pTmpB->m_bLock || !pTmpB->m_bMovable )
			continue;
		
		int nMoveCount = 0;
		for( int j=0; j<pTmpB->m_vForced.size(); j++ )
		{
			if( !pTmpB->m_vForced[j]->m_pInstance->hasBeenMoved() )
				nMoveCount++;
		}

		if( nMoveCount + m_vMovedInstance.size() > m_pDesign->getMaxCellConstraint() )
			continue;

		int nTmpForced = 0;
		nTmpForced = pTmpB->m_nGain;
		//nTmpForced = pTmpB->m_nPosF;
		//cout << pTmpB->m_pNetwork->m_pNet->getName() << " " << pTmpB->m_cType << " " << nTmpForced << endl;
		if( nMaxForced < nTmpForced )
		{
			pBestB = pTmpB;
			nMaxForced = nTmpForced;
			nLength = ( pBestB->m_pNetwork->m_nMaxX - pBestB->m_pNetwork->m_nMinX ) + ( pBestB->m_pNetwork->m_nMaxY - pBestB->m_pNetwork->m_nMinY );
		}
		else if( nMaxForced == nTmpForced )
		{
			int nTmpLength = ( pTmpB->m_pNetwork->m_nMaxX - pTmpB->m_pNetwork->m_nMinX ) + ( pTmpB->m_pNetwork->m_nMaxY - pTmpB->m_pNetwork->m_nMinY );
			if( nTmpLength > nLength )
			{
				nLength = nTmpLength;
				pBestB = pTmpB;
			}
		}
	}
	
	return pBestB;
	/*
	if( pBestB == NULL )
		return pBestB;
	else
	{
		for( int i=0; i < pBestB->m_vForced.size(); i++ )
			vInst.push_back( pBestB->m_vForced[i]->m_pInstance );
		
		return vInst;
	}
	*/
}

boundry_C* router_C::pickInstanceToMove_ver4()
{
	cout<<"Picking Instances..."<<endl;
	boundry_C* pBestB = NULL;
	int nMaxForced = 0;
	vector< instance_C* > vInst;

	//cout << "Cal Instance Forced" << endl;
	for( int i=0; i<m_vBoundry.size(); i++ )
	{
		boundry_C* pTmpB = m_vBoundry[i];
		if( pTmpB->m_bLock || !pTmpB->m_bMovable )
			continue;
		
		int nMoveCount = 0;
		for( int j=0; j<pTmpB->m_vForced.size(); j++ )
		{
			if( !pTmpB->m_vForced[j]->m_pInstance->hasBeenMoved() )
				nMoveCount++;
		}

		if( nMoveCount + m_vMovedInstance.size() > m_pDesign->getMaxCellConstraint() )
			continue;

		int nTmpForced = 0;
		nTmpForced = pTmpB->m_nGain;
		//nTmpForced = pTmpB->m_nPosF;
		//cout << pTmpB->m_pNetwork->m_pNet->getName() << " " << pTmpB->m_cType << " " << nTmpForced << endl;
		if( nMaxForced < nTmpForced )
		{
			pBestB = pTmpB;
			nMaxForced = nTmpForced;
		}
	}
	
	return pBestB;
	/*
	if( pBestB == NULL )
		return pBestB;
	else
	{
		for( int i=0; i < pBestB->m_vForced.size(); i++ )
			vInst.push_back( pBestB->m_vForced[i]->m_pInstance );
		
		return vInst;
	}
	*/
}

boundry_C* router_C::pickInstanceToMove_ver3()
{
	cout<<"Picking Instances..."<<endl;
	boundry_C* pBestB = NULL;
	int nMaxForced = 0;
	vector< instance_C* > vInst;

	//cout << "Cal Instance Forced" << endl;
	for( int i=0; i<m_vBoundry.size(); i++ )
	{
		boundry_C* pTmpB = m_vBoundry[i];
		if( pTmpB->m_bLock || !pTmpB->m_bMovable )
			continue;
		
		int nMoveCount = 0;
		for( int j=0; j<pTmpB->m_vForced.size(); j++ )
		{
			if( !pTmpB->m_vForced[j]->m_pInstance->hasBeenMoved() )
				nMoveCount++;
		}

		if( nMoveCount + m_vMovedInstance.size() > m_pDesign->getMaxCellConstraint() )
			continue;

		int nTmpForced = 0;
		if( pTmpB->m_cType == 'L' )
		{
			nTmpForced = pTmpB->m_nR - pTmpB->m_nL - pTmpB->m_nCY;
	//		nTmpForced = nTmpForced + max( 0, pTmpB->m_nT + pTmpB->m_nD - pTmpB->m_nCX );
		}
		else if( pTmpB->m_cType == 'R' )
		{
			nTmpForced = pTmpB->m_nL - pTmpB->m_nR - pTmpB->m_nCY;
	//		nTmpForced = nTmpForced + max( 0, pTmpB->m_nT + pTmpB->m_nD - pTmpB->m_nCX );
		}
		else if( pTmpB->m_cType == 'D' )
		{
			nTmpForced = pTmpB->m_nT - pTmpB->m_nD - pTmpB->m_nCX;
	//		nTmpForced = nTmpForced + max( 0, pTmpB->m_nR + pTmpB->m_nL - pTmpB->m_nCY );
		}
		else if( pTmpB->m_cType == 'T' )
		{
			nTmpForced = pTmpB->m_nD - pTmpB->m_nT - pTmpB->m_nCX;
	//		nTmpForced = nTmpForced + max( 0, pTmpB->m_nR + pTmpB->m_nL - pTmpB->m_nCY ) ;
		}
		//cout << pTmpB->m_pNetwork->m_pNet->getName() << " " << pTmpB->m_cType << " " << nTmpForced << endl;
		if( nMaxForced < nTmpForced )
		{
			pBestB = pTmpB;
			nMaxForced = nTmpForced;
		}
	}
	
	return pBestB;
	/*
	if( pBestB == NULL )
		return pBestB;
	else
	{
		for( int i=0; i < pBestB->m_vForced.size(); i++ )
			vInst.push_back( pBestB->m_vForced[i]->m_pInstance );
		
		return vInst;
	}
	*/
}

instance_C *router_C::pickInstanceToMove_ver2()
{
	cout<<"Picking Instance..."<<endl;
	vector<int> vSumOfForced;
	vector<int> vMaxForced;
	vector<int> vArea;
	int nMaxSForced = 0; // sum of forced;

	vector<forced_C *> vForced;
	/*
	for (int i = 0; i < m_vForced.size(); i++)
	{
		vForced.push_back(&m_vForced[i]);
	}
	*/
	int nGlobalSumOfForced = 0;
	int nGlobalMaxForced = 0;
	//int nMinArea = INT_MAX;
	int nMaxArea = 0;
	int nMinAreaId = -1;
	int nPickId = -1;
	int nMinConnection = 0;
	for (int i = 0; i < m_vForced.size(); i++)
	{
		forced_C *pF = &m_vForced[i];
		int nFX;
		if (pF->m_bLockInX)
			nFX = 0;
		else
			nFX = abs(pF->m_nR - pF->m_nL) - pF->m_nC;
		int nFY;
		if (pF->m_bLockInY)
			nFY = 0;
		else
			nFY = abs(pF->m_nT - pF->m_nD) - pF->m_nC;;
		int nSumOfForced = nFX + nFY;
		int nMaxForced = max(nFX, nFY);
		//int nArea = ( pF->m_nBoundX2 - pF->m_nBoundX1 +1 )*( pF->m_nBoundY2 - pF->m_nBoundY1 + 1 );
		//nMaxSForced = max(nMaxSForced, nSumOfForced);
		//cout << nArea <<endl;
		if( nMaxSForced < nSumOfForced )
		{
			nMaxSForced = nSumOfForced;
			//nMinConnection = pF->m_vNetwork.size();
			nPickId = i;
			//nMaxArea = nArea;
		}
		vSumOfForced.push_back(nSumOfForced);
		vMaxForced.push_back(nMaxForced);
	}
	//cout<<nPickMaxForced<<endl;
	//int nPickId = nMinAreaId;
	if (nPickId < 0)
	{
		//cout<<"No instances are picked"<<endl;
		return NULL;
	}
	cout<<"Pick instance: "<<m_vForced[nPickId].m_pInstance->getName() << " Max forced: " << nMaxSForced <<endl;
	//cout<<nPickId<<endl;
	return m_vForced[nPickId].m_pInstance;
}

instance_C *router_C::pickInstanceToMove()
{
	//cout<<"Picking Instance..."<<endl;
	vector<int> vSumOfForced;
	vector<int> vMaxForced;
	vector<int> vArea;
	int nMaxSForced = 0; // sum of forced;

	vector<forced_C *> vForced;
	/*
	for (int i = 0; i < m_vForced.size(); i++)
	{
		vForced.push_back(&m_vForced[i]);
	}
	*/
	int nGlobalSumOfForced = 0;
	int nGlobalMaxForced = 0;
	//int nMinArea = INT_MAX;
	int nMaxArea = 0;
	int nMinAreaId = -1;
	int nPickId = -1;
	int nMinConnection = 0;
	for (int i = 0; i < m_vForced.size(); i++)
	{
		forced_C *pF = &m_vForced[i];
		int nFX;
		if (pF->m_bLockInX)
			nFX = 0;
		else
			nFX = abs(pF->m_nR - pF->m_nL);
		int nFY;
		if (pF->m_bLockInY)
			nFY = 0;
		else
			nFY = abs(pF->m_nT - pF->m_nD);
		int nSumOfForced = nFX + nFY;
		int nMaxForced = max(nFX, nFY);
		//int nArea = ( pF->m_nBoundX2 - pF->m_nBoundX1 +1 )*( pF->m_nBoundY2 - pF->m_nBoundY1 + 1 );
		//nMaxSForced = max(nMaxSForced, nSumOfForced);
		//cout << nArea <<endl;
		if( nMaxSForced < nSumOfForced )
		{
			nMaxSForced = nSumOfForced;
			//nMinConnection = pF->m_vNetwork.size();
			nPickId = i;
			//nMaxArea = nArea;
		}
		//else if( nMaxSForced == nSumOfForced )
		//{
		//	if( nMinConnection > pF->m_vNetwork.size() )
		//	{
		//		nMinConnection = pF->m_vNetwork.size();
		//		nPickId = i;
		//	}
			//if( nMaxArea < nArea )
			//	nMaxArea = nArea;
		//}
		vSumOfForced.push_back(nSumOfForced);
		vMaxForced.push_back(nMaxForced);
		//vArea.push_back( nArea );
		/*
		if( nSumOfForced > nGlobalSumOfForced )
		{
			nGlobalSumOfForced = nSumOfForced;
			nGlobalMaxForced = nMaxForced;
			nMinArea = nArea;
			nMinAreaId = i;
		}
		else if( nSumOfForced == nGlobalSumOfForced )
		{
			if( nMaxForced > nGlobalMaxForced )
			{
				nMaxForced = nGlobalMaxForced;
				nMinArea = nArea;
				nMinAreaId = i;
			}
			else if( nMaxForced ==  nGlobalMaxForced )
			{
				if( nMinArea < nArea )
				{
					nMinArea = nArea;
					nMinAreaId = i;
				}
			}
		}
		*/
	}
	// change at 0711 21:00
	/*	
	vector< int > vForcedId;
	for( int i=0; i<vSumOfForced.size(); i++ )
	{
		if( vSumOfForced[i] == nMaxSForced )
			vForcedId.push_back(i);
	}

	int nPickMaxForced = 0;
	int nPickId = -1;
	for( int i=0; i<vForcedId.size(); i++ )
	{
		int nId = vForcedId[i];
		if( vMaxForced[nId] > nPickMaxForced )
		{
			nPickMaxForced = vMaxForced[nId];
			nPickId = nId;
		}
	}
	*/
	//cout<<"Max"<<nMaxSForced<<endl;
	/*
	int nPickId = -1;
	
	for (int i = 0; i < vSumOfForced.size(); i++)
	{
		if (vSumOfForced[i] == nMaxSForced && vSumOfForced[i] != 0 )
		{
			nPickId = i;
			//cout<<" Forced: "<<vSumOfForced[i];
			break;
		}
	}
	*/
	/*
	int nMaxGain = 0;
	for( int i=0; i<vForced.size(); i++ )
	{
		if( !vForced[i]->m_bLockInX && !vForced[i]->m_bLockInY && vForced[i]->m_nGain > nMaxGain )
		{
			nMaxGain = vForced[i]->m_nGain;
			nPickId = i;
		}
	}
	*/

	// end change

	//cout<<nPickMaxForced<<endl;
	//int nPickId = nMinAreaId;
	if (nPickId < 0)
	{
		//cout<<"No instances are picked"<<endl;
		return NULL;
	}
	cout<<"Pick instance: "<<m_vForced[nPickId].m_pInstance->getName() << " Max forced: " << nMaxSForced <<endl;
	//cout<<nPickId<<endl;
	return m_vForced[nPickId].m_pInstance;
}

// added at 0705 02:00
//ripup version
vector<gGrid_C *> router_C::findPlaceToMove(instance_C *pInst, vector<net_C *> &vRipNet)
{
	int nOrigX = pInst->getPlacedX();
	int nOrigY = pInst->getPlacedY();

	vector<gGrid_C *> vBestGrid;

	forced_C cF;
	if (pInst == m_vForced[pInst->getId()].m_pInstance)
	{
		cF = m_vForced[pInst->getId()];
	}
	else
	{
		for (int i = 0; i < m_vForced.size(); i++)
		{
			if (m_vForced[i].m_pInstance == pInst)
			{
				cF = m_vForced[i];
				break;
			}
		}
	}
	// setup search direction
	int nDX = 0;
	int nDY = 0;
	if (!cF.m_bLockInX)
		nDX = cF.m_nR - cF.m_nL;
	if (!cF.m_bLockInY)
		nDY = cF.m_nT - cF.m_nD;

	if (nDX == 0 && nDY == 0)
	{
		cout << "Warning: No operations will be executed. This instance has balanced forced" << endl;
		return vBestGrid;
		//	return NULL;
	}
	if (abs(nDX) >= abs(nDY))
	{
		nDX = nDX / abs(nDX);
		nDY = 0;
	}
	else
	{
		nDY = nDY / abs(nDY);
		nDX = 0;
	}

	// collecting all the correlated network
	vector<networkForced_C *> vNetWork = cF.m_vNetwork;
	vector<vector<instance_C *>> vBoundingBox;
	for (int i = 0; i < vNetWork.size(); i++)
	{
		vector<instance_C *> vInst;
		networkForced_C *pNF = vNetWork[i];
		vector<forced_C *> vF = pNF->m_vForced;
		for (int j = 0; j < vF.size(); j++)
		{
			instance_C *pTmpInst = vF[j]->m_pInstance;
			if (pTmpInst != pInst)
				vInst.push_back(pTmpInst);
		}
		vBoundingBox.push_back(vInst);
	}

	vector<int> vMin;
	vector<int> vMax;
	for (int i = 0; i < vBoundingBox.size(); i++)
	{
		vector<instance_C *> vInst = vBoundingBox[i];
		int nMax;
		int nMin;
		if (nDX != 0)
		{
			nMax = m_nDX;
			nMin = m_nTX;
			;
		}
		else
		{
			nMax = m_nDY;
			nMin = m_nTY;
		}

		for (int j = 0; j < vInst.size(); j++)
		{
			instance_C *pTmpInst = vInst[j];
			if (nDX != 0)
			{
				nMax = max(pTmpInst->getPlacedX(), nMax);
				nMin = min(pTmpInst->getPlacedX(), nMin);
			}
			else
			{
				nMax = max(pTmpInst->getPlacedY(), nMax);
				nMin = min(pTmpInst->getPlacedY(), nMin);
			}
		}
		vMin.push_back(nMin);
		vMax.push_back(nMax);
	}
	// moving instance
	int nBest;
	int nTmp;
	if (nDX != 0)
		nTmp = pInst->getPlacedX();
	else
		nTmp = pInst->getPlacedY();

	nBest = boundingBoxCost(nTmp, vMin, vMax);
	gGrid_C *pBestGrid = NULL;
	gGrid_C *pGrid = getGrid(m_pDesign, pInst->getPlacedX(), pInst->getPlacedY(), m_nDZ);
	while (graphTravel(m_pDesign, pGrid, nDX, nDY, 0) != NULL)
	{
		pGrid = graphTravel(m_pDesign, pGrid, nDX, nDY, 0);
		// check placeable
		if (!isPlaceable(pGrid, pInst, vRipNet))
			continue;
		//
		int nTmpX, nTmpY, nTmpZ;
		pGrid->getPosition(nTmpX, nTmpY, nTmpZ);
		if (nDX != 0)
			nTmp = nTmpX;
		else
			nTmp = nTmpY;

		int nTmpBest = boundingBoxCost(nTmp, vMin, vMax);
		if (nTmpBest <= nBest) // change from <
		{
			nBest = nTmpBest;
			pBestGrid = pGrid;
			vBestGrid.push_back(pBestGrid);
		}
		else
			break;
		//cout<<nBest<<endl;
	}
	pInst->setPlaced(nOrigX, nOrigY);
	//return pBestGrid;
	return vBestGrid;
}
// end added at 0705 02:00

// version 2
/*
gGrid_C* router_C::findPlaceToMove( instance_C* pInst )
{
	forced_C cF;
	if( pInst == m_vForced[ pInst->getId() ].m_pInstance )
	{
		cF = m_vForced[ pInst->getId() ];
	}
	else
	{
		for( int i=0; i<m_vForced.size(); i++ )
		{
			if( m_vForced[i].m_pInstance == pInst )
			{
				cF = m_vForced[i];
				break;
			}
		}
	}
	// setup search direction
	int nDX = 0;
	int nDY = 0;
	if( !cF.m_bLockInX )
		nDX = cF.m_nR - cF.m_nL;
	if( !cF.m_bLockInY )
		nDY = cF.m_nT - cF.m_nD;
	
	if( nDX == 0 && nDY == 0 )
	{
		cout<<"Warning: No operations will be executed. This instance has balanced forced"<<endl;
		return NULL;
	}
	if( abs( nDX ) >= abs( nDY ) )
	{
		nDX = nDX/abs(nDX); nDY = 0;
	}
	else
	{
		nDY = nDY/abs(nDY); nDX = 0; 
	}

	// collecting all the correlated network
	vector< networkForced_C* > vNetWork = cF.m_vNetwork;
	vector< vector< instance_C*> > vBoundingBox;
	for( int i=0; i<vNetWork.size(); i++ )
	{
		vector< instance_C* > vInst;
		networkForced_C* pNF = vNetWork[i];
		vector< forced_C* > vF = pNF->m_vForced;
		for( int j=0; j<vF.size(); j++ )
		{
			instance_C* pTmpInst = vF[j]->m_pInstance;
			if( pTmpInst != pInst )
				vInst.push_back( pTmpInst );
		}
		vBoundingBox.push_back( vInst );
	}
	
	vector<int> vMin;
	vector<int> vMax;
	vector<int> vPosition;
	for( int i=0; i<vBoundingBox.size(); i++ )
	{
		vector< instance_C* > vInst = vBoundingBox[i];
		int nMax;
		int nMin;
		if( nDX != 0 )
		{
			nMax = m_nDX;
			nMin = m_nTX;;
		}
		else
		{
			nMax = m_nDY;
			nMin = m_nTY;
		}


		for( int j=0; j<vInst.size(); j++ )
		{
			instance_C* pTmpInst = vInst[j];
			if( nDX != 0 )
			{
				nMax = max( pTmpInst->getPlacedX(), nMax );
				nMin = min( pTmpInst->getPlacedX(), nMin );
			}
			else
			{
				nMax = max( pTmpInst->getPlacedY(), nMax );
				nMin = min( pTmpInst->getPlacedY(), nMin );
			}
		}
		vMin.push_back( nMin );
		vMax.push_back( nMax );
		vPosition.push_back( nMax );
		vPosition.push_back( nMin );
	}

	//if( nDX != 0 )
	//	vPosition.push_back( pInst->getPlacedX() );
	//else
	//	vPosition.push_back( pInst->getPlacedY() );

	sort( vPosition.begin(), vPosition.end() );
	//for( int i=vPosition.size()-2; i>=0; i-- )
	//{
	//	if( vPosition[i] == vPosition[i+1] )
	//	{
	//		vPosition.erase( vPosition.begin() + i );	
	//	}
	//}
	// moving instance
	int nBest;
	int nTmp;
	
	//if( nDX != 0 )
	//	nTmp = pInst->getPlacedX();
	//else
	//	nTmp = pInst->getPlacedY();
	
	//for( int i=0; i<vPosition.size(); i++ )
	//{
	//	cout<<vPosition[i]<<" ";
	//}
	//cout<<endl;

	if( vPosition.size() % 2 != 0 )
	{
		int nPosition = ( vPosition.size() + 1 )/2 - 1;
		nTmp = vPosition[ nPosition ];
	}
	else
	{	
		int nPosition = vPosition.size()/2 - 1;
		//cout<<nPosition<<endl;
		if( nDX + nDY > 0 )
			nTmp = vPosition[ nPosition + 1 ];
		else 
			nTmp = vPosition[ nPosition ];
	}
	//cout<<nDX<<" "<<nDY<<" "<<nTmp<<endl;
	//nBest = boundingBoxCost( nTmp, vMin, vMax );
	gGrid_C* pBestGrid = NULL;
	gGrid_C* pGrid = NULL;
	if( nDX != 0 )
	{ 
		pGrid = getGrid( m_pDesign, nTmp, pInst->getPlacedY(), m_nDZ );
	}
	else
	{
		pGrid = getGrid( m_pDesign, pInst->getPlacedX(), nTmp, m_nDZ );
	}
	
	gGrid_C* pPlaceGrid = getGrid( m_pDesign, pInst->getPlacedX(), pInst->getPlacedY(), m_nDZ );
	
	while( pGrid != NULL && pGrid != pPlaceGrid )
	{
		// check placeable
		if( !isPlaceable( pGrid, pInst ) )
		{
			pGrid = graphTravel( m_pDesign, pGrid, -nDX, -nDY, 0 );
		}
		else
		{
			pBestGrid = pGrid;
			break;
		}
			//pGrid = graphTravel( m_pDesign, pGrid, -nDX, -nDY, 0 );
	}

	if( nDX != 0 )
		lockInstance( pInst, 'X' );
	else
		lockInstance( pInst, 'Y' );

	return pBestGrid;
}
*/

// added at 0705 02:00
vector<gGrid_C *> router_C::findPlaceToMove_ver4(instance_C *pInst, set< net_C* > &sNet, set< instance_C* > &sInst )
{
	int nOrigX = pInst->getPlacedX();
	int nOrigY = pInst->getPlacedY();

	vector<gGrid_C *> vBestGrid;

	forced_C *cF;
	if (pInst == m_vForced[pInst->getId()].m_pInstance)
	{
		cF = &m_vForced[pInst->getId()];
	}
	else
	{
		for (int i = 0; i < m_vForced.size(); i++)
		{
			if (m_vForced[i].m_pInstance == pInst)
			{
				cF = &m_vForced[i];
				break;
			}
		}
	}

	int &nStartX = cF->m_nBoundX1;
	int &nEndX = cF->m_nBoundX2;
	int &nStartY = cF->m_nBoundY1;
	int &nEndY = cF->m_nBoundY2;

	//if( pInst->getName() == "C1200" )
	//	cout<<"Range: "<<nStartX<<" "<<nStartY<<" "<<nEndX<<" "<<nEndY<<endl;

	for (int nX = nStartX; nX <= nEndX; nX++)
	{
		for (int nY = nStartY; nY <= nEndY; nY++)
		{
			gGrid_C *pPGrid = getGrid(m_pDesign, nX, nY, m_nOffsetZ);
			vBestGrid.push_back(pPGrid);
		}
	}
	// end added at 0711 02:00

	vector<int> &vMinX = cF->m_vMinX;
	vector<int> &vMaxX = cF->m_vMaxX;

	vector<int> &vMinY = cF->m_vMinY;
	vector<int> &vMaxY = cF->m_vMaxY;

	vector<int> vBestGridCost;
	for (int i = 0; i < vBestGrid.size(); i++)
	{
		int nTmpX, nTmpY, nTmpZ;
		gGrid_C *pG = vBestGrid[i];
		pG->getPosition(nTmpX, nTmpY, nTmpZ);

		int nCost = boundingBoxCost(nTmpX, vMinX, vMaxX) + boundingBoxCost(nTmpY, vMinY, vMaxY);

		vBestGridCost.push_back(nCost);
	}

	// added at 0706 19:30
	for (int i = 1; i < vBestGrid.size(); i++)
	{
		for (int j = i - 1; j >= 0; j--)
		{
			//if( !cF.m_bLockInX )
			//	nDX = cF.m_nR - cF.m_nL;
			//if( !cF.m_bLockInY )
			//	nDY = cF.m_nT - cF.m_nD;

			//int nTmpX, nTmpY, nTmpZ, nTmpB, nTmpF;
			gGrid_C *pBG = vBestGrid[j + 1];
			//pBG->getPosition( nTmpX, nTmpY, nTmpZ );

			//int nBCost = boundingBoxCost( nTmpX, vMinX, vMaxX )
			//	   + boundingBoxCost( nTmpY, vMinY, vMaxY ) ;
			int nBCost = vBestGridCost[j + 1];
			int nFCost = vBestGridCost[j];

			gGrid_C *pFG = vBestGrid[j];
			//pFG->getPosition( nTmpX, nTmpY, nTmpZ );

			//int nFCost = boundingBoxCost( nTmpX, vMinX, vMaxX )
			//	   + boundingBoxCost( nTmpY, vMinY, vMaxY ) ;

			if (nFCost >= nBCost)
				break;
			else
			{
				vBestGrid[j + 1] = pFG;
				vBestGrid[j] = pBG;
				vBestGridCost[j + 1] = nFCost;
				vBestGridCost[j] = nBCost;
			}
		}
	}
	// endadded at 0706 19:30

	pInst->setPlaced(nOrigX, nOrigY);
	//return pBestGrid;
	return vBestGrid;
}

vector<gGrid_C *> router_C::findPlaceToMove_ver4(instance_C *pInst)
{
	int nOrigX = pInst->getPlacedX();
	int nOrigY = pInst->getPlacedY();

	vector<gGrid_C *> vBestGrid;

	forced_C *cF;
	if (pInst == m_vForced[pInst->getId()].m_pInstance)
	{
		cF = &m_vForced[pInst->getId()];
	}
	else
	{
		for (int i = 0; i < m_vForced.size(); i++)
		{
			if (m_vForced[i].m_pInstance == pInst)
			{
				cF = &m_vForced[i];
				break;
			}
		}
	}

	//cout << cF << endl;
	vector<int> &vMinX = cF->m_vMinX;
	vector<int> &vMaxX = cF->m_vMaxX;

	vector<int> &vMinY = cF->m_vMinY;
	vector<int> &vMaxY = cF->m_vMaxY;

	//cout << "Orig: "<< nOrigX << " " << nOrigY << endl;
	int nStartX = nOrigX;
	int nEndX = nOrigX;
	int nStartY = nOrigY;
	int nEndY = nOrigY;
	/*
	cout << "X" << endl;
	for( int i=0; i<vMinX.size(); i++ )
	{
		cout << vMinX[i] << " " << vMaxX[i] << endl;
	}
	
	cout << "Y" << endl;
	for( int i=0; i<vMinY.size(); i++ )
	{
		cout << vMinY[i] << " " << vMaxY[i] << endl;
	}
	*/
	if( vMinX.size() != 0 && vMaxX.size() != 0 && vMinY.size() != 0 && vMaxY.size() != 0 )
	{	
		int nOriCostX = boundingBoxCost(nOrigX, vMinX, vMaxX); 
		int nOriCostY = boundingBoxCost(nOrigY, vMinY, vMaxY);
		//cout << nOriCostX << " " <<nOriCostY << endl;

		for( int i=nStartX; i>=m_nOffsetX; i-- )
		{
			if( nOriCostX >= boundingBoxCost(i, vMinX, vMaxX ) )
				nStartX = i;
			else
				break;
		}
		for( int i=nStartY; i>=m_nOffsetY; i-- )
		{
			if( nOriCostY >= boundingBoxCost(i, vMinY, vMaxY ) )
				nStartY = i;
			else
				break;
		}
		for( int i=nEndX; i<=m_vGraph[0][0].size(); i++ )
		{
			if( nOriCostX >= boundingBoxCost(i, vMinX, vMaxX ) )
				nEndX = i;
			else
				break;
		}
		for( int i=nEndY; i<=m_vGraph[0].size(); i++ )
		{
			if( nOriCostY >= boundingBoxCost(i, vMinY, vMaxY ) )
				nEndY = i;
			else
				break;
		}
	}
	//if( pInst->getName() == "C1200" )
	//	cout<<"Range: "<<nStartX<<" "<<nStartY<<" "<<nEndX<<" "<<nEndY<<endl;

	for (int nX = nStartX; nX <= nEndX; nX++)
	{
		for (int nY = nStartY; nY <= nEndY; nY++)
		{
			gGrid_C *pPGrid = getGrid(m_pDesign, nX, nY, m_nOffsetZ);
			vBestGrid.push_back(pPGrid);
		}
	}
	// end added at 0711 02:00

	vector<int> vBestGridCost;
	for (int i = 0; i < vBestGrid.size(); i++)
	{
		int nTmpX, nTmpY, nTmpZ;
		gGrid_C *pG = vBestGrid[i];
		pG->getPosition(nTmpX, nTmpY, nTmpZ);

		int nCost = boundingBoxCost(nTmpX, vMinX, vMaxX) + boundingBoxCost(nTmpY, vMinY, vMaxY);

		vBestGridCost.push_back(nCost);
	}

	// added at 0706 19:30
	for (int i = 1; i < vBestGrid.size(); i++)
	{
		for (int j = i - 1; j >= 0; j--)
		{
			//if( !cF.m_bLockInX )
			//	nDX = cF.m_nR - cF.m_nL;
			//if( !cF.m_bLockInY )
			//	nDY = cF.m_nT - cF.m_nD;

			//int nTmpX, nTmpY, nTmpZ, nTmpB, nTmpF;
			gGrid_C *pBG = vBestGrid[j + 1];
			//pBG->getPosition( nTmpX, nTmpY, nTmpZ );

			//int nBCost = boundingBoxCost( nTmpX, vMinX, vMaxX )
			//	   + boundingBoxCost( nTmpY, vMinY, vMaxY ) ;
			int nBCost = vBestGridCost[j + 1];
			int nFCost = vBestGridCost[j];

			gGrid_C *pFG = vBestGrid[j];
			//pFG->getPosition( nTmpX, nTmpY, nTmpZ );

			//int nFCost = boundingBoxCost( nTmpX, vMinX, vMaxX )
			//	   + boundingBoxCost( nTmpY, vMinY, vMaxY ) ;

			if (nFCost >= nBCost)
				break;
			else
			{
				vBestGrid[j + 1] = pFG;
				vBestGrid[j] = pBG;
				vBestGridCost[j + 1] = nFCost;
				vBestGridCost[j] = nBCost;
			}
		}
	}
	// endadded at 0706 19:30

	pInst->setPlaced(nOrigX, nOrigY);
	//return pBestGrid;
	return vBestGrid;
}

vector<gGrid_C *> router_C::findPlaceToMove_ver4(instance_C *pInst, boundry_C* pBound, set< instance_C* > &sUnplacedInst )
{
	int nOrigX = pInst->getPlacedX();
	int nOrigY = pInst->getPlacedY();
	sUnplacedInst.erase( pInst );
	vector<gGrid_C *> vBestGrid;

	forced_C *cF;
	if (pInst == m_vForced[pInst->getId()].m_pInstance)
	{
		cF = &m_vForced[pInst->getId()];
	}
	else
	{
		for (int i = 0; i < m_vForced.size(); i++)
		{
			if (m_vForced[i].m_pInstance == pInst)
			{
				cF = &m_vForced[i];
				break;
			}
		}
	}

	int nStartX = cF->m_nBoundX1;
	int nEndX = cF->m_nBoundX2;
	int nStartY = cF->m_nBoundY1;
	int nEndY = cF->m_nBoundY2;
//
	vector< net_C* > vNet;
	vector< int > vExtraCost;
	for( int i=0; i<cF->m_vNetwork.size(); i++ )
	{
		vNet.push_back( cF->m_vNetwork[i]->m_pNet );
		vExtraCost.push_back( cF->m_vNetwork[i]->m_pNet->getConstraintLayerId() );
	}


//

	if( pBound->m_cType == 'T' || pBound->m_cType =='D' )
	{
		nStartX = pBound->m_nBoundX1;
		nEndX = pBound->m_nBoundX2;
		/*
		if( pBound->m_cType == 'T' && pBound->m_nMove == 1 )
			nEndX--;
		else if( pBound->m_cType == 'T' && pBound->m_nMove == -1 )
			nStartX++;
		
		if( pBound->m_cType == 'D' && pBound->m_nMove == 1 )
			nStartX++;
		else if( pBound->m_cType == 'D' && pBound->m_nMove == -1 )
			nEndX--;
		*/
		/*
		if( pBound->m_nBoundX1 > cF->m_nBoundX1 )
			nStartX = pBound->m_nBoundX1;
		else if( pBound->m_nBoundX2 < cF->m_nBoundX1 )
			nStartX = pBound->m_nBoundX2;
		else
			nStartX = cF->m_nBoundX1;
		if( pBound->m_nBoundX2 < cF->m_nBoundX2 )
			nEndX = pBound->m_nBoundX2;
		else if( pBound->m_nBoundX1 > cF->m_nBoundX2 )
			nEndX = pBound->m_nBoundX1;
		else
			nEndX = cF->m_nBoundX2;
		*/
		
		//if( pBound->m_nBoundY1 > cF->m_nBoundY1 )
		//	nStartY = pBound->m_nBoundY1;
		//else if( pBound->m_nBoundY2 < cF->m_nBoundY1 )
		//	nStartY = pBound->m_nBoundY2;
		//else
		//	nStartY = cF->m_nBoundY1;
		
		nStartY = pBound->m_nBoundY1;
		
		//if( pBound->m_nBoundY2 < cF->m_nBoundY2 )
		//	nEndY = pBound->m_nBoundY2;
		//else if( pBound->m_nBoundY1 > cF->m_nBoundY2 )
		//	nEndY = pBound->m_nBoundY1;
		//else
		//	nEndY = cF->m_nBoundY2;
		
		nEndY = pBound->m_nBoundY2;
	}
	else
	{
		nStartY = pBound->m_nBoundY1;
		nEndY = pBound->m_nBoundY2;
		/*	
		if( pBound->m_cType == 'R' && pBound->m_nMove == 1 )
			nEndY--;
		else if( pBound->m_cType == 'R' && pBound->m_nMove == -1 )
			nStartY++;
		
		if( pBound->m_cType == 'L' && pBound->m_nMove == 1 )
			nStartY++;
		else if( pBound->m_cType == 'L' && pBound->m_nMove == -1 )
			nEndY--;
		*/
		/*
		if( pBound->m_nBoundY1 > cF->m_nBoundY1 )
			nStartY = pBound->m_nBoundY1;
		else if( pBound->m_nBoundY2 < cF->m_nBoundY1 )
			nStartY = pBound->m_nBoundY2;
		else
			nStartY = cF->m_nBoundY1;
		if( pBound->m_nBoundY2 < cF->m_nBoundY2 )
			nEndY = pBound->m_nBoundY2;
		else if( pBound->m_nBoundY1 > cF->m_nBoundY2 )
			nEndY = pBound->m_nBoundY1;
		else
			nEndY = cF->m_nBoundY2;
		*/
		
		//if( pBound->m_nBoundX1 > cF->m_nBoundX1 )
		//	nStartX = pBound->m_nBoundX1;
		//else if( pBound->m_nBoundX2 < cF->m_nBoundX1 )
		//	nStartX = pBound->m_nBoundX2;
		//else
		//	nStartX = cF->m_nBoundX1;
		
		nStartX = pBound->m_nBoundX1;
		
		//if( pBound->m_nBoundX2 < cF->m_nBoundX2 )
		//	nEndX = pBound->m_nBoundX2;
		//else if( pBound->m_nBoundX1 > cF->m_nBoundX2 )
		//	nEndX = pBound->m_nBoundX1;
		//else
		//	nEndX = cF->m_nBoundX2;
		
		nEndX = pBound->m_nBoundX2;
	}
	//if( pInst->getName() == "C1200" )
		cout<<"Range: "<<nStartX<<" "<<nStartY<<" "<<nEndX<<" "<<nEndY<<endl;

	for (int nX = nStartX; nX <= nEndX; nX++)
	{
		for (int nY = nStartY; nY <= nEndY; nY++)
		{
			gGrid_C *pPGrid = getGrid(m_pDesign, nX, nY, m_nOffsetZ);
			vector< net_C* > vRipNet;
			//if( isPlaceable_ver5( pPGrid, pInst, vRipNet ) )
				vBestGrid.push_back(pPGrid);
		}
	}
	// end added at 0711 02:00
	/*
	vector<int> &vMinX = cF->m_vMinX;
	vector<int> &vMaxX = cF->m_vMaxX;

	vector<int> &vMinY = cF->m_vMinY;
	vector<int> &vMaxY = cF->m_vMaxY;
	*/
	/*
	vector<int> &vMinX = pBound->m_vMinX;
	vector<int> &vMaxX = pBound->m_vMaxX;

	vector<int> &vMinY = pBound->m_vMinY;
	vector<int> &vMaxY = pBound->m_vMaxY;
	*/
	vector<int> vMinX;
	vector<int> vMaxX;

	vector<int> vMinY;
	vector<int> vMaxY;

	for( int i=0; i<cF->m_vNetwork.size(); i++ )
	{
		for( int j=0; j<pBound->m_vNetwork.size(); j++ )
		{
			if( cF->m_vNetwork[i] == pBound->m_vNetwork[j] )
			{
				vMinX.push_back( pBound->m_vMinX[j] );
				vMaxX.push_back( pBound->m_vMaxX[j] );
				vMinY.push_back( pBound->m_vMinY[j] );
				vMaxY.push_back( pBound->m_vMaxY[j] );
				break;
			}
		}
	}

	vector<int> vBestGridCost;
	vector<int> vDelta;
	for (int i = 0; i < vBestGrid.size(); i++)
	{
		int nTmpX, nTmpY, nTmpZ;
		gGrid_C *pG = vBestGrid[i];
		pG->getPosition(nTmpX, nTmpY, nTmpZ);

		int nCost = boundingBoxCost(nTmpX, vMinX, vMaxX) + boundingBoxCost(nTmpY, vMinY, vMaxY);

//		
		int nExtra = 0;
		vector< instance_C* > vTmpInst = vBestGrid[i]->getInstance();
		vector< bool > vNetInfo;
		for( int j=0; j<vNet.size(); j++ )
			vNetInfo.push_back( false );
		for( int j=0; j<vTmpInst.size(); j++ )
		{
			forced_C* pTF = &m_vForced[ vTmpInst[j]->getId() ];
			vector< networkForced_C* > &vTNF = pTF->m_vNetwork;
			for( int c=0; c<vNet.size(); c++ )
			{
				for( int n=0; n<vTNF.size(); n++ )
				{
					if( vTNF[n]->m_pNet == vNet[c] )
					{
						vNetInfo[ c ] = true;
						break;
					}
				}
			}	
		}
		
		for( int j=0; j<vNet.size(); j++ )
		{
			if( !vNetInfo[j] )
				nExtra = nExtra + vExtraCost[j];
		}
		nCost = nCost + nExtra;
//

		pInst->setPlaced( nTmpX, nTmpY );
		/*
		int nCost = 0;
		for( int j=0; j<vNet.size(); j++ )
		{
			nCost = nCost + routeNet_ideal( vNet[j], sUnplacedInst );
		}
		*/
		vBestGridCost.push_back(nCost);
		if( pBound->m_cType == 'T' || pBound->m_cType == 'D' )
			vDelta.push_back( abs( nOrigX - nTmpX ) );
		else
			vDelta.push_back( abs( nOrigY - nTmpY ) );	
	}

	// added at 0706 19:30
	for (int i = 1; i < vBestGrid.size(); i++)
	{
		for (int j = i - 1; j >= 0; j--)
		{
			//if( !cF.m_bLockInX )
			//	nDX = cF.m_nR - cF.m_nL;
			//if( !cF.m_bLockInY )
			//	nDY = cF.m_nT - cF.m_nD;

			//int nTmpX, nTmpY, nTmpZ, nTmpB, nTmpF;
			gGrid_C *pBG = vBestGrid[j + 1];
			//pBG->getPosition( nTmpX, nTmpY, nTmpZ );

			//int nBCost = boundingBoxCost( nTmpX, vMinX, vMaxX )
			//	   + boundingBoxCost( nTmpY, vMinY, vMaxY ) ;
			int nBCost = vBestGridCost[j + 1];
			int nFCost = vBestGridCost[j];
			int nBDelta = vDelta[j+1];
			int nFDelta = vDelta[j];

			gGrid_C *pFG = vBestGrid[j];
			//pFG->getPosition( nTmpX, nTmpY, nTmpZ );

			//int nFCost = boundingBoxCost( nTmpX, vMinX, vMaxX )
			//	   + boundingBoxCost( nTmpY, vMinY, vMaxY ) ;

			if (nFCost >= nBCost)
			{
				//if( pBound->m_nMove == 1 )
				//{
				/*
					if( nFDelta <= nBDelta )
						break;
					else
					{
						vBestGrid[j + 1] = pFG;
						vBestGrid[j] = pBG;
						vBestGridCost[j + 1] = nFCost;
						vBestGridCost[j] = nBCost;
						vDelta[j + 1] = nFDelta;
						vDelta[j] = nBDelta;	
					}
				*/
				//}
				//else // == -1
				//{
				/*
					if( nFDelta >= nBDelta )
						break;
					else
					{
						vBestGrid[j + 1] = pFG;
						vBestGrid[j] = pBG;
						vBestGridCost[j + 1] = nFCost;
						vBestGridCost[j] = nBCost;
						vDelta[j + 1] = nFDelta;
						vDelta[j] = nBDelta;	
					}	
				*/
				//}
				break;
			}
			else
			{
				vBestGrid[j + 1] = pFG;
				vBestGrid[j] = pBG;
				vBestGridCost[j + 1] = nFCost;
				vBestGridCost[j] = nBCost;
				//vDelta[j + 1] = nFDelta;
				//vDelta[j] = nBDelta;
			}
		}
	}
	// endadded at 0706 19:30

	pInst->setPlaced(nOrigX, nOrigY);
	sUnplacedInst.insert( pInst );
	//return pBestGrid;
	return vBestGrid;
}


vector<gGrid_C *> router_C::findPlaceToMove_ver4(instance_C *pInst, boundry_C* pBound )
{
	int nOrigX = pInst->getPlacedX();
	int nOrigY = pInst->getPlacedY();

	vector<gGrid_C *> vBestGrid;

	forced_C *cF;
	if (pInst == m_vForced[pInst->getId()].m_pInstance)
	{
		cF = &m_vForced[pInst->getId()];
	}
	else
	{
		for (int i = 0; i < m_vForced.size(); i++)
		{
			if (m_vForced[i].m_pInstance == pInst)
			{
				cF = &m_vForced[i];
				break;
			}
		}
	}

	int nStartX = cF->m_nBoundX1;
	int nEndX = cF->m_nBoundX2;
	int nStartY = cF->m_nBoundY1;
	int nEndY = cF->m_nBoundY2;
//
	vector< net_C* > vNet;
	vector< int > vExtraCost;
	for( int i=0; i<cF->m_vNetwork.size(); i++ )
	{
		vNet.push_back( cF->m_vNetwork[i]->m_pNet );
		vExtraCost.push_back( cF->m_vNetwork[i]->m_pNet->getConstraintLayerId() );
	}


//

	if( pBound->m_cType == 'T' || pBound->m_cType =='D' )
	{
		nStartX = pBound->m_nBoundX1;
		nEndX = pBound->m_nBoundX2;
		/*
		if( pBound->m_cType == 'T' && pBound->m_nMove == 1 )
			nEndX--;
		else if( pBound->m_cType == 'T' && pBound->m_nMove == -1 )
			nStartX++;
		
		if( pBound->m_cType == 'D' && pBound->m_nMove == 1 )
			nStartX++;
		else if( pBound->m_cType == 'D' && pBound->m_nMove == -1 )
			nEndX--;
		*/
		/*
		if( pBound->m_nBoundX1 > cF->m_nBoundX1 )
			nStartX = pBound->m_nBoundX1;
		else if( pBound->m_nBoundX2 < cF->m_nBoundX1 )
			nStartX = pBound->m_nBoundX2;
		else
			nStartX = cF->m_nBoundX1;
		if( pBound->m_nBoundX2 < cF->m_nBoundX2 )
			nEndX = pBound->m_nBoundX2;
		else if( pBound->m_nBoundX1 > cF->m_nBoundX2 )
			nEndX = pBound->m_nBoundX1;
		else
			nEndX = cF->m_nBoundX2;
		*/
		
		//if( pBound->m_nBoundY1 > cF->m_nBoundY1 )
		//	nStartY = pBound->m_nBoundY1;
		//else if( pBound->m_nBoundY2 < cF->m_nBoundY1 )
		//	nStartY = pBound->m_nBoundY2;
		//else
		//	nStartY = cF->m_nBoundY1;
		
		nStartY = pBound->m_nBoundY1;
		
		//if( pBound->m_nBoundY2 < cF->m_nBoundY2 )
		//	nEndY = pBound->m_nBoundY2;
		//else if( pBound->m_nBoundY1 > cF->m_nBoundY2 )
		//	nEndY = pBound->m_nBoundY1;
		//else
		//	nEndY = cF->m_nBoundY2;
		
		nEndY = pBound->m_nBoundY2;
	}
	else
	{
		nStartY = pBound->m_nBoundY1;
		nEndY = pBound->m_nBoundY2;
		/*	
		if( pBound->m_cType == 'R' && pBound->m_nMove == 1 )
			nEndY--;
		else if( pBound->m_cType == 'R' && pBound->m_nMove == -1 )
			nStartY++;
		
		if( pBound->m_cType == 'L' && pBound->m_nMove == 1 )
			nStartY++;
		else if( pBound->m_cType == 'L' && pBound->m_nMove == -1 )
			nEndY--;
		*/
		/*
		if( pBound->m_nBoundY1 > cF->m_nBoundY1 )
			nStartY = pBound->m_nBoundY1;
		else if( pBound->m_nBoundY2 < cF->m_nBoundY1 )
			nStartY = pBound->m_nBoundY2;
		else
			nStartY = cF->m_nBoundY1;
		if( pBound->m_nBoundY2 < cF->m_nBoundY2 )
			nEndY = pBound->m_nBoundY2;
		else if( pBound->m_nBoundY1 > cF->m_nBoundY2 )
			nEndY = pBound->m_nBoundY1;
		else
			nEndY = cF->m_nBoundY2;
		*/
		
		//if( pBound->m_nBoundX1 > cF->m_nBoundX1 )
		//	nStartX = pBound->m_nBoundX1;
		//else if( pBound->m_nBoundX2 < cF->m_nBoundX1 )
		//	nStartX = pBound->m_nBoundX2;
		//else
		//	nStartX = cF->m_nBoundX1;
		
		nStartX = pBound->m_nBoundX1;
		
		//if( pBound->m_nBoundX2 < cF->m_nBoundX2 )
		//	nEndX = pBound->m_nBoundX2;
		//else if( pBound->m_nBoundX1 > cF->m_nBoundX2 )
		//	nEndX = pBound->m_nBoundX1;
		//else
		//	nEndX = cF->m_nBoundX2;
		
		nEndX = pBound->m_nBoundX2;
	}
	//if( pInst->getName() == "C1200" )
		cout<<"Range: "<<nStartX<<" "<<nStartY<<" "<<nEndX<<" "<<nEndY<<endl;

	for (int nX = nStartX; nX <= nEndX; nX++)
	{
		for (int nY = nStartY; nY <= nEndY; nY++)
		{
			gGrid_C *pPGrid = getGrid(m_pDesign, nX, nY, m_nOffsetZ);
			vector< net_C* > vRipNet;
			//if( isPlaceable_ver5( pPGrid, pInst, vRipNet ) )
				vBestGrid.push_back(pPGrid);
		}
	}
	// end added at 0711 02:00
	/*
	vector<int> &vMinX = cF->m_vMinX;
	vector<int> &vMaxX = cF->m_vMaxX;

	vector<int> &vMinY = cF->m_vMinY;
	vector<int> &vMaxY = cF->m_vMaxY;
	*/
	/*
	vector<int> &vMinX = pBound->m_vMinX;
	vector<int> &vMaxX = pBound->m_vMaxX;

	vector<int> &vMinY = pBound->m_vMinY;
	vector<int> &vMaxY = pBound->m_vMaxY;
	*/
	vector<int> vMinX;
	vector<int> vMaxX;

	vector<int> vMinY;
	vector<int> vMaxY;

	for( int i=0; i<cF->m_vNetwork.size(); i++ )
	{
		for( int j=0; j<pBound->m_vNetwork.size(); j++ )
		{
			if( cF->m_vNetwork[i] == pBound->m_vNetwork[j] )
			{
				vMinX.push_back( pBound->m_vMinX[j] );
				vMaxX.push_back( pBound->m_vMaxX[j] );
				vMinY.push_back( pBound->m_vMinY[j] );
				vMaxY.push_back( pBound->m_vMaxY[j] );
				break;
			}
		}
	}

	vector<int> vBestGridCost;
	vector<int> vDelta;
	for (int i = 0; i < vBestGrid.size(); i++)
	{
		int nTmpX, nTmpY, nTmpZ;
		gGrid_C *pG = vBestGrid[i];
		pG->getPosition(nTmpX, nTmpY, nTmpZ);

		int nCost = boundingBoxCost(nTmpX, vMinX, vMaxX) + boundingBoxCost(nTmpY, vMinY, vMaxY);

//			
		int nExtra = 0;
		vector< instance_C* > vTmpInst = vBestGrid[i]->getInstance();
		vector< bool > vNetInfo;
		for( int j=0; j<vNet.size(); j++ )
			vNetInfo.push_back( false );
		for( int j=0; j<vTmpInst.size(); j++ )
		{
			forced_C* pTF = &m_vForced[ vTmpInst[j]->getId() ];
			vector< networkForced_C* > &vTNF = pTF->m_vNetwork;
			for( int c=0; c<vNet.size(); c++ )
			{
				for( int n=0; n<vTNF.size(); n++ )
				{
					if( vTNF[n]->m_pNet == vNet[c] )
					{
						vNetInfo[ c ] = true;
						break;
					}
				}
			}	
		}
		
		for( int j=0; j<vNet.size(); j++ )
		{
			if( !vNetInfo[j] )
				nExtra = nExtra + vExtraCost[j];
		}
		nCost = nCost + nExtra;
//

/*
		int nCost = 0;
		for( int j=0; j<vNet.size(); j++ )
		{
			nCost = nCost + routeNet_ideal( vNet[j] );
		}
*/
		vBestGridCost.push_back(nCost);
		if( pBound->m_cType == 'T' || pBound->m_cType == 'D' )
			vDelta.push_back( abs( nOrigX - nTmpX ) );
		else
			vDelta.push_back( abs( nOrigY - nTmpY ) );	
	}

	// added at 0706 19:30
	for (int i = 1; i < vBestGrid.size(); i++)
	{
		for (int j = i - 1; j >= 0; j--)
		{
			//if( !cF.m_bLockInX )
			//	nDX = cF.m_nR - cF.m_nL;
			//if( !cF.m_bLockInY )
			//	nDY = cF.m_nT - cF.m_nD;

			//int nTmpX, nTmpY, nTmpZ, nTmpB, nTmpF;
			gGrid_C *pBG = vBestGrid[j + 1];
			//pBG->getPosition( nTmpX, nTmpY, nTmpZ );

			//int nBCost = boundingBoxCost( nTmpX, vMinX, vMaxX )
			//	   + boundingBoxCost( nTmpY, vMinY, vMaxY ) ;
			int nBCost = vBestGridCost[j + 1];
			int nFCost = vBestGridCost[j];
			int nBDelta = vDelta[j+1];
			int nFDelta = vDelta[j];

			gGrid_C *pFG = vBestGrid[j];
			//pFG->getPosition( nTmpX, nTmpY, nTmpZ );

			//int nFCost = boundingBoxCost( nTmpX, vMinX, vMaxX )
			//	   + boundingBoxCost( nTmpY, vMinY, vMaxY ) ;

			if (nFCost >= nBCost)
			{
				//if( pBound->m_nMove == 1 )
				//{
				/*
					if( nFDelta <= nBDelta )
						break;
					else
					{
						vBestGrid[j + 1] = pFG;
						vBestGrid[j] = pBG;
						vBestGridCost[j + 1] = nFCost;
						vBestGridCost[j] = nBCost;
						vDelta[j + 1] = nFDelta;
						vDelta[j] = nBDelta;	
					}
				*/
				//}
				//else // == -1
				//{
				/*
					if( nFDelta >= nBDelta )
						break;
					else
					{
						vBestGrid[j + 1] = pFG;
						vBestGrid[j] = pBG;
						vBestGridCost[j + 1] = nFCost;
						vBestGridCost[j] = nBCost;
						vDelta[j + 1] = nFDelta;
						vDelta[j] = nBDelta;	
					}	
				*/
				//}
			}
			else
			{
				vBestGrid[j + 1] = pFG;
				vBestGrid[j] = pBG;
				vBestGridCost[j + 1] = nFCost;
				vBestGridCost[j] = nBCost;
				//vDelta[j + 1] = nFDelta;
				//vDelta[j] = nBDelta;
			}
		}
	}
	// endadded at 0706 19:30

	pInst->setPlaced(nOrigX, nOrigY);
	//return pBestGrid;
	return vBestGrid;
}

vector<gGrid_C *> router_C::findPlaceToMove_ver3(instance_C *pInst, boundry_C* pBound )
{
	int nOrigX = pInst->getPlacedX();
	int nOrigY = pInst->getPlacedY();

	vector<gGrid_C *> vBestGrid;

	forced_C *cF;
	if (pInst == m_vForced[pInst->getId()].m_pInstance)
	{
		cF = &m_vForced[pInst->getId()];
	}
	else
	{
		for (int i = 0; i < m_vForced.size(); i++)
		{
			if (m_vForced[i].m_pInstance == pInst)
			{
				cF = &m_vForced[i];
				break;
			}
		}
	}

	int nStartX = cF->m_nBoundX1;
	int nEndX = cF->m_nBoundX2;
	int nStartY = cF->m_nBoundY1;
	int nEndY = cF->m_nBoundY2;

//
	vector< net_C* > vNet;
	vector< int > vExtraCost;
	for( int i=0; i<cF->m_vNetwork.size(); i++ )
	{
		vNet.push_back( cF->m_vNetwork[i]->m_pNet );
		vExtraCost.push_back( cF->m_vNetwork[i]->m_pNet->getConstraintLayerId() );
	}


//
	if( pBound->m_cType == 'T' || pBound->m_cType =='D' )
	{
		nStartX = pBound->m_nBoundX1;
		nEndX = pBound->m_nBoundX2;
		/*
		if( pBound->m_cType == 'T' && pBound->m_nMove == 1 )
			nEndX--;
		else if( pBound->m_cType == 'T' && pBound->m_nMove == -1 )
			nStartX++;
		
		if( pBound->m_cType == 'D' && pBound->m_nMove == 1 )
			nStartX++;
		else if( pBound->m_cType == 'D' && pBound->m_nMove == -1 )
			nEndX--;
		*/
		/*
		if( pBound->m_nBoundX1 > cF->m_nBoundX1 )
			nStartX = pBound->m_nBoundX1;
		else if( pBound->m_nBoundX2 < cF->m_nBoundX1 )
			nStartX = pBound->m_nBoundX2;
		else
			nStartX = cF->m_nBoundX1;
		if( pBound->m_nBoundX2 < cF->m_nBoundX2 )
			nEndX = pBound->m_nBoundX2;
		else if( pBound->m_nBoundX1 > cF->m_nBoundX2 )
			nEndX = pBound->m_nBoundX1;
		else
			nEndX = cF->m_nBoundX2;
		*/
		
		if( pBound->m_nBoundY1 > cF->m_nBoundY1 )
			nStartY = pBound->m_nBoundY1;
		else if( pBound->m_nBoundY2 < cF->m_nBoundY1 )
			nStartY = pBound->m_nBoundY2;
		else
			nStartY = cF->m_nBoundY1;
		
		//nStartY = pBound->m_nBoundY1;
		
		if( pBound->m_nBoundY2 < cF->m_nBoundY2 )
			nEndY = pBound->m_nBoundY2;
		else if( pBound->m_nBoundY1 > cF->m_nBoundY2 )
			nEndY = pBound->m_nBoundY1;
		else
			nEndY = cF->m_nBoundY2;
		
		//nEndY = pBound->m_nBoundY2;
	}
	else
	{
		nStartY = pBound->m_nBoundY1;
		nEndY = pBound->m_nBoundY2;
		/*	
		if( pBound->m_cType == 'R' && pBound->m_nMove == 1 )
			nEndY--;
		else if( pBound->m_cType == 'R' && pBound->m_nMove == -1 )
			nStartY++;
		
		if( pBound->m_cType == 'L' && pBound->m_nMove == 1 )
			nStartY++;
		else if( pBound->m_cType == 'L' && pBound->m_nMove == -1 )
			nEndY--;
		*/
		/*
		if( pBound->m_nBoundY1 > cF->m_nBoundY1 )
			nStartY = pBound->m_nBoundY1;
		else if( pBound->m_nBoundY2 < cF->m_nBoundY1 )
			nStartY = pBound->m_nBoundY2;
		else
			nStartY = cF->m_nBoundY1;
		if( pBound->m_nBoundY2 < cF->m_nBoundY2 )
			nEndY = pBound->m_nBoundY2;
		else if( pBound->m_nBoundY1 > cF->m_nBoundY2 )
			nEndY = pBound->m_nBoundY1;
		else
			nEndY = cF->m_nBoundY2;
		*/
		
		if( pBound->m_nBoundX1 > cF->m_nBoundX1 )
			nStartX = pBound->m_nBoundX1;
		else if( pBound->m_nBoundX2 < cF->m_nBoundX1 )
			nStartX = pBound->m_nBoundX2;
		else
			nStartX = cF->m_nBoundX1;
		
		
		//nStartX = pBound->m_nBoundX1;
		
		if( pBound->m_nBoundX2 < cF->m_nBoundX2 )
			nEndX = pBound->m_nBoundX2;
		else if( pBound->m_nBoundX1 > cF->m_nBoundX2 )
			nEndX = pBound->m_nBoundX1;
		else
			nEndX = cF->m_nBoundX2;
		
		//nEndX = pBound->m_nBoundX2;
	}
	//if( pInst->getName() == "C1200" )
		cout<<"Range: "<<nStartX<<" "<<nStartY<<" "<<nEndX<<" "<<nEndY<<endl;

	for (int nX = nStartX; nX <= nEndX; nX++)
	{
		for (int nY = nStartY; nY <= nEndY; nY++)
		{
			gGrid_C *pPGrid = getGrid(m_pDesign, nX, nY, m_nOffsetZ);
			vBestGrid.push_back(pPGrid);
		}
	}
	// end added at 0711 02:00
	/*
	vector<int> &vMinX = cF->m_vMinX;
	vector<int> &vMaxX = cF->m_vMaxX;

	vector<int> &vMinY = cF->m_vMinY;
	vector<int> &vMaxY = cF->m_vMaxY;
	*/
	
	vector<int> &vMinX = pBound->m_vMinX;
	vector<int> &vMaxX = pBound->m_vMaxX;

	vector<int> &vMinY = pBound->m_vMinY;
	vector<int> &vMaxY = pBound->m_vMaxY;

	vector<int> vBestGridCost;
	vector<int> vDelta;
	for (int i = 0; i < vBestGrid.size(); i++)
	{
		int nTmpX, nTmpY, nTmpZ;
		gGrid_C *pG = vBestGrid[i];
		pG->getPosition(nTmpX, nTmpY, nTmpZ);

		int nCost = boundingBoxCost(nTmpX, vMinX, vMaxX) + boundingBoxCost(nTmpY, vMinY, vMaxY);
//		
		int nExtra = 0;
		vector< instance_C* > vTmpInst = vBestGrid[i]->getInstance();
		vector< bool > vNetInfo;
		for( int j=0; j<vNet.size(); j++ )
			vNetInfo.push_back( false );
		for( int j=0; j<vTmpInst.size(); j++ )
		{
			forced_C* pTF = &m_vForced[ vTmpInst[j]->getId() ];
			vector< networkForced_C* > &vTNF = pTF->m_vNetwork;
			for( int c=0; c<vNet.size(); c++ )
			{
				for( int n=0; n<vTNF.size(); n++ )
				{
					if( vTNF[n]->m_pNet == vNet[c] )
					{
						vNetInfo[ c ] = true;
						break;
					}
				}
			}	
		}
		
		for( int j=0; j<vNet.size(); j++ )
		{
			if( !vNetInfo[j] )
				nExtra = nExtra + vExtraCost[j];
		}
		nCost = nCost + nExtra;
//
/*
		int nCost = 0;
		for( int j=0; j<vNet.size(); j++ )
		{
			nCost = nCost + routeNet_ideal( vNet[j] );
		}
*/


		vBestGridCost.push_back(nCost);
		if( pBound->m_cType == 'T' || pBound->m_cType == 'D' )
			vDelta.push_back( abs( nOrigX - nTmpX ) );
		else
			vDelta.push_back( abs( nOrigY - nTmpY ) );	
	}

	// added at 0706 19:30
	for (int i = 1; i < vBestGrid.size(); i++)
	{
		for (int j = i - 1; j >= 0; j--)
		{
			//if( !cF.m_bLockInX )
			//	nDX = cF.m_nR - cF.m_nL;
			//if( !cF.m_bLockInY )
			//	nDY = cF.m_nT - cF.m_nD;

			//int nTmpX, nTmpY, nTmpZ, nTmpB, nTmpF;
			gGrid_C *pBG = vBestGrid[j + 1];
			//pBG->getPosition( nTmpX, nTmpY, nTmpZ );

			//int nBCost = boundingBoxCost( nTmpX, vMinX, vMaxX )
			//	   + boundingBoxCost( nTmpY, vMinY, vMaxY ) ;
			int nBCost = vBestGridCost[j + 1];
			int nFCost = vBestGridCost[j];
			int nBDelta = vDelta[j+1];
			int nFDelta = vDelta[j];

			gGrid_C *pFG = vBestGrid[j];
			//pFG->getPosition( nTmpX, nTmpY, nTmpZ );

			//int nFCost = boundingBoxCost( nTmpX, vMinX, vMaxX )
			//	   + boundingBoxCost( nTmpY, vMinY, vMaxY ) ;

			if (nFCost >= nBCost)
			{
				//if( pBound->m_nMove == 1 )
				//{
					if( nFDelta <= nBDelta )
						break;
					else
					{
						vBestGrid[j + 1] = pFG;
						vBestGrid[j] = pBG;
						vBestGridCost[j + 1] = nFCost;
						vBestGridCost[j] = nBCost;
						vDelta[j + 1] = nFDelta;
						vDelta[j] = nBDelta;	
					}
				//}
				//else // == -1
				//{
				/*
					if( nFDelta >= nBDelta )
						break;
					else
					{
						vBestGrid[j + 1] = pFG;
						vBestGrid[j] = pBG;
						vBestGridCost[j + 1] = nFCost;
						vBestGridCost[j] = nBCost;
						vDelta[j + 1] = nFDelta;
						vDelta[j] = nBDelta;	
					}	
				*/
				//}
			}
			else
			{
				vBestGrid[j + 1] = pFG;
				vBestGrid[j] = pBG;
				vBestGridCost[j + 1] = nFCost;
				vBestGridCost[j] = nBCost;
				vDelta[j + 1] = nFDelta;
				vDelta[j] = nBDelta;
			}
		}
	}
	// endadded at 0706 19:30

	pInst->setPlaced(nOrigX, nOrigY);
	//return pBestGrid;
	return vBestGrid;
}

vector<gGrid_C *> router_C::findPlaceToMove_ver3(instance_C *pInst)
{
	int nOrigX = pInst->getPlacedX();
	int nOrigY = pInst->getPlacedY();

	vector<gGrid_C *> vBestGrid;

	forced_C *cF;
	if (pInst == m_vForced[pInst->getId()].m_pInstance)
	{
		cF = &m_vForced[pInst->getId()];
	}
	else
	{
		for (int i = 0; i < m_vForced.size(); i++)
		{
			if (m_vForced[i].m_pInstance == pInst)
			{
				cF = &m_vForced[i];
				break;
			}
		}
	}

	int &nStartX = cF->m_nBoundX1;
	int &nEndX = cF->m_nBoundX2;
	int &nStartY = cF->m_nBoundY1;
	int &nEndY = cF->m_nBoundY2;

	//if( pInst->getName() == "C1200" )
		cout<<"Range: "<<nStartX<<" "<<nStartY<<" "<<nEndX<<" "<<nEndY<<endl;

	for (int nX = nStartX; nX <= nEndX; nX++)
	{
		for (int nY = nStartY; nY <= nEndY; nY++)
		{
			gGrid_C *pPGrid = getGrid(m_pDesign, nX, nY, m_nOffsetZ);
			vBestGrid.push_back(pPGrid);
		}
	}
	// end added at 0711 02:00

	vector<int> &vMinX = cF->m_vMinX;
	vector<int> &vMaxX = cF->m_vMaxX;

	vector<int> &vMinY = cF->m_vMinY;
	vector<int> &vMaxY = cF->m_vMaxY;

	vector<int> vBestGridCost;
	for (int i = 0; i < vBestGrid.size(); i++)
	{
		int nTmpX, nTmpY, nTmpZ;
		gGrid_C *pG = vBestGrid[i];
		pG->getPosition(nTmpX, nTmpY, nTmpZ);

		int nCost = boundingBoxCost(nTmpX, vMinX, vMaxX) + boundingBoxCost(nTmpY, vMinY, vMaxY);

		vBestGridCost.push_back(nCost);
	}

	// added at 0706 19:30
	for (int i = 1; i < vBestGrid.size(); i++)
	{
		for (int j = i - 1; j >= 0; j--)
		{
			//if( !cF.m_bLockInX )
			//	nDX = cF.m_nR - cF.m_nL;
			//if( !cF.m_bLockInY )
			//	nDY = cF.m_nT - cF.m_nD;

			//int nTmpX, nTmpY, nTmpZ, nTmpB, nTmpF;
			gGrid_C *pBG = vBestGrid[j + 1];
			//pBG->getPosition( nTmpX, nTmpY, nTmpZ );

			//int nBCost = boundingBoxCost( nTmpX, vMinX, vMaxX )
			//	   + boundingBoxCost( nTmpY, vMinY, vMaxY ) ;
			int nBCost = vBestGridCost[j + 1];
			int nFCost = vBestGridCost[j];

			gGrid_C *pFG = vBestGrid[j];
			//pFG->getPosition( nTmpX, nTmpY, nTmpZ );

			//int nFCost = boundingBoxCost( nTmpX, vMinX, vMaxX )
			//	   + boundingBoxCost( nTmpY, vMinY, vMaxY ) ;

			if (nFCost >= nBCost)
				break;
			else
			{
				vBestGrid[j + 1] = pFG;
				vBestGrid[j] = pBG;
				vBestGridCost[j + 1] = nFCost;
				vBestGridCost[j] = nBCost;
			}
		}
	}
	// endadded at 0706 19:30

	pInst->setPlaced(nOrigX, nOrigY);
	//return pBestGrid;
	return vBestGrid;
}
// end added at 0705 02:00

// added at 0704 22:00
vector<gGrid_C *> router_C::findPlaceToMove(instance_C *pInst)
{
	int nOrigX = pInst->getPlacedX();
	int nOrigY = pInst->getPlacedY();

	vector<gGrid_C *> vBestGrid;

	forced_C cF;
	if (pInst == m_vForced[pInst->getId()].m_pInstance)
	{
		cF = m_vForced[pInst->getId()];
	}
	else
	{
		for (int i = 0; i < m_vForced.size(); i++)
		{
			if (m_vForced[i].m_pInstance == pInst)
			{
				cF = m_vForced[i];
				break;
			}
		}
	}
	// setup search direction
	int nDX = 0;
	int nDY = 0;
	if (!cF.m_bLockInX)
		nDX = cF.m_nR - cF.m_nL;
	if (!cF.m_bLockInY)
		nDY = cF.m_nT - cF.m_nD;

	if (nDX == 0 && nDY == 0)
	{
		cout << "Warning: No operations will be executed. This instance has balanced forced" << endl;
		return vBestGrid;
		//	return NULL;
	}
	if (abs(nDX) >= abs(nDY))
	{
		nDX = nDX / abs(nDX);
		nDY = 0;
	}
	else
	{
		nDY = nDY / abs(nDY);
		nDX = 0;
	}

	// collecting all the correlated network
	vector<networkForced_C *> vNetWork = cF.m_vNetwork;
	vector<vector<instance_C *>> vBoundingBox;
	for (int i = 0; i < vNetWork.size(); i++)
	{
		vector<instance_C *> vInst;
		networkForced_C *pNF = vNetWork[i];
		vector<forced_C *> vF = pNF->m_vForced;
		for (int j = 0; j < vF.size(); j++)
		{
			instance_C *pTmpInst = vF[j]->m_pInstance;
			if (pTmpInst != pInst)
				vInst.push_back(pTmpInst);
		}
		vBoundingBox.push_back(vInst);
	}

	vector<int> vMin;
	vector<int> vMax;
	for (int i = 0; i < vBoundingBox.size(); i++)
	{
		vector<instance_C *> vInst = vBoundingBox[i];
		int nMax;
		int nMin;
		if (nDX != 0)
		{
			nMax = m_nDX;
			nMin = m_nTX;
			;
		}
		else
		{
			nMax = m_nDY;
			nMin = m_nTY;
		}

		for (int j = 0; j < vInst.size(); j++)
		{
			instance_C *pTmpInst = vInst[j];
			if (nDX != 0)
			{
				nMax = max(pTmpInst->getPlacedX(), nMax);
				nMin = min(pTmpInst->getPlacedX(), nMin);
			}
			else
			{
				nMax = max(pTmpInst->getPlacedY(), nMax);
				nMin = min(pTmpInst->getPlacedY(), nMin);
			}
		}
		vMin.push_back(nMin);
		vMax.push_back(nMax);
	}
	// moving instance
	int nBest;
	int nTmp;
	if (nDX != 0)
		nTmp = pInst->getPlacedX();
	else
		nTmp = pInst->getPlacedY();

	nBest = boundingBoxCost(nTmp, vMin, vMax);
	gGrid_C *pBestGrid = NULL;
	gGrid_C *pGrid = getGrid(m_pDesign, pInst->getPlacedX(), pInst->getPlacedY(), m_nDZ);
	while (graphTravel(m_pDesign, pGrid, nDX, nDY, 0) != NULL)
	{
		pGrid = graphTravel(m_pDesign, pGrid, nDX, nDY, 0);
		// check placeable
		if (!isPlaceable(pGrid, pInst))
			continue;
		//
		int nTmpX, nTmpY, nTmpZ;
		pGrid->getPosition(nTmpX, nTmpY, nTmpZ);
		if (nDX != 0)
			nTmp = nTmpX;
		else
			nTmp = nTmpY;
		// added at 0705 15:40
		//int nTmpBest = boundingBoxCost( nTmp, vMin, vMax );
		//if( nTmpBest <= nBest ) // change from <
		//{
		//	nBest = nTmpBest;
		//	pBestGrid = pGrid;
		//	vBestGrid.push_back( pBestGrid );
		//}
		//else
		//	break;
		int nTmpBest = boundingBoxCost(nTmp, vMin, vMax);
		if (nTmpBest <= nBest)
		{
			pBestGrid = pGrid;
			vBestGrid.push_back(pBestGrid);
		}
		else
			break;
		// endadded at 0705 15:40
	}

	// added at 0705 15:40
	for (int i = 1; i < vBestGrid.size(); i++)
	{
		for (int j = i - 1; j >= 0; j--)
		{
			int nTmpX, nTmpY, nTmpZ, nTmpB, nTmpF;
			gGrid_C *pBG = vBestGrid[j + 1];
			pBG->getPosition(nTmpX, nTmpY, nTmpZ);
			if (nDX != 0)
				nTmpB = nTmpX;
			else
				nTmpB = nTmpY;
			int nBCost = boundingBoxCost(nTmpB, vMin, vMax);

			gGrid_C *pFG = vBestGrid[j];
			pFG->getPosition(nTmpX, nTmpY, nTmpZ);
			if (nDX != 0)
				nTmpF = nTmpX;
			else
				nTmpF = nTmpY;
			int nFCost = boundingBoxCost(nTmpF, vMin, vMax);
			if (nFCost >= nBCost)
				break;
			else
			{
				vBestGrid[j + 1] = pFG;
				vBestGrid[j] = pBG;
			}
		}
	}
	// endadded at 0705 15:40

	pInst->setPlaced(nOrigX, nOrigY);
	//return pBestGrid;
	return vBestGrid;
}
// end added at 0704 22:00

//version 1
/*
gGrid_C* router_C::findPlaceToMove( instance_C* pInst )
{
	int nOrigX = pInst->getPlacedX();
	int nOrigY = pInst->getPlacedY();

	forced_C cF;
	if( pInst == m_vForced[ pInst->getId() ].m_pInstance )
	{
		cF = m_vForced[ pInst->getId() ];
	}
	else
	{
		for( int i=0; i<m_vForced.size(); i++ )
		{
			if( m_vForced[i].m_pInstance == pInst )
			{
				cF = m_vForced[i];
				break;
			}
		}
	}
	// setup search direction
	int nDX = 0;
	int nDY = 0;
	if( !cF.m_bLockInX )
		nDX = cF.m_nR - cF.m_nL;
	if( !cF.m_bLockInY )
		nDY = cF.m_nT - cF.m_nD;
	
	if( nDX == 0 && nDY == 0 )
	{
		cout<<"Warning: No operations will be executed. This instance has balanced forced"<<endl;
	//	return NULL;
	}
	if( abs( nDX ) >= abs( nDY ) )
	{
		nDX = nDX/abs(nDX); nDY = 0;
	}
	else
	{
		nDY = nDY/abs(nDY); nDX = 0; 
	}

	// collecting all the correlated network
	vector< networkForced_C* > vNetWork = cF.m_vNetwork;
	vector< vector< instance_C*> > vBoundingBox;
	for( int i=0; i<vNetWork.size(); i++ )
	{
		vector< instance_C* > vInst;
		networkForced_C* pNF = vNetWork[i];
		vector< forced_C* > vF = pNF->m_vForced;
		for( int j=0; j<vF.size(); j++ )
		{
			instance_C* pTmpInst = vF[j]->m_pInstance;
			if( pTmpInst != pInst )
				vInst.push_back( pTmpInst );
		}
		vBoundingBox.push_back( vInst );
	}
	
	vector<int> vMin;
	vector<int> vMax;
	for( int i=0; i<vBoundingBox.size(); i++ )
	{
		vector< instance_C* > vInst = vBoundingBox[i];
		int nMax;
		int nMin;
		if( nDX != 0 )
		{
			nMax = m_nDX;
			nMin = m_nTX;;
		}
		else
		{
			nMax = m_nDY;
			nMin = m_nTY;
		}


		for( int j=0; j<vInst.size(); j++ )
		{
			instance_C* pTmpInst = vInst[j];
			if( nDX != 0 )
			{
				nMax = max( pTmpInst->getPlacedX(), nMax );
				nMin = min( pTmpInst->getPlacedX(), nMin );
			}
			else
			{
				nMax = max( pTmpInst->getPlacedY(), nMax );
				nMin = min( pTmpInst->getPlacedY(), nMin );
			}
		}
		vMin.push_back( nMin );
		vMax.push_back( nMax );
	}
	// moving instance
	int nBest;
	int nTmp;
	if( nDX != 0 )
		nTmp = pInst->getPlacedX();
	else
		nTmp = pInst->getPlacedY();

	nBest = boundingBoxCost( nTmp, vMin, vMax );
	gGrid_C* pBestGrid = NULL;
	gGrid_C* pGrid = getGrid( m_pDesign, pInst->getPlacedX(), pInst->getPlacedY(), m_nDZ);
	while( graphTravel( m_pDesign, pGrid, nDX, nDY, 0 ) != NULL )
	{
		pGrid = graphTravel( m_pDesign, pGrid, nDX, nDY, 0 );
		// check placeable
		if( !isPlaceable( pGrid, pInst ) )
			continue;
		// 
		int nTmpX, nTmpY, nTmpZ;
		pGrid->getPosition( nTmpX, nTmpY, nTmpZ );
		if( nDX != 0 )
			nTmp = nTmpX;
		else
			nTmp = nTmpY;
		
		int nTmpBest = boundingBoxCost( nTmp, vMin, vMax );
		if( nTmpBest <= nBest ) // change from <
		{
			nBest = nTmpBest;
			pBestGrid = pGrid;
		}
		else
			break;
		//cout<<nBest<<endl;
	}
	pInst->setPlaced( nOrigX, nOrigY );
	return pBestGrid;
}
*/

inline int boundingBoxCost(int nTmp, vector<int> &vMin, vector<int> &vMax)
{
	int nCost = 0;
	int nSize = vMin.size();
	for (int i = 0; i < nSize; i++)
	{
		if( vMin[i] == -1 || vMax[i] == -1 )
			continue;
		int nMin = min(nTmp, vMin[i]);
		int nMax = max(nTmp, vMax[i]);
		nCost = nCost + (nMax - nMin + 1);
		/*
		if( nMin == nTmp && nMax == nTmp )
		{
		
		}
		else
			nCost++;
		*/
	}
	return nCost;
}

vector<gGrid_C *> router_C::routingGridAnalysis(int nDX, int nTX, int nDY, int nTY)
{

	//return true;
}

vector<gGrid_C *> router_C::adjBoundingBoxAnalysis(instance_C *pInst)
{
	//return true;
}

vector<gGrid_C *> router_C::routingGridOrdering(vector<gGrid_C *> &vRGA, vector<gGrid_C *> &vBBA)
{
	//return true;
}

inline bool router_C::lockInstance(instance_C *pInst)
{
	if (pInst == m_vForced[pInst->getId()].m_pInstance)
	{
		m_vForced[pInst->getId()].m_bLockInX = true;
		m_vForced[pInst->getId()].m_bLockInY = true;
	}
	else
	{
		for (int i = 0; i < m_vForced.size(); i++)
		{
			if (m_vForced[i].m_pInstance == pInst)
			{
				m_vForced[i].m_bLockInX = true;
				m_vForced[i].m_bLockInY = true;
				break;
			}
		}
	}
	return true;
}

inline bool router_C::freeInstance(instance_C *pInst)
{
	if (!pInst->isMovable())
		return true;

	if (pInst == m_vForced[pInst->getId()].m_pInstance)
	{
		m_vForced[pInst->getId()].m_bLockInX = false;
		m_vForced[pInst->getId()].m_bLockInX = false;
	}
	else
	{
		for (int i = 0; i < m_vForced.size(); i++)
		{
			if (m_vForced[i].m_pInstance == pInst)
			{
				m_vForced[i].m_bLockInX = false;
				m_vForced[i].m_bLockInX = false;
				break;
			}
		}
	}
	return true;
}

bool router_C::lockInstance(instance_C *pInst, char cDir)
{
	if (pInst == m_vForced[pInst->getId()].m_pInstance)
	{
		if (cDir == 'X')
			m_vForced[pInst->getId()].m_bLockInX = true;
		else
			m_vForced[pInst->getId()].m_bLockInY = true;
	}
	else
	{
		for (int i = 0; i < m_vForced.size(); i++)
		{
			if (m_vForced[i].m_pInstance == pInst)
			{
				if (cDir == 'X')
				{
					m_vForced[i].m_bLockInX = true;
					//m_vForced[i].m_bLockInY = false;
				}
				else
				{
					m_vForced[i].m_bLockInY = true;
					//m_vForced[i].m_bLockInX = false;
				}
				break;
			}
		}
	}
	return true;
}

bool router_C::freeInstance(instance_C *pInst, char cDir)
{
	for (int i = 0; i < m_vForced.size(); i++)
	{
		if (m_vForced[i].m_pInstance == pInst)
		{
			if (cDir == 'X')
				m_vForced[i].m_bLockInX = false;
			else
				m_vForced[i].m_bLockInX = false;
			break;
		}
	}
	return true;
}

// added at 0711 23:30
bool router_C::isPlaceable_ver3(gGrid_C *pGrid, instance_C *pInst, vector<net_C *> &vRipNet)
{
	int nOrigX = pInst->getPlacedX();
	int nOrigY = pInst->getPlacedY();

	//bool bIsOverflow = false;
	int nX, nY, nZ;
	pGrid->getPosition(nX, nY, nZ);
	nZ = m_nOffsetZ;
	// routing information

	// added at 0705 02:30
	vector<networkForced_C *> vNF = m_vForced[pInst->getId()].m_vNetwork;
	vector<net_C *> vNet;
	for (int i = 0; i < vNF.size(); i++)
	{
		vNet.push_back(vNF[i]->m_pNet);
	}
	ripupNet(vNet);
	// end added at 0705 02:30

	putInstOnGraph(pInst, nX, nY, 0);

	//calNeighborCellDemand( m_pDesign, pGrid );
	int nNumLayer = m_pDesign->getLayer().size();
	vector<int> vPinDemand;
	for (int i = 0; i < nNumLayer; i++)
	{
		vPinDemand.push_back(0);
	}

	vector<pin_C> vPin = pInst->getPin();
	for (int i = 0; i < vPin.size(); i++)
	{
		pin_C *pPin = &vPin[i];
		int nLayerId = pPin->getLayerId();
		vPinDemand[nLayerId - nZ]++;
	}

	// added at 0705 02:30
	vector<net_C *> vCanRemovedNet; // record the net that can be removed
	set<net_C *> sCanRemovedNet;	// query the net that can be removed
	vector<int> vNetCount;			// record the net's occurance in the grids

	vector<gGrid_C *> vOverflowGrid; // record the overflow grid
	vector<int> vGridRemand;		 // record the grid remain with overflow
	vector<vector<net_C *>> vGNet;	 // record the net routed through the grid
	// end added at 0705 02:30

	gGrid_C *pLGrid = pGrid;
	// added at 0705 06:30
	set<net_C *> sTmpNet;
	vector<instance_C *> vTmpInst = pGrid->getInstance();
	for (int i = 0; i < vTmpInst.size(); i++)
	{
		forced_C &cF = m_vForced[vTmpInst[i]->getId()];
		vector<networkForced_C *> vNF = cF.m_vNetwork;
		for (int n = 0; n < vNF.size(); n++)
			sTmpNet.insert(vNF[n]->m_pNet);
	}
	// end added at 0705 06:30

	for (int l = 0; l < nNumLayer; l++)
	{
		// added at 0705 02:30
		int nNetCost = 0;
		for (int i = 0; i < vNet.size(); i++)
		{
			if (pLGrid->findNet(vNet[i]))
				nNetCost++;
		}
		// end added at 0705 02:30
		// added at 0705 06:30
		set<net_C *> sTmpNet;
		for (int i = 0; i < vNet.size(); i++)
		{
			if (pLGrid->findNet(vNet[i]))
				nNetCost++;
		}
		// end added at 0705 06:30
		int nRemand = pLGrid->getRemand() - vPinDemand[l] + nNetCost;
		if (nRemand < 0)
		{
			//bIsOverflow = true;
			//break;
			// added at 0705 03:30
			vOverflowGrid.push_back(pLGrid);
			vGridRemand.push_back(pLGrid->getRemand() - vPinDemand[l] + nNetCost);
			vector<net_C *> vTmpNet = pLGrid->getNet();
			for (int n = vTmpNet.size() - 1; n >= 0; n--)
			{
				if (sTmpNet.count(vTmpNet[n]) != 0)
					vTmpNet.erase(vTmpNet.begin() + n);
			}
			vGNet.push_back(vTmpNet);
			// end added at 0705 03:30
		}
		pLGrid = graphTravel(m_pDesign, pLGrid, 0, 0, 1);
	}

	// added at 0705 06:30
	sTmpNet.clear();
	vTmpInst.clear();
	// end added at 0705 06:30

	/*	
	if( bIsOverflow )
	{
		removeInstOnGraph( pInst );
		return false;
	}
	*/
	gGrid_C *pNGrid;

	pNGrid = graphTravel(m_pDesign, pGrid, 0, 1, 0);

	// added at 0705 06:30
	if (pNGrid != NULL)
	{
		vTmpInst = pNGrid->getInstance();
		for (int i = 0; i < vTmpInst.size(); i++)
		{
			forced_C &cF = m_vForced[vTmpInst[i]->getId()];
			vector<networkForced_C *> vNF = cF.m_vNetwork;
			//			for( int n=0; n<vNF.size(); n++ )
			//				sTmpNet.insert( vNF[n]->m_pNet );
		}
	}
	// end added at 0705 06:30

	if (pNGrid != NULL)
	{
		//calNeighborCellDemand( m_pDesign, pNGrid );

		pLGrid = pNGrid;
		for (int l = 0; l < nNumLayer; l++)
		{
			int nRemand = pLGrid->getRemand();
			if (nRemand < 0)
			{
				//bIsOverflow = true;
				//break;
				// added at 0705 03:30
				vOverflowGrid.push_back(pLGrid);
				vGridRemand.push_back(pLGrid->getRemand());
				vector<net_C *> vTmpNet = pLGrid->getNet();
				for (int n = vTmpNet.size() - 1; n >= 0; n--)
				{
					if (sTmpNet.count(vTmpNet[n]) != 0)
						vTmpNet.erase(vTmpNet.begin() + n);
				}
				vGNet.push_back(vTmpNet);
				// end added at 0705 03:30
			}
			pLGrid = graphTravel(m_pDesign, pLGrid, 0, 0, 1);
		}
		/*
		if( bIsOverflow )
		{
			removeInstOnGraph( pInst );
			return false;
		}
		*/
	}
	// added at 0705 06:30
	sTmpNet.clear();
	vTmpInst.clear();
	// end added at 0705 06:30

	pNGrid = graphTravel(m_pDesign, pGrid, 0, -1, 0);
	// added at 0705 06:30
	if (pNGrid != NULL)
	{
		vTmpInst = pNGrid->getInstance();
		for (int i = 0; i < vTmpInst.size(); i++)
		{
			forced_C &cF = m_vForced[vTmpInst[i]->getId()];
			vector<networkForced_C *> vNF = cF.m_vNetwork;
			//			for( int n=0; n<vNF.size(); n++ )
			//				sTmpNet.insert( vNF[n]->m_pNet );
		}
	}
	// end added at 0705 06:30
	if (pNGrid != NULL)
	{
		//calNeighborCellDemand( m_pDesign, pNGrid );

		pLGrid = pNGrid;
		for (int l = 0; l < nNumLayer; l++)
		{
			int nRemand = pLGrid->getRemand();
			if (nRemand < 0)
			{
				//bIsOverflow = true;
				//break;
				// added at 0705 03:30
				vOverflowGrid.push_back(pLGrid);
				vGridRemand.push_back(pLGrid->getRemand());
				vector<net_C *> vTmpNet = pLGrid->getNet();
				for (int n = vTmpNet.size() - 1; n >= 0; n--)
				{
					if (sTmpNet.count(vTmpNet[n]) != 0)
						vTmpNet.erase(vTmpNet.begin() + n);
				}
				vGNet.push_back(vTmpNet);
				// end added at 0705 03:30
			}
			pLGrid = graphTravel(m_pDesign, pLGrid, 0, 0, 1);
		}
		/*
		if( bIsOverflow )
		{
			removeInstOnGraph( pInst );
			return false;
		}
		*/
	}
	// added at 0705 06:30
	sTmpNet.clear();
	vTmpInst.clear();
	// end added at 0705 06:30

	// added at 0705 03:30
	// see if the net can be removed

	//vector< net_C* > vCanRemovedNet;	// record the net that can be removed
	//set< net_C* > sCanRemovedNet;		// query the net that can be removed
	//vector< int > vNetCount;		// record the net's occurance in the grids

	//vector< gGrid_C* > vOverflowGrid;	// record the overflow grid
	//vector< int > vGridRemand;		// record the grid remain with overflow
	//vector< vector< net_C*> > vGNet;	// record the net routed through the grid

	set<net_C *> sINet; // net connected instance
	for (int i = 0; i < vNet.size(); i++)
	{
		sINet.insert(vNet[i]);
	}
	bool bIsOverflow = true;

	// added at 0705 15:20
	if (vOverflowGrid.size() == 0)
		bIsOverflow = false;
	// end added at 0705 15:20

	while (bIsOverflow)
	{
		for (int i = 0; i < vOverflowGrid.size(); i++)
		{
			if (vGridRemand[i] < 0)
			{

				if (vGridRemand[i] + vGNet[i].size() < 0)
				{
					bIsOverflow = false;
					vRipNet.clear();
					break;
				}
				else
				{
					vGridRemand[i] = vGridRemand[i] + vGNet[i].size();
					for (int n = 0; n < vGNet[i].size(); n++)
					{
						net_C *pGNet = vGNet[i][n];
						if (sCanRemovedNet.count(pGNet) == 0)
						{
							sCanRemovedNet.insert(pGNet);
							vRipNet.push_back(pGNet);
						}
					}
				}
			}
		}
	}
	// end added at 0705 03:30

	removeInstOnGraph(pInst);
	pInst->setPlaced(nOrigX, nOrigY);
	for (int i = 0; i < vNet.size(); i++)
	{
		addNetOnGraph(m_pDesign, vNet[i]);
	}
	//	return true;

	// added at 0705 03:30
	if (bIsOverflow)
		return false;
	else
		return true;
	// end added at 0705 03:30
}
// /end added at 0711 23:30

bool router_C::isPlaceable(gGrid_C *pGrid, vector< instance_C* > &vInst, vector<net_C *> &vRipNet)
{
	instance_C* pInst = vInst[0];
	int nOrigX = pInst->getPlacedX();
	int nOrigY = pInst->getPlacedY();

	//bool bIsOverflow = false;
	int nX, nY, nZ;
	pGrid->getPosition(nX, nY, nZ);
	nZ = m_nOffsetZ;
	// routing information

	// added at 0705 02:30
	vector<net_C *> vNet;
	for( int i=0; i<vInst.size(); i++ )
	{
		vector<networkForced_C *> &vNF = m_vForced[vInst[i]->getId()].m_vNetwork;
		for (int j = 0; j < vNF.size(); j++)
		{
			vNet.push_back(vNF[j]->m_pNet);
		}
	}
	ripupNet(vNet);
	// end added at 0705 02:30
	for( int i=0; i<vInst.size(); i++ )
		putInstOnGraph( vInst[i], nX, nY, 0);

	//calNeighborCellDemand( m_pDesign, pGrid );
	int nNumLayer = m_pDesign->getLayer().size();
	vector<int> vPinDemand;
	for (int i = 0; i < nNumLayer; i++)
	{
		vPinDemand.push_back(0);
	}
	
	for( int c=0; c<vInst.size(); c++ )
	{
		vector<pin_C> vPin = vInst[c]->getPin();
		for (int i = 0; i < vPin.size(); i++)
		{
			pin_C *pPin = &vPin[i];
			int nLayerId = pPin->getLayerId();
			vPinDemand[nLayerId - nZ]++;
	}
	}
	// added at 0705 02:30
	vector<net_C *> vCanRemovedNet; // record the net that can be removed
	set<net_C *> sCanRemovedNet;	// query the net that can be removed
	vector<int> vNetCount;			// record the net's occurance in the grids

	vector<gGrid_C *> vOverflowGrid; // record the overflow grid
	vector<int> vGridRemand;		 // record the grid remain with overflow
	vector<vector<net_C *>> vGNet;	 // record the net routed through the grid
	// end added at 0705 02:30

	gGrid_C *pLGrid = pGrid;
	// added at 0705 06:30
	set<net_C *> sTmpNet;
	vector<instance_C *> vTmpInst = pGrid->getInstance();
	for (int i = 0; i < vTmpInst.size(); i++)
	{
		forced_C &cF = m_vForced[vTmpInst[i]->getId()];
		vector<networkForced_C *> vNF = cF.m_vNetwork;
		for (int n = 0; n < vNF.size(); n++)
			sTmpNet.insert(vNF[n]->m_pNet);
	}
	// end added at 0705 06:30

	for (int l = 0; l < nNumLayer; l++)
	{
		// added at 0705 02:30
		int nNetCost = 0;
		/*
		for( int i=0; i<vNet.size(); i++ )
		{
			if( pLGrid->findNet( vNet[i] ) )
				nNetCost++;
		}
		*/
		// end added at 0705 02:30
		// added at 0705 06:30
		//set<net_C *> sTmpNet;
		/*
		for( int i=0; i<vNet.size(); i++ )
		{
			if( pLGrid->findNet( vNet[i] ) )
				nNetCost++;
		}
		*/
		// end added at 0705 06:30
		int nRemand = pLGrid->getRemand() - vPinDemand[l] + nNetCost;
		if (nRemand < 0)
		{
			//bIsOverflow = true;
			//break;
			// added at 0705 03:30
			vOverflowGrid.push_back(pLGrid);
			vGridRemand.push_back(pLGrid->getRemand() - vPinDemand[l] + nNetCost);
			vector<net_C *> vTmpNet = pLGrid->getNet();
			for (int n = vTmpNet.size() - 1; n >= 0; n--)
			{
				if (sTmpNet.count(vTmpNet[n]) != 0)
					vTmpNet.erase(vTmpNet.begin() + n);
			}
			vGNet.push_back(vTmpNet);
			// end added at 0705 03:30
		}
		pLGrid = graphTravel(m_pDesign, pLGrid, 0, 0, 1);
	}

	// added at 0705 06:30
	sTmpNet.clear();
	vTmpInst.clear();
	// end added at 0705 06:30

	/*	
	if( bIsOverflow )
	{
		removeInstOnGraph( pInst );
		return false;
	}
	*/
	gGrid_C *pNGrid;

	pNGrid = graphTravel(m_pDesign, pGrid, 1, 0, 0);
	// added at 0705 06:30
	if (pNGrid != NULL)
	{
		vTmpInst = pNGrid->getInstance();
		for (int i = 0; i < vTmpInst.size(); i++)
		{
			forced_C &cF = m_vForced[vTmpInst[i]->getId()];
			vector<networkForced_C *> vNF = cF.m_vNetwork;
			for (int n = 0; n < vNF.size(); n++)
				sTmpNet.insert(vNF[n]->m_pNet);
		}
	}
	// end added at 0705 06:30

	if (pNGrid != NULL)
	{
		//calNeighborCellDemand( m_pDesign, pNGrid );

		pLGrid = pNGrid;
		for (int l = 0; l < nNumLayer; l++)
		{
			int nRemand = pLGrid->getRemand();
			if (nRemand < 0)
			{
				//bIsOverflow = true;
				//break;
				// added at 0705 03:30
				vOverflowGrid.push_back(pLGrid);
				vGridRemand.push_back(pLGrid->getRemand());
				vector<net_C *> vTmpNet = pLGrid->getNet();
				for (int n = vTmpNet.size() - 1; n >= 0; n--)
				{
					if (sTmpNet.count(vTmpNet[n]) != 0)
						vTmpNet.erase(vTmpNet.begin() + n);
				}
				vGNet.push_back(vTmpNet);
				// end added at 0705 03:30
			}
			pLGrid = graphTravel(m_pDesign, pLGrid, 0, 0, 1);
		}
		/*
		if( bIsOverflow )
		{
			removeInstOnGraph( pInst );
			return false;
		}	
		*/
	}

	// added at 0705 06:30
	sTmpNet.clear();
	vTmpInst.clear();
	// end added at 0705 06:30

	pNGrid = graphTravel(m_pDesign, pGrid, -1, 0, 0);

	// added at 0705 06:30
	if (pNGrid != NULL)
	{
		vTmpInst = pNGrid->getInstance();
		for (int i = 0; i < vTmpInst.size(); i++)
		{
			forced_C &cF = m_vForced[vTmpInst[i]->getId()];
			vector<networkForced_C *> vNF = cF.m_vNetwork;
			for (int n = 0; n < vNF.size(); n++)
				sTmpNet.insert(vNF[n]->m_pNet);
		}
	}
	// end added at 0705 06:30

	if (pNGrid != NULL)
	{
		//calNeighborCellDemand( m_pDesign, pNGrid );

		pLGrid = pNGrid;
		for (int l = 0; l < nNumLayer; l++)
		{
			int nRemand = pLGrid->getRemand();
			if (nRemand < 0)
			{
				//bIsOverflow = true;
				//break;
				// added at 0705 03:30
				vOverflowGrid.push_back(pLGrid);
				vGridRemand.push_back(pLGrid->getRemand());
				vector<net_C *> vTmpNet = pLGrid->getNet();
				for (int n = vTmpNet.size() - 1; n >= 0; n--)
				{
					if (sTmpNet.count(vTmpNet[n]) != 0)
						vTmpNet.erase(vTmpNet.begin() + n);
				}
				vGNet.push_back(vTmpNet);
				// end added at 0705 03:30
			}
			pLGrid = graphTravel(m_pDesign, pLGrid, 0, 0, 1);
		}
		/*
		if( bIsOverflow )
		{
			removeInstOnGraph( pInst );
			return false;
		}
		*/
	}
	// added at 0705 06:30
	sTmpNet.clear();
	vTmpInst.clear();
	// end added at 0705 06:30

	pNGrid = graphTravel(m_pDesign, pGrid, 0, 1, 0);

	// added at 0705 06:30
	if (pNGrid != NULL)
	{
		vTmpInst = pNGrid->getInstance();
		for (int i = 0; i < vTmpInst.size(); i++)
		{
			forced_C &cF = m_vForced[vTmpInst[i]->getId()];
			vector<networkForced_C *> vNF = cF.m_vNetwork;
			for (int n = 0; n < vNF.size(); n++)
				sTmpNet.insert(vNF[n]->m_pNet);
		}
	}
	// end added at 0705 06:30

	if (pNGrid != NULL)
	{
		//calNeighborCellDemand( m_pDesign, pNGrid );

		pLGrid = pNGrid;
		for (int l = 0; l < nNumLayer; l++)
		{
			int nRemand = pLGrid->getRemand();
			if (nRemand < 0)
			{
				//bIsOverflow = true;
				//break;
				// added at 0705 03:30
				vOverflowGrid.push_back(pLGrid);
				vGridRemand.push_back(pLGrid->getRemand());
				vector<net_C *> vTmpNet = pLGrid->getNet();
				for (int n = vTmpNet.size() - 1; n >= 0; n--)
				{
					if (sTmpNet.count(vTmpNet[n]) != 0)
						vTmpNet.erase(vTmpNet.begin() + n);
				}
				vGNet.push_back(vTmpNet);
				// end added at 0705 03:30
			}
			pLGrid = graphTravel(m_pDesign, pLGrid, 0, 0, 1);
		}
		/*
		if( bIsOverflow )
		{
			removeInstOnGraph( pInst );
			return false;
		}
		*/
	}
	// added at 0705 06:30
	sTmpNet.clear();
	vTmpInst.clear();
	// end added at 0705 06:30

	pNGrid = graphTravel(m_pDesign, pGrid, 0, -1, 0);
	// added at 0705 06:30
	if (pNGrid != NULL)
	{
		vTmpInst = pNGrid->getInstance();
		for (int i = 0; i < vTmpInst.size(); i++)
		{
			forced_C &cF = m_vForced[vTmpInst[i]->getId()];
			vector<networkForced_C *> vNF = cF.m_vNetwork;
			for (int n = 0; n < vNF.size(); n++)
				sTmpNet.insert(vNF[n]->m_pNet);
		}
	}
	// end added at 0705 06:30
	if (pNGrid != NULL)
	{
		//calNeighborCellDemand( m_pDesign, pNGrid );

		pLGrid = pNGrid;
		for (int l = 0; l < nNumLayer; l++)
		{
			int nRemand = pLGrid->getRemand();
			if (nRemand < 0)
			{
				//bIsOverflow = true;
				//break;
				// added at 0705 03:30
				vOverflowGrid.push_back(pLGrid);
				vGridRemand.push_back(pLGrid->getRemand());
				vector<net_C *> vTmpNet = pLGrid->getNet();
				for (int n = vTmpNet.size() - 1; n >= 0; n--)
				{
					if (sTmpNet.count(vTmpNet[n]) != 0)
						vTmpNet.erase(vTmpNet.begin() + n);
				}
				vGNet.push_back(vTmpNet);
				// end added at 0705 03:30
			}
			pLGrid = graphTravel(m_pDesign, pLGrid, 0, 0, 1);
		}
		/*
		if( bIsOverflow )
		{
			removeInstOnGraph( pInst );
			return false;
		}
		*/
	}
	// added at 0705 06:30
	sTmpNet.clear();
	vTmpInst.clear();
	// end added at 0705 06:30

	// added at 0705 03:30
	// see if the net can be removed

	//vector< net_C* > vCanRemovedNet;	// record the net that can be removed
	//set< net_C* > sCanRemovedNet;		// query the net that can be removed
	//vector< int > vNetCount;		// record the net's occurance in the grids

	//vector< gGrid_C* > vOverflowGrid;	// record the overflow grid
	//vector< int > vGridRemand;		// record the grid remain with overflow
	//vector< vector< net_C*> > vGNet;	// record the net routed through the grid

	set<net_C *> sINet; // net connected instance
	for (int i = 0; i < vNet.size(); i++)
	{
		sINet.insert(vNet[i]);
	}
	bool bIsOverflow = true;

	// added at 0705 15:20
	if (vOverflowGrid.size() == 0)
		bIsOverflow = false;
	// end added at 0705 15:20
	while (bIsOverflow)
	{
		int nGridCount = 0;
		for (int i = 0; i < vOverflowGrid.size(); i++)
		{
			if (vGridRemand[i] <= 0)
			{
				for (int n = 0; n < vGNet[i].size(); n++)
				{
					net_C *pGNet = vGNet[i][n];
					// if the pGNet is the net connect to the pInst, then do nothing
					if (sINet.count(pGNet) != 0)
						continue;

					bool bFind = false;
					for (int r = 0; r < vCanRemovedNet.size(); r++)
					{
						if (pGNet == vCanRemovedNet[r])
						{
							bFind = true;
							vNetCount[r]++;
							break;
						}
					}
					if (!bFind)
					{
						vCanRemovedNet.push_back(pGNet);
						vNetCount.push_back(1);
					}
				}
			}
			else
			{
				nGridCount++ ;
			}
		}
		int nIndex = -1;
		int nMaxNet = 0;
		for (int i = 0; i < vNetCount.size(); i++)
		{
			if (vNetCount[i] > nMaxNet)
			{
				nMaxNet = vNetCount[i];
				nIndex = i;
			}
		}

		if (nIndex == -1) // no net can eliminate overflow
		{
			if( nGridCount == (int)vOverflowGrid.size() )
				bIsOverflow = false;
			else
				vRipNet.clear();
			break;
		}
		else
		{
			net_C *pRemovedNet = vCanRemovedNet[nIndex];
			for (int i = 0; i < vOverflowGrid.size(); i++)
			{
				if (vOverflowGrid[i]->findNet(pRemovedNet))
				{
					vGridRemand[i]++;
					for (int n = vGNet[i].size() - 1; n >= 0; n--)
					{
						if (vGNet[i][n] == pRemovedNet)
						{
							vGNet[i].erase(vGNet[i].begin() + n);
							break;
						}
					}
				}
			}
			vRipNet.push_back(pRemovedNet);
		}
		vCanRemovedNet.clear();
		vNetCount.clear();
	}
	// end added at 0705 03:30
	for( int i=0; i<vInst.size(); i++ )
	{
		removeInstOnGraph( vInst[i] );
		vInst[i]->setPlaced(nOrigX, nOrigY);
	}
	for (int i = 0; i < vNet.size(); i++)
	{
		addNetOnGraph(m_pDesign, vNet[i]);
	}
	//	return true;

	// added at 0705 03:30
	if (bIsOverflow)
		return false;
	else
		return true;
	// end added at 0705 03:30
}
// end added 0705 02:30
// added 0705 02:30

bool router_C::isPlaceable_ver5(gGrid_C *pGrid, instance_C *pInst, vector<net_C *> &vRipNet)
{
	//cout << "Check Placealbe "<<endl;
	int nOrigX = pInst->getPlacedX();
	int nOrigY = pInst->getPlacedY();

	//bool bIsOverflow = false;
	int nX, nY, nZ;
	pGrid->getPosition(nX, nY, nZ);
	nZ = m_nOffsetZ;
	// routing information

	// added at 0705 02:30
	vector<networkForced_C *> vNF = m_vForced[pInst->getId()].m_vNetwork;
	vector<net_C *> vNet;
	for (int i = 0; i < vNF.size(); i++)
	{
		vNet.push_back(vNF[i]->m_pNet);
	}
	//ripupNet(vNet);
	// end added at 0705 02:30

	putInstOnGraph(pInst, nX, nY, 0);

	//calNeighborCellDemand( m_pDesign, pGrid );
	int nNumLayer = m_pDesign->getLayer().size();
	vector<int> vPinDemand;
	for (int i = 0; i < nNumLayer; i++)
	{
		vPinDemand.push_back(0);
	}

	vector<pin_C> vPin = pInst->getPin();
	for (int i = 0; i < vPin.size(); i++)
	{
		pin_C *pPin = &vPin[i];
		int nLayerId = pPin->getLayerId();
		int nLayerConstraint = nLayerId;
		if( pPin->getNet() != NULL )
			nLayerConstraint = max( nLayerId, pPin->getNet()->getConstraintLayerId() );

		//vPinDemand[nLayerId - nZ]++;
		for( int n=nLayerId; n<=nLayerConstraint; n++ )
			vPinDemand[ n-nZ ]++;
	}

	// added at 0705 02:30
	vector<net_C *> vCanRemovedNet; // record the net that can be removed
	set<net_C *> sCanRemovedNet;	// query the net that can be removed
	vector<int> vNetCount;			// record the net's occurance in the grids

	vector<gGrid_C *> vOverflowGrid; // record the overflow grid
	vector<int> vGridRemand;		 // record the grid remain with overflow
	vector<vector<net_C *>> vGNet;	 // record the net routed through the grid
	// end added at 0705 02:30

	gGrid_C *pLGrid = pGrid;
	// added at 0705 06:30
	set<net_C *> sTmpNet;
	//vector<instance_C *> vTmpInst;
	
	vector<instance_C *> vTmpInst = pGrid->getInstance();
	for (int i = 0; i < vTmpInst.size(); i++)
	{
		forced_C &cF = m_vForced[vTmpInst[i]->getId()];
		vector<networkForced_C *> vNF = cF.m_vNetwork;
		for (int n = 0; n < vNF.size(); n++)
			sTmpNet.insert(vNF[n]->m_pNet);
	}
	
	// end added at 0705 06:30

	for (int l = 0; l < nNumLayer; l++)
	{
		// added at 0705 02:30
		int nNetCost = 0;
		/*
		for( int i=0; i<vNet.size(); i++ )
		{
			if( pLGrid->findNet( vNet[i] ) )
				nNetCost++;
		}
		*/
		// end added at 0705 02:30
		// added at 0705 06:30
		/*
		for( int i=0; i<vNet.size(); i++ )
		{
			if( pLGrid->findNet( vNet[i] ) )
				nNetCost++;
		}
		*/
		// end added at 0705 06:30
		int nRemand = pLGrid->getRemand() - pLGrid->getPinDemand() - vPinDemand[l] + nNetCost;
		/*
		if( pInst->getName() == "C1112" )
		{
			int nCX, nCY, nCZ;
			pLGrid->getPosition( nCX, nCY, nCZ );
			cout << "at grid: "<< nCX << " " << nCY << " " << nCZ << endl;
			cout << pLGrid->getRemand() << " " << pLGrid->getPinDemand() << " " << vPinDemand[l] << endl;	
		}
		*/
		if (nRemand < 0)
		{
			//bIsOverflow = true;
			//break;
			// added at 0705 03:30
			vOverflowGrid.push_back(pLGrid);
			//vGridRemand.push_back(pLGrid->getRemand() - vPinDemand[l] + nNetCost);
			vGridRemand.push_back( nRemand );
			vector<net_C *> vTmpNet = pLGrid->getNet();
			for (int n = vTmpNet.size() - 1; n >= 0; n--)
			{
				if (sTmpNet.count(vTmpNet[n]) != 0)
					vTmpNet.erase(vTmpNet.begin() + n);
			}
			vGNet.push_back(vTmpNet);
			// end added at 0705 03:30
		}
		pLGrid = graphTravel(m_pDesign, pLGrid, 0, 0, 1);
	}

	// added at 0705 06:30
	sTmpNet.clear();
	vTmpInst.clear();
	// end added at 0705 06:30

	/*	
	if( bIsOverflow )
	{
		removeInstOnGraph( pInst );
		return false;
	}
	*/
	gGrid_C *pNGrid;

	pNGrid = graphTravel(m_pDesign, pGrid, 1, 0, 0);
	// added at 0705 06:30
	if (pNGrid != NULL)
	{
		vTmpInst = pNGrid->getInstance();
		for (int i = 0; i < vTmpInst.size(); i++)
		{
			forced_C &cF = m_vForced[vTmpInst[i]->getId()];
			vector<networkForced_C *> vNF = cF.m_vNetwork;
			for (int n = 0; n < vNF.size(); n++)
				sTmpNet.insert(vNF[n]->m_pNet);
		}
	}
	// end added at 0705 06:30

	if (pNGrid != NULL)
	{
		//calNeighborCellDemand( m_pDesign, pNGrid );

		pLGrid = pNGrid;
		for (int l = 0; l < nNumLayer; l++)
		{
			int nRemand = pLGrid->getRemand() - pLGrid->getPinDemand();
			if (nRemand < 0)
			{
				//bIsOverflow = true;
				//break;
				// added at 0705 03:30
				vOverflowGrid.push_back(pLGrid);
				//vGridRemand.push_back( pLGrid->getRemand());
				vGridRemand.push_back( nRemand );
				vector<net_C *> vTmpNet = pLGrid->getNet();
				for (int n = vTmpNet.size() - 1; n >= 0; n--)
				{
					if (sTmpNet.count(vTmpNet[n]) != 0)
						vTmpNet.erase(vTmpNet.begin() + n);
				}
				vGNet.push_back(vTmpNet);
				// end added at 0705 03:30
			}
			pLGrid = graphTravel(m_pDesign, pLGrid, 0, 0, 1);
		}
		/*
		if( bIsOverflow )
		{
			removeInstOnGraph( pInst );
			return false;
		}	
		*/
	}

	// added at 0705 06:30
	sTmpNet.clear();
	vTmpInst.clear();
	// end added at 0705 06:30

	pNGrid = graphTravel(m_pDesign, pGrid, -1, 0, 0);

	// added at 0705 06:30
	if (pNGrid != NULL)
	{
		vTmpInst = pNGrid->getInstance();
		for (int i = 0; i < vTmpInst.size(); i++)
		{
			forced_C &cF = m_vForced[vTmpInst[i]->getId()];
			vector<networkForced_C *> vNF = cF.m_vNetwork;
			for (int n = 0; n < vNF.size(); n++)
				sTmpNet.insert(vNF[n]->m_pNet);
		}
	}
	// end added at 0705 06:30

	if (pNGrid != NULL)
	{
		//calNeighborCellDemand( m_pDesign, pNGrid );

		pLGrid = pNGrid;
		for (int l = 0; l < nNumLayer; l++)
		{
			int nRemand = pLGrid->getRemand() - pLGrid->getPinDemand() ;
			if (nRemand < 0)
			{
				//bIsOverflow = true;
				//break;
				// added at 0705 03:30
				vOverflowGrid.push_back(pLGrid);
				//vGridRemand.push_back(pLGrid->getRemand());
				vGridRemand.push_back( nRemand );
				vector<net_C *> vTmpNet = pLGrid->getNet();
				for (int n = vTmpNet.size() - 1; n >= 0; n--)
				{
					if (sTmpNet.count(vTmpNet[n]) != 0)
						vTmpNet.erase(vTmpNet.begin() + n);
				}
				vGNet.push_back(vTmpNet);
				// end added at 0705 03:30
			}
			pLGrid = graphTravel(m_pDesign, pLGrid, 0, 0, 1);
		}
		/*
		if( bIsOverflow )
		{
			removeInstOnGraph( pInst );
			return false;
		}
		*/
	}
	// added at 0705 06:30
	sTmpNet.clear();
	vTmpInst.clear();
	// end added at 0705 06:30

	pNGrid = graphTravel(m_pDesign, pGrid, 0, 1, 0);

	// added at 0705 06:30
	if (pNGrid != NULL)
	{
		vTmpInst = pNGrid->getInstance();
		for (int i = 0; i < vTmpInst.size(); i++)
		{
			forced_C &cF = m_vForced[vTmpInst[i]->getId()];
			vector<networkForced_C *> vNF = cF.m_vNetwork;
			for (int n = 0; n < vNF.size(); n++)
				sTmpNet.insert(vNF[n]->m_pNet);
		}
	}
	// end added at 0705 06:30

	if (pNGrid != NULL)
	{
		//calNeighborCellDemand( m_pDesign, pNGrid );

		pLGrid = pNGrid;
		for (int l = 0; l < nNumLayer; l++)
		{
			int nRemand = pLGrid->getRemand() - pLGrid->getPinDemand();
			if (nRemand < 0)
			{
				//bIsOverflow = true;
				//break;
				// added at 0705 03:30
				vOverflowGrid.push_back(pLGrid);
				//vGridRemand.push_back(pLGrid->getRemand());
				vGridRemand.push_back( nRemand );
				vector<net_C *> vTmpNet = pLGrid->getNet();
				for (int n = vTmpNet.size() - 1; n >= 0; n--)
				{
					if (sTmpNet.count(vTmpNet[n]) != 0)
						vTmpNet.erase(vTmpNet.begin() + n);
				}
				vGNet.push_back(vTmpNet);
				// end added at 0705 03:30
			}
			pLGrid = graphTravel(m_pDesign, pLGrid, 0, 0, 1);
		}
		/*
		if( bIsOverflow )
		{
			removeInstOnGraph( pInst );
			return false;
		}
		*/
	}
	// added at 0705 06:30
	sTmpNet.clear();
	vTmpInst.clear();
	// end added at 0705 06:30

	pNGrid = graphTravel(m_pDesign, pGrid, 0, -1, 0);
	// added at 0705 06:30
	if (pNGrid != NULL)
	{
		vTmpInst = pNGrid->getInstance();
		for (int i = 0; i < vTmpInst.size(); i++)
		{
			forced_C &cF = m_vForced[vTmpInst[i]->getId()];
			vector<networkForced_C *> vNF = cF.m_vNetwork;
			for (int n = 0; n < vNF.size(); n++)
				sTmpNet.insert(vNF[n]->m_pNet);
		}
	}
	// end added at 0705 06:30
	if (pNGrid != NULL)
	{
		//calNeighborCellDemand( m_pDesign, pNGrid );

		pLGrid = pNGrid;
		for (int l = 0; l < nNumLayer; l++)
		{
			int nRemand = pLGrid->getRemand() - pLGrid->getPinDemand();
			if (nRemand < 0)
			{
				//bIsOverflow = true;
				//break;
				// added at 0705 03:30
				vOverflowGrid.push_back(pLGrid);
				//vGridRemand.push_back(pLGrid->getRemand());
				vGridRemand.push_back( nRemand );
				vector<net_C *> vTmpNet = pLGrid->getNet();
				for (int n = vTmpNet.size() - 1; n >= 0; n--)
				{
					if (sTmpNet.count(vTmpNet[n]) != 0)
						vTmpNet.erase(vTmpNet.begin() + n);
				}
				vGNet.push_back(vTmpNet);
				// end added at 0705 03:30
			}
			pLGrid = graphTravel(m_pDesign, pLGrid, 0, 0, 1);
		}
		/*
		if( bIsOverflow )
		{
			removeInstOnGraph( pInst );
			return false;
		}
		*/
	}
	// added at 0705 06:30
	sTmpNet.clear();
	vTmpInst.clear();
	// end added at 0705 06:30

	// added at 0705 03:30
	// see if the net can be removed

	//vector< net_C* > vCanRemovedNet;	// record the net that can be removed
	//set< net_C* > sCanRemovedNet;		// query the net that can be removed
	//vector< int > vNetCount;		// record the net's occurance in the grids

	//vector< gGrid_C* > vOverflowGrid;	// record the overflow grid
	//vector< int > vGridRemand;		// record the grid remain with overflow
	//vector< vector< net_C*> > vGNet;	// record the net routed through the grid

	set<net_C *> sINet; // net connected instance
	for (int i = 0; i < vNet.size(); i++)
	{
		sINet.insert(vNet[i]);
	}
	bool bIsOverflow = true;

	// added at 0705 15:20
	if (vOverflowGrid.size() == 0)
		bIsOverflow = false;
	// end added at 0705 15:20

	while (bIsOverflow)
	{
		int nNoOverflow = 0;
		for (int i = 0; i < vOverflowGrid.size(); i++)
		{
			//vNetCount.push_back(0);
			if (vGridRemand[i] < 0)
			{
				for (int n = 0; n < vGNet[i].size(); n++)
				{
					net_C *pGNet = vGNet[i][n];
					// if the pGNet is the net connect to the pInst, then do nothing
					if (sINet.count(pGNet) != 0)
						continue;

					bool bFind = false;
					for (int r = 0; r < vCanRemovedNet.size(); r++)
					{
						if (pGNet == vCanRemovedNet[r])
						{
							bFind = true;
							vNetCount[r]++;
							break;
						}
					}
					if (!bFind)
					{
						vCanRemovedNet.push_back(pGNet);
						vNetCount.push_back(1);
						//vNetCount[r]++;
					}
				}
			}
			else
			{
				nNoOverflow++;
			}
		}
		int nIndex = -1;
		int nMaxNet = 0;
		for (int i = 0; i < vNetCount.size(); i++)
		{
			if (vNetCount[i] > nMaxNet)
			{
				nMaxNet = vNetCount[i];
				nIndex = i;
			}
		}

		if (nIndex == -1) // no net can eliminate overflow
		{
			if( nNoOverflow != (int)vOverflowGrid.size() )
				vRipNet.clear();
			else
				bIsOverflow = false;
			break;
		}
		else
		{
			net_C *pRemovedNet = vCanRemovedNet[nIndex];
			for (int i = 0; i < vOverflowGrid.size(); i++)
			{
				if (vOverflowGrid[i]->findNet(pRemovedNet))
				{
					vGridRemand[i]++;
					for (int n = vGNet[i].size() - 1; n >= 0; n--)
					{
						if (vGNet[i][n] == pRemovedNet)
						{
							vGNet[i].erase(vGNet[i].begin() + n);
							break;
						}
					}
				}
			}
			vRipNet.push_back(pRemovedNet);
		}
		vCanRemovedNet.clear();
		vNetCount.clear();
	}
	// end added at 0705 03:30

	removeInstOnGraph(pInst);
	pInst->setPlaced(nOrigX, nOrigY);
	/*
	for (int i = 0; i < vNet.size(); i++)
	{
		addNetOnGraph(m_pDesign, vNet[i]);
	}
	*/
	//	return true;

	// added at 0705 03:30
	if (bIsOverflow)
		return false;
	else
		return true;
	// end added at 0705 03:30
}

bool router_C::isPlaceable_ver4(gGrid_C *pGrid, instance_C *pInst, vector<net_C *> &vRipNet)
{
	int nOrigX = pInst->getPlacedX();
	int nOrigY = pInst->getPlacedY();

	//bool bIsOverflow = false;
	int nX, nY, nZ;
	pGrid->getPosition(nX, nY, nZ);
	nZ = m_nOffsetZ;
	// routing information

	// added at 0705 02:30
	vector<networkForced_C *> vNF = m_vForced[pInst->getId()].m_vNetwork;
	vector<net_C *> vNet;
	for (int i = 0; i < vNF.size(); i++)
	{
		vNet.push_back(vNF[i]->m_pNet);
	}
	//ripupNet(vNet);
	// end added at 0705 02:30

	putInstOnGraph(pInst, nX, nY, 0);

	//calNeighborCellDemand( m_pDesign, pGrid );
	int nNumLayer = m_pDesign->getLayer().size();
	vector<int> vPinDemand;
	for (int i = 0; i < nNumLayer; i++)
	{
		vPinDemand.push_back(0);
	}

	vector<pin_C> vPin = pInst->getPin();
	for (int i = 0; i < vPin.size(); i++)
	{
		pin_C *pPin = &vPin[i];
		int nLayerId = pPin->getLayerId();
		vPinDemand[nLayerId - nZ]++;
	}

	// added at 0705 02:30
	vector<net_C *> vCanRemovedNet; // record the net that can be removed
	set<net_C *> sCanRemovedNet;	// query the net that can be removed
	vector<int> vNetCount;			// record the net's occurance in the grids

	vector<gGrid_C *> vOverflowGrid; // record the overflow grid
	vector<int> vGridRemand;		 // record the grid remain with overflow
	vector<vector<net_C *>> vGNet;	 // record the net routed through the grid
	// end added at 0705 02:30

	gGrid_C *pLGrid = pGrid;
	// added at 0705 06:30
	set<net_C *> sTmpNet;
	vector<instance_C *> vTmpInst = pGrid->getInstance();
	for (int i = 0; i < vTmpInst.size(); i++)
	{
		forced_C &cF = m_vForced[vTmpInst[i]->getId()];
		vector<networkForced_C *> vNF = cF.m_vNetwork;
		for (int n = 0; n < vNF.size(); n++)
			sTmpNet.insert(vNF[n]->m_pNet);
	}
	// end added at 0705 06:30

	for (int l = 0; l < nNumLayer; l++)
	{
		// added at 0705 02:30
		int nNetCost = 0;
		/*
		for( int i=0; i<vNet.size(); i++ )
		{
			if( pLGrid->findNet( vNet[i] ) )
				nNetCost++;
		}
		*/
		// end added at 0705 02:30
		// added at 0705 06:30
		set<net_C *> sTmpNet;
		/*
		for( int i=0; i<vNet.size(); i++ )
		{
			if( pLGrid->findNet( vNet[i] ) )
				nNetCost++;
		}
		*/
		// end added at 0705 06:30
		int nRemand = pLGrid->getRemand() - vPinDemand[l] + nNetCost;
		if (nRemand < 0)
		{
			//bIsOverflow = true;
			//break;
			// added at 0705 03:30
			vOverflowGrid.push_back(pLGrid);
			vGridRemand.push_back(pLGrid->getRemand() - vPinDemand[l] + nNetCost);
			vector<net_C *> vTmpNet = pLGrid->getNet();
			for (int n = vTmpNet.size() - 1; n >= 0; n--)
			{
				if (sTmpNet.count(vTmpNet[n]) != 0)
					vTmpNet.erase(vTmpNet.begin() + n);
			}
			vGNet.push_back(vTmpNet);
			// end added at 0705 03:30
		}
		pLGrid = graphTravel(m_pDesign, pLGrid, 0, 0, 1);
	}

	// added at 0705 06:30
	sTmpNet.clear();
	vTmpInst.clear();
	// end added at 0705 06:30

	/*	
	if( bIsOverflow )
	{
		removeInstOnGraph( pInst );
		return false;
	}
	*/
	gGrid_C *pNGrid;

	pNGrid = graphTravel(m_pDesign, pGrid, 1, 0, 0);
	// added at 0705 06:30
	if (pNGrid != NULL)
	{
		vTmpInst = pNGrid->getInstance();
		for (int i = 0; i < vTmpInst.size(); i++)
		{
			forced_C &cF = m_vForced[vTmpInst[i]->getId()];
			vector<networkForced_C *> vNF = cF.m_vNetwork;
			for (int n = 0; n < vNF.size(); n++)
				sTmpNet.insert(vNF[n]->m_pNet);
		}
	}
	// end added at 0705 06:30

	if (pNGrid != NULL)
	{
		//calNeighborCellDemand( m_pDesign, pNGrid );

		pLGrid = pNGrid;
		for (int l = 0; l < nNumLayer; l++)
		{
			int nRemand = pLGrid->getRemand();
			if (nRemand < 0)
			{
				//bIsOverflow = true;
				//break;
				// added at 0705 03:30
				vOverflowGrid.push_back(pLGrid);
				vGridRemand.push_back(pLGrid->getRemand());
				vector<net_C *> vTmpNet = pLGrid->getNet();
				for (int n = vTmpNet.size() - 1; n >= 0; n--)
				{
					if (sTmpNet.count(vTmpNet[n]) != 0)
						vTmpNet.erase(vTmpNet.begin() + n);
				}
				vGNet.push_back(vTmpNet);
				// end added at 0705 03:30
			}
			pLGrid = graphTravel(m_pDesign, pLGrid, 0, 0, 1);
		}
		/*
		if( bIsOverflow )
		{
			removeInstOnGraph( pInst );
			return false;
		}	
		*/
	}

	// added at 0705 06:30
	sTmpNet.clear();
	vTmpInst.clear();
	// end added at 0705 06:30

	pNGrid = graphTravel(m_pDesign, pGrid, -1, 0, 0);

	// added at 0705 06:30
	if (pNGrid != NULL)
	{
		vTmpInst = pNGrid->getInstance();
		for (int i = 0; i < vTmpInst.size(); i++)
		{
			forced_C &cF = m_vForced[vTmpInst[i]->getId()];
			vector<networkForced_C *> vNF = cF.m_vNetwork;
			for (int n = 0; n < vNF.size(); n++)
				sTmpNet.insert(vNF[n]->m_pNet);
		}
	}
	// end added at 0705 06:30

	if (pNGrid != NULL)
	{
		//calNeighborCellDemand( m_pDesign, pNGrid );

		pLGrid = pNGrid;
		for (int l = 0; l < nNumLayer; l++)
		{
			int nRemand = pLGrid->getRemand();
			if (nRemand < 0)
			{
				//bIsOverflow = true;
				//break;
				// added at 0705 03:30
				vOverflowGrid.push_back(pLGrid);
				vGridRemand.push_back(pLGrid->getRemand());
				vector<net_C *> vTmpNet = pLGrid->getNet();
				for (int n = vTmpNet.size() - 1; n >= 0; n--)
				{
					if (sTmpNet.count(vTmpNet[n]) != 0)
						vTmpNet.erase(vTmpNet.begin() + n);
				}
				vGNet.push_back(vTmpNet);
				// end added at 0705 03:30
			}
			pLGrid = graphTravel(m_pDesign, pLGrid, 0, 0, 1);
		}
		/*
		if( bIsOverflow )
		{
			removeInstOnGraph( pInst );
			return false;
		}
		*/
	}
	// added at 0705 06:30
	sTmpNet.clear();
	vTmpInst.clear();
	// end added at 0705 06:30

	pNGrid = graphTravel(m_pDesign, pGrid, 0, 1, 0);

	// added at 0705 06:30
	if (pNGrid != NULL)
	{
		vTmpInst = pNGrid->getInstance();
		for (int i = 0; i < vTmpInst.size(); i++)
		{
			forced_C &cF = m_vForced[vTmpInst[i]->getId()];
			vector<networkForced_C *> vNF = cF.m_vNetwork;
			for (int n = 0; n < vNF.size(); n++)
				sTmpNet.insert(vNF[n]->m_pNet);
		}
	}
	// end added at 0705 06:30

	if (pNGrid != NULL)
	{
		//calNeighborCellDemand( m_pDesign, pNGrid );

		pLGrid = pNGrid;
		for (int l = 0; l < nNumLayer; l++)
		{
			int nRemand = pLGrid->getRemand();
			if (nRemand < 0)
			{
				//bIsOverflow = true;
				//break;
				// added at 0705 03:30
				vOverflowGrid.push_back(pLGrid);
				vGridRemand.push_back(pLGrid->getRemand());
				vector<net_C *> vTmpNet = pLGrid->getNet();
				for (int n = vTmpNet.size() - 1; n >= 0; n--)
				{
					if (sTmpNet.count(vTmpNet[n]) != 0)
						vTmpNet.erase(vTmpNet.begin() + n);
				}
				vGNet.push_back(vTmpNet);
				// end added at 0705 03:30
			}
			pLGrid = graphTravel(m_pDesign, pLGrid, 0, 0, 1);
		}
		/*
		if( bIsOverflow )
		{
			removeInstOnGraph( pInst );
			return false;
		}
		*/
	}
	// added at 0705 06:30
	sTmpNet.clear();
	vTmpInst.clear();
	// end added at 0705 06:30

	pNGrid = graphTravel(m_pDesign, pGrid, 0, -1, 0);
	// added at 0705 06:30
	if (pNGrid != NULL)
	{
		vTmpInst = pNGrid->getInstance();
		for (int i = 0; i < vTmpInst.size(); i++)
		{
			forced_C &cF = m_vForced[vTmpInst[i]->getId()];
			vector<networkForced_C *> vNF = cF.m_vNetwork;
			for (int n = 0; n < vNF.size(); n++)
				sTmpNet.insert(vNF[n]->m_pNet);
		}
	}
	// end added at 0705 06:30
	if (pNGrid != NULL)
	{
		//calNeighborCellDemand( m_pDesign, pNGrid );

		pLGrid = pNGrid;
		for (int l = 0; l < nNumLayer; l++)
		{
			int nRemand = pLGrid->getRemand();
			if (nRemand < 0)
			{
				//bIsOverflow = true;
				//break;
				// added at 0705 03:30
				vOverflowGrid.push_back(pLGrid);
				vGridRemand.push_back(pLGrid->getRemand());
				vector<net_C *> vTmpNet = pLGrid->getNet();
				for (int n = vTmpNet.size() - 1; n >= 0; n--)
				{
					if (sTmpNet.count(vTmpNet[n]) != 0)
						vTmpNet.erase(vTmpNet.begin() + n);
				}
				vGNet.push_back(vTmpNet);
				// end added at 0705 03:30
			}
			pLGrid = graphTravel(m_pDesign, pLGrid, 0, 0, 1);
		}
		/*
		if( bIsOverflow )
		{
			removeInstOnGraph( pInst );
			return false;
		}
		*/
	}
	// added at 0705 06:30
	sTmpNet.clear();
	vTmpInst.clear();
	// end added at 0705 06:30

	// added at 0705 03:30
	// see if the net can be removed

	//vector< net_C* > vCanRemovedNet;	// record the net that can be removed
	//set< net_C* > sCanRemovedNet;		// query the net that can be removed
	//vector< int > vNetCount;		// record the net's occurance in the grids

	//vector< gGrid_C* > vOverflowGrid;	// record the overflow grid
	//vector< int > vGridRemand;		// record the grid remain with overflow
	//vector< vector< net_C*> > vGNet;	// record the net routed through the grid

	set<net_C *> sINet; // net connected instance
	for (int i = 0; i < vNet.size(); i++)
	{
		sINet.insert(vNet[i]);
	}
	bool bIsOverflow = true;

	// added at 0705 15:20
	if (vOverflowGrid.size() == 0)
		bIsOverflow = false;
	// end added at 0705 15:20

	while (bIsOverflow)
	{
		int nNoOverflow = 0;
		for (int i = 0; i < vOverflowGrid.size(); i++)
		{
			//vNetCount.push_back(0);
			if (vGridRemand[i] < 0)
			{
				for (int n = 0; n < vGNet[i].size(); n++)
				{
					net_C *pGNet = vGNet[i][n];
					// if the pGNet is the net connect to the pInst, then do nothing
					if (sINet.count(pGNet) != 0)
						continue;

					bool bFind = false;
					for (int r = 0; r < vCanRemovedNet.size(); r++)
					{
						if (pGNet == vCanRemovedNet[r])
						{
							bFind = true;
							vNetCount[r]++;
							break;
						}
					}
					if (!bFind)
					{
						vCanRemovedNet.push_back(pGNet);
						vNetCount.push_back(1);
						//vNetCount[r]++;
					}
				}
			}
			else
			{
				nNoOverflow++;
			}
		}
		int nIndex = -1;
		int nMaxNet = 0;
		for (int i = 0; i < vNetCount.size(); i++)
		{
			if (vNetCount[i] > nMaxNet)
			{
				nMaxNet = vNetCount[i];
				nIndex = i;
			}
		}

		if (nIndex == -1) // no net can eliminate overflow
		{
			if( nNoOverflow != (int)vOverflowGrid.size() )
				vRipNet.clear();
			else
				bIsOverflow = false;
			break;
		}
		else
		{
			net_C *pRemovedNet = vCanRemovedNet[nIndex];
			for (int i = 0; i < vOverflowGrid.size(); i++)
			{
				if (vOverflowGrid[i]->findNet(pRemovedNet))
				{
					vGridRemand[i]++;
					for (int n = vGNet[i].size() - 1; n >= 0; n--)
					{
						if (vGNet[i][n] == pRemovedNet)
						{
							vGNet[i].erase(vGNet[i].begin() + n);
							break;
						}
					}
				}
			}
			vRipNet.push_back(pRemovedNet);
		}
		vCanRemovedNet.clear();
		vNetCount.clear();
	}
	// end added at 0705 03:30

	removeInstOnGraph(pInst);
	pInst->setPlaced(nOrigX, nOrigY);
	/*
	for (int i = 0; i < vNet.size(); i++)
	{
		addNetOnGraph(m_pDesign, vNet[i]);
	}
	*/
	//	return true;

	// added at 0705 03:30
	if (bIsOverflow)
		return false;
	else
		return true;
	// end added at 0705 03:30
}
//
bool router_C::isPlaceable(gGrid_C *pGrid, instance_C *pInst, vector<net_C *> &vRipNet)
{
	int nOrigX = pInst->getPlacedX();
	int nOrigY = pInst->getPlacedY();

	//bool bIsOverflow = false;
	int nX, nY, nZ;
	pGrid->getPosition(nX, nY, nZ);
	nZ = m_nOffsetZ;
	// routing information

	// added at 0705 02:30
	vector<networkForced_C *> vNF = m_vForced[pInst->getId()].m_vNetwork;
	vector<net_C *> vNet;
	for (int i = 0; i < vNF.size(); i++)
	{
		vNet.push_back(vNF[i]->m_pNet);
	}
	ripupNet(vNet);
	// end added at 0705 02:30

	putInstOnGraph(pInst, nX, nY, 0);

	//calNeighborCellDemand( m_pDesign, pGrid );
	int nNumLayer = m_pDesign->getLayer().size();
	vector<int> vPinDemand;
	for (int i = 0; i < nNumLayer; i++)
	{
		vPinDemand.push_back(0);
	}

	vector<pin_C> vPin = pInst->getPin();
	for (int i = 0; i < vPin.size(); i++)
	{
		pin_C *pPin = &vPin[i];
		int nLayerId = pPin->getLayerId();
		vPinDemand[nLayerId - nZ]++;
	}

	// added at 0705 02:30
	vector<net_C *> vCanRemovedNet; // record the net that can be removed
	set<net_C *> sCanRemovedNet;	// query the net that can be removed
	vector<int> vNetCount;			// record the net's occurance in the grids

	vector<gGrid_C *> vOverflowGrid; // record the overflow grid
	vector<int> vGridRemand;		 // record the grid remain with overflow
	vector<vector<net_C *>> vGNet;	 // record the net routed through the grid
	// end added at 0705 02:30

	gGrid_C *pLGrid = pGrid;
	// added at 0705 06:30
	set<net_C *> sTmpNet;
	vector<instance_C *> vTmpInst = pGrid->getInstance();
	for (int i = 0; i < vTmpInst.size(); i++)
	{
		forced_C &cF = m_vForced[vTmpInst[i]->getId()];
		vector<networkForced_C *> vNF = cF.m_vNetwork;
		for (int n = 0; n < vNF.size(); n++)
			sTmpNet.insert(vNF[n]->m_pNet);
	}
	// end added at 0705 06:30

	for (int l = 0; l < nNumLayer; l++)
	{
		// added at 0705 02:30
		int nNetCost = 0;
		/*
		for( int i=0; i<vNet.size(); i++ )
		{
			if( pLGrid->findNet( vNet[i] ) )
				nNetCost++;
		}
		*/
		// end added at 0705 02:30
		// added at 0705 06:30
		set<net_C *> sTmpNet;
		/*
		for( int i=0; i<vNet.size(); i++ )
		{
			if( pLGrid->findNet( vNet[i] ) )
				nNetCost++;
		}
		*/
		// end added at 0705 06:30
		int nRemand = pLGrid->getRemand() - vPinDemand[l] + nNetCost;
		if (nRemand < 0)
		{
			//bIsOverflow = true;
			//break;
			// added at 0705 03:30
			vOverflowGrid.push_back(pLGrid);
			vGridRemand.push_back(pLGrid->getRemand() - vPinDemand[l] + nNetCost);
			vector<net_C *> vTmpNet = pLGrid->getNet();
			for (int n = vTmpNet.size() - 1; n >= 0; n--)
			{
				if (sTmpNet.count(vTmpNet[n]) != 0)
					vTmpNet.erase(vTmpNet.begin() + n);
			}
			vGNet.push_back(vTmpNet);
			// end added at 0705 03:30
		}
		pLGrid = graphTravel(m_pDesign, pLGrid, 0, 0, 1);
	}

	// added at 0705 06:30
	sTmpNet.clear();
	vTmpInst.clear();
	// end added at 0705 06:30

	/*	
	if( bIsOverflow )
	{
		removeInstOnGraph( pInst );
		return false;
	}
	*/
	gGrid_C *pNGrid;

	pNGrid = graphTravel(m_pDesign, pGrid, 1, 0, 0);
	// added at 0705 06:30
	if (pNGrid != NULL)
	{
		vTmpInst = pNGrid->getInstance();
		for (int i = 0; i < vTmpInst.size(); i++)
		{
			forced_C &cF = m_vForced[vTmpInst[i]->getId()];
			vector<networkForced_C *> vNF = cF.m_vNetwork;
			for (int n = 0; n < vNF.size(); n++)
				sTmpNet.insert(vNF[n]->m_pNet);
		}
	}
	// end added at 0705 06:30

	if (pNGrid != NULL)
	{
		//calNeighborCellDemand( m_pDesign, pNGrid );

		pLGrid = pNGrid;
		for (int l = 0; l < nNumLayer; l++)
		{
			int nRemand = pLGrid->getRemand();
			if (nRemand < 0)
			{
				//bIsOverflow = true;
				//break;
				// added at 0705 03:30
				vOverflowGrid.push_back(pLGrid);
				vGridRemand.push_back(pLGrid->getRemand());
				vector<net_C *> vTmpNet = pLGrid->getNet();
				for (int n = vTmpNet.size() - 1; n >= 0; n--)
				{
					if (sTmpNet.count(vTmpNet[n]) != 0)
						vTmpNet.erase(vTmpNet.begin() + n);
				}
				vGNet.push_back(vTmpNet);
				// end added at 0705 03:30
			}
			pLGrid = graphTravel(m_pDesign, pLGrid, 0, 0, 1);
		}
		/*
		if( bIsOverflow )
		{
			removeInstOnGraph( pInst );
			return false;
		}	
		*/
	}

	// added at 0705 06:30
	sTmpNet.clear();
	vTmpInst.clear();
	// end added at 0705 06:30

	pNGrid = graphTravel(m_pDesign, pGrid, -1, 0, 0);

	// added at 0705 06:30
	if (pNGrid != NULL)
	{
		vTmpInst = pNGrid->getInstance();
		for (int i = 0; i < vTmpInst.size(); i++)
		{
			forced_C &cF = m_vForced[vTmpInst[i]->getId()];
			vector<networkForced_C *> vNF = cF.m_vNetwork;
			for (int n = 0; n < vNF.size(); n++)
				sTmpNet.insert(vNF[n]->m_pNet);
		}
	}
	// end added at 0705 06:30

	if (pNGrid != NULL)
	{
		//calNeighborCellDemand( m_pDesign, pNGrid );

		pLGrid = pNGrid;
		for (int l = 0; l < nNumLayer; l++)
		{
			int nRemand = pLGrid->getRemand();
			if (nRemand < 0)
			{
				//bIsOverflow = true;
				//break;
				// added at 0705 03:30
				vOverflowGrid.push_back(pLGrid);
				vGridRemand.push_back(pLGrid->getRemand());
				vector<net_C *> vTmpNet = pLGrid->getNet();
				for (int n = vTmpNet.size() - 1; n >= 0; n--)
				{
					if (sTmpNet.count(vTmpNet[n]) != 0)
						vTmpNet.erase(vTmpNet.begin() + n);
				}
				vGNet.push_back(vTmpNet);
				// end added at 0705 03:30
			}
			pLGrid = graphTravel(m_pDesign, pLGrid, 0, 0, 1);
		}
		/*
		if( bIsOverflow )
		{
			removeInstOnGraph( pInst );
			return false;
		}
		*/
	}
	// added at 0705 06:30
	sTmpNet.clear();
	vTmpInst.clear();
	// end added at 0705 06:30

	pNGrid = graphTravel(m_pDesign, pGrid, 0, 1, 0);

	// added at 0705 06:30
	if (pNGrid != NULL)
	{
		vTmpInst = pNGrid->getInstance();
		for (int i = 0; i < vTmpInst.size(); i++)
		{
			forced_C &cF = m_vForced[vTmpInst[i]->getId()];
			vector<networkForced_C *> vNF = cF.m_vNetwork;
			for (int n = 0; n < vNF.size(); n++)
				sTmpNet.insert(vNF[n]->m_pNet);
		}
	}
	// end added at 0705 06:30

	if (pNGrid != NULL)
	{
		//calNeighborCellDemand( m_pDesign, pNGrid );

		pLGrid = pNGrid;
		for (int l = 0; l < nNumLayer; l++)
		{
			int nRemand = pLGrid->getRemand();
			if (nRemand < 0)
			{
				//bIsOverflow = true;
				//break;
				// added at 0705 03:30
				vOverflowGrid.push_back(pLGrid);
				vGridRemand.push_back(pLGrid->getRemand());
				vector<net_C *> vTmpNet = pLGrid->getNet();
				for (int n = vTmpNet.size() - 1; n >= 0; n--)
				{
					if (sTmpNet.count(vTmpNet[n]) != 0)
						vTmpNet.erase(vTmpNet.begin() + n);
				}
				vGNet.push_back(vTmpNet);
				// end added at 0705 03:30
			}
			pLGrid = graphTravel(m_pDesign, pLGrid, 0, 0, 1);
		}
		/*
		if( bIsOverflow )
		{
			removeInstOnGraph( pInst );
			return false;
		}
		*/
	}
	// added at 0705 06:30
	sTmpNet.clear();
	vTmpInst.clear();
	// end added at 0705 06:30

	pNGrid = graphTravel(m_pDesign, pGrid, 0, -1, 0);
	// added at 0705 06:30
	if (pNGrid != NULL)
	{
		vTmpInst = pNGrid->getInstance();
		for (int i = 0; i < vTmpInst.size(); i++)
		{
			forced_C &cF = m_vForced[vTmpInst[i]->getId()];
			vector<networkForced_C *> vNF = cF.m_vNetwork;
			for (int n = 0; n < vNF.size(); n++)
				sTmpNet.insert(vNF[n]->m_pNet);
		}
	}
	// end added at 0705 06:30
	if (pNGrid != NULL)
	{
		//calNeighborCellDemand( m_pDesign, pNGrid );

		pLGrid = pNGrid;
		for (int l = 0; l < nNumLayer; l++)
		{
			int nRemand = pLGrid->getRemand();
			if (nRemand < 0)
			{
				//bIsOverflow = true;
				//break;
				// added at 0705 03:30
				vOverflowGrid.push_back(pLGrid);
				vGridRemand.push_back(pLGrid->getRemand());
				vector<net_C *> vTmpNet = pLGrid->getNet();
				for (int n = vTmpNet.size() - 1; n >= 0; n--)
				{
					if (sTmpNet.count(vTmpNet[n]) != 0)
						vTmpNet.erase(vTmpNet.begin() + n);
				}
				vGNet.push_back(vTmpNet);
				// end added at 0705 03:30
			}
			pLGrid = graphTravel(m_pDesign, pLGrid, 0, 0, 1);
		}
		/*
		if( bIsOverflow )
		{
			removeInstOnGraph( pInst );
			return false;
		}
		*/
	}
	// added at 0705 06:30
	sTmpNet.clear();
	vTmpInst.clear();
	// end added at 0705 06:30

	// added at 0705 03:30
	// see if the net can be removed

	//vector< net_C* > vCanRemovedNet;	// record the net that can be removed
	//set< net_C* > sCanRemovedNet;		// query the net that can be removed
	//vector< int > vNetCount;		// record the net's occurance in the grids

	//vector< gGrid_C* > vOverflowGrid;	// record the overflow grid
	//vector< int > vGridRemand;		// record the grid remain with overflow
	//vector< vector< net_C*> > vGNet;	// record the net routed through the grid

	set<net_C *> sINet; // net connected instance
	for (int i = 0; i < vNet.size(); i++)
	{
		sINet.insert(vNet[i]);
	}
	bool bIsOverflow = true;

	// added at 0705 15:20
	if (vOverflowGrid.size() == 0)
		bIsOverflow = false;
	// end added at 0705 15:20

	while (bIsOverflow)
	{
		int nNoOverflow = 0;
		for (int i = 0; i < vOverflowGrid.size(); i++)
		{
			if (vGridRemand[i] < 0)
			{
				for (int n = 0; n < vGNet[i].size(); n++)
				{
					net_C *pGNet = vGNet[i][n];
					// if the pGNet is the net connect to the pInst, then do nothing
					if (sINet.count(pGNet) != 0)
						continue;

					bool bFind = false;
					for (int r = 0; r < vCanRemovedNet.size(); r++)
					{
						if (pGNet == vCanRemovedNet[r])
						{
							bFind = true;
							vNetCount[r]++;
							break;
						}
					}
					if (!bFind)
					{
						vCanRemovedNet.push_back(pGNet);
						vNetCount.push_back(1);
					}
				}
			}
			else
			{
				nNoOverflow++;
			}
		}
		int nIndex = -1;
		int nMaxNet = 0;
		for (int i = 0; i < vNetCount.size(); i++)
		{
			if (vNetCount[i] > nMaxNet)
			{
				nMaxNet = vNetCount[i];
				nIndex = i;
			}
		}

		if (nIndex == -1) // no net can eliminate overflow
		{
			if( nNoOverflow != (int)vOverflowGrid.size() )
				vRipNet.clear();
			else
				bIsOverflow = false;
			break;
		}
		else
		{
			net_C *pRemovedNet = vCanRemovedNet[nIndex];
			for (int i = 0; i < vOverflowGrid.size(); i++)
			{
				if (vOverflowGrid[i]->findNet(pRemovedNet))
				{
					vGridRemand[i]++;
					for (int n = vGNet[i].size() - 1; n >= 0; n--)
					{
						if (vGNet[i][n] == pRemovedNet)
						{
							vGNet[i].erase(vGNet[i].begin() + n);
							break;
						}
					}
				}
			}
			vRipNet.push_back(pRemovedNet);
		}
		vCanRemovedNet.clear();
		vNetCount.clear();
	}
	// end added at 0705 03:30

	removeInstOnGraph(pInst);
	pInst->setPlaced(nOrigX, nOrigY);
	for (int i = 0; i < vNet.size(); i++)
	{
		addNetOnGraph(m_pDesign, vNet[i]);
	}
	//	return true;

	// added at 0705 03:30
	if (bIsOverflow)
		return false;
	else
		return true;
	// end added at 0705 03:30
}
// end added 0705 02:30

bool router_C::isPlaceable(gGrid_C *pGrid, instance_C *pInst)
{
	int nOrigX = pInst->getPlacedX();
	int nOrigY = pInst->getPlacedY();

	bool bIsOverflow = false;
	int nX, nY, nZ;
	pGrid->getPosition(nX, nY, nZ);
	nZ = m_nOffsetZ;
	// routing information

	// added at 0705 02:30
	vector<networkForced_C *> vNF = m_vForced[pInst->getId()].m_vNetwork;
	vector<net_C *> vNet;
	for (int i = 0; i < vNF.size(); i++)
	{
		vNet.push_back(vNF[i]->m_pNet);
	}
	// end added at 0705 02:30

	putInstOnGraph(pInst, nX, nY, 0);

	calNeighborCellDemand(m_pDesign, pGrid);
	int nNumLayer = m_pDesign->getLayer().size();
	vector<int> vPinDemand;
	for (int i = 0; i < nNumLayer; i++)
	{
		vPinDemand.push_back(0);
	}

	vector<pin_C> vPin = pInst->getPin();

	for (int i = 0; i < vPin.size(); i++)
	{
		pin_C *pPin = &vPin[i];
		int nLayerId = pPin->getLayerId();
		vPinDemand[nLayerId - nZ]++;
	}

	gGrid_C *pLGrid = pGrid;
	for (int l = 0; l < nNumLayer; l++)
	{
		// added at 0705 02:30
		int nNetCost = 0;
		for (int i = 0; i < vNet.size(); i++)
		{
			if (pLGrid->findNet(vNet[i]))
				nNetCost++;
		}
		// end added at 0705 02:30
		int nRemand = pLGrid->getRemand() - vPinDemand[l] + nNetCost;
		if (nRemand < 0)
		{
			bIsOverflow = true;
			break;
		}
		pLGrid = graphTravel(m_pDesign, pLGrid, 0, 0, 1);
	}

	if (bIsOverflow)
	{
		removeInstOnGraph(pInst);
		return false;
	}

	gGrid_C *pNGrid;
	pNGrid = graphTravel(m_pDesign, pGrid, 1, 0, 0);
	if (pNGrid != NULL)
	{
		calNeighborCellDemand(m_pDesign, pNGrid);

		pLGrid = pNGrid;
		for (int l = 0; l < nNumLayer; l++)
		{
			int nRemand = pLGrid->getRemand();
			if (nRemand < 0)
			{
				bIsOverflow = true;
				break;
			}
			pLGrid = graphTravel(m_pDesign, pLGrid, 0, 0, 1);
		}

		if (bIsOverflow)
		{
			removeInstOnGraph(pInst);
			return false;
		}
	}

	pNGrid = graphTravel(m_pDesign, pGrid, -1, 0, 0);
	if (pNGrid != NULL)
	{
		calNeighborCellDemand(m_pDesign, pNGrid);

		pLGrid = pNGrid;
		for (int l = 0; l < nNumLayer; l++)
		{
			int nRemand = pLGrid->getRemand();
			if (nRemand < 0)
			{
				bIsOverflow = true;
				break;
			}
			pLGrid = graphTravel(m_pDesign, pLGrid, 0, 0, 1);
		}

		if (bIsOverflow)
		{
			removeInstOnGraph(pInst);
			return false;
		}
	}

	pNGrid = graphTravel(m_pDesign, pGrid, 0, 1, 0);
	if (pNGrid != NULL)
	{
		calNeighborCellDemand(m_pDesign, pNGrid);

		pLGrid = pNGrid;
		for (int l = 0; l < nNumLayer; l++)
		{
			int nRemand = pLGrid->getRemand();
			if (nRemand < 0)
			{
				bIsOverflow = true;
				break;
			}
			pLGrid = graphTravel(m_pDesign, pLGrid, 0, 0, 1);
		}

		if (bIsOverflow)
		{
			removeInstOnGraph(pInst);
			return false;
		}
	}

	pNGrid = graphTravel(m_pDesign, pGrid, 0, -1, 0);
	if (pNGrid != NULL)
	{
		calNeighborCellDemand(m_pDesign, pNGrid);

		pLGrid = pNGrid;
		for (int l = 0; l < nNumLayer; l++)
		{
			int nRemand = pLGrid->getRemand();
			if (nRemand < 0)
			{
				bIsOverflow = true;
				break;
			}
			pLGrid = graphTravel(m_pDesign, pLGrid, 0, 0, 1);
		}

		if (bIsOverflow)
		{
			removeInstOnGraph(pInst);
			return false;
		}
	}

	removeInstOnGraph(pInst);
	pInst->setPlaced(nOrigX, nOrigY);
	return true;
}

bool router_C::checkConnectivity(net_C *pNet)
{
}

bool router_C::fixConnection(net_C *pNet)
{
}

bool router_C::isExchangable(gGrid_C *pGrid, instance_C *pPInst, instance_C *pRInst)
{
	int nPX = pPInst->getPlacedX();
	int nPY = pPInst->getPlacedY();

	if (!pRInst->isMovable())
		return false;

	removeInstOnGraph(pRInst);
	putInstOnGraph(pPInst, pRInst->getPlacedX(), pRInst->getPlacedY(), m_nOffsetZ);
	vector<instance_C *> vGInst = pGrid->getInstance();
	vector<net_C *> vGNet = pGrid->getNet();
	set<net_C *> sNet;
	for (int i = 0; i < vGInst.size(); i++)
	{
		instance_C *pGInst = vGInst[i];
		forced_C &cF = m_vForced[pGInst->getId()];
		vector<networkForced_C *> vNetwork = cF.m_vNetwork;
		for (int n = 0; n < vNetwork.size(); n++)
		{
			sNet.insert(vNetwork[n]->m_pNet);
		}
	}

	vector<int> vNetCost;
	int nDX = 0;
	int nDY = 0;
	int nDZ = 1;
	gGrid_C *pTmpGrid = getGrid(m_pDesign, pRInst->getPlacedX(), pRInst->getPlacedY(), m_nOffsetZ);
	do
	{
		vector<net_C *> vTmpNet = pTmpGrid->getNet();
		int nNumNet = 0;
		for (int n = 0; n < vTmpNet.size(); n++)
		{
			net_C *pNet = vTmpNet[n];
			if (sNet.count(pNet) > 0)
				nNumNet++;
		}
		vNetCost.push_back(nNumNet);
		pTmpGrid = graphTravel(m_pDesign, pTmpGrid, nDX, nDY, nDZ);
	} while (pTmpGrid != NULL);

	vector<pin_C> vPin = pPInst->getPin();
	for (int i = 0; i < vPin.size(); i++)
	{
		int nLayer = vPin[i].getLayerId();
		vNetCost[nLayer - m_nOffsetZ]++;
	}

	bool bIsExchangable = true;
	pTmpGrid = getGrid(m_pDesign, pRInst->getPlacedX(), pRInst->getPlacedY(), m_nOffsetZ);
	for (int i = 0; i < vNetCost.size(); i++)
	{
		int nSupply = pTmpGrid->getSupply();
		int nDemand = pTmpGrid->getDemand();
		int nExtraDemand = pTmpGrid->getExtraDemand();
		int nNetCost = vNetCost[i];
		if (nSupply - (nDemand + nExtraDemand + nNetCost) < 0)
		{
			bIsExchangable = false;
			break;
		}
	}

	removeInstOnGraph(pPInst);
	putInstOnGraph(pRInst, pRInst->getPlacedX(), pRInst->getPlacedY(), m_nOffsetZ);
	return bIsExchangable;
}

bool router_C::globalRoute(vector<net_C *> &vNet)
{
	bool bRouteSuccess = true;
	// calculate the wirelength for length constraint
	vector<int> vLengthConstraint;
	for (int i = 0; i < vNet.size(); i++)
	{
		vLengthConstraint.push_back(calWireLength(vNet[i]));
	}

	// back up and reset the routing nets
	backupNet(vNet);
	ripupNet(vNet);
	cleanWire(vNet);

	//sort the wire length in increasing order
	for (int i = 1; i < vLengthConstraint.size(); i++)
	{
		for (int j = i - 1; j >= 0; j--)
		{
			int nFLength = vLengthConstraint[j];
			int nBLength = vLengthConstraint[j + 1];
			if (nFLength <= nBLength)
				break;
			else
			{
				net_C *pFNet = vNet[j];
				net_C *pBNet = vNet[j + 1];
				vNet[j] = pBNet;
				vNet[j + 1] = pFNet;
				vLengthConstraint[j] = nBLength;
				vLengthConstraint[j + 1] = nFLength;
			}
		}
	}

	// start global route
	int nFailedCount = 0;
	for (int i = 0; i < vNet.size(); i++)
	{
		vector<gGrid_C *> vResult = routeNet_length_constraint(vNet[i], vLengthConstraint[i]);
		if (vResult.size() == 0)
		{
			nFailedCount++;
			bRouteSuccess = false;
		}
		else
		{
			saveNet(vNet[i], vResult);
			//addNetOnGraph( m_pDesign, vNet[i] );
		}
	}

	cout << "Failed Net: " << nFailedCount << endl;
	// check if the routing is success or not
	if (!bRouteSuccess)
	{
		ripupNet(vNet);
		recoverNet(vNet);
		for (int i = 0; i < vNet.size(); i++)
		{
			addNetOnGraph(m_pDesign, vNet[i]);
		}
	}

	if (bRouteSuccess)
		return true;
	else
		return false;
}

bool router_C::reroute( vector< instance_C * > &vInst, vector<net_C *> &vRerouteNet)
{

	for( int i=0; i<vInst.size(); i++ )
	{
		instance_C* pInst = vInst[i];
		vector<networkForced_C *> &vNF = m_vForced[pInst->getId()].m_vNetwork;
		for (int n = 0; n < vNF.size(); n++)
		{
			vRerouteNet.push_back(vNF[n]->m_pNet);
		}
	}
	// remove double exist nets
	for (int n = 0; n < vRerouteNet.size(); n++)
	{
		//for( int m=n+1; m<vRerouteNet.size(); m++ )
		for (int m = vRerouteNet.size() - 1; m >= n + 1; m--)
		{
			if (vRerouteNet[n] == vRerouteNet[m])
			{
				vRerouteNet.erase(vRerouteNet.begin() + m);
				//m--;
			}
		}
	}

	// sort by length
	//vector< int > vTmpLength;
	//for( int i=0; i<vRerouteNet.size(); i++ )
	//{
	//	vTmpLength.push_back( calWireLength( vRerouteNet[i] ) );
	//}
	for (int i = 1; i < vRerouteNet.size(); i++)
	{
		for (int j = i - 1; j >= 0; j--)
		{
			//int nLength_b = calWireLength( vRerouteNet[j+1] );
			//int nLength_f = calWireLength( vRerouteNet[j] );
			//int nLength_b = vTmpLength[j+1];
			//int nLength_f = vTmpLength[j];
			int nLength_b = vRerouteNet[j + 1]->getLength();
			int nLength_f = vRerouteNet[j]->getLength();
			if (nLength_f <= nLength_b)
				break;
			else
			{
				net_C *pNet = vRerouteNet[j + 1];
				vRerouteNet[j + 1] = vRerouteNet[j];
				vRerouteNet[j] = pNet;
				//vTmpLength[j] = nLength_b;
				//vTmpLength[j+1] = nLength_f;
			}
		}
	}

	int nPreviousLength = 0;
	for (int n = 0; n < vRerouteNet.size(); n++)
	{
		//nPreviousLength = nPreviousLength + calWireLength( vRerouteNet[n] );
		//nPreviousLength = nPreviousLength + vTmpLength[n];
		nPreviousLength = nPreviousLength + vRerouteNet[n]->getLength();
	}
	//cout<<"backup net"<<endl;
	backupNet(vRerouteNet);
	//cout<<"ripup net"<<endl;
	ripupNet(vRerouteNet);
	cleanWire(vRerouteNet);
	//cout<<"route the net..."<<endl;

	bool bFail = false;
	int nNetIndex = vRerouteNet.size();
	// change at 0711 20:50
	int nLengthConstraint = nPreviousLength;
	// end change
	for (int r = 0; r < vRerouteNet.size(); r++)
	{
		// change at 0711 20:50
		//vector< gGrid_C* > vResult = routeNet( vRerouteNet[r] );
		//vector< gGrid_C* > vResult = routeNet_length_constraint( vRerouteNet[r], nLengthConstraint );
		vector<vector<gGrid_C *>> vResult = routeNet_length_constraint_ver2(vRerouteNet[r], nLengthConstraint);
		// end change
		if (vResult.size() == 0)
		{
			nNetIndex = r;
			bFail = true;
			break;
		}

		else
		{
			bool bOverflow = false;
		//	int nOX, nOY, nOZ;
			for (int g = 0; g < vResult.size(); g++)
			{
				for (int gg = 0; gg < vResult[g].size(); gg++)
					if (vResult[g][gg]->getRemand() - 1 < 0)
					{
//						vResult[g][gg]->getPosition( nOX, nOY, nOZ );
						bOverflow = true;
						break;
					}
			}
			if (bOverflow)
			{
//				vector< pin_C* > vTmpPin = vRerouteNet[r]->getPin();
//				cout << "Overflow at: " << nOX << " " << nOY << " " << nOZ << endl;
//				cout << "Pin lists: " << endl;
//				for( int p=0; p<vTmpPin.size(); p++ )
//				{
//					int nPX, nPY, nPZ;
					//vTmpPin[p]->getCell()->getPosition( nPX, nPY, nPZ );
					//cout << nPX << " " << nPY << " " << nPZ << endl;
//					cout << ((instance_C*)vTmpPin[p]->getCell())->getPlacedX() << " " << ((instance_C*)vTmpPin[p]->getCell())->getPlacedY() << " " << vTmpPin[p]->getLayerId()+1 << endl;
//				}
//				getchar();
				nNetIndex = r;
				vResult.clear();
				bFail = true;
				break;
			}
		}

		saveNet(vRerouteNet[r], vResult);
		addNetOnGraph(m_pDesign, vRerouteNet[r]);
		// change at 0711 20:50
		//nLengthConstraint = nLengthConstraint - calWireLength( vRerouteNet[r] );
		nLengthConstraint = nLengthConstraint - vRerouteNet[r]->getLength();
		// end change
	}
	//cout<<"Here"<<endl;
	int nNewLength = 0;
	if (!bFail)
	{
		for (int n = 0; n < vRerouteNet.size(); n++)
		{
			//nNewLength = nNewLength + calWireLength( vRerouteNet[n] );
			nNewLength = nNewLength + vRerouteNet[n]->getLength();
		}
		//cout<<"Previous wire length: "<<nPreviousLength<<endl;
		//cout<<"New wire length: "<<nNewLength<<endl;
		//m_nFailed++;
		//cout<<"failed"<<endl;
	}

	if (bFail || nNewLength >= nPreviousLength) // change from ">" to ">=" at 0711 16:30
	{
		//cout<<"Failed to optimize the design, recover."<<endl;
		vector<net_C *> vRoutedNet;

		for (int rr = 0; rr < nNetIndex; rr++)
			vRoutedNet.push_back(vRerouteNet[rr]);

		ripupNet(vRoutedNet);
		recoverNet(vRerouteNet);

		for (int r = 0; r < vRerouteNet.size(); r++)
		{
			addNetOnGraph(m_pDesign, vRerouteNet[r]);
		}
		//m_nFailed++;
		//cout<<"failed"<<endl;
		/*	
		for( int i=0; i<m_vBackupNet.size(); i++ )
		{
			m_vBackupNet[i].cleanWire();
		}
		m_vBackupNet.clear();
		*/
#ifdef _DEBUG_MODE
		nTmpFailedRoute++;
#endif
		return false;
	}
	else
	{

		for (int i = 0; i < vRerouteNet.size(); i++)
			vRerouteNet[i]->addReroute();

		for (int i = 0; i < m_vBackupNet.size(); i++)
		{
			m_vBackupNet[i].cleanWire();
		}
		m_vBackupNet.clear();
		//m_nSuccess++;
		//cout<<"complete"<<endl;
		return true;
	}
}
bool router_C::reroute(instance_C *pInst, vector<net_C *> &vRerouteNet)
{

	if (pInst == m_vForced[pInst->getId()].m_pInstance)
	{
		vector<networkForced_C *> &vNF = m_vForced[pInst->getId()].m_vNetwork;
		for (int n = 0; n < vNF.size(); n++)
		{
			vRerouteNet.push_back(vNF[n]->m_pNet);
		}
	}
	else
	{
		for (int f = 0; f < m_vForced.size(); f++)
		{
			if (pInst == m_vForced[f].m_pInstance)
			{
				vector<networkForced_C *> &vNF = m_vForced[f].m_vNetwork;
				for (int n = 0; n < vNF.size(); n++)
				{
					vRerouteNet.push_back(vNF[n]->m_pNet);
				}
				break;
			}
		}
	}

	// remove double exist nets
	for (int n = 0; n < vRerouteNet.size(); n++)
	{
		//for( int m=n+1; m<vRerouteNet.size(); m++ )
		for (int m = vRerouteNet.size() - 1; m >= n + 1; m--)
		{
			if (vRerouteNet[n] == vRerouteNet[m])
			{
				vRerouteNet.erase(vRerouteNet.begin() + m);
				//m--;
			}
		}
	}

	// sort by length
	//vector< int > vTmpLength;
	//for( int i=0; i<vRerouteNet.size(); i++ )
	//{
	//	vTmpLength.push_back( calWireLength( vRerouteNet[i] ) );
	//}
	for (int i = 1; i < vRerouteNet.size(); i++)
	{
		for (int j = i - 1; j >= 0; j--)
		{
			//int nLength_b = calWireLength( vRerouteNet[j+1] );
			//int nLength_f = calWireLength( vRerouteNet[j] );
			//int nLength_b = vTmpLength[j+1];
			//int nLength_f = vTmpLength[j];
			int nLength_b = vRerouteNet[j + 1]->getLength();
			int nLength_f = vRerouteNet[j]->getLength();
			if (nLength_f <= nLength_b)
				break;
			else
			{
				net_C *pNet = vRerouteNet[j + 1];
				vRerouteNet[j + 1] = vRerouteNet[j];
				vRerouteNet[j] = pNet;
				//vTmpLength[j] = nLength_b;
				//vTmpLength[j+1] = nLength_f;
			}
		}
	}

	int nPreviousLength = 0;
	for (int n = 0; n < vRerouteNet.size(); n++)
	{
		//nPreviousLength = nPreviousLength + calWireLength( vRerouteNet[n] );
		//nPreviousLength = nPreviousLength + vTmpLength[n];
		nPreviousLength = nPreviousLength + vRerouteNet[n]->getLength();
	}
	//cout<<"backup net"<<endl;
	backupNet(vRerouteNet);
	//cout<<"ripup net"<<endl;
	ripupNet(vRerouteNet);
	cleanWire(vRerouteNet);
	//cout<<"route the net..."<<endl;

	bool bFail = false;
	int nNetIndex = vRerouteNet.size();
	// change at 0711 20:50
	int nLengthConstraint = nPreviousLength;
	// end change
	for (int r = 0; r < vRerouteNet.size(); r++)
	{
		// change at 0711 20:50
		//vector< gGrid_C* > vResult = routeNet( vRerouteNet[r] );
		//vector< gGrid_C* > vResult = routeNet_length_constraint( vRerouteNet[r], nLengthConstraint );
		vector<vector<gGrid_C *>> vResult = routeNet_length_constraint_ver2(vRerouteNet[r], nLengthConstraint);
		// end change
		if (vResult.size() == 0)
		{
			nNetIndex = r;
			bFail = true;
			break;
		}

		else
		{
			bool bOverflow = false;
			for (int g = 0; g < vResult.size(); g++)
			{
				for (int gg = 0; gg < vResult[g].size(); gg++)
					if (vResult[g][gg]->getRemand() - 1 < 0)
					{
						bOverflow = true;
						break;
					}
			}
			if (bOverflow)
			{
				nNetIndex = r;
				vResult.clear();
				bFail = true;
				break;
			}
		}

		saveNet(vRerouteNet[r], vResult);
		addNetOnGraph(m_pDesign, vRerouteNet[r]);
		// change at 0711 20:50
		//nLengthConstraint = nLengthConstraint - calWireLength( vRerouteNet[r] );
		nLengthConstraint = nLengthConstraint - vRerouteNet[r]->getLength();
		// end change
	}
	//cout<<"Here"<<endl;
	int nNewLength = 0;
	if (!bFail)
	{
		for (int n = 0; n < vRerouteNet.size(); n++)
		{
			//nNewLength = nNewLength + calWireLength( vRerouteNet[n] );
			nNewLength = nNewLength + vRerouteNet[n]->getLength();
		}
		//cout<<"Previous wire length: "<<nPreviousLength<<endl;
		//cout<<"New wire length: "<<nNewLength<<endl;
		//m_nFailed++;
		//cout<<"failed"<<endl;
	}

	if (bFail || nNewLength >= nPreviousLength) // change from ">" to ">=" at 0711 16:30
	{
		//cout<<"Failed to optimize the design, recover."<<endl;
		vector<net_C *> vRoutedNet;

		for (int rr = 0; rr < nNetIndex; rr++)
			vRoutedNet.push_back(vRerouteNet[rr]);

		ripupNet(vRoutedNet);
		recoverNet(vRerouteNet);

		for (int r = 0; r < vRerouteNet.size(); r++)
		{
			addNetOnGraph(m_pDesign, vRerouteNet[r]);
		}
		m_vBackupNet.clear();
		//m_nFailed++;
		//cout<<"failed"<<endl;
		/*	
		for( int i=0; i<m_vBackupNet.size(); i++ )
		{
			m_vBackupNet[i].cleanWire();
		}
		m_vBackupNet.clear();
		*/
#ifdef _DEBUG_MODE
		nTmpFailedRoute++;
#endif
		return false;
	}
	else
	{
		/*
		if( m_vInstHistory.find( pInst ) == m_vInstHistory.end() )
		{	
			vector< instance_C > vTmpInst;
			vTmpInst.push_back( m_cBackupInstance );
			m_vInstHistory[ pInst ] = vTmpInst;
		}
		else
		{
			m_vInstHistory.find( pInst )->second.push_back( m_cBackupInstance );
		}
		*/
		for (int i = 0; i < vRerouteNet.size(); i++)
			vRerouteNet[i]->addReroute();

		for (int i = 0; i < m_vBackupNet.size(); i++)
		{
			m_vBackupNet[i].cleanWire();
		}
		m_vBackupNet.clear();
		//m_nSuccess++;
		//cout<<"complete"<<endl;
		return true;
	}
}

bool router_C::moveCell( vector< instance_C* > &vInst, boundry_C* pBound )
{
	//cout << "at moveCell"<<endl;
	bool bResult;
	char cBound = pBound->m_cType;
	if( vInst.size() == 1 )
	{
		cout << "at single cell" << endl;
	//	bResult = singleCellMovement_ver3( vInst[0], cBound );
		//bResult = singleCellMovement_ver3( vInst[0], pBound ); // exit bug
		//bResult = multipleCellMovement_ver4( vInst, pBound );
		bResult = multipleCellMovement_ver4_2( vInst, pBound );
		//bResult = multipleCellMovement_ver5( vInst, pBound );
		if( bResult )
			nSglCount++;
		else
			nSglFCount++;
	}
	else
	{
		cout << "at multiple cell" << endl;
		//bResult = multipleCellMovement_ver3( vInst );		
		//bResult = multipleCellMovement_ver4( vInst );		
		//bResult = multipleCellMovement_ver4( vInst, pBound );		
		bResult = multipleCellMovement_ver4_2( vInst, pBound );		
		//bResult = multipleCellMovement_ver5( vInst, pBound );		
	/*	
		for( int i=0; i<vInst.size(); i++ )
		{
			bResult = singleCellMovement_ver3( vInst[i], pBound );
		}
	*/	
		
		if( bResult )
			nMulCount++;
		else
			nMulFCount++;
	}
	//cout << "return moveCell result"<<endl;
	return bResult;
}

bool router_C::iterPlaceInst_ver2( int &nTotalInst, int nNumInst, vector< instance_C* > &vInst, vector< vector< gGrid_C* > > &vBestGrid, vector< net_C* > vRipNet, int nLengthConstraint, vector< instance_C* > vNonMovedInst, vector< int > vIndex, vector< instance_C* > &vSInst, const int nSwapCost )
{
	//cout << nLengthConstraint << endl;
	bool bSuccess = false;
	instance_C* pInst = vInst[ nNumInst- 1 ];
	vector< gGrid_C* > &vTmpBestGrid = vBestGrid[ nNumInst-1 ];
	gGrid_C* pOGrid = getGrid( m_pDesign, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ );	
	set< net_C* > sNet;
	vector< net_C* > vLocalBackupNet;
	instance_C cLocalBackupInst;
	vIndex.push_back( 0 );
	for( int i=0; i<vRipNet.size(); i++ )
		sNet.insert( vRipNet[i] );
	//cout << "Backup net before place: " <<m_vBackupNet.size() << endl;
	for( int b = vTmpBestGrid.size() - 1; b>=0; b-- )
	{
		vIndex.back() = b;
		vector< net_C* > vTmpRip;
		int nX, nY, nZ;
		gGrid_C* pBGrid = vTmpBestGrid[b];
		pBGrid->getPosition( nX, nY, nZ );
		//if( isPlaceable_ver4( pBGrid, pInst, vTmpRip ) )
		if( isPlaceable_ver5( pBGrid, pInst, vTmpRip ) )
		{
			if( vTmpRip.size() != 0 )	
			{
				for( int n=0; n<vTmpRip.size(); n++ )
				{
					if( sNet.count( vTmpRip[n] ) != 0 )
					{
						vTmpRip.erase( vTmpRip.begin() + n );
						n--;
					}
					else
						sNet.insert( vTmpRip[n] );	
				}
				backupNet( vTmpRip );
				//cout << "Local ripup: "<<endl;
				for( int n=0; n<vTmpRip.size(); n++ )
				{
					nLengthConstraint = nLengthConstraint + vTmpRip[n]->getLength();
					ripupNet( vTmpRip[n] );	
					vRipNet.push_back( vTmpRip[n] );
					//cout << vTmpRip[n]->getName() << " " << vTmpRip[n]->getLength() << endl;
				}
				cleanWire( vTmpRip );
			}
			//cout << nNumInst << " " << vTmpRip.size() << endl;
			//cout << "Local ripnet: " << vTmpRip.size() << endl;
			//cout << "Place at: " << nX << ", "<< nY <<", "<< nZ << endl;
			putInstOnGraph( pInst, nX, nY, nZ );
			calPseudoPinDemand( pInst, nX, nY, nZ );
			bool bFindSol;

			if( nTotalInst == nNumInst ) // do routing
			{	
				//cout << "Place result: "<<endl;
				//for( int i=0; i<vInst.size(); i++ )
				//	cout << vInst[i]->getName() << " " << vInst[i]->getPlacedX() << " " << vInst[i]->getPlacedY() << endl;
				for( int i=0; i<vInst.size(); i++ )
					delPseudoPinDemand( vInst[i], vInst[i]->getPlacedX(), vInst[i]->getPlacedY(), m_nOffsetZ );
				
				//cout << "Check instances: "<<endl;
				//for( int i=0; i<vInst.size(); i++ )
				//	cout << vInst[i]->getName() << " (" << vInst[i]->getPlacedX() << " " << vInst[i]->getPlacedY() <<") ";
				//cout << endl;
				//cout << "Check boundary: "<<endl;
				for( int n=0; n<vRipNet.size(); n++ )
				{
					networkForced_C &cNF = m_vNetworkForced[ vRipNet[n]->getId() ];
					//cout << vRipNet[n]->getName () << " " << cNF.m_nMinX << " " << cNF.m_nMinY << " " << cNF.m_nMaxX << " " << cNF.m_nMaxY << " " << vRipNet[n]->getLength()<< endl;
				}
				//cout << endl;

					//bFindSol = multiNetRouting( vRipNet, nLengthConstraint );
					bFindSol = multiNetRouting_ver2( vRipNet, nLengthConstraint, vTmpRip );
					if( !bFindSol )
						for( int i=0; i<vInst.size(); i++ )
						calPseudoPinDemand( vInst[i], vInst[i]->getPlacedX(), vInst[i]->getPlacedY(), m_nOffsetZ );
				//for( int i=0; i<vIndex.size(); i++ )
				//	cout << vIndex[i] << " ";
				//cout << endl;
			}
			else // place next cell
			{
				bFindSol = iterPlaceInst( nTotalInst, nNumInst+1, vInst, vBestGrid, vRipNet, nLengthConstraint, vNonMovedInst, vIndex );
			}
			
			//cout << nNumInst << " " << vTmpRip.size() << endl;

			if( bFindSol )
			{
				bSuccess = true;
				break;
			}
			else
			{
				bSuccess = false;
				delPseudoPinDemand( pInst, nX, nY, nZ );
				removeInstOnGraph( pInst );
				recoverNet( vTmpRip );
				//cout << "Backup Lib: "<<endl;
				//for( int r=0; r<m_vBackupNet.size(); r++ )
				//{
				//	cout << m_vBackupNet[r].getName() << endl;
				//}
				//cout << "Local recover: " << vTmpRip.size() << endl;
				for( int r=0; r<vTmpRip.size(); r++ )
				{
					//cout << vTmpRip[r]->getName() << " " << vTmpRip[r]->getLength() << endl;
					sNet.erase( vTmpRip[r] );
					//cout << "erase on data base"<<endl;
					nLengthConstraint = nLengthConstraint - vTmpRip[r]->getLength();
					//cout << "put net on graph" << endl;
					addNetOnGraph( m_pDesign, vTmpRip[r] );
					//cout << vTmpRip[r]->getName() << " " << vTmpRip[r]->getLength() << endl;
				}
				
				for( int r=0; r<vTmpRip.size(); r++ )
				{
					m_vBackupNet.erase( m_vBackupNet.end() - 1 );
					vRipNet.erase( vRipNet.end() -1 );
				}
				//cout << "Backup net after failed: " <<m_vBackupNet.size() << endl;
			}
		}
		else //swap 
		{
			//cout << "Swap Instance"<<endl;
			vector< instance_C* > vSwapInstance = findSwapInstance( pBGrid, pOGrid, pInst );
			set< instance_C* > sSInst;
			for( int i=0; i<vInst.size(); i++ )
				sSInst.insert( vInst[i] );	
			
			for( int i=0; i<vSInst.size(); i++ )
				sSInst.insert( vSInst[i] );	
			
			for( int s=0; s < vSwapInstance.size(); s++ )
			{
				instance_C* pSInst = vSwapInstance[s];
				if( sSInst.count( pSInst ) != 0 )
					continue;
				int nOX, nOY, nOZ;
				pOGrid->getPosition( nOX, nOY, nOZ );
				//cout << "Swap: "<< pSInst->getName() << " from (" << pSInst->getPlacedX() << " " << pSInst->getPlacedY() << ") to (" << nOX << " " << nOY << ")" << endl;
				cLocalBackupInst.setPlaced( pSInst->getPlacedX(), pSInst->getPlacedY() );
				removeInstOnGraph( pSInst );
				int nNewSwapCost = nSwapCost;
				if( !pSInst->hasBeenMoved() )
					nNewSwapCost--;

				if( swapInstance( pBGrid, pOGrid, pInst, pSInst, vTmpRip ) && nNewSwapCost >= 0 )
				{
					//cout << "Able to swap"<<endl;
					if( vTmpRip.size() != 0 )	
					{
						for( int n=0; n<vTmpRip.size(); n++ )
						{
							if( sNet.count( vTmpRip[n] ) != 0 )
							{
								vTmpRip.erase( vTmpRip.begin() + n );
								n--;
							}
							else
								sNet.insert( vTmpRip[n] );	
						}
						backupNet( vTmpRip );
						//cout << "Local ripup: "<<endl;
						for( int n=0; n<vTmpRip.size(); n++ )
						{
							nLengthConstraint = nLengthConstraint + vTmpRip[n]->getLength();
							ripupNet( vTmpRip[n] );	
							vRipNet.push_back( vTmpRip[n] );
							//cout << vTmpRip[n]->getName() << " " << vTmpRip[n]->getLength() << endl;
						}
						cleanWire( vTmpRip );
					}
					int nTX, nTY, nTZ;
					pBGrid->getPosition( nTX, nTY, nTZ );
					putInstOnGraph( pInst, nTX, nTY, nTZ );
					calPseudoPinDemand( pInst, nTX, nTY, nTZ );
					pOGrid->getPosition( nTX, nTY, nTZ );
					putInstOnGraph( pSInst, nTX, nTY, nTZ );
					calPseudoPinDemand( pSInst, nTX, nTY, nTZ );
					vSInst.push_back( pSInst );	

					bool bFindSol;
					if( nTotalInst == nNumInst )
					{
						for( int i=0; i<vInst.size(); i++ )
							delPseudoPinDemand( vInst[i], vInst[i]->getPlacedX(), vInst[i]->getPlacedY(), m_nOffsetZ );
					
						for( int i=0; i<vSInst.size(); i++ )
							delPseudoPinDemand( vSInst[i], vSInst[i]->getPlacedX(), vSInst[i]->getPlacedY(), m_nOffsetZ );

						//cout << "Check instances: "<<endl;
						//for( int i=0; i<vInst.size(); i++ )
						//	cout << vInst[i]->getName() << " (" << vInst[i]->getPlacedX() << " " << vInst[i]->getPlacedY() <<") ";
						for( int n=0; n<vRipNet.size(); n++ )
						{
							networkForced_C &cNF = m_vNetworkForced[ vRipNet[n]->getId() ];
						}
						//cout << endl;

							//bFindSol = multiNetRouting( vRipNet, nLengthConstraint );
							bFindSol = multiNetRouting_ver2( vRipNet, nLengthConstraint, vTmpRip );
							if( !bFindSol )
							{
								for( int i=0; i<vInst.size(); i++ )
									calPseudoPinDemand( vInst[i], vInst[i]->getPlacedX(), vInst[i]->getPlacedY(), m_nOffsetZ );
								for( int i=0; i<vSInst.size(); i++ )
									calPseudoPinDemand( vSInst[i], vSInst[i]->getPlacedX(), vSInst[i]->getPlacedY(), m_nOffsetZ );
								
							}
					}
					else
					{
						bFindSol = iterPlaceInst_ver2( nTotalInst, nNumInst+1, vInst, vBestGrid, vRipNet, nLengthConstraint, vNonMovedInst, vIndex, vSInst, nNewSwapCost );
					
					}

					if( bFindSol )
					{
						m_vBackupInstance.push_back( cLocalBackupInst );	
						bSuccess = true;
						break;
					}
					else
					{
						//cout << "Remove at here "<< endl;
						removeInstOnGraph( pInst );
						removeInstOnGraph( pSInst );
						//cout << "Remove at here end " << pSInst->getName() << " " << pSInst->getPlacedX() << " " << pSInst->getPlacedY() << endl;
						pSInst->setPlaced( cLocalBackupInst.getPlacedX(), cLocalBackupInst.getPlacedY() );
						
						putInstOnGraph( pSInst, pSInst->getPlacedX(), pSInst->getPlacedY(), m_nOffsetZ );
						vSInst.erase( vSInst.end() - 1 );
						recoverNet( vTmpRip );
						for( int r=0; r<vTmpRip.size(); r++ )
						{
							//cout << vTmpRip[r]->getName() << " " << vTmpRip[r]->getLength() << endl;
							sNet.erase( vTmpRip[r] );
							//cout << "erase on data base"<<endl;
							nLengthConstraint = nLengthConstraint - vTmpRip[r]->getLength();
							//cout << "put net on graph" << endl;
							addNetOnGraph( m_pDesign, vTmpRip[r] );
							//cout << vTmpRip[r]->getName() << " " << vTmpRip[r]->getLength() << endl;
						}
						
						for( int r=0; r<vTmpRip.size(); r++ )
						{
							m_vBackupNet.erase( m_vBackupNet.end() - 1 );
							vRipNet.erase( vRipNet.end() -1 );
						}
					}
				}
				else
				{
					pSInst->setPlaced( cLocalBackupInst.getPlacedX(), cLocalBackupInst.getPlacedY() );
					putInstOnGraph( pSInst, pSInst->getPlacedX(), pSInst->getPlacedY(), m_nOffsetZ );
					
				}
			}
			if( bSuccess )
				break;
		}
	}

	return bSuccess;
}
	
bool router_C::iterPlaceInst( int &nTotalInst, int nNumInst, vector< instance_C* > &vInst, vector< vector< gGrid_C* > > &vBestGrid, vector< net_C* > vRipNet, int nLengthConstraint, vector< instance_C* > vNonMovedInst, vector< int > vIndex )
{
	//cout << nLengthConstraint << endl;
	bool bSuccess = false;
	instance_C* pInst = vInst[ nNumInst- 1 ];
	vector< gGrid_C* > &vTmpBestGrid = vBestGrid[ nNumInst-1 ];
	gGrid_C* pOGrid = getGrid( m_pDesign, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ );	
	set< net_C* > sNet;
	vector< net_C* > vLocalBackupNet;
	vIndex.push_back( 0 );
	for( int i=0; i<vRipNet.size(); i++ )
		sNet.insert( vRipNet[i] );
	//cout << "Backup net before place: " <<m_vBackupNet.size() << endl;
	for( int b = vTmpBestGrid.size() - 1; b>=0; b-- )
	{
		vIndex.back() = b;
		vector< net_C* > vTmpRip;
		int nX, nY, nZ;
		gGrid_C* pBGrid = vTmpBestGrid[b];
		pBGrid->getPosition( nX, nY, nZ );
		//if( isPlaceable_ver4( pBGrid, pInst, vTmpRip ) )
		if( isPlaceable_ver5( pBGrid, pInst, vTmpRip ) )
		{
			if( vTmpRip.size() != 0 )	
			{
				for( int n=0; n<vTmpRip.size(); n++ )
				{
					if( sNet.count( vTmpRip[n] ) != 0 )
					{
						vTmpRip.erase( vTmpRip.begin() + n );
						n--;
					}
					else
						sNet.insert( vTmpRip[n] );	
				}
				backupNet( vTmpRip );
				//cout << "Local ripup: "<<endl;
				for( int n=0; n<vTmpRip.size(); n++ )
				{
					nLengthConstraint = nLengthConstraint + vTmpRip[n]->getLength();
					ripupNet( vTmpRip[n] );	
					vRipNet.push_back( vTmpRip[n] );
					//cout << vTmpRip[n]->getName() << " " << vTmpRip[n]->getLength() << endl;
				}
				cleanWire( vTmpRip );
			}
			//cout << nNumInst << " " << vTmpRip.size() << endl;
			//cout << "Local ripnet: " << vTmpRip.size() << endl;
			//cout << "Place at: " << nX << ", "<< nY <<", "<< nZ << endl;
			putInstOnGraph( pInst, nX, nY, nZ );
			calPseudoPinDemand( pInst, nX, nY, nZ );
			bool bFindSol;

			if( nTotalInst == nNumInst ) // do routing
			{	
				for( int i=0; i<vInst.size(); i++ )
					delPseudoPinDemand( vInst[i], vInst[i]->getPlacedX(), vInst[i]->getPlacedY(), m_nOffsetZ );
				
				//cout << "Check boundary: "<<endl;
				for( int n=0; n<vRipNet.size(); n++ )
				{
					networkForced_C &cNF = m_vNetworkForced[ vRipNet[n]->getId() ];
					//cout << vRipNet[n]->getName () << " " << cNF.m_nMinX << " " << cNF.m_nMinY << " " << cNF.m_nMaxX << " " << cNF.m_nMaxY << " " << vRipNet[n]->getLength()<< endl;
				}
				//cout << endl;

					//bFindSol = multiNetRouting( vRipNet, nLengthConstraint );
					bFindSol = multiNetRouting_ver2( vRipNet, nLengthConstraint, vTmpRip );
					if( !bFindSol )
						for( int i=0; i<vInst.size(); i++ )
						calPseudoPinDemand( vInst[i], vInst[i]->getPlacedX(), vInst[i]->getPlacedY(), m_nOffsetZ );
			}
			else // place next cell
			{
				bFindSol = iterPlaceInst( nTotalInst, nNumInst+1, vInst, vBestGrid, vRipNet, nLengthConstraint, vNonMovedInst, vIndex );
			}
			
			//cout << nNumInst << " " << vTmpRip.size() << endl;

			if( bFindSol )
			{
				bSuccess = true;
				break;
			}
			else
			{
				bSuccess = false;
				delPseudoPinDemand( pInst, nX, nY, nZ );
				removeInstOnGraph( pInst );
				recoverNet( vTmpRip );
				//cout << "Backup Lib: "<<endl;
				//for( int r=0; r<m_vBackupNet.size(); r++ )
				//{
				//	cout << m_vBackupNet[r].getName() << endl;
				//}
				//cout << "Local recover: " << vTmpRip.size() << endl;
				for( int r=0; r<vTmpRip.size(); r++ )
				{
					//cout << vTmpRip[r]->getName() << " " << vTmpRip[r]->getLength() << endl;
					sNet.erase( vTmpRip[r] );
					//cout << "erase on data base"<<endl;
					nLengthConstraint = nLengthConstraint - vTmpRip[r]->getLength();
					//cout << "put net on graph" << endl;
					addNetOnGraph( m_pDesign, vTmpRip[r] );
					//cout << vTmpRip[r]->getName() << " " << vTmpRip[r]->getLength() << endl;
				}
				
				for( int r=0; r<vTmpRip.size(); r++ )
				{
					m_vBackupNet.erase( m_vBackupNet.end() - 1 );
					vRipNet.erase( vRipNet.end() -1 );
				}
				//cout << "Backup net after failed: " <<m_vBackupNet.size() << endl;
			}
		}
	}

	return bSuccess;
}

bool router_C::multipleCellMovement_ver5( vector< instance_C* > &vInst, boundry_C* pBound )
{	
	cout << "at multipleCellMovement"<<endl;
	char cBound = pBound->m_cType;
	int nMoveCount = vInst.size();
	set< instance_C* > sInst;	
	vector< instance_C* > vIInst = vInst;
	for( int i=0; i<vIInst.size(); i++ )
	{
		sInst.insert( vIInst[i] );
		//vInst[ nMoveCount - 1 - i ] = vIInst[i];	
	}
	
	vector< net_C* > vRipNet;
	set< net_C* > sNet;
	set< instance_C* > sUnplacedInst;
	//cout << "Collect the ripup nets"<<endl;
	for( int i=0; i<vInst.size(); i++ )
	{
		instance_C* pInst = vInst[i];
		if( pInst->hasBeenMoved() )
			nMoveCount--;
		vector< networkForced_C* > &vTmpNF = m_vForced[ pInst->getId() ].m_vNetwork;
		for( int j=0; j<vTmpNF.size(); j++ )
		{
			if( sNet.count( vTmpNF[j]->m_pNet ) == 0 )
			{
				sNet.insert( vTmpNF[j]->m_pNet );
				vRipNet.push_back( vTmpNF[j]->m_pNet );
			}
		}
		sUnplacedInst.insert( pInst );
	}
	//cout << vRipNet.size() << endl;
	//cout << "Calculating wire length for constraint"<<endl;
	int nPreviousLength = 0;
	for( int n=0; n<vRipNet.size(); n++ )
	{
		nPreviousLength = nPreviousLength + vRipNet[n]->getLength();
	}
	
	//cout << "Backup instance and routing information"<<endl;
	backupInstance( vInst );
	backupNet( vRipNet );
	//cout << vRipNet.size() << endl;

	//cout << "Remove instance on the graph"<<endl;
	for( int i=0; i<vInst.size(); i++ )
	{
		removeInstOnGraph( vInst[i] );
		delPseudoPinDemand( vInst[i], vInst[i]->getPlacedX(), vInst[i]->getPlacedY(), m_nOffsetZ );
	}
	
	vector< gGrid_C* > vPseudoPinGrid;
	
	//cout << "Remove routing nets on the graph"<<endl;
	for( int i=0; i<vRipNet.size(); i++ )
	{
		vector< gGrid_C* >  vTmpGrid = addPseudoPinDemand( vRipNet[i], sUnplacedInst );
		//cout << "Rip " << vRipNet[i]->getName()<<endl;
		ripupNet( vRipNet[i] );	
		for( int g=0; g<vTmpGrid.size(); g++ )
			vPseudoPinGrid.push_back( vTmpGrid[g] );
	}
	//cout << "Clean up wires"<<endl;
	cleanWire( vRipNet );


	int nNumInstance = vInst.size();
	vector< instance_C* > vNonMovedInst;
	//cout << "Collect the best placement candidates"<<endl;
	bool bSuccess = true;
	vector< vector< gGrid_C* > > vBestGrid;
	for( int i=0; i<vInst.size(); i++ )
	{
		instance_C* pInst = vInst[i];
		//vector< gGrid_C* > vTmpBestGrid = findPlaceToMove_ver3( pInst, pBound );
		vector< gGrid_C* > vTmpBestGrid = findPlaceToMove_ver4( pInst, pBound );
		if( vTmpBestGrid.size() == 0 )
		{
			bSuccess = false;
			break;
		}
		else
		{
			vBestGrid.push_back( vTmpBestGrid );
		}
	}

	int nSwapCost = (int)m_pDesign->getMaxCellConstraint() - m_vMovedInstance.size() - nMoveCount;
	vector< instance_C* > vSInst;
	if( bSuccess )
	{
		//cout << "Start place instance: "<<endl;
		vector< int > vIndex;
		//bSuccess = iterPlaceInst( nNumInstance, 1, vInst, vBestGrid, vRipNet, nPreviousLength, vNonMovedInst, vIndex );
		bSuccess = iterPlaceInst_ver2( nNumInstance, 1, vInst, vBestGrid, vRipNet, nPreviousLength, vNonMovedInst, vIndex, vSInst, nSwapCost );
		cout << "Num Swap Instance " << vSInst.size() << endl;
	}
	for( int i=0; i<vNonMovedInst.size(); i++ )
	{
		resetPseudoPinDemand( vNonMovedInst[i], vNonMovedInst[i]->getPlacedX(), vNonMovedInst[i]->getPlacedY(), m_nOffsetZ );
	}

	if( !bSuccess ) // recover all the information
	{
		//cout << "recover information" << endl;
		for( int s=vSInst.size()-1; s>=0; s-- )
			vInst.push_back( vSInst[s] );

		recoverInstance( vInst );
		for( int i=0; i<vInst.size(); i++ )
		{
			instance_C* pInst = vInst[i];
			
			putInstOnGraph( pInst, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ );
			resetPseudoPinDemand( pInst, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ );
			//if( pInst->getName() == "C1" )
			//	cout << "C1: "<<pInst->getPlacedX() <<", "<<pInst->getPlacedY() << endl;
			//calForcedModel_ver3( m_vForced[ pInst->getId() ] );
			calForcedModel_ver4( m_vForced[ pInst->getId() ] );
		}
		recoverNet( vRipNet );
		//cout << "recover " << vRipNet.size() << endl;
		for( int i=0; i<vRipNet.size(); i++ )
		{
		//	cout << vRipNet[i]->getName() << " " << vRipNet[i]->getLength() << endl;
			addNetOnGraph( m_pDesign, vRipNet[i] );	
		}
		for( int i=0; i<vInst.size(); i++ )
		{
			updateForcedModel_ver2( vInst[i] );	
		}
	}
	else
	{
		for( int s=vSInst.size()-1; s>=0; s-- )
			vInst.push_back( vSInst[s] );
		
		for( int i=0; i<vInst.size(); i++ )
			cout << vInst[i]->getName() << " (" <<vInst[i]->getPlacedX() << " " << vInst[i]->getPlacedY() << ") ";
		cout << endl;
		cout << "update model " << m_vBackupInstance.size() << endl;
		for( int i=0; i<vInst.size(); i++ )
		{
			instance_C* pInst = vInst[i];
			if ( !pInst->hasBeenMoved() && !( pInst->getPlacedX() == m_vBackupInstance[i].getPlacedX() && pInst->getPlacedY() == m_vBackupInstance[i].getPlacedY() ) )
			//if ( !pInst->hasBeenMoved() )
			{
				pInst->hasMoved();
				m_vMovedInstance.push_back(pInst);
			}
		
			//m_nSuccess++;
			updateForcedModel_ver2(pInst);
			freeBoundry( pInst );
			freeForcedModel(pInst);
		}
		for( int i=0; i<m_vBackupInstance.size(); i++ )
		{
			instance_C* pInst = &m_vBackupInstance[i];
			resetPseudoPinDemand( pInst, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ );	
		}
		
		for( int i=0; i<m_vBackupNet.size(); i++ )
		{
			m_vBackupNet[i].cleanWire();
		}
	}

	for( int i=0; i<vPseudoPinGrid.size(); i++ )
	{
		vPseudoPinGrid[i]->resetPinDemand();
	}
	m_vBackupNet.clear();
	m_vBackupInstance.clear();	

	cout << "return  multipleCellMovement result"<<endl;
	return bSuccess;
}


bool router_C::multipleCellMovement_ver4_2( vector< instance_C* > &vInst, boundry_C* pBound )
{	
	cout << "at multipleCellMovement"<<endl;
	char cBound = pBound->m_cType;
	int nMoveCount = vInst.size();
	set< instance_C* > sInst;	
	vector< instance_C* > vIInst = vInst;
	for( int i=0; i<vIInst.size(); i++ )
	{
		sInst.insert( vIInst[i] );
		vInst[ nMoveCount - 1 - i ] = vIInst[i];	
	}
	
	vector< net_C* > vRipNet;
	set< net_C* > sNet;
	set< net_C* > sRipNet;
	set< instance_C* > sUnplacedInst;
	vector< gGrid_C* > vPseudoPinGrid;
	for( int i=0; i<vInst.size(); i++ )
	{
		instance_C* pInst = vInst[i];
		if( pInst->hasBeenMoved() )
			nMoveCount--;
		vector< networkForced_C* > &vTmpNF = m_vForced[ pInst->getId() ].m_vNetwork;
		for( int j=0; j<vTmpNF.size(); j++ )
		{
			if( sNet.count( vTmpNF[j]->m_pNet ) == 0 )
			{
				sNet.insert( vTmpNF[j]->m_pNet );
				vRipNet.push_back( vTmpNF[j]->m_pNet );
			}
		}
		sUnplacedInst.insert( pInst );
	}
	//cout << vRipNet.size() << endl;
	vector< instance_C* > vNonMovedInst;
	
	//cout << "Check 16 3 1: " << getGrid( m_pDesign, 16, 3, 1)->getRemand() << " " << getGrid( m_pDesign, 16, 3, 1 )->getPinDemand() << endl;
	for( int i=0; i<vRipNet.size(); i++ )
	{
		vector< pin_C* > vPin = vRipNet[i]->getPin();
		for( int j=0; j<vPin.size(); j++ )
		{
			pin_C* pPin = vPin[j];
			instance_C* pInst = ( instance_C* )pPin->getCell();
			if( sUnplacedInst.count( pInst ) != 0 )
			{
				continue;
			}
			int nZ = pPin->getLayerId();
			int nX = pInst->getPlacedX();
			int nY = pInst->getPlacedY();
			
			gGrid_C* pGrid = getGrid( m_pDesign, nX, nY, nZ );
			pGrid->addPinDemand( vRipNet[i] );
			if( nX == 16 && nY == 3 )
				cout << pInst->getName() << " " << vRipNet[i]->getName() << endl;		
			int nLayerConstraint = max( nZ, vRipNet[i]->getConstraintLayerId() );
			for( int l=nZ+1; l<=nLayerConstraint; l++ )
			{
				pGrid = getGrid( m_pDesign, nX, nY, l );
				pGrid->addPinDemand( vRipNet[i] );
				vPseudoPinGrid.push_back( pGrid );
			}
			vNonMovedInst.push_back( pInst );
		}
	}
	
	int nSwapCost = m_pDesign->getMaxCellConstraint() - m_vMovedInstance.size() - nMoveCount;

	backupInstance( vInst );
	backupNet( vRipNet );
	//cout << vRipNet.size() << endl;
	//cout << "Check 16 3 1: " << getGrid( m_pDesign, 16, 3, 1)->getRemand() << " " << getGrid( m_pDesign, 16, 3, 1 )->getPinDemand() << endl;

	for( int i=0; i<vInst.size(); i++ )
	{
		removeInstOnGraph( vInst[i] );
	}
	//cout << "Check 16 3 1: " << getGrid( m_pDesign, 16, 3, 1)->getRemand() << " " << getGrid( m_pDesign, 16, 3, 1 )->getPinDemand() << endl;
	cout << "Origin Rip net: "<<endl;
	for( int i=0; i<vRipNet.size(); i++ )
	{
		cout << vRipNet[i]->getName() << endl;
		ripupNet( vRipNet[i] );
		sRipNet.insert( vRipNet[i] );
	}
	cleanWire( vRipNet );
	//cout << "Check 16 3 1: " << getGrid( m_pDesign, 16, 3, 1)->getRemand() << " " << getGrid( m_pDesign, 16, 3, 1 )->getPinDemand() << endl;

	bool bFindAllPlace = true;
	vector< instance_C* > vTmpSwap;
	//vector< instance_C > vSwapBackup;
	vector< instance_C* > vPlacedInst;

	//cout << "place instance"<<endl;
	// place the instance
	int nNumInstance = vInst.size();
	//cout << "Move Cell"<<endl;
	vector< instance_C* > vRecordPse;
	vector< int > vBestRecord; 
	int nMaxBest = 0;
	cout << "X Min: "<< pBound->m_pNetwork->m_nMinX << " " << pBound->m_pNetwork->m_n2MinX << endl;
	cout << "X Max: "<< pBound->m_pNetwork->m_nMaxX << " " << pBound->m_pNetwork->m_n2MaxX << endl;
	cout << "Y Min: "<< pBound->m_pNetwork->m_nMinY << " " << pBound->m_pNetwork->m_n2MinY << endl;
	cout << "Y Max: "<< pBound->m_pNetwork->m_nMaxY << " " << pBound->m_pNetwork->m_n2MaxY << endl;
	for( int i=0; i<nNumInstance; i++ )
	{
		instance_C* pInst = vInst[i];
		//cout << "Place "<<pInst->getName() << " ";
		gGrid_C* pOGrid = getGrid( m_pDesign, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ );	
			
		//cout << "Find"<<endl;
		//vector< gGrid_C* > vBestGrid = findPlaceToMove_ver4( pInst );		
		//vector< gGrid_C* > vBestGrid = findPlaceToMove_ver3( pInst );		
		//vector< gGrid_C* > vBestGrid = findPlaceToMove_ver3( pInst, pBound );		
		vector< gGrid_C* > vBestGrid = findPlaceToMove_ver4( pInst, pBound, sUnplacedInst );		
			
		cout << "Choice "<< vBestGrid.size() << endl;
		for( int i=vBestGrid.size()-1; i>=0; i-- )
		{
			int nX, nY, nZ;
			vBestGrid[i]->getPosition( nX, nY, nZ );
			cout <<"( " << nX << " " << nY << " )";
		}
		cout << endl;
		
		gGrid_C* pBGrid = NULL;
		if( vBestGrid.size() == 0 )
		{
			//lockInstance( pInst );
			for( int j=0; j<i; j++ )
			{
				resetPseudoPinDemand( vInst[j], vInst[j]->getPlacedX(), vInst[j]->getPlacedY(), m_nOffsetZ );
				removeInstOnGraph( vInst[j] );
			}
			for( int j=0; j<vTmpSwap.size(); j++ )
			{
				resetPseudoPinDemand( vTmpSwap[j], vTmpSwap[j]->getPlacedX(), vTmpSwap[j]->getPlacedY(), m_nOffsetZ );
				removeInstOnGraph( vTmpSwap[j] );
			}
			/*
			for( int n=0; n<vLocalNet.size(); n++ )
				addNetOnGraph( m_pDesign, vLocalNet[n] );
			*/
			for( int n=0; n<vRipNet.size(); n++ )
			{
				if( sRipNet.count( vRipNet[n] ) != 0 )
				{
					addNetOnGraph( m_pDesign, vRipNet[n] );
				}
			}
			bFindAllPlace = false;
			break;
		}
		else
		{
			bool bPlace = false;
			vector< net_C *> vTmpRip;
			for( int b=vBestGrid.size() - 1; b>=0; b-- )
			{
				int nX, nY, nZ;
				pBGrid = vBestGrid[b];
				pBGrid->getPosition( nX, nY, nZ );
				//if( isPlaceable( pBGrid, pInst ) )
				//if( isPlaceable_ver4( pBGrid, pInst, vTmpRip ) )
				if( isPlaceable_ver5( pBGrid, pInst, vTmpRip ) )
				{
//
					vBestRecord.push_back(b);
//					
					if( vTmpRip.size() != 0 )	
					{
						for( int n=0; n<vTmpRip.size(); n++ )
						{
							if( sRipNet.count( vTmpRip[n] ) != 0 )
							{
								vTmpRip.erase( vTmpRip.begin() + n );
								n--;
							}
							else
								sRipNet.insert( vTmpRip[n] );	
						}
						cout << "Local Rip "<<endl;
						backupNet( vTmpRip );
						for( int n=0; n<vTmpRip.size(); n++ )
						{
							cout << vTmpRip[n]->getName()<<endl;
							ripupNet( vTmpRip[n] );	
							vRipNet.push_back( vTmpRip[n] );
						}
						cleanWire( vTmpRip );
					}

					//cout << "Place at: " << nX << ", "<< nY <<", "<< nZ << endl;
					putInstOnGraph( pInst, nX, nY, nZ );
					calPseudoPinDemand( pInst, nX, nY, nZ );
					bPlace = true;
					sUnplacedInst.erase( pInst );
					break;
				}
					
				else if( pBGrid != pOGrid && nSwapCost > 0 )
				{
					vector<instance_C *> vSwapInstance = findSwapInstance_ver2(pBGrid, pOGrid, pInst);
					cout << "Swap Instance: ";
					for( int s=0; s<vSwapInstance.size(); s++ )
						cout << vSwapInstance[s]->getName() << " ";
					cout << endl;
					for (int s = 0; s < vSwapInstance.size(); s++)
					{
						bool bHasSwap = false;
						for( int t=0; t<vTmpSwap.size(); t++ )
						{
							if( vTmpSwap[t] == vSwapInstance[s] )
							{
								bHasSwap = true;
								break;
							}		
						}

						for( int t=0; t<vInst.size(); t++ )
						{
							if( vInst[t] == vSwapInstance[s] )
							{
								bHasSwap = true;
								break;
							}
						}
						if( bHasSwap )
							continue;

						vTmpRip.clear();
						instance_C *pSInst = vSwapInstance[s];
						instance_C cLocalBackupInstance;
						// local backup pSInst
						cLocalBackupInstance.setName( pSInst->getName() );
						cLocalBackupInstance.setPlaced(pSInst->getPlacedX(), pSInst->getPlacedY());
						removeInstOnGraph(pSInst);
						if( swapInstance_ver3(pBGrid, pOGrid, pInst, pSInst, vTmpRip, vRipNet ) )
						//if( swapInstance_ver2(pBGrid, pOGrid, pInst, pSInst, vTmpRip ) )
						{
							//cout << "Swap: "<<pSInst->getName() << " at " << cLocalBackupInstance.getPlacedX() <<", " << cLocalBackupInstance.getPlacedY() << endl;
							cout << "Swap Rip Net: ";
							for( int ss=0; ss<vTmpRip.size(); ss++ )
							{
								cout << vTmpRip[ss]->getName() << " ";
							}
							cout << endl;
							vBestRecord.push_back(b);
//							
							if( vTmpRip.size() != 0 )
							{
								for( int n=0; n<vTmpRip.size(); n++ )
								{
									if( sRipNet.count( vTmpRip[n] ) != 0 )
									{
										vTmpRip.erase( vTmpRip.begin() + n );
										n--;
									}
									else
										sRipNet.insert( vTmpRip[n] );	
								}
								
								backupNet( vTmpRip );
								for( int n=0; n<vTmpRip.size(); n++ )
								{
									ripupNet( vTmpRip[n] );	
									vRipNet.push_back( vTmpRip[n] );
								}
								cleanWire( vTmpRip );
								
							}

							int nTmpX, nTmpY, nTmpZ;
							pBGrid->getPosition(nTmpX, nTmpY, nTmpZ);
							putInstOnGraph(pInst, nTmpX, nTmpY, nTmpZ);
							calPseudoPinDemand( pInst, nTmpX, nTmpY, nTmpZ );
						
							sInst.insert( pSInst );
							pOGrid->getPosition(nTmpX, nTmpY, nTmpZ);
							putInstOnGraph(pSInst, nTmpX, nTmpY, nTmpZ);
							calPseudoPinDemand( pSInst, nTmpX, nTmpY, nTmpZ );
							for( int l=0; l<vTmpRip.size(); l++ )
							{
								vector< pin_C* > vTmpPin = vTmpRip[l]->getPin();
								for( int p=0; p<vTmpPin.size(); p++ )
								{
									instance_C* pPInst = ( instance_C* )vTmpPin[p]->getCell();
									if( sInst.count( pPInst ) == 0 )
									{
										int nPX = pPInst->getPlacedX();
										int nPY = pPInst->getPlacedY();
										int nPZ = vTmpPin[p]->getLayerId();
										gGrid_C* pPGrid = getGrid( m_pDesign, nPX, nPY, nPZ );
										pPGrid->addPinDemand( vTmpPin[p]->getNet() );
										for( int z=nPZ + 1; z<= vTmpRip[l]->getConstraintLayerId(); z++ )
										{	
											pPGrid = getGrid( m_pDesign, nPX, nPY, nPZ );
											pPGrid->addPinDemand( vTmpRip[l] );
										}
										vNonMovedInst.push_back( pPInst );
									}
								}
							}
							
							vTmpSwap.push_back( pSInst );
							vInst.push_back( pSInst );
							m_vBackupInstance.push_back( cLocalBackupInstance );
							bPlace = true;
							nSwapCost--;
							cout << "Place at: " << nX << ", "<< nY <<", "<< nZ << endl;
							cout << "Swap from: " << pSInst->getPlacedX() << ", "<< pSInst->getPlacedY() <<", "<< nZ << endl;
							break;
						}
						else
						{
							// recover instance, put instance back on the graph
							int nTmpX, nTmpY, nTmpZ;
							pBGrid->getPosition(nTmpX, nTmpY, nTmpZ);
							putInstOnGraph(pSInst, nTmpX, nTmpY, nTmpZ);
							pOGrid->getPosition( nTmpX, nTmpY, nTmpZ );
							pInst->setPlaced(nTmpX, nTmpY);
						}
					}
					if( bPlace )
						break;	
				}
					
				
			}
			if( !bPlace )
			{
				//lockInstance( pInst );
				for( int j=0; j<i; j++ )
				{
					resetPseudoPinDemand( vInst[j], vInst[j]->getPlacedX(), vInst[j]->getPlacedY(), m_nOffsetZ );
					removeInstOnGraph( vInst[j] );
				}
				for( int j=0; j<vTmpSwap.size(); j++ )
				{
					resetPseudoPinDemand( vTmpSwap[j], vTmpSwap[j]->getPlacedX(), vTmpSwap[j]->getPlacedY(), m_nOffsetZ );
					removeInstOnGraph( vTmpSwap[j] );
				}
				bFindAllPlace = false;
				/*	
				for( int n=0; n<vLocalNet.size(); n++ )
					addNetOnGraph( m_pDesign, vLocalNet[n] );
				*/
				/*
				for( int n=0; n<vRipNet.size(); n++ )
				{
					addNetOnGraph( m_pDesign, vRipNet[n] );
				}
				*/
				break;	
			}
			else
			{
				sUnplacedInst.erase( pInst );	
				/*
				for( int n=0; n<vLocalNet.size(); n++ )
					addNetOnGraph( m_pDesign, vLocalNet[n] );
				*/
			}
		}
		//cout << "Check 16 3 1: " << getGrid( m_pDesign, 16, 3, 1)->getRemand() << " " << getGrid( m_pDesign, 16, 3, 1 )->getPinDemand() << endl;
		
		if( !bFindAllPlace )
			break;
//
	}

	bool bSuccess = false;
	//cout << "Before reset " << getGrid( m_pDesign, 16, 3, 1)->getRemand() << " " << getGrid( m_pDesign, 16, 3, 1 )->getPinDemand() << endl;
	
	for( int i=0; i<vPseudoPinGrid.size(); i++ )
	{
		//vPseudoPinGrid[i]->delPinDemand();
		vPseudoPinGrid[i]->resetPinDemand();
	}
	
	for( int i=0; i<vNonMovedInst.size(); i++ )
	{
		resetPseudoPinDemand( vNonMovedInst[i], vNonMovedInst[i]->getPlacedX(), vNonMovedInst[i]->getPlacedY(), m_nOffsetZ );
	}

	//cout << "After reset " << getGrid( m_pDesign, 16, 3, 1)->getRemand() << " " << getGrid( m_pDesign, 16, 3, 1 )->getPinDemand() << endl;

	if( bFindAllPlace )
	{
		//cout << "route net: "<< vRipNet.size() <<endl;
		for( int i=0; i<vInst.size(); i++ )
		{
			resetPseudoPinDemand( vInst[i], vInst[i]->getPlacedX(), vInst[i]->getPlacedY(), m_nOffsetZ );
		}
		cout << "Place Result in total best: "<< nMaxBest <<endl;
		checkPseudoPin();
		
		for( int i=0; i<vInst.size(); i++ )
		{
			cout << vInst[i]->getName() << " " << vInst[i]->getPlacedX() << " " << vInst[i]->getPlacedY();
			if( i < nNumInstance )
				cout << " at " << vBestRecord[i] << endl;
			else
				cout << endl;
		}
		
		bool bRouteAllNet = true;
		bool bBetterSol = false;
		//record previous result
		int nPreviousLength = 0;
		int nNewLength = 0;
		for( int n=0; n<vRipNet.size(); n++ )
		{
			nPreviousLength = nPreviousLength + vRipNet[n]->getLength();
		}

		// route the net
		int nSuccessCount = 0;
		
		for (int i = 1; i < vRipNet.size(); i++)
		{
			for (int j = i - 1; j >= 0; j--)
			{
				int nLength_b = vRipNet[j + 1]->getLength();
				int nLength_f = vRipNet[j]->getLength();
				if (nLength_f <= nLength_b)
					break;
				else
				{
					net_C *pNet = vRipNet[j + 1];
					vRipNet[j + 1] = vRipNet[j];
					vRipNet[j] = pNet;
				}
			}
		}

		int nLengthConstraint = nPreviousLength;
	
		vector< net_C* > vTmpRip;
		cout << "Route net"<<endl;
		bRouteAllNet = multiNetRouting_ver2( vRipNet, nLengthConstraint, vTmpRip );
//		bRouteAllNet = multiNetRouting( vRipNet, nLengthConstraint, nSuccessCount );
		
		cout << "check result" << endl;
		// check if the new result is better
		if( bRouteAllNet )
			bSuccess = true;
		else
			bSuccess = false;
	}
	else
	{
		bSuccess = false;
		/*
		for( int i=0; i<vInst.size(); i++ )
			removeInstOnGraph( vInst[i] );
		*/
	}

	if( !bSuccess ) // recover all the information
	{
		cout << "recover information" << endl;
		if( bFindAllPlace )
		{
			for( int i=0; i<vInst.size(); i++ )
				removeInstOnGraph( vInst[i] );
		}
		
		recoverInstance( vInst );
		for( int i=0; i<vInst.size(); i++ )
		{
			instance_C* pInst = vInst[i];
			
			///if( i==0 )
			//	lockInstance( pInst );
			putInstOnGraph( pInst, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ );
			resetPseudoPinDemand( pInst, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ );
			//if( pInst->getName() == "C1" )
			//	cout << "C1: "<<pInst->getPlacedX() <<", "<<pInst->getPlacedY() << endl;
			//calForcedModel_ver3( m_vForced[ pInst->getId() ] );
			calForcedModel_ver4( m_vForced[ pInst->getId() ] );
		}
		recoverNet( vRipNet );
		for( int i=0; i<vRipNet.size(); i++ )
		{
			addNetOnGraph( m_pDesign, vRipNet[i] );	
		}
		/*
		for( int i=0; i<vInst.size(); i++ )
		{
			updateForcedModel_ver2( vInst[i] );	
		}
		*/
	}
	else
	{
		cout << "update model" << endl;
		for( int i=0; i<vInst.size(); i++ )
		{
			instance_C* pInst = vInst[i];
			if ( !pInst->hasBeenMoved() && !( pInst->getPlacedX() == m_vBackupInstance[i].getPlacedX() && pInst->getPlacedY() == m_vBackupInstance[i].getPlacedY() ) )
			//if ( !pInst->hasBeenMoved() )
			{
				pInst->hasMoved();
				m_vMovedInstance.push_back(pInst);
			}
		
			//m_nSuccess++;
			updateForcedModel_ver2(pInst);
			freeBoundry( pInst );
			freeForcedModel(pInst);
		}
		for( int i=0; i<m_vBackupInstance.size(); i++ )
		{
			instance_C* pInst = &m_vBackupInstance[i];
			resetPseudoPinDemand( pInst, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ );	
		}
		for( int i=0; i<m_vBackupNet.size(); i++ )
		{
			m_vBackupNet[i].cleanWire();
		}
		
	}

	m_vBackupNet.clear();
	m_vBackupInstance.clear();	

	cout << "return  multipleCellMovement result"<<endl;
	return bSuccess;
}

bool router_C::multipleCellMovement_ver4( vector< instance_C* > &vInst, boundry_C* pBound )
{	
	cout << "at multipleCellMovement"<<endl;
	char cBound = pBound->m_cType;
	int nMoveCount = vInst.size();
	set< instance_C* > sInst;	
	vector< instance_C* > vIInst = vInst;
	for( int i=0; i<vIInst.size(); i++ )
	{
		sInst.insert( vIInst[i] );
		vInst[ nMoveCount - 1 - i ] = vIInst[i];	
	}
	
	vector< net_C* > vRipNet;
	set< net_C* > sNet;
	set< net_C* > sRipNet;
	set< instance_C* > sUnplacedInst;
	for( int i=0; i<vInst.size(); i++ )
	{
		instance_C* pInst = vInst[i];
		if( pInst->hasBeenMoved() )
			nMoveCount--;
		vector< networkForced_C* > &vTmpNF = m_vForced[ pInst->getId() ].m_vNetwork;
		for( int j=0; j<vTmpNF.size(); j++ )
		{
			if( sNet.count( vTmpNF[j]->m_pNet ) == 0 )
			{
				sNet.insert( vTmpNF[j]->m_pNet );
				vRipNet.push_back( vTmpNF[j]->m_pNet );
			}
		}
		sUnplacedInst.insert( pInst );
	}
	//cout << vRipNet.size() << endl;
	set< net_C* > sSpecialNet;
//
/*
	for( int i=0; i<1; i++ )
	{
		instance_C* pInst = vInst[i];
		vector< networkForced_C* > &vTmpNF = m_vForced[ pInst->getId() ].m_vNetwork;
		for( int j=0; j<vTmpNF.size(); j++ )
		{
			if( sSpecialNet.count( vTmpNF[j]->m_pNet ) == 0 )
			{
				sSpecialNet.insert( vTmpNF[j]->m_pNet );
			}
		}
	}
*/
//


	int nSwapCost = m_pDesign->getMaxCellConstraint() - m_vMovedInstance.size() - nMoveCount;

	backupInstance( vInst );
	backupNet( vRipNet );
	//cout << vRipNet.size() << endl;

	for( int i=0; i<vInst.size(); i++ )
	{
		removeInstOnGraph( vInst[i] );
		delPseudoPinDemand( vInst[i], vInst[i]->getPlacedX(), vInst[i]->getPlacedY(), m_nOffsetZ );
		//calForcedModel_ver4( m_vForced[ vInst[i]->getId() ], sSpecialNet, cBound );
	}
	/*
	for( int i=0; i<vRipNet.size(); i++ )
	{
		//cout << vRipNet[i]->getName() << endl;
		ripupNet( vRipNet[i] );	
	}
	cleanWire( vRipNet );
	*/
	//cout << vRipNet.size() << endl;


	bool bFindAllPlace = true;
	vector< instance_C* > vTmpSwap;
	//vector< instance_C > vSwapBackup;
	vector< instance_C* > vPlacedInst;

	//cout << "place instance"<<endl;
	// place the instance
	int nNumInstance = vInst.size();
// update the networkforced model considering unplaced instance
	//cout << "Update first"<<endl;
	/*	
	for( int i=0; i<vRipNet.size(); i++ )
	{
		calForcedNetwork( m_vNetworkForced[ vRipNet[i]->getId() ], sUnplacedInst );
	}
	*/
//
	//cout << "Move Cell"<<endl;
	vector< instance_C* > vRecordPse;
	vector< instance_C* > vNonMovedInst;
	vector< int > vBestRecord; 
	int nMaxBest = 0;
	cout << "X Min: "<< pBound->m_pNetwork->m_nMinX << " " << pBound->m_pNetwork->m_n2MinX << endl;
	cout << "X Max: "<< pBound->m_pNetwork->m_nMaxX << " " << pBound->m_pNetwork->m_n2MaxX << endl;
	cout << "Y Min: "<< pBound->m_pNetwork->m_nMinY << " " << pBound->m_pNetwork->m_n2MinY << endl;
	cout << "Y Max: "<< pBound->m_pNetwork->m_nMaxY << " " << pBound->m_pNetwork->m_n2MaxY << endl;
	for( int i=0; i<nNumInstance; i++ )
	{
		instance_C* pInst = vInst[i];
		//cout << "Place "<<pInst->getName() << " ";
		gGrid_C* pOGrid = getGrid( m_pDesign, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ );	
			
// calculate new forced considering ripup nets
		//cout << "Cal"<<endl;
		//calForcedModel_ver2( m_vForced[ pInst->getId() ] );
// //
		
		vector< net_C* > vLocalNet;
		set< net_C* > sLocalNet;
		vector< networkForced_C* > &vTmpNF = m_vForced[ pInst->getId() ].m_vNetwork;
		for( int n=0; n<vTmpNF.size(); n++ )
		{
			if( sRipNet.count( vTmpNF[n]->m_pNet ) == 0 )
			{
				vLocalNet.push_back( vTmpNF[n]->m_pNet );
				sLocalNet.insert( vTmpNF[n]->m_pNet );
				sRipNet.insert( vTmpNF[n]->m_pNet );
			}
		}
		ripupNet( vLocalNet );

		//cout << "Find"<<endl;
		//vector< gGrid_C* > vBestGrid = findPlaceToMove_ver4( pInst );		
		//vector< gGrid_C* > vBestGrid = findPlaceToMove_ver3( pInst );		
		vector< gGrid_C* > vBestGrid = findPlaceToMove_ver3( pInst, pBound );		
			
		cout << "Choice "<< vBestGrid.size() << endl;
		for( int i=vBestGrid.size()-1; i>=0; i-- )
		{
			int nX, nY, nZ;
			vBestGrid[i]->getPosition( nX, nY, nZ );
			cout <<"( " << nX << " " << nY << " )";
		}
		cout << endl;
		
		gGrid_C* pBGrid = NULL;
		if( vBestGrid.size() == 0 )
		{
			//lockInstance( pInst );
			for( int j=0; j<i; j++ )
			{
				resetPseudoPinDemand( vInst[j], vInst[j]->getPlacedX(), vInst[j]->getPlacedY(), m_nOffsetZ );
				removeInstOnGraph( vInst[j] );
			}
			for( int j=0; j<vTmpSwap.size(); j++ )
			{
				resetPseudoPinDemand( vTmpSwap[j], vTmpSwap[j]->getPlacedX(), vTmpSwap[j]->getPlacedY(), m_nOffsetZ );
				removeInstOnGraph( vTmpSwap[j] );
			}
			/*
			for( int n=0; n<vLocalNet.size(); n++ )
				addNetOnGraph( m_pDesign, vLocalNet[n] );
			*/
			for( int n=0; n<vRipNet.size(); n++ )
			{
				if( sRipNet.count( vRipNet[n] ) != 0 )
				{
					addNetOnGraph( m_pDesign, vRipNet[n] );
				}
			}
			bFindAllPlace = false;
			break;
		}
		else
		{
			bool bPlace = false;
			vector< net_C *> vTmpRip;
			for( int b=vBestGrid.size() - 1; b>=0; b-- )
			{
				int nX, nY, nZ;
				pBGrid = vBestGrid[b];
				pBGrid->getPosition( nX, nY, nZ );
				//cout << "Best: " << nX << " " << nY << endl;
				//if( isPlaceable( pBGrid, pInst ) )
				//if( isPlaceable_ver4( pBGrid, pInst, vTmpRip ) )
				if( isPlaceable_ver5( pBGrid, pInst, vTmpRip ) )
				{
//
					vBestRecord.push_back(b);
//					
					if( vTmpRip.size() != 0 )	
					{
						for( int n=0; n<vTmpRip.size(); n++ )
						{
							if( sNet.count( vTmpRip[n] ) != 0 )
							{
								vTmpRip.erase( vTmpRip.begin() + n );
								n--;
							}
							else
								sNet.insert( vTmpRip[n] );	
						}
						backupNet( vTmpRip );
						for( int n=0; n<vTmpRip.size(); n++ )
						{
							sRipNet.insert( vTmpRip[n] );
							ripupNet( vTmpRip[n] );	
							vRipNet.push_back( vTmpRip[n] );
						}
						//cleanWire( vTmpRip );
					}

					//cout << "Place at: " << nX << ", "<< nY <<", "<< nZ << endl;
					putInstOnGraph( pInst, nX, nY, nZ );
					calPseudoPinDemand( pInst, nX, nY, nZ );
					vRecordPse.push_back( pInst );	
				// non moved instance need to set pseudo pin for ripup net
					for( int l=0; l<vLocalNet.size(); l++ )
					{
						vector< pin_C* > vTmpPin = vLocalNet[l]->getPin();
						for( int p=0; p<vTmpPin.size(); p++ )
						{
							instance_C* pPInst = ( instance_C* )vTmpPin[p]->getCell();
							if( sInst.count( pPInst ) == 0 )
							{
								int nPX = pPInst->getPlacedX();
								int nPY = pPInst->getPlacedY();
								int nPZ = vTmpPin[p]->getLayerId();
								gGrid_C* pPGrid = getGrid( m_pDesign, nPX, nPY, nPZ );
								pPGrid->addPinDemand();
								vNonMovedInst.push_back( pPInst );
							}
						}
					}
					for( int l=0; l<vTmpRip.size(); l++ )
					{
						vector< pin_C* > vTmpPin = vTmpRip[l]->getPin();
						for( int p=0; p<vTmpPin.size(); p++ )
						{
							instance_C* pPInst = ( instance_C* )vTmpPin[p]->getCell();
							if( sInst.count( pPInst ) == 0 )
							{
								int nPX = pPInst->getPlacedX();
								int nPY = pPInst->getPlacedY();
								int nPZ = vTmpPin[p]->getLayerId();
								gGrid_C* pPGrid = getGrid( m_pDesign, nPX, nPY, nPZ );
								pPGrid->addPinDemand();
								vNonMovedInst.push_back( pPInst );
							}
						}
					}

					/*
					if( pInst->getName() == "C1112" )
					{
						cout << "Ripup: " << endl;
						for( int t=0; t<vTmpRip.size(); t++ )
							cout << vTmpRip[t]->getName() << " ";
						cout << endl;
					}
					*/
					bPlace = true;
					sUnplacedInst.erase( pInst );
					break;
				}
				/*	
				else if( pBGrid != pOGrid && nSwapCost > 0 )
				{
					vector<instance_C *> vSwapInstance = findSwapInstance(pBGrid, pOGrid, pInst);
					for (int s = 0; s < vSwapInstance.size(); s++)
					{
						bool bHasSwap = false;
						for( int t=0; t<vTmpSwap.size(); t++ )
						{
							if( vTmpSwap[t] == vSwapInstance[s] )
							{
								bHasSwap = true;
								break;
							}		
						}

						for( int t=0; t<vInst.size(); t++ )
						{
							if( vInst[t] == vSwapInstance[s] )
							{
								bHasSwap = true;
								break;
							}
						}
						if( bHasSwap )
							continue;

						vTmpRip.clear();
						instance_C *pSInst = vSwapInstance[s];
						instance_C cLocalBackupInstance;
						// local backup pSInst
						cLocalBackupInstance.setName( pSInst->getName() );
						cLocalBackupInstance.setPlaced(pSInst->getPlacedX(), pSInst->getPlacedY());
						removeInstOnGraph(pSInst);
						//if( swapInstance(pBGrid, pOGrid, pInst, pSInst, vTmpRip ) )
						if( swapInstance_ver2(pBGrid, pOGrid, pInst, pSInst, vTmpRip ) )
						{
							//cout << "Swap: "<<pSInst->getName() << " at " << cLocalBackupInstance.getPlacedX() <<", " << cLocalBackupInstance.getPlacedY() << endl;
//
							vBestRecord.push_back(b);
//							
							if( vTmpRip.size() != 0 )
							{
								for( int n=0; n<vTmpRip.size(); n++ )
								{
									if( sNet.count( vTmpRip[n] ) != 0 )
									{
										vTmpRip.erase( vTmpRip.begin() + n );
										n--;
									}
									else
										sNet.insert( vTmpRip[n] );	
								}
								
								backupNet( vTmpRip );
								for( int n=0; n<vTmpRip.size(); n++ )
								{
									sRipNet.insert( vTmpRip[n] );
									ripupNet( vTmpRip[n] );	
									vRipNet.push_back( vTmpRip[n] );
								}
								//cleanWire( vTmpRip );
								
							}

							int nTmpX, nTmpY, nTmpZ;
							pBGrid->getPosition(nTmpX, nTmpY, nTmpZ);
							putInstOnGraph(pInst, nTmpX, nTmpY, nTmpZ);
							calPseudoPinDemand( pInst, nTmpX, nTmpY, nTmpZ );
							for( int l=0; l<vLocalNet.size(); l++ )
							{
								vector< pin_C* > vTmpPin = vLocalNet[l]->getPin();
								for( int p=0; p<vTmpPin.size(); p++ )
								{
									instance_C* pPInst = ( instance_C* )vTmpPin[p]->getCell();
									if( sInst.count( pPInst ) == 0 )
									{
										int nPX = pPInst->getPlacedX();
										int nPY = pPInst->getPlacedY();
										int nPZ = vTmpPin[p]->getLayerId();
										gGrid_C* pPGrid = getGrid( m_pDesign, nPX, nPY, nPZ );
										pPGrid->addPinDemand();
										vNonMovedInst.push_back( pPInst );
									}
								}
							}
					vRecordPse.push_back( pInst );	
						
							sInst.insert( pSInst );
							pOGrid->getPosition(nTmpX, nTmpY, nTmpZ);
							putInstOnGraph(pSInst, nTmpX, nTmpY, nTmpZ);
							calPseudoPinDemand( pSInst, nTmpX, nTmpY, nTmpZ );
							for( int l=0; l<vTmpRip.size(); l++ )
							{
								vector< pin_C* > vTmpPin = vTmpRip[l]->getPin();
								for( int p=0; p<vTmpPin.size(); p++ )
								{
									instance_C* pPInst = ( instance_C* )vTmpPin[p]->getCell();
									if( sInst.count( pPInst ) == 0 )
									{
										int nPX = pPInst->getPlacedX();
										int nPY = pPInst->getPlacedY();
										int nPZ = vTmpPin[p]->getLayerId();
										gGrid_C* pPGrid = getGrid( m_pDesign, nPX, nPY, nPZ );
										pPGrid->addPinDemand();
										vNonMovedInst.push_back( pPInst );
									}
								}
							}
					vRecordPse.push_back( pInst );	
							
							vTmpSwap.push_back( pSInst );
							vInst.push_back( pSInst );
							m_vBackupInstance.push_back( cLocalBackupInstance );
							bPlace = true;
							nSwapCost--;
							cout << "Place at: " << nX << ", "<< nY <<", "<< nZ << endl;
							cout << "Swap from: " << pSInst->getPlacedX() << ", "<< pSInst->getPlacedY() <<", "<< nZ << endl;
							break;
						}
						else
						{
							// recover instance, put instance back on the graph
							int nTmpX, nTmpY, nTmpZ;
							pBGrid->getPosition(nTmpX, nTmpY, nTmpZ);
							putInstOnGraph(pSInst, nTmpX, nTmpY, nTmpZ);
							pOGrid->getPosition( nTmpX, nTmpY, nTmpZ );
							pInst->setPlaced(nTmpX, nTmpY);
						}
					}
					
					if( bPlace )
						break;
				}*/
				
				
			}
			if( !bPlace )
			{
				//lockInstance( pInst );
				for( int j=0; j<i; j++ )
				{
					resetPseudoPinDemand( vInst[j], vInst[j]->getPlacedX(), vInst[j]->getPlacedY(), m_nOffsetZ );
					removeInstOnGraph( vInst[j] );
				}
				for( int j=0; j<vTmpSwap.size(); j++ )
				{
					resetPseudoPinDemand( vTmpSwap[j], vTmpSwap[j]->getPlacedX(), vTmpSwap[j]->getPlacedY(), m_nOffsetZ );
					removeInstOnGraph( vTmpSwap[j] );
				}
				bFindAllPlace = false;
				/*	
				for( int n=0; n<vLocalNet.size(); n++ )
					addNetOnGraph( m_pDesign, vLocalNet[n] );
				*/
				for( int n=0; n<vRipNet.size(); n++ )
				{
					if( sRipNet.count( vRipNet[n] ) != 0 )
						addNetOnGraph( m_pDesign, vRipNet[n] );
				}
				break;	
			}
			else
			{
				sUnplacedInst.erase( pInst );	
				/*
				for( int n=0; n<vLocalNet.size(); n++ )
					addNetOnGraph( m_pDesign, vLocalNet[n] );
				*/
			}
		}
// update the nets networkforced model considering unplaced instance that influence by placed instance
		//cout << "Update second"<<endl;
		/*
		vector< networkForced_C* > &vTTmpNF = m_vForced[ pInst->getId() ].m_vNetwork;
		for( int n=0; n<vTTmpNF.size(); n++ )
			calForcedNetwork( *vTTmpNF[n], sUnplacedInst );
		*/
		/*
		for( int g=1; g<=3; g++ )
		{
			gGrid_C* gG = getGrid( m_pDesign, 20, 4, g );
			cout << gG->getRemand() << " ";
			cout << gG->getRemand() - gG->getPinDemand() << " ";
		}
		cout << endl;
		*/
		if( !bFindAllPlace )
			break;
//
	}

	bool bSuccess = false;
	/*
	cout << "Pre "<< endl;
	for( int g=1; g<=1; g++ )
	{
		gGrid_C* gG = getGrid( m_pDesign, 20, 4, g );
		cout << gG->getSupply() << " " << gG->getExtraDemand() << " " << gG->getNet().size() << endl;
		cout << gG->getRemand() << " ";
		cout << gG->getRemand() - gG->getPinDemand() << " ";
	}
	cout << endl;
	*/
	// route the net 
	/*
	for( int i=0; i<vRipNet.size(); i++ )
	{
		//cout << vRipNet[i]->getName() << endl;
		ripupNet( vRipNet[i] );	
	}
	*/
	for( int i=0; i<vNonMovedInst.size(); i++ )
	{
		resetPseudoPinDemand( vNonMovedInst[i], vNonMovedInst[i]->getPlacedX(), vNonMovedInst[i]->getPlacedY(), m_nOffsetZ );
	}
	cleanWire( vRipNet );
	/*
	cout << "Post "<< endl;
	for( int g=1; g<=3; g++ )
	{
		gGrid_C* gG = getGrid( m_pDesign, 20, 4, g );
		cout << gG->getSupply() << " " << gG->getExtraDemand() << " " << gG->getNet().size() << endl;
		cout << gG->getRemand() << " ";
		cout << gG->getRemand() - gG->getPinDemand() << " ";
	}
	cout << endl;
	*/
	if( bFindAllPlace )
	{
		//cout << "route net: "<< vRipNet.size() <<endl;
		for( int i=0; i<vInst.size(); i++ )
		{
			resetPseudoPinDemand( vInst[i], vInst[i]->getPlacedX(), vInst[i]->getPlacedY(), m_nOffsetZ );
		}
		/*
		gGrid_C* pTmpGrid = getGrid( m_pDesign, 20, 4, 1 );
		vector< instance_C* > vGInst = pTmpGrid->getInstance();
		cout << "Instance in 7 5 1:" << endl;
		for( int i=0; i<vGInst.size(); i++ )
		{
			cout << vGInst[i]->getName() << " ";
		}
		cout << endl;
		
		cout << "Record:" << endl;
		for( int i=0; i<vRecordPse.size(); i++ )
		{
			cout << vRecordPse[i]->getName() << " ";
		}
		cout << endl;
		*/
		cout << "Place Result in total best: "<< nMaxBest <<endl;
		
		for( int i=0; i<vInst.size(); i++ )
		{
			resetPseudoPinDemand( vInst[i], vInst[i]->getPlacedX(), vInst[i]->getPlacedY(), m_nOffsetZ );
		}
		
		for( int i=0; i<vInst.size(); i++ )
		{
			cout << vInst[i]->getName() << " " << vInst[i]->getPlacedX() << " " << vInst[i]->getPlacedY();
			if( i < nNumInstance )
				cout << " at " << vBestRecord[i] << endl;
			else
				cout << endl;
		}
		
		bool bRouteAllNet = true;
		bool bBetterSol = false;
		//record previous result
		int nPreviousLength = 0;
		int nNewLength = 0;
		for( int n=0; n<vRipNet.size(); n++ )
		{
			nPreviousLength = nPreviousLength + vRipNet[n]->getLength();
		}

		// route the net
		int nSuccessCount = 0;
		
		for (int i = 1; i < vRipNet.size(); i++)
		{
			for (int j = i - 1; j >= 0; j--)
			{
				int nLength_b = vRipNet[j + 1]->getLength();
				int nLength_f = vRipNet[j]->getLength();
				if (nLength_f <= nLength_b)
					break;
				else
				{
					net_C *pNet = vRipNet[j + 1];
					vRipNet[j + 1] = vRipNet[j];
					vRipNet[j] = pNet;
				}
			}
		}

		int nLengthConstraint = nPreviousLength;
	
		bRouteAllNet = multiNetRouting( vRipNet, nLengthConstraint, nSuccessCount );

		// check if the new result is better
		if( bRouteAllNet )
		{
			for( int n=0; n<vRipNet.size(); n++ )
			{
				nNewLength = nNewLength + vRipNet[n]->getLength();
			}
			if( nNewLength < nPreviousLength )
			{
				bBetterSol = true;
			}
			else
			{
				bBetterSol = false;
			}
		}
		
		// deal with the routing result
		if( !bRouteAllNet || !bBetterSol )
		{
			if( !bRouteAllNet )
			{
				///cout << "Some net failed"<<endl;
			}
			else
			{
				cout << "No better solution in routing "<<endl;
				cout << "Original Length: " << nPreviousLength << " New Length: "<< nNewLength<<endl;
			}
			for( int n=0; n<nSuccessCount; n++ )
			{
				ripupNet( vRipNet[n] );
			}
			cleanWire( vRipNet );
			bSuccess = false;
			for( int i=0; i<vInst.size(); i++ )
				removeInstOnGraph( vInst[i] );
		}
		else //( bRouteAllNet && bBetterSol )
		{	
			bSuccess = true;
		}
	}
	else
	{
		bSuccess = false;
		/*
		for( int i=0; i<vInst.size(); i++ )
			removeInstOnGraph( vInst[i] );
		*/
	}

	if( !bSuccess ) // recover all the information
	{
		//cout << "recover information" << endl;
		recoverInstance( vInst );
		for( int i=0; i<vInst.size(); i++ )
		{
			instance_C* pInst = vInst[i];
			
			///if( i==0 )
			//	lockInstance( pInst );
			putInstOnGraph( pInst, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ );
			resetPseudoPinDemand( pInst, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ );
			//if( pInst->getName() == "C1" )
			//	cout << "C1: "<<pInst->getPlacedX() <<", "<<pInst->getPlacedY() << endl;
			//calForcedModel_ver3( m_vForced[ pInst->getId() ] );
			calForcedModel_ver4( m_vForced[ pInst->getId() ] );
		}
		recoverNet( vRipNet );
		for( int i=0; i<vRipNet.size(); i++ )
		{
			addNetOnGraph( m_pDesign, vRipNet[i] );	
		}
		for( int i=0; i<vInst.size(); i++ )
		{
			updateForcedModel_ver2( vInst[i] );	
		}
	}
	else
	{
		//cout << "update model" << endl;
		for( int i=0; i<vInst.size(); i++ )
		{
			instance_C* pInst = vInst[i];
			if ( !pInst->hasBeenMoved() && !( pInst->getPlacedX() == m_vBackupInstance[i].getPlacedX() && pInst->getPlacedY() == m_vBackupInstance[i].getPlacedY() ) )
			//if ( !pInst->hasBeenMoved() )
			{
				pInst->hasMoved();
				m_vMovedInstance.push_back(pInst);
			}
		
			//m_nSuccess++;
			updateForcedModel_ver2(pInst);
			freeBoundry( pInst );
			freeForcedModel(pInst);
		}
		for( int i=0; i<m_vBackupInstance.size(); i++ )
		{
			instance_C* pInst = &m_vBackupInstance[i];
			resetPseudoPinDemand( pInst, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ );	
		}
		for( int i=0; i<m_vBackupNet.size(); i++ )
		{
			m_vBackupNet[i].cleanWire();
		}
		
	}

	m_vBackupNet.clear();
	m_vBackupInstance.clear();	

	cout << "return  multipleCellMovement result"<<endl;
	return bSuccess;
}

bool router_C::multipleCellMovement_ver4( vector< instance_C* > &vInst )
{	
	cout << "at multipleCellMovement"<<endl;
	int nMoveCount = vInst.size();
	set< instance_C* > sInst;	
	vector< instance_C* > vIInst = vInst;
	for( int i=0; i<vIInst.size(); i++ )
	{
		sInst.insert( vIInst[i] );
		//vInst[ nMoveCount - 1 - i ] = vIInst[i];	
	}
	
	vector< net_C* > vRipNet;
	set< net_C* > sNet;
	set< net_C* > sRipNet;
	set< instance_C* > sUnplacedInst;
	for( int i=0; i<vInst.size(); i++ )
	{
		instance_C* pInst = vInst[i];
		if( pInst->hasBeenMoved() )
			nMoveCount--;
		vector< networkForced_C* > &vTmpNF = m_vForced[ pInst->getId() ].m_vNetwork;
		for( int j=0; j<vTmpNF.size(); j++ )
		{
			if( sNet.count( vTmpNF[j]->m_pNet ) == 0 )
			{
				sNet.insert( vTmpNF[j]->m_pNet );
				vRipNet.push_back( vTmpNF[j]->m_pNet );
			}
		}
		sUnplacedInst.insert( pInst );
	}
	//cout << vRipNet.size() << endl;
//
	set< net_C* > sSpecialNet;
	for( int i=0; i<1; i++ )
	{
		instance_C* pInst = vInst[i];
		vector< networkForced_C* > &vTmpNF = m_vForced[ pInst->getId() ].m_vNetwork;
		for( int j=0; j<vTmpNF.size(); j++ )
		{
			if( sSpecialNet.count( vTmpNF[j]->m_pNet ) == 0 )
			{
				sSpecialNet.insert( vTmpNF[j]->m_pNet );
			}
		}
	}
//


	int nSwapCost = m_pDesign->getMaxCellConstraint() - m_vMovedInstance.size() - nMoveCount;

	backupInstance( vInst );
	backupNet( vRipNet );
	//cout << vRipNet.size() << endl;

	for( int i=0; i<vInst.size(); i++ )
	{
		removeInstOnGraph( vInst[i] );
		delPseudoPinDemand( vInst[i], vInst[i]->getPlacedX(), vInst[i]->getPlacedY(), m_nOffsetZ );
		calForcedModel_ver4( m_vForced[ vInst[i]->getId() ], sSpecialNet );
	}
	/*
	for( int i=0; i<vRipNet.size(); i++ )
	{
		//cout << vRipNet[i]->getName() << endl;
		ripupNet( vRipNet[i] );	
	}
	cleanWire( vRipNet );
	*/
	//cout << vRipNet.size() << endl;


	bool bFindAllPlace = true;
	vector< instance_C* > vTmpSwap;
	//vector< instance_C > vSwapBackup;
	vector< instance_C* > vPlacedInst;

	//cout << "place instance"<<endl;
	// place the instance
	int nNumInstance = vInst.size();
// update the networkforced model considering unplaced instance
	//cout << "Update first"<<endl;
	/*	
	for( int i=0; i<vRipNet.size(); i++ )
	{
		calForcedNetwork( m_vNetworkForced[ vRipNet[i]->getId() ], sUnplacedInst );
	}
	*/
//
	//cout << "Move Cell"<<endl;
	vector< instance_C* > vRecordPse;
	vector< instance_C* > vNonMovedInst;
	for( int i=0; i<nNumInstance; i++ )
	{
		instance_C* pInst = vInst[i];
		//cout << "Place "<<pInst->getName() << " ";
		gGrid_C* pOGrid = getGrid( m_pDesign, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ );	
			
// calculate new forced considering ripup nets
		//cout << "Cal"<<endl;
		//calForcedModel_ver2( m_vForced[ pInst->getId() ] );
// //
		
		vector< net_C* > vLocalNet;
		set< net_C* > sLocalNet;
		vector< networkForced_C* > &vTmpNF = m_vForced[ pInst->getId() ].m_vNetwork;
		for( int n=0; n<vTmpNF.size(); n++ )
		{
			if( sRipNet.count( vTmpNF[n]->m_pNet ) == 0 )
			{
				vLocalNet.push_back( vTmpNF[n]->m_pNet );
				sLocalNet.insert( vTmpNF[n]->m_pNet );
				sRipNet.insert( vTmpNF[n]->m_pNet );
			}
		}
		ripupNet( vLocalNet );

		//cout << "Find"<<endl;
		//vector< gGrid_C* > vBestGrid = findPlaceToMove_ver4( pInst );		
		vector< gGrid_C* > vBestGrid = findPlaceToMove_ver3( pInst );		
		//cout << "Choice "<< vBestGrid.size() << endl;
		gGrid_C* pBGrid = NULL;
		if( vBestGrid.size() == 0 )
		{
			//lockInstance( pInst );
			for( int j=0; j<i; j++ )
			{
				resetPseudoPinDemand( vInst[j], vInst[j]->getPlacedX(), vInst[j]->getPlacedY(), m_nOffsetZ );
				removeInstOnGraph( vInst[j] );
			}
			for( int j=0; j<vTmpSwap.size(); j++ )
			{
				resetPseudoPinDemand( vTmpSwap[j], vTmpSwap[j]->getPlacedX(), vTmpSwap[j]->getPlacedY(), m_nOffsetZ );
				removeInstOnGraph( vTmpSwap[j] );
			}
			/*
			for( int n=0; n<vLocalNet.size(); n++ )
				addNetOnGraph( m_pDesign, vLocalNet[n] );
			*/
			for( int n=0; n<vRipNet.size(); n++ )
			{
				if( sRipNet.count( vRipNet[n] ) != 0 )
				{
					addNetOnGraph( m_pDesign, vRipNet[n] );
				}
			}
			bFindAllPlace = false;
			break;
		}
		else
		{
			bool bPlace = false;
			vector< net_C *> vTmpRip;
			for( int b=vBestGrid.size() - 1; b>=0; b-- )
			{
				int nX, nY, nZ;
				pBGrid = vBestGrid[b];
				pBGrid->getPosition( nX, nY, nZ );
				//cout << "Best: " << nX << " " << nY << endl;
				//if( isPlaceable( pBGrid, pInst ) )
				//if( isPlaceable_ver4( pBGrid, pInst, vTmpRip ) )
				if( isPlaceable_ver5( pBGrid, pInst, vTmpRip ) )
				{
					if( vTmpRip.size() != 0 )	
					{
						for( int n=0; n<vTmpRip.size(); n++ )
						{
							if( sNet.count( vTmpRip[n] ) != 0 )
							{
								vTmpRip.erase( vTmpRip.begin() + n );
								n--;
							}
							else
								sNet.insert( vTmpRip[n] );	
						}
						backupNet( vTmpRip );
						for( int n=0; n<vTmpRip.size(); n++ )
						{
							sRipNet.insert( vTmpRip[n] );
							ripupNet( vTmpRip[n] );	
							vRipNet.push_back( vTmpRip[n] );
						}
						//cleanWire( vTmpRip );
					}

					//cout << "Place at: " << nX << ", "<< nY <<", "<< nZ << endl;
					putInstOnGraph( pInst, nX, nY, nZ );
					calPseudoPinDemand( pInst, nX, nY, nZ );
					vRecordPse.push_back( pInst );	
				// non moved instance need to set pseudo pin for ripup net
					for( int l=0; l<vLocalNet.size(); l++ )
					{
						vector< pin_C* > vTmpPin = vLocalNet[l]->getPin();
						for( int p=0; p<vTmpPin.size(); p++ )
						{
							instance_C* pPInst = ( instance_C* )vTmpPin[p]->getCell();
							if( sInst.count( pPInst ) == 0 )
							{
								int nPX = pPInst->getPlacedX();
								int nPY = pPInst->getPlacedY();
								int nPZ = vTmpPin[p]->getLayerId();
								gGrid_C* pPGrid = getGrid( m_pDesign, nPX, nPY, nPZ );
								pPGrid->addPinDemand();
								vNonMovedInst.push_back( pPInst );
							}
						}
					}
					for( int l=0; l<vTmpRip.size(); l++ )
					{
						vector< pin_C* > vTmpPin = vTmpRip[l]->getPin();
						for( int p=0; p<vTmpPin.size(); p++ )
						{
							instance_C* pPInst = ( instance_C* )vTmpPin[p]->getCell();
							if( sInst.count( pPInst ) == 0 )
							{
								int nPX = pPInst->getPlacedX();
								int nPY = pPInst->getPlacedY();
								int nPZ = vTmpPin[p]->getLayerId();
								gGrid_C* pPGrid = getGrid( m_pDesign, nPX, nPY, nPZ );
								pPGrid->addPinDemand();
								vNonMovedInst.push_back( pPInst );
							}
						}
					}

					/*
					if( pInst->getName() == "C1112" )
					{
						cout << "Ripup: " << endl;
						for( int t=0; t<vTmpRip.size(); t++ )
							cout << vTmpRip[t]->getName() << " ";
						cout << endl;
					}
					*/
					bPlace = true;
					sUnplacedInst.erase( pInst );
					break;
				}
				else if( pBGrid != pOGrid && nSwapCost > 0 )
				{
					vector<instance_C *> vSwapInstance = findSwapInstance(pBGrid, pOGrid, pInst);
					for (int s = 0; s < vSwapInstance.size(); s++)
					{
						bool bHasSwap = false;
						for( int t=0; t<vTmpSwap.size(); t++ )
						{
							if( vTmpSwap[t] == vSwapInstance[s] )
							{
								bHasSwap = true;
								break;
							}		
						}

						for( int t=0; t<vInst.size(); t++ )
						{
							if( vInst[t] == vSwapInstance[s] )
							{
								bHasSwap = true;
								break;
							}
						}
						if( bHasSwap )
							continue;

						vTmpRip.clear();
						instance_C *pSInst = vSwapInstance[s];
						instance_C cLocalBackupInstance;
						// local backup pSInst
						cLocalBackupInstance.setName( pSInst->getName() );
						cLocalBackupInstance.setPlaced(pSInst->getPlacedX(), pSInst->getPlacedY());
						removeInstOnGraph(pSInst);
						//if( swapInstance(pBGrid, pOGrid, pInst, pSInst, vTmpRip ) )
						if( swapInstance_ver2(pBGrid, pOGrid, pInst, pSInst, vTmpRip ) )
						{
							//cout << "Swap: "<<pSInst->getName() << " at " << cLocalBackupInstance.getPlacedX() <<", " << cLocalBackupInstance.getPlacedY() << endl;
							if( vTmpRip.size() != 0 )
							{
								for( int n=0; n<vTmpRip.size(); n++ )
								{
									if( sNet.count( vTmpRip[n] ) != 0 )
									{
										vTmpRip.erase( vTmpRip.begin() + n );
										n--;
									}
									else
										sNet.insert( vTmpRip[n] );	
								}
								
								backupNet( vTmpRip );
								for( int n=0; n<vTmpRip.size(); n++ )
								{
									sRipNet.insert( vTmpRip[n] );
									ripupNet( vTmpRip[n] );	
									vRipNet.push_back( vTmpRip[n] );
								}
								//cleanWire( vTmpRip );
								
							}

							int nTmpX, nTmpY, nTmpZ;
							pBGrid->getPosition(nTmpX, nTmpY, nTmpZ);
							putInstOnGraph(pInst, nTmpX, nTmpY, nTmpZ);
							calPseudoPinDemand( pInst, nTmpX, nTmpY, nTmpZ );
							for( int l=0; l<vLocalNet.size(); l++ )
							{
								vector< pin_C* > vTmpPin = vLocalNet[l]->getPin();
								for( int p=0; p<vTmpPin.size(); p++ )
								{
									instance_C* pPInst = ( instance_C* )vTmpPin[p]->getCell();
									if( sInst.count( pPInst ) == 0 )
									{
										int nPX = pPInst->getPlacedX();
										int nPY = pPInst->getPlacedY();
										int nPZ = vTmpPin[p]->getLayerId();
										gGrid_C* pPGrid = getGrid( m_pDesign, nPX, nPY, nPZ );
										pPGrid->addPinDemand();
										vNonMovedInst.push_back( pPInst );
									}
								}
							}
					vRecordPse.push_back( pInst );	
						
							sInst.insert( pSInst );
							pOGrid->getPosition(nTmpX, nTmpY, nTmpZ);
							putInstOnGraph(pSInst, nTmpX, nTmpY, nTmpZ);
							calPseudoPinDemand( pSInst, nTmpX, nTmpY, nTmpZ );
							for( int l=0; l<vTmpRip.size(); l++ )
							{
								vector< pin_C* > vTmpPin = vTmpRip[l]->getPin();
								for( int p=0; p<vTmpPin.size(); p++ )
								{
									instance_C* pPInst = ( instance_C* )vTmpPin[p]->getCell();
									if( sInst.count( pPInst ) == 0 )
									{
										int nPX = pPInst->getPlacedX();
										int nPY = pPInst->getPlacedY();
										int nPZ = vTmpPin[p]->getLayerId();
										gGrid_C* pPGrid = getGrid( m_pDesign, nPX, nPY, nPZ );
										pPGrid->addPinDemand();
										vNonMovedInst.push_back( pPInst );
									}
								}
							}
					vRecordPse.push_back( pInst );	
							
							vTmpSwap.push_back( pSInst );
							vInst.push_back( pSInst );
							m_vBackupInstance.push_back( cLocalBackupInstance );
							bPlace = true;
							nSwapCost--;
							cout << "Place at: " << nX << ", "<< nY <<", "<< nZ << endl;
							cout << "Swap from: " << pSInst->getPlacedX() << ", "<< pSInst->getPlacedY() <<", "<< nZ << endl;
							break;
						}
						else
						{
							// recover instance, put instance back on the graph
							int nTmpX, nTmpY, nTmpZ;
							pBGrid->getPosition(nTmpX, nTmpY, nTmpZ);
							putInstOnGraph(pSInst, nTmpX, nTmpY, nTmpZ);
							pOGrid->getPosition( nTmpX, nTmpY, nTmpZ );
							pInst->setPlaced(nTmpX, nTmpY);
						}
					}
					
					if( bPlace )
						break;
				}
				
			}
			if( !bPlace )
			{
				//lockInstance( pInst );
				for( int j=0; j<i; j++ )
				{
					resetPseudoPinDemand( vInst[j], vInst[j]->getPlacedX(), vInst[j]->getPlacedY(), m_nOffsetZ );
					removeInstOnGraph( vInst[j] );
				}
				for( int j=0; j<vTmpSwap.size(); j++ )
				{
					resetPseudoPinDemand( vTmpSwap[j], vTmpSwap[j]->getPlacedX(), vTmpSwap[j]->getPlacedY(), m_nOffsetZ );
					removeInstOnGraph( vTmpSwap[j] );
				}
				bFindAllPlace = false;
				/*	
				for( int n=0; n<vLocalNet.size(); n++ )
					addNetOnGraph( m_pDesign, vLocalNet[n] );
				*/
				for( int n=0; n<vRipNet.size(); n++ )
				{
					if( sRipNet.count( vRipNet[n] ) != 0 )
						addNetOnGraph( m_pDesign, vRipNet[n] );
				}
				break;	
			}
			else
			{
				sUnplacedInst.erase( pInst );	
				/*
				for( int n=0; n<vLocalNet.size(); n++ )
					addNetOnGraph( m_pDesign, vLocalNet[n] );
				*/
			}
		}
// update the nets networkforced model considering unplaced instance that influence by placed instance
		//cout << "Update second"<<endl;
		/*
		vector< networkForced_C* > &vTTmpNF = m_vForced[ pInst->getId() ].m_vNetwork;
		for( int n=0; n<vTTmpNF.size(); n++ )
			calForcedNetwork( *vTTmpNF[n], sUnplacedInst );
		*/
		/*
		for( int g=1; g<=3; g++ )
		{
			gGrid_C* gG = getGrid( m_pDesign, 20, 4, g );
			cout << gG->getRemand() << " ";
			cout << gG->getRemand() - gG->getPinDemand() << " ";
		}
		cout << endl;
		*/
		if( !bFindAllPlace )
			break;
//
	}

	bool bSuccess = false;
	/*
	cout << "Pre "<< endl;
	for( int g=1; g<=1; g++ )
	{
		gGrid_C* gG = getGrid( m_pDesign, 20, 4, g );
		cout << gG->getSupply() << " " << gG->getExtraDemand() << " " << gG->getNet().size() << endl;
		cout << gG->getRemand() << " ";
		cout << gG->getRemand() - gG->getPinDemand() << " ";
	}
	cout << endl;
	*/
	// route the net 
	/*
	for( int i=0; i<vRipNet.size(); i++ )
	{
		//cout << vRipNet[i]->getName() << endl;
		ripupNet( vRipNet[i] );	
	}
	*/
	for( int i=0; i<vNonMovedInst.size(); i++ )
	{
		resetPseudoPinDemand( vNonMovedInst[i], vNonMovedInst[i]->getPlacedX(), vNonMovedInst[i]->getPlacedY(), m_nOffsetZ );
	}
	cleanWire( vRipNet );
	/*
	cout << "Post "<< endl;
	for( int g=1; g<=3; g++ )
	{
		gGrid_C* gG = getGrid( m_pDesign, 20, 4, g );
		cout << gG->getSupply() << " " << gG->getExtraDemand() << " " << gG->getNet().size() << endl;
		cout << gG->getRemand() << " ";
		cout << gG->getRemand() - gG->getPinDemand() << " ";
	}
	cout << endl;
	*/
	if( bFindAllPlace )
	{
		//cout << "route net: "<< vRipNet.size() <<endl;
		for( int i=0; i<vInst.size(); i++ )
		{
			resetPseudoPinDemand( vInst[i], vInst[i]->getPlacedX(), vInst[i]->getPlacedY(), m_nOffsetZ );
		}
		/*
		gGrid_C* pTmpGrid = getGrid( m_pDesign, 20, 4, 1 );
		vector< instance_C* > vGInst = pTmpGrid->getInstance();
		cout << "Instance in 7 5 1:" << endl;
		for( int i=0; i<vGInst.size(); i++ )
		{
			cout << vGInst[i]->getName() << " ";
		}
		cout << endl;
		
		cout << "Record:" << endl;
		for( int i=0; i<vRecordPse.size(); i++ )
		{
			cout << vRecordPse[i]->getName() << " ";
		}
		cout << endl;
		*/
		cout << "Place Result: "<<endl;
		
		for( int i=0; i<vInst.size(); i++ )
		{
			resetPseudoPinDemand( vInst[i], vInst[i]->getPlacedX(), vInst[i]->getPlacedY(), m_nOffsetZ );
		}
		
		for( int i=0; i<vInst.size(); i++ )
		{
			cout << vInst[i]->getName() << " " << vInst[i]->getPlacedX() << " " << vInst[i]->getPlacedY() << endl;
		}
		
		bool bRouteAllNet = true;
		bool bBetterSol = false;
		//record previous result
		int nPreviousLength = 0;
		int nNewLength = 0;
		for( int n=0; n<vRipNet.size(); n++ )
		{
			nPreviousLength = nPreviousLength + vRipNet[n]->getLength();
		}

		// route the net
		int nSuccessCount = 0;
		
		for (int i = 1; i < vRipNet.size(); i++)
		{
			for (int j = i - 1; j >= 0; j--)
			{
				int nLength_b = vRipNet[j + 1]->getLength();
				int nLength_f = vRipNet[j]->getLength();
				if (nLength_f <= nLength_b)
					break;
				else
				{
					net_C *pNet = vRipNet[j + 1];
					vRipNet[j + 1] = vRipNet[j];
					vRipNet[j] = pNet;
				}
			}
		}

		int nLengthConstraint = nPreviousLength;
	
		bRouteAllNet = multiNetRouting( vRipNet, nLengthConstraint, nSuccessCount );

		// check if the new result is better
		if( bRouteAllNet )
		{
			for( int n=0; n<vRipNet.size(); n++ )
			{
				nNewLength = nNewLength + vRipNet[n]->getLength();
			}
			if( nNewLength < nPreviousLength )
			{
				bBetterSol = true;
			}
			else
			{
				bBetterSol = false;
			}
		}
		
		// deal with the routing result
		if( !bRouteAllNet || !bBetterSol )
		{
			if( !bRouteAllNet )
				cout << "Some net failed"<<endl;
			else
			{
				cout << "No better solution in routing "<<endl;
				cout << "Original Length: " << nPreviousLength << " New Length: "<< nNewLength<<endl;
			}
			for( int n=0; n<nSuccessCount; n++ )
			{
				ripupNet( vRipNet[n] );
			}
			cleanWire( vRipNet );
			bSuccess = false;
			for( int i=0; i<vInst.size(); i++ )
				removeInstOnGraph( vInst[i] );
		}
		else //( bRouteAllNet && bBetterSol )
		{	
			bSuccess = true;
		}
	}
	else
	{
		bSuccess = false;
		/*
		for( int i=0; i<vInst.size(); i++ )
			removeInstOnGraph( vInst[i] );
		*/
	}

	if( !bSuccess ) // recover all the information
	{
		//cout << "recover information" << endl;
		recoverInstance( vInst );
		for( int i=0; i<vInst.size(); i++ )
		{
			instance_C* pInst = vInst[i];
			
			if( i==0 )
				lockInstance( pInst );
			putInstOnGraph( pInst, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ );
			resetPseudoPinDemand( pInst, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ );
			//if( pInst->getName() == "C1" )
			//	cout << "C1: "<<pInst->getPlacedX() <<", "<<pInst->getPlacedY() << endl;
			//calForcedModel_ver3( m_vForced[ pInst->getId() ] );
			calForcedModel_ver4( m_vForced[ pInst->getId() ] );
		}
		recoverNet( vRipNet );
		for( int i=0; i<vRipNet.size(); i++ )
		{
			addNetOnGraph( m_pDesign, vRipNet[i] );	
		}
		for( int i=0; i<vInst.size(); i++ )
		{
			updateForcedModel_ver2( vInst[i] );	
		}
	}
	else
	{
		//cout << "update model" << endl;
		for( int i=0; i<vInst.size(); i++ )
		{
			instance_C* pInst = vInst[i];
			if ( !pInst->hasBeenMoved() && !( pInst->getPlacedX() == m_vBackupInstance[i].getPlacedX() && pInst->getPlacedY() == m_vBackupInstance[i].getPlacedY() ) )
			//if ( !pInst->hasBeenMoved() )
			{
				pInst->hasMoved();
				m_vMovedInstance.push_back(pInst);
			}
		
			//m_nSuccess++;
			updateForcedModel_ver2(pInst);
			freeBoundry( pInst );
			freeForcedModel(pInst);
		}
		for( int i=0; i<m_vBackupInstance.size(); i++ )
		{
			instance_C* pInst = &m_vBackupInstance[i];
			resetPseudoPinDemand( pInst, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ );	
		}
		
	}

	for( int i=0; i<m_vBackupNet.size(); i++ )
	{
		m_vBackupNet[i].cleanWire();
	}
	m_vBackupNet.clear();
	m_vBackupInstance.clear();	

	cout << "return  multipleCellMovement result"<<endl;
	return bSuccess;
}

bool router_C::multipleCellMovement_ver3( vector< instance_C* > &vInst )
{	
	cout << "at multipleCellMovement"<<endl;
	int nMoveCount = vInst.size();
	
	vector< instance_C* > vIInst = vInst;
	for( int i=0; i<vIInst.size(); i++ )
	{
		vInst[ nMoveCount - 1 - i ] = vIInst[i];	
	}
	
	vector< net_C* > vRipNet;
	set< net_C* > sNet;
	set< net_C* > sRipNet;
	set< instance_C* > sUnplacedInst;
	for( int i=0; i<vInst.size(); i++ )
	{
		instance_C* pInst = vInst[i];
		if( pInst->hasBeenMoved() )
			nMoveCount--;
		vector< networkForced_C* > &vTmpNF = m_vForced[ pInst->getId() ].m_vNetwork;
		for( int j=0; j<vTmpNF.size(); j++ )
		{
			if( sNet.count( vTmpNF[j]->m_pNet ) == 0 )
			{
				sNet.insert( vTmpNF[j]->m_pNet );
				vRipNet.push_back( vTmpNF[j]->m_pNet );
			}
		}
		sUnplacedInst.insert( pInst );
	}
	//cout << vRipNet.size() << endl;
//
	set< net_C* > sSpecialNet;
	for( int i=0; i<1; i++ )
	{
		instance_C* pInst = vInst[i];
		vector< networkForced_C* > &vTmpNF = m_vForced[ pInst->getId() ].m_vNetwork;
		for( int j=0; j<vTmpNF.size(); j++ )
		{
			if( sSpecialNet.count( vTmpNF[j]->m_pNet ) == 0 )
			{
				sSpecialNet.insert( vTmpNF[j]->m_pNet );
			}
		}
	}
//


	int nSwapCost = m_pDesign->getMaxCellConstraint() - m_vMovedInstance.size() - nMoveCount;

	backupInstance( vInst );
	backupNet( vRipNet );
	//cout << vRipNet.size() << endl;

	for( int i=0; i<vInst.size(); i++ )
	{
		removeInstOnGraph( vInst[i] );
		delPseudoPinDemand( vInst[i], vInst[i]->getPlacedX(), vInst[i]->getPlacedY(), m_nOffsetZ );
		if( i == 0 )
			continue;
		calForcedModel( m_vForced[ vInst[i]->getId() ], sSpecialNet );
	}
	/*
	for( int i=0; i<vRipNet.size(); i++ )
	{
		//cout << vRipNet[i]->getName() << endl;
		ripupNet( vRipNet[i] );	
	}
	cleanWire( vRipNet );
	*/
	//cout << vRipNet.size() << endl;


	bool bFindAllPlace = true;
	vector< instance_C* > vTmpSwap;
	//vector< instance_C > vSwapBackup;
	vector< instance_C* > vPlacedInst;

	//cout << "place instance"<<endl;
	// place the instance
	int nNumInstance = vInst.size();
// update the networkforced model considering unplaced instance
	//cout << "Update first"<<endl;
	/*	
	for( int i=0; i<vRipNet.size(); i++ )
	{
		calForcedNetwork( m_vNetworkForced[ vRipNet[i]->getId() ], sUnplacedInst );
	}
	*/
//
	//cout << "Move Cell"<<endl;
	vector< instance_C* > vRecordPse;
	for( int i=0; i<nNumInstance; i++ )
	{
		instance_C* pInst = vInst[i];
		//cout << "Place "<<pInst->getName() << " ";
		gGrid_C* pOGrid = getGrid( m_pDesign, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ );	
			
// calculate new forced considering ripup nets
		//cout << "Cal"<<endl;
		//calForcedModel_ver2( m_vForced[ pInst->getId() ] );
// //
		
		vector< net_C* > vLocalNet;
		set< net_C* > sLocalNet;
		vector< networkForced_C* > &vTmpNF = m_vForced[ pInst->getId() ].m_vNetwork;
		for( int n=0; n<vTmpNF.size(); n++ )
		{
			if( sRipNet.count( vTmpNF[n]->m_pNet ) == 0 )
			{
				vLocalNet.push_back( vTmpNF[n]->m_pNet );
				sLocalNet.insert( vTmpNF[n]->m_pNet );
				sRipNet.insert( vTmpNF[n]->m_pNet );
			}
		}
		ripupNet( vLocalNet );

		//cout << "Find"<<endl;
		vector< gGrid_C* > vBestGrid = findPlaceToMove_ver4( pInst );		
		//vector< gGrid_C* > vBestGrid = findPlaceToMove_ver3( pInst );		
		//cout << "Choice "<< vBestGrid.size() << endl;
		gGrid_C* pBGrid = NULL;
		if( vBestGrid.size() == 0 )
		{
			//lockInstance( pInst );
			for( int j=0; j<i; j++ )
			{
				resetPseudoPinDemand( vInst[j], vInst[j]->getPlacedX(), vInst[j]->getPlacedY(), m_nOffsetZ );
				removeInstOnGraph( vInst[j] );
			}
			for( int j=0; j<vTmpSwap.size(); j++ )
			{
				resetPseudoPinDemand( vTmpSwap[j], vTmpSwap[j]->getPlacedX(), vTmpSwap[j]->getPlacedY(), m_nOffsetZ );
				removeInstOnGraph( vTmpSwap[j] );
			}
			/*
			for( int n=0; n<vLocalNet.size(); n++ )
				addNetOnGraph( m_pDesign, vLocalNet[n] );
			*/
			for( int n=0; n<vRipNet.size(); n++ )
			{
				if( sRipNet.count( vRipNet[n] ) != 0 )
				{
					addNetOnGraph( m_pDesign, vRipNet[n] );
				}
			}
			bFindAllPlace = false;
			break;
		}
		else
		{
			bool bPlace = false;
			vector< net_C *> vTmpRip;
			for( int b=vBestGrid.size() - 1; b>=0; b-- )
			{
				int nX, nY, nZ;
				pBGrid = vBestGrid[b];
				pBGrid->getPosition( nX, nY, nZ );
				//cout << "Best: " << nX << " " << nY << endl;
				//if( isPlaceable( pBGrid, pInst ) )
				//if( isPlaceable_ver4( pBGrid, pInst, vTmpRip ) )
				if( isPlaceable_ver5( pBGrid, pInst, vTmpRip ) )
				{
					if( vTmpRip.size() != 0 )	
					{
						for( int n=0; n<vTmpRip.size(); n++ )
						{
							if( sNet.count( vTmpRip[n] ) != 0 )
							{
								vTmpRip.erase( vTmpRip.begin() + n );
								n--;
							}
							else
								sNet.insert( vTmpRip[n] );	
						}
						backupNet( vTmpRip );
						for( int n=0; n<vTmpRip.size(); n++ )
						{
							sRipNet.insert( vTmpRip[n] );
							ripupNet( vTmpRip[n] );	
							vRipNet.push_back( vTmpRip[n] );
						}
						//cleanWire( vTmpRip );
					}

					//cout << "Place at: " << nX << ", "<< nY <<", "<< nZ << endl;
					putInstOnGraph( pInst, nX, nY, nZ );
					calPseudoPinDemand( pInst, nX, nY, nZ );
					vRecordPse.push_back( pInst );	
					/*
					if( pInst->getName() == "C1112" )
					{
						cout << "Ripup: " << endl;
						for( int t=0; t<vTmpRip.size(); t++ )
							cout << vTmpRip[t]->getName() << " ";
						cout << endl;
					}
					*/
					bPlace = true;
					sUnplacedInst.erase( pInst );
					break;
				}
				else if( pBGrid != pOGrid && nSwapCost > 0 )
				{
					vector<instance_C *> vSwapInstance = findSwapInstance(pBGrid, pOGrid, pInst);
					for (int s = 0; s < vSwapInstance.size(); s++)
					{
						bool bHasSwap = false;
						for( int t=0; t<vTmpSwap.size(); t++ )
						{
							if( vTmpSwap[t] == vSwapInstance[s] )
							{
								bHasSwap = true;
								break;
							}		
						}

						for( int t=0; t<vInst.size(); t++ )
						{
							if( vInst[t] == vSwapInstance[s] )
							{
								bHasSwap = true;
								break;
							}
						}
						if( bHasSwap )
							continue;

						vTmpRip.clear();
						instance_C *pSInst = vSwapInstance[s];
						instance_C cLocalBackupInstance;
						// local backup pSInst
						cLocalBackupInstance.setName( pSInst->getName() );
						cLocalBackupInstance.setPlaced(pSInst->getPlacedX(), pSInst->getPlacedY());
						removeInstOnGraph(pSInst);
						//if( swapInstance(pBGrid, pOGrid, pInst, pSInst, vTmpRip ) )
						if( swapInstance_ver2(pBGrid, pOGrid, pInst, pSInst, vTmpRip ) )
						{
							//cout << "Swap: "<<pSInst->getName() << " at " << cLocalBackupInstance.getPlacedX() <<", " << cLocalBackupInstance.getPlacedY() << endl;
							if( vTmpRip.size() != 0 )
							{
								for( int n=0; n<vTmpRip.size(); n++ )
								{
									if( sNet.count( vTmpRip[n] ) != 0 )
									{
										vTmpRip.erase( vTmpRip.begin() + n );
										n--;
									}
									else
										sNet.insert( vTmpRip[n] );	
								}
								
								backupNet( vTmpRip );
								for( int n=0; n<vTmpRip.size(); n++ )
								{
									sRipNet.insert( vTmpRip[n] );
									ripupNet( vTmpRip[n] );	
									vRipNet.push_back( vTmpRip[n] );
								}
								//cleanWire( vTmpRip );
								
							}

							int nTmpX, nTmpY, nTmpZ;
							pBGrid->getPosition(nTmpX, nTmpY, nTmpZ);
							putInstOnGraph(pInst, nTmpX, nTmpY, nTmpZ);
							calPseudoPinDemand( pInst, nTmpX, nTmpY, nTmpZ );
					vRecordPse.push_back( pInst );	
							
							pOGrid->getPosition(nTmpX, nTmpY, nTmpZ);
							putInstOnGraph(pSInst, nTmpX, nTmpY, nTmpZ);
							calPseudoPinDemand( pSInst, nTmpX, nTmpY, nTmpZ );
					vRecordPse.push_back( pInst );	
							
							vTmpSwap.push_back( pSInst );
							vInst.push_back( pSInst );
							m_vBackupInstance.push_back( cLocalBackupInstance );
							bPlace = true;
							nSwapCost--;
							cout << "Place at: " << nX << ", "<< nY <<", "<< nZ << endl;
							cout << "Swap from: " << pSInst->getPlacedX() << ", "<< pSInst->getPlacedY() <<", "<< nZ << endl;
							break;
						}
						else
						{
							// recover instance, put instance back on the graph
							int nTmpX, nTmpY, nTmpZ;
							pBGrid->getPosition(nTmpX, nTmpY, nTmpZ);
							putInstOnGraph(pSInst, nTmpX, nTmpY, nTmpZ);
							pOGrid->getPosition( nTmpX, nTmpY, nTmpZ );
							pInst->setPlaced(nTmpX, nTmpY);
						}
					}
					
					if( bPlace )
						break;
				}
				
			}
			if( !bPlace )
			{
				//lockInstance( pInst );
				for( int j=0; j<i; j++ )
				{
					resetPseudoPinDemand( vInst[j], vInst[j]->getPlacedX(), vInst[j]->getPlacedY(), m_nOffsetZ );
					removeInstOnGraph( vInst[j] );
				}
				for( int j=0; j<vTmpSwap.size(); j++ )
				{
					resetPseudoPinDemand( vTmpSwap[j], vTmpSwap[j]->getPlacedX(), vTmpSwap[j]->getPlacedY(), m_nOffsetZ );
					removeInstOnGraph( vTmpSwap[j] );
				}
				bFindAllPlace = false;
				/*	
				for( int n=0; n<vLocalNet.size(); n++ )
					addNetOnGraph( m_pDesign, vLocalNet[n] );
				*/
				for( int n=0; n<vRipNet.size(); n++ )
				{
					if( sRipNet.count( vRipNet[n] ) != 0 )
						addNetOnGraph( m_pDesign, vRipNet[n] );
				}
				break;	
			}
			else
			{
				sUnplacedInst.erase( pInst );	
				/*
				for( int n=0; n<vLocalNet.size(); n++ )
					addNetOnGraph( m_pDesign, vLocalNet[n] );
				*/
			}
		}
// update the nets networkforced model considering unplaced instance that influence by placed instance
		//cout << "Update second"<<endl;
		/*
		vector< networkForced_C* > &vTTmpNF = m_vForced[ pInst->getId() ].m_vNetwork;
		for( int n=0; n<vTTmpNF.size(); n++ )
			calForcedNetwork( *vTTmpNF[n], sUnplacedInst );
		*/
		/*
		for( int g=1; g<=3; g++ )
		{
			gGrid_C* gG = getGrid( m_pDesign, 20, 4, g );
			cout << gG->getRemand() << " ";
			cout << gG->getRemand() - gG->getPinDemand() << " ";
		}
		cout << endl;
		*/
		if( !bFindAllPlace )
			break;
//
	}

	bool bSuccess = false;
	/*
	cout << "Pre "<< endl;
	for( int g=1; g<=1; g++ )
	{
		gGrid_C* gG = getGrid( m_pDesign, 20, 4, g );
		cout << gG->getSupply() << " " << gG->getExtraDemand() << " " << gG->getNet().size() << endl;
		cout << gG->getRemand() << " ";
		cout << gG->getRemand() - gG->getPinDemand() << " ";
	}
	cout << endl;
	*/
	// route the net 
	/*
	for( int i=0; i<vRipNet.size(); i++ )
	{
		//cout << vRipNet[i]->getName() << endl;
		ripupNet( vRipNet[i] );	
	}
	*/
	cleanWire( vRipNet );
	/*
	cout << "Post "<< endl;
	for( int g=1; g<=3; g++ )
	{
		gGrid_C* gG = getGrid( m_pDesign, 20, 4, g );
		cout << gG->getSupply() << " " << gG->getExtraDemand() << " " << gG->getNet().size() << endl;
		cout << gG->getRemand() << " ";
		cout << gG->getRemand() - gG->getPinDemand() << " ";
	}
	cout << endl;
	*/
	if( bFindAllPlace )
	{
		//cout << "route net: "<< vRipNet.size() <<endl;
		for( int i=0; i<vInst.size(); i++ )
		{
			resetPseudoPinDemand( vInst[i], vInst[i]->getPlacedX(), vInst[i]->getPlacedY(), m_nOffsetZ );
		}
		/*
		gGrid_C* pTmpGrid = getGrid( m_pDesign, 20, 4, 1 );
		vector< instance_C* > vGInst = pTmpGrid->getInstance();
		cout << "Instance in 7 5 1:" << endl;
		for( int i=0; i<vGInst.size(); i++ )
		{
			cout << vGInst[i]->getName() << " ";
		}
		cout << endl;
		
		cout << "Record:" << endl;
		for( int i=0; i<vRecordPse.size(); i++ )
		{
			cout << vRecordPse[i]->getName() << " ";
		}
		cout << endl;
		*/
		cout << "Place Result: "<<endl;
		
		for( int i=0; i<vInst.size(); i++ )
		{
			resetPseudoPinDemand( vInst[i], vInst[i]->getPlacedX(), vInst[i]->getPlacedY(), m_nOffsetZ );
		}
		
		for( int i=0; i<vInst.size(); i++ )
		{
			cout << vInst[i]->getName() << " " << vInst[i]->getPlacedX() << " " << vInst[i]->getPlacedY() << endl;
		}
		
		bool bRouteAllNet = true;
		bool bBetterSol = false;
		//record previous result
		int nPreviousLength = 0;
		int nNewLength = 0;
		for( int n=0; n<vRipNet.size(); n++ )
		{
			nPreviousLength = nPreviousLength + vRipNet[n]->getLength();
		}

		// route the net
		int nSuccessCount = 0;
		
		for (int i = 1; i < vRipNet.size(); i++)
		{
			for (int j = i - 1; j >= 0; j--)
			{
				int nLength_b = vRipNet[j + 1]->getLength();
				int nLength_f = vRipNet[j]->getLength();
				if (nLength_f <= nLength_b)
					break;
				else
				{
					net_C *pNet = vRipNet[j + 1];
					vRipNet[j + 1] = vRipNet[j];
					vRipNet[j] = pNet;
				}
			}
		}

		int nLengthConstraint = nPreviousLength;
		for (int r = 0; r < vRipNet.size(); r++)
		{
			//cout << "->route: " << vRipNet[r]->getName() << endl;
			/*
			vector< pin_C* > vTmpPin = vRipNet[r]->getPin();
			for( int p=0; p<vTmpPin.size(); p++ )
			{
				pin_C *pPin = vTmpPin[p];
				int nPX, nPY, nPZ;
				instance_C *pPInst = (instance_C *)pPin->getCell();
				nPX = pPInst->getPlacedX();
				nPY = pPInst->getPlacedY();
				nPZ = pPin->getLayerId();
				gGrid_C* pTGrid = getGrid( m_pDesign, nPX, nPY, nPZ );
				pTGrid->delPinDemand();
			}
			*/
			vector<vector<gGrid_C *>> vResult = routeNet_length_constraint_ver2(vRipNet[r], nLengthConstraint);
			if (vResult.size() == 0)
			{
				bRouteAllNet = false;
				break;
			}
			else
			{
				
				bool bOverflow = false;
				int nOX, nOY, nOZ;
				for (int g = 0; g < vResult.size(); g++)
				{
					for (int gg = 0; gg < vResult[g].size(); gg++)
						if (vResult[g][gg]->getRemand() - 1 < 0)
						{
							vResult[g][gg]->getPosition( nOX, nOY, nOZ );
							bOverflow = true;
							break;
						}
				}
				if (bOverflow)
				{
					cout << "Net " << vRipNet[r]->getName() <<" Routing Result: " << endl;
					int nRX, nRY, nRZ;
					for (int g = 0; g < vResult.size(); g++)
					{
						for (int gg = 0; gg < vResult[g].size(); gg++)
						{
							vResult[g][gg]->getPosition( nRX, nRY, nRZ );
							cout << "(" << nRX << " " << nRY << " " << nRZ << ")  ";
						}
						cout << endl;
					}
					vector< pin_C* > vTmpPin = vRipNet[r]->getPin();
					cout << "Overflow at: " << nOX << " " << nOY << " " << nOZ << endl;
					cout << "Detailed "<< endl;
					for( int g=1; g<=2; g++ )
					{
						gGrid_C* gG = getGrid( m_pDesign, nOX, nOY, g );
						cout << gG->getSupply() << " " << gG->getExtraDemand() << " " << gG->getNet().size() << endl;
						cout << gG->getRemand() << " ";
						cout << gG->getRemand() - gG->getPinDemand() << " ";
					}
					cout << endl;
					cout << "Pin lists: " << endl;
					for( int p=0; p<vTmpPin.size(); p++ )
					{
						int nPX, nPY, nPZ;
						//vTmpPin[p]->getCell()->getPosition( nPX, nPY, nPZ );
						//cout << nPX << " " << nPY << " " << nPZ << endl;
						cout << ((instance_C*)vTmpPin[p]->getCell())->getPlacedX() << " " << ((instance_C*)vTmpPin[p]->getCell())->getPlacedY() << " " << vTmpPin[p]->getLayerId() << endl;
					}
					//getchar();
					vResult.clear();
					bRouteAllNet = false;
					break;
				}
				else
				{
					cout << "Net " << vRipNet[r]->getName() <<" Routing Result: " << endl;
					int nRX, nRY, nRZ;
					for (int g = 0; g < vResult.size(); g++)
					{
						for (int gg = 0; gg < vResult[g].size(); gg++)
						{
							vResult[g][gg]->getPosition( nRX, nRY, nRZ );
							cout << "(" << nRX << " " << nRY << " " << nRZ << ")  ";
						}
						cout << endl;
					}	
				}
				
			}
			


			saveNet(vRipNet[r], vResult);
			//if( vRipNet[r]->getName() == "N4" )
			//	cout << "N4: " <<vRipNet[r]->getLength();
			addNetOnGraph(m_pDesign, vRipNet[r]);
			nLengthConstraint = nLengthConstraint - vRipNet[r]->getLength();
			nSuccessCount++;
		}
		
		// check if the new result is better
		if( bRouteAllNet )
		{
			for( int n=0; n<vRipNet.size(); n++ )
			{
				nNewLength = nNewLength + vRipNet[n]->getLength();
			}
			if( nNewLength < nPreviousLength )
			{
				bBetterSol = true;
			}
			else
			{
				bBetterSol = false;
			}
		}
		
		// deal with the routing result
		if( !bRouteAllNet || !bBetterSol )
		{
			if( !bRouteAllNet )
				cout << "Some net failed"<<endl;
			else
			{
				cout << "No better solution in routing "<<endl;
				cout << "Original Length: " << nPreviousLength << " New Length: "<< nNewLength<<endl;
			}
			for( int n=0; n<nSuccessCount; n++ )
			{
				ripupNet( vRipNet[n] );
			}
			cleanWire( vRipNet );
			bSuccess = false;
			for( int i=0; i<vInst.size(); i++ )
				removeInstOnGraph( vInst[i] );
		}
		else //( bRouteAllNet && bBetterSol )
		{	
			bSuccess = true;
		}
	}
	else
	{
		bSuccess = false;
		/*
		for( int i=0; i<vInst.size(); i++ )
			removeInstOnGraph( vInst[i] );
		*/
	}

	if( !bSuccess ) // recover all the information
	{
		//cout << "recover information" << endl;
		recoverInstance( vInst );
		for( int i=0; i<vInst.size(); i++ )
		{
			instance_C* pInst = vInst[i];
			
			if( i==0 )
				lockInstance( pInst );
			putInstOnGraph( pInst, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ );
			resetPseudoPinDemand( pInst, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ );
			//if( pInst->getName() == "C1" )
			//	cout << "C1: "<<pInst->getPlacedX() <<", "<<pInst->getPlacedY() << endl;
			calForcedModel( m_vForced[ pInst->getId() ] );
		}
		recoverNet( vRipNet );
		for( int i=0; i<vRipNet.size(); i++ )
		{
			addNetOnGraph( m_pDesign, vRipNet[i] );	
		}
		for( int i=0; i<vInst.size(); i++ )
		{
			updateForcedModel( vInst[i] );	
		}
	}
	else
	{
		//cout << "update model" << endl;
		for( int i=0; i<vInst.size(); i++ )
		{
			instance_C* pInst = vInst[i];
			if ( !pInst->hasBeenMoved() && !( pInst->getPlacedX() == m_vBackupInstance[i].getPlacedX() && pInst->getPlacedY() == m_vBackupInstance[i].getPlacedY() ) )
			//if ( !pInst->hasBeenMoved() )
			{
				pInst->hasMoved();
				m_vMovedInstance.push_back(pInst);
			}
		
			//m_nSuccess++;
			updateForcedModel(pInst);
			freeForcedModel(pInst);
		}
		for( int i=0; i<m_vBackupInstance.size(); i++ )
		{
			instance_C* pInst = &m_vBackupInstance[i];
			resetPseudoPinDemand( pInst, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ );	
		}
		
	}

	for( int i=0; i<m_vBackupNet.size(); i++ )
	{
		m_vBackupNet[i].cleanWire();
	}
	m_vBackupNet.clear();
	m_vBackupInstance.clear();	

	cout << "return  multipleCellMovement result"<<endl;
	return bSuccess;
}

bool router_C::multipleCellMovement_ver2( vector< instance_C* > &vInst )
{	
	cout << "at multipleCellMovement"<<endl;
	int nMoveCount = vInst.size();
	
	vector< instance_C* > vIInst = vInst;
	for( int i=0; i<vIInst.size(); i++ )
	{
		vInst[ nMoveCount - 1 - i ] = vIInst[i];	
	}
	
	vector< net_C* > vRipNet;
	set< net_C* > sNet;
	set< instance_C* > sUnplacedInst;
	for( int i=0; i<vInst.size(); i++ )
	{
		instance_C* pInst = vInst[i];
		if( pInst->hasBeenMoved() )
			nMoveCount--;
		vector< networkForced_C* > &vTmpNF = m_vForced[ pInst->getId() ].m_vNetwork;
		for( int j=0; j<vTmpNF.size(); j++ )
		{
			if( sNet.count( vTmpNF[j]->m_pNet ) == 0 )
			{
				sNet.insert( vTmpNF[j]->m_pNet );
				vRipNet.push_back( vTmpNF[j]->m_pNet );
			}
		}
		sUnplacedInst.insert( pInst );
	}
	//cout << vRipNet.size() << endl;
//
	set< net_C* > sSpecialNet;
	for( int i=0; i<1; i++ )
	{
		instance_C* pInst = vInst[i];
		vector< networkForced_C* > &vTmpNF = m_vForced[ pInst->getId() ].m_vNetwork;
		for( int j=0; j<vTmpNF.size(); j++ )
		{
			if( sSpecialNet.count( vTmpNF[j]->m_pNet ) == 0 )
			{
				sSpecialNet.insert( vTmpNF[j]->m_pNet );
			}
		}
	}
//


	int nSwapCost = m_pDesign->getMaxCellConstraint() - m_vMovedInstance.size() - nMoveCount;

	backupInstance( vInst );
	backupNet( vRipNet );
	//cout << vRipNet.size() << endl;

	for( int i=0; i<vInst.size(); i++ )
	{
		removeInstOnGraph( vInst[i] ); 
		if( i == 0 )
			continue;
		calForcedModel( m_vForced[ vInst[i]->getId() ], sSpecialNet );
	}
	/*
	for( int i=0; i<vRipNet.size(); i++ )
	{
		//cout << vRipNet[i]->getName() << endl;
		ripupNet( vRipNet[i] );	
	}
	cleanWire( vRipNet );
	*/
	//cout << vRipNet.size() << endl;


	bool bFindAllPlace = true;
	vector< instance_C* > vTmpSwap;
	//vector< instance_C > vSwapBackup;
	vector< instance_C* > vPlacedInst;

	//cout << "place instance"<<endl;
	// place the instance
	int nNumInstance = vInst.size();
// update the networkforced model considering unplaced instance
	//cout << "Update first"<<endl;
	/*
	for( int i=0; i<vRipNet.size(); i++ )
	{
		calForcedNetwork( m_vNetworkForced[ vRipNet[i]->getId() ], sUnplacedInst );
	}
	*/
//
	//cout << "Move Cell"<<endl;
	vector< instance_C* > vRecordPse;
	for( int i=0; i<nNumInstance; i++ )
	{
		instance_C* pInst = vInst[i];
		//cout << "Place "<<pInst->getName() << " ";
		gGrid_C* pOGrid = getGrid( m_pDesign, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ );	
			
// calculate new forced considering ripup nets
		//cout << "Cal"<<endl;
		//calForcedModel_ver2( m_vForced[ pInst->getId() ] );
// //
		
		vector< net_C* > vLocalNet;
		set< net_C* > sLocalNet;
		vector< networkForced_C* > &vTmpNF = m_vForced[ pInst->getId() ].m_vNetwork;
		for( int n=0; n<vTmpNF.size(); n++ )
		{
			vLocalNet.push_back( vTmpNF[n]->m_pNet );
			sLocalNet.insert( vTmpNF[n]->m_pNet );
		}
		ripupNet( vLocalNet );

		//cout << "Find"<<endl;
		vector< gGrid_C* > vBestGrid = findPlaceToMove_ver4( pInst );		
		//vector< gGrid_C* > vBestGrid = findPlaceToMove_ver3( pInst );		
		//cout << "Choice "<< vBestGrid.size() << endl;
		gGrid_C* pBGrid = NULL;
		if( vBestGrid.size() == 0 )
		{
			//lockInstance( pInst );
			for( int j=0; j<i; j++ )
			{
				resetPseudoPinDemand( vInst[j], vInst[j]->getPlacedX(), vInst[j]->getPlacedY(), m_nOffsetZ );
				removeInstOnGraph( vInst[j] );
			}
			for( int j=0; j<vTmpSwap.size(); j++ )
			{
				resetPseudoPinDemand( vTmpSwap[j], vTmpSwap[j]->getPlacedX(), vTmpSwap[j]->getPlacedY(), m_nOffsetZ );
				removeInstOnGraph( vTmpSwap[j] );
			}

			for( int n=0; n<vLocalNet.size(); n++ )
				addNetOnGraph( m_pDesign, vLocalNet[n] );

			bFindAllPlace = false;
			break;
		}
		else
		{
			bool bPlace = false;
			vector< net_C *> vTmpRip;
			for( int b=vBestGrid.size() - 1; b>=0; b-- )
			{
				int nX, nY, nZ;
				pBGrid = vBestGrid[b];
				pBGrid->getPosition( nX, nY, nZ );
				//cout << "Best: " << nX << " " << nY << endl;
				//if( isPlaceable( pBGrid, pInst ) )
				//if( isPlaceable_ver4( pBGrid, pInst, vTmpRip ) )
				if( isPlaceable_ver5( pBGrid, pInst, vTmpRip ) )
				{
					if( vTmpRip.size() != 0 )	
					{
						for( int n=0; n<vTmpRip.size(); n++ )
						{
							if( sNet.count( vTmpRip[n] ) != 0 )
							{
								vTmpRip.erase( vTmpRip.begin() + n );
								n--;
							}
							else
								sNet.insert( vTmpRip[n] );	
						}
						backupNet( vTmpRip );
						for( int n=0; n<vTmpRip.size(); n++ )
						{
						//	ripupNet( vTmpRip[n] );	
							vRipNet.push_back( vTmpRip[n] );
						}
						//cleanWire( vTmpRip );
					}

					//cout << "Place at: " << nX << ", "<< nY <<", "<< nZ << endl;
					putInstOnGraph( pInst, nX, nY, nZ );
					calPseudoPinDemand( pInst, nX, nY, nZ );
					vRecordPse.push_back( pInst );	

					if( pInst->getName() == "C1112" )
					{
						cout << "Ripup: " << endl;
						for( int t=0; t<vTmpRip.size(); t++ )
							cout << vTmpRip[t]->getName() << " ";
						cout << endl;
					}
					bPlace = true;
					sUnplacedInst.erase( pInst );
					break;
				}
				else if( pBGrid != pOGrid && nSwapCost > 0 )
				{
					vector<instance_C *> vSwapInstance = findSwapInstance(pBGrid, pOGrid, pInst);
					for (int s = 0; s < vSwapInstance.size(); s++)
					{
						bool bHasSwap = false;
						for( int t=0; t<vTmpSwap.size(); t++ )
						{
							if( vTmpSwap[t] == vSwapInstance[s] )
							{
								bHasSwap = true;
								break;
							}		
						}

						for( int t=0; t<vInst.size(); t++ )
						{
							if( vInst[t] == vSwapInstance[s] )
							{
								bHasSwap = true;
								break;
							}
						}
						if( bHasSwap )
							continue;

						vTmpRip.clear();
						instance_C *pSInst = vSwapInstance[s];
						instance_C cLocalBackupInstance;
						// local backup pSInst
						cLocalBackupInstance.setName( pSInst->getName() );
						cLocalBackupInstance.setPlaced(pSInst->getPlacedX(), pSInst->getPlacedY());
						removeInstOnGraph(pSInst);
						//if( swapInstance(pBGrid, pOGrid, pInst, pSInst, vTmpRip ) )
						if( swapInstance_ver2(pBGrid, pOGrid, pInst, pSInst, vTmpRip ) )
						{
							//cout << "Swap: "<<pSInst->getName() << " at " << cLocalBackupInstance.getPlacedX() <<", " << cLocalBackupInstance.getPlacedY() << endl;
							if( vTmpRip.size() != 0 )
							{
								for( int n=0; n<vTmpRip.size(); n++ )
								{
									if( sNet.count( vTmpRip[n] ) != 0 )
									{
										vTmpRip.erase( vTmpRip.begin() + n );
										n--;
									}
									else
										sNet.insert( vTmpRip[n] );	
								}
								
								backupNet( vTmpRip );
								for( int n=0; n<vTmpRip.size(); n++ )
								{
								//	ripupNet( vTmpRip[n] );	
									vRipNet.push_back( vTmpRip[n] );
								}
								//cleanWire( vTmpRip );
								
							}

							int nTmpX, nTmpY, nTmpZ;
							pBGrid->getPosition(nTmpX, nTmpY, nTmpZ);
							putInstOnGraph(pInst, nTmpX, nTmpY, nTmpZ);
							calPseudoPinDemand( pInst, nTmpX, nTmpY, nTmpZ );
					vRecordPse.push_back( pInst );	
							
							pOGrid->getPosition(nTmpX, nTmpY, nTmpZ);
							putInstOnGraph(pSInst, nTmpX, nTmpY, nTmpZ);
							calPseudoPinDemand( pSInst, nTmpX, nTmpY, nTmpZ );
					vRecordPse.push_back( pInst );	
							
							vTmpSwap.push_back( pSInst );
							vInst.push_back( pSInst );
							m_vBackupInstance.push_back( cLocalBackupInstance );
							bPlace = true;
							nSwapCost--;
							cout << "Place at: " << nX << ", "<< nY <<", "<< nZ << endl;
							cout << "Swap from: " << pSInst->getPlacedX() << ", "<< pSInst->getPlacedY() <<", "<< nZ << endl;
							break;
						}
						else
						{
							// recover instance, put instance back on the graph
							int nTmpX, nTmpY, nTmpZ;
							pBGrid->getPosition(nTmpX, nTmpY, nTmpZ);
							putInstOnGraph(pSInst, nTmpX, nTmpY, nTmpZ);
							pOGrid->getPosition( nTmpX, nTmpY, nTmpZ );
							pInst->setPlaced(nTmpX, nTmpY);
						}
					}
					
					if( bPlace )
						break;
				}
				
			}
			if( !bPlace )
			{
				//lockInstance( pInst );
				for( int j=0; j<i; j++ )
				{
					resetPseudoPinDemand( vInst[j], vInst[j]->getPlacedX(), vInst[j]->getPlacedY(), m_nOffsetZ );
					removeInstOnGraph( vInst[j] );
				}
				for( int j=0; j<vTmpSwap.size(); j++ )
				{
					resetPseudoPinDemand( vTmpSwap[j], vTmpSwap[j]->getPlacedX(), vTmpSwap[j]->getPlacedY(), m_nOffsetZ );
					removeInstOnGraph( vTmpSwap[j] );
				}
				bFindAllPlace = false;
				/*
				for( int n=0; n<vLocalNet.size(); n++ )
					addNetOnGraph( m_pDesign, vLocalNet[n] );
				*/
				break;	
			}
			else
			{
				sUnplacedInst.erase( pInst );	
				
				for( int n=0; n<vLocalNet.size(); n++ )
					addNetOnGraph( m_pDesign, vLocalNet[n] );
				
			}
		}
// update the nets networkforced model considering unplaced instance that influence by placed instance
		//cout << "Update second"<<endl;
		/*
		vector< networkForced_C* > &vTmpNF = m_vForced[ pInst->getId() ].m_vNetwork;
		for( int n=0; n<vTmpNF.size(); n++ )
			calForcedNetwork( *vTmpNF[n], sUnplacedInst );
		*/
		for( int g=1; g<=3; g++ )
		{
			gGrid_C* gG = getGrid( m_pDesign, 20, 4, g );
			cout << gG->getRemand() << " ";
			cout << gG->getRemand() - gG->getPinDemand() << " ";
		}
		cout << endl;
		if( !bFindAllPlace )
			break;
//
	}

	bool bSuccess = false;
	cout << "Pre "<< endl;
	for( int g=1; g<=1; g++ )
	{
		gGrid_C* gG = getGrid( m_pDesign, 20, 4, g );
		cout << gG->getSupply() << " " << gG->getExtraDemand() << " " << gG->getNet().size() << endl;
		cout << gG->getRemand() << " ";
		cout << gG->getRemand() - gG->getPinDemand() << " ";
	}
	cout << endl;
	// route the net 
	
	for( int i=0; i<vRipNet.size(); i++ )
	{
		//cout << vRipNet[i]->getName() << endl;
		ripupNet( vRipNet[i] );	
	}
	cleanWire( vRipNet );
	
	cout << "Post "<< endl;
	for( int g=1; g<=3; g++ )
	{
		gGrid_C* gG = getGrid( m_pDesign, 20, 4, g );
		cout << gG->getSupply() << " " << gG->getExtraDemand() << " " << gG->getNet().size() << endl;
		cout << gG->getRemand() << " ";
		cout << gG->getRemand() - gG->getPinDemand() << " ";
	}
	cout << endl;
	if( bFindAllPlace )
	{
		//cout << "route net: "<< vRipNet.size() <<endl;
		gGrid_C* pTmpGrid = getGrid( m_pDesign, 20, 4, 1 );
		vector< instance_C* > vGInst = pTmpGrid->getInstance();
		cout << "Instance in 7 5 1:" << endl;
		for( int i=0; i<vGInst.size(); i++ )
		{
			cout << vGInst[i]->getName() << " ";
		}
		cout << endl;
		
		cout << "Record:" << endl;
		for( int i=0; i<vRecordPse.size(); i++ )
		{
			cout << vRecordPse[i]->getName() << " ";
		}
		cout << endl;

		cout << "Place Result: "<<endl;
		
		for( int i=0; i<vInst.size(); i++ )
		{
			resetPseudoPinDemand( vInst[i], vInst[i]->getPlacedX(), vInst[i]->getPlacedY(), m_nOffsetZ );
		}
		
		for( int i=0; i<vInst.size(); i++ )
		{
			cout << vInst[i]->getName() << " " << vInst[i]->getPlacedX() << " " << vInst[i]->getPlacedY() << endl;
		}
		
		bool bRouteAllNet = true;
		bool bBetterSol = false;
		//record previous result
		int nPreviousLength = 0;
		int nNewLength = 0;
		for( int n=0; n<vRipNet.size(); n++ )
		{
			nPreviousLength = nPreviousLength + vRipNet[n]->getLength();
		}

		// route the net
		int nSuccessCount = 0;
		
		for (int i = 1; i < vRipNet.size(); i++)
		{
			for (int j = i - 1; j >= 0; j--)
			{
				int nLength_b = vRipNet[j + 1]->getLength();
				int nLength_f = vRipNet[j]->getLength();
				if (nLength_f <= nLength_b)
					break;
				else
				{
					net_C *pNet = vRipNet[j + 1];
					vRipNet[j + 1] = vRipNet[j];
					vRipNet[j] = pNet;
				}
			}
		}

		int nLengthConstraint = nPreviousLength;
		for (int r = 0; r < vRipNet.size(); r++)
		{
			//cout << "->route: " << vRipNet[r]->getName() << endl;
			/*
			vector< pin_C* > vTmpPin = vRipNet[r]->getPin();
			for( int p=0; p<vTmpPin.size(); p++ )
			{
				pin_C *pPin = vTmpPin[p];
				int nPX, nPY, nPZ;
				instance_C *pPInst = (instance_C *)pPin->getCell();
				nPX = pPInst->getPlacedX();
				nPY = pPInst->getPlacedY();
				nPZ = pPin->getLayerId();
				gGrid_C* pTGrid = getGrid( m_pDesign, nPX, nPY, nPZ );
				pTGrid->delPinDemand();
			}
			*/
			vector<vector<gGrid_C *>> vResult = routeNet_length_constraint_ver2(vRipNet[r], nLengthConstraint);
			if (vResult.size() == 0)
			{
				bRouteAllNet = false;
				break;
			}
			else
			{
				
				bool bOverflow = false;
				int nOX, nOY, nOZ;
				for (int g = 0; g < vResult.size(); g++)
				{
					for (int gg = 0; gg < vResult[g].size(); gg++)
						if (vResult[g][gg]->getRemand() - 1 < 0)
						{
							vResult[g][gg]->getPosition( nOX, nOY, nOZ );
							bOverflow = true;
							break;
						}
				}
				if (bOverflow)
				{
					cout << "Net " << vRipNet[r]->getName() <<" Routing Result: " << endl;
					int nRX, nRY, nRZ;
					for (int g = 0; g < vResult.size(); g++)
					{
						for (int gg = 0; gg < vResult[g].size(); gg++)
						{
							vResult[g][gg]->getPosition( nRX, nRY, nRZ );
							cout << "(" << nRX << " " << nRY << " " << nRZ << ")  ";
						}
						cout << endl;
					}
					vector< pin_C* > vTmpPin = vRipNet[r]->getPin();
					cout << "Overflow at: " << nOX << " " << nOY << " " << nOZ << endl;
					cout << "Detailed "<< endl;
					for( int g=1; g<=2; g++ )
					{
						gGrid_C* gG = getGrid( m_pDesign, nOX, nOY, g );
						cout << gG->getSupply() << " " << gG->getExtraDemand() << " " << gG->getNet().size() << endl;
						cout << gG->getRemand() << " ";
						cout << gG->getRemand() - gG->getPinDemand() << " ";
					}
					cout << endl;
					cout << "Pin lists: " << endl;
					for( int p=0; p<vTmpPin.size(); p++ )
					{
						int nPX, nPY, nPZ;
						//vTmpPin[p]->getCell()->getPosition( nPX, nPY, nPZ );
						//cout << nPX << " " << nPY << " " << nPZ << endl;
						cout << ((instance_C*)vTmpPin[p]->getCell())->getPlacedX() << " " << ((instance_C*)vTmpPin[p]->getCell())->getPlacedY() << " " << vTmpPin[p]->getLayerId() << endl;
					}
					//getchar();
					vResult.clear();
					bRouteAllNet = false;
					break;
				}
				else
				{
					cout << "Net " << vRipNet[r]->getName() <<" Routing Result: " << endl;
					int nRX, nRY, nRZ;
					for (int g = 0; g < vResult.size(); g++)
					{
						for (int gg = 0; gg < vResult[g].size(); gg++)
						{
							vResult[g][gg]->getPosition( nRX, nRY, nRZ );
							cout << "(" << nRX << " " << nRY << " " << nRZ << ")  ";
						}
						cout << endl;
					}	
				}
				
			}
			


			saveNet(vRipNet[r], vResult);
			//if( vRipNet[r]->getName() == "N4" )
			//	cout << "N4: " <<vRipNet[r]->getLength();
			addNetOnGraph(m_pDesign, vRipNet[r]);
			nLengthConstraint = nLengthConstraint - vRipNet[r]->getLength();
			nSuccessCount++;
		}
		
		// check if the new result is better
		if( bRouteAllNet )
		{
			for( int n=0; n<vRipNet.size(); n++ )
			{
				nNewLength = nNewLength + vRipNet[n]->getLength();
			}
			if( nNewLength < nPreviousLength )
			{
				bBetterSol = true;
			}
			else
			{
				bBetterSol = false;
			}
		}
		
		// deal with the routing result
		if( !bRouteAllNet || !bBetterSol )
		{
			if( !bRouteAllNet )
				cout << "Some net failed"<<endl;
			else
			{
				cout << "No better solution in routing "<<endl;
				cout << "Original Length: " << nPreviousLength << " New Length: "<< nNewLength<<endl;
			}
			for( int n=0; n<nSuccessCount; n++ )
			{
				ripupNet( vRipNet[n] );
			}
			cleanWire( vRipNet );
			bSuccess = false;
			for( int i=0; i<vInst.size(); i++ )
				removeInstOnGraph( vInst[i] );
		}
		else //( bRouteAllNet && bBetterSol )
		{	
			bSuccess = true;
		}
	}
	else
	{
		bSuccess = false;
		/*
		for( int i=0; i<vInst.size(); i++ )
			removeInstOnGraph( vInst[i] );
		*/
	}

	if( !bSuccess ) // recover all the information
	{
		//cout << "recover information" << endl;
		recoverInstance( vInst );
		for( int i=0; i<vInst.size(); i++ )
		{
			instance_C* pInst = vInst[i];
			if( i==0 )
				lockInstance( pInst );
			putInstOnGraph( pInst, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ );
			//if( pInst->getName() == "C1" )
			//	cout << "C1: "<<pInst->getPlacedX() <<", "<<pInst->getPlacedY() << endl;
			calForcedModel( m_vForced[ pInst->getId() ] );
		}
		recoverNet( vRipNet );
		for( int i=0; i<vRipNet.size(); i++ )
		{
			addNetOnGraph( m_pDesign, vRipNet[i] );	
		}
		for( int i=0; i<vInst.size(); i++ )
		{
			updateForcedModel( vInst[i] );	
		}
	}
	else
	{
		//cout << "update model" << endl;
		for( int i=0; i<vInst.size(); i++ )
		{
			instance_C* pInst = vInst[i];
			if ( !pInst->hasBeenMoved() && !( pInst->getPlacedX() == m_vBackupInstance[i].getPlacedX() && pInst->getPlacedY() == m_vBackupInstance[i].getPlacedY() ) )
			//if ( !pInst->hasBeenMoved() )
			{
				pInst->hasMoved();
				m_vMovedInstance.push_back(pInst);
			}
		
			//m_nSuccess++;
			updateForcedModel(pInst);
			freeForcedModel(pInst);
		}
		
	}

	for( int i=0; i<m_vBackupNet.size(); i++ )
	{
		m_vBackupNet[i].cleanWire();
	}
	for( int i=0; i<vInst.size(); i++ )
	{
		resetPseudoPinDemand( vInst[i], vInst[i]->getPlacedX(), vInst[i]->getPlacedY(), m_nOffsetZ );
	}
	m_vBackupNet.clear();
	m_vBackupInstance.clear();	

	cout << "return  multipleCellMovement result"<<endl;
	return bSuccess;
}

bool router_C::multipleCellMovement( vector< instance_C* > &vInst )
{	
	cout << "at multipleCellMovement"<<endl;
	int nMoveCount = vInst.size();
	
	vector< instance_C* > vIInst = vInst;
	for( int i=0; i<vIInst.size(); i++ )
	{
		vInst[ nMoveCount - 1 - i ] = vIInst[i];	
	}
	
	vector< net_C* > vRipNet;
	set< net_C* > sNet;
	set< instance_C* > sUnplacedInst;
	for( int i=0; i<vInst.size(); i++ )
	{
		instance_C* pInst = vInst[i];
		if( pInst->hasBeenMoved() )
			nMoveCount--;
		vector< networkForced_C* > &vTmpNF = m_vForced[ pInst->getId() ].m_vNetwork;
		for( int j=0; j<vTmpNF.size(); j++ )
		{
			if( sNet.count( vTmpNF[j]->m_pNet ) == 0 )
			{
				sNet.insert( vTmpNF[j]->m_pNet );
				vRipNet.push_back( vTmpNF[j]->m_pNet );
			}
		}
		sUnplacedInst.insert( pInst );
	}
	//cout << vRipNet.size() << endl;
//
	set< net_C* > sSpecialNet;
	for( int i=0; i<1; i++ )
	{
		instance_C* pInst = vInst[i];
		vector< networkForced_C* > &vTmpNF = m_vForced[ pInst->getId() ].m_vNetwork;
		for( int j=0; j<vTmpNF.size(); j++ )
		{
			if( sSpecialNet.count( vTmpNF[j]->m_pNet ) == 0 )
			{
				sSpecialNet.insert( vTmpNF[j]->m_pNet );
			}
		}
	}
//


	int nSwapCost = m_pDesign->getMaxCellConstraint() - m_vMovedInstance.size() - nMoveCount;

	backupInstance( vInst );
	backupNet( vRipNet );
	//cout << vRipNet.size() << endl;

	for( int i=0; i<vInst.size(); i++ )
	{
		removeInstOnGraph( vInst[i] ); 
		if( i == 0 )
			continue;
		calForcedModel( m_vForced[ vInst[i]->getId() ], sSpecialNet );
	}
	for( int i=0; i<vRipNet.size(); i++ )
	{
		//cout << vRipNet[i]->getName() << endl;
		ripupNet( vRipNet[i] );	
	}
	cleanWire( vRipNet );
	//cout << vRipNet.size() << endl;


	bool bFindAllPlace = true;
	vector< instance_C* > vTmpSwap;
	//vector< instance_C > vSwapBackup;
	vector< instance_C* > vPlacedInst;

	//cout << "place instance"<<endl;
	// place the instance
	int nNumInstance = vInst.size();
// update the networkforced model considering unplaced instance
	//cout << "Update first"<<endl;
	/*
	for( int i=0; i<vRipNet.size(); i++ )
	{
		calForcedNetwork( m_vNetworkForced[ vRipNet[i]->getId() ], sUnplacedInst );
	}
	*/
//
	//cout << "Move Cell"<<endl;
	for( int i=0; i<nNumInstance; i++ )
	{
		instance_C* pInst = vInst[i];
		//cout << "Place "<<pInst->getName() << " ";
		gGrid_C* pOGrid = getGrid( m_pDesign, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ );	

// calculate new forced considering ripup nets
		//cout << "Cal"<<endl;
		//calForcedModel_ver2( m_vForced[ pInst->getId() ] );
// //
		//cout << "Find"<<endl;
		vector< gGrid_C* > vBestGrid = findPlaceToMove_ver4( pInst );		
		//vector< gGrid_C* > vBestGrid = findPlaceToMove_ver3( pInst );		
		//cout << "Choice "<< vBestGrid.size() << endl;
		gGrid_C* pBGrid = NULL;
		if( vBestGrid.size() == 0 )
		{
			//lockInstance( pInst );
			for( int j=0; j<i; j++ )
			{
				resetPseudoPinDemand( vInst[j], vInst[j]->getPlacedX(), vInst[j]->getPlacedY(), m_nOffsetZ );
				removeInstOnGraph( vInst[j] );
			}
			for( int j=0; j<vTmpSwap.size(); j++ )
			{
				resetPseudoPinDemand( vTmpSwap[j], vTmpSwap[j]->getPlacedX(), vTmpSwap[j]->getPlacedY(), m_nOffsetZ );
				removeInstOnGraph( vTmpSwap[j] );
			}
			bFindAllPlace = false;
			break;
		}
		else
		{
			bool bPlace = false;
			vector< net_C *> vTmpRip;
			for( int b=vBestGrid.size() - 1; b>=0; b-- )
			{
				int nX, nY, nZ;
				pBGrid = vBestGrid[b];
				pBGrid->getPosition( nX, nY, nZ );
				//cout << "Best: " << nX << " " << nY << endl;
				//if( isPlaceable( pBGrid, pInst ) )
				//if( isPlaceable_ver4( pBGrid, pInst, vTmpRip ) )
				if( isPlaceable_ver5( pBGrid, pInst, vTmpRip ) )
				{
					if( vTmpRip.size() != 0 )	
					{
						for( int n=0; n<vTmpRip.size(); n++ )
						{
							if( sNet.count( vTmpRip[n] ) != 0 )
							{
								vTmpRip.erase( vTmpRip.begin() + n );
								n--;
							}
							else
								sNet.insert( vTmpRip[n] );	
						}
						backupNet( vTmpRip );
						for( int n=0; n<vTmpRip.size(); n++ )
						{
							ripupNet( vTmpRip[n] );	
							vRipNet.push_back( vTmpRip[n] );
						}
						cleanWire( vTmpRip );
					}

					//cout << "Place at: " << nX << ", "<< nY <<", "<< nZ << endl;
					putInstOnGraph( pInst, nX, nY, nZ );
					calPseudoPinDemand( pInst, nX, nY, nZ );
					
					bPlace = true;
					sUnplacedInst.erase( pInst );
					break;
				}
				else if( pBGrid != pOGrid && nSwapCost > 0 )
				{
					vector<instance_C *> vSwapInstance = findSwapInstance(pBGrid, pOGrid, pInst);
					for (int s = 0; s < vSwapInstance.size(); s++)
					{
						bool bHasSwap = false;
						for( int t=0; t<vTmpSwap.size(); t++ )
						{
							if( vTmpSwap[t] == vSwapInstance[s] )
							{
								bHasSwap = true;
								break;
							}		
						}

						for( int t=0; t<vInst.size(); t++ )
						{
							if( vInst[t] == vSwapInstance[s] )
							{
								bHasSwap = true;
								break;
							}
						}
						if( bHasSwap )
							continue;

						vTmpRip.clear();
						instance_C *pSInst = vSwapInstance[s];
						instance_C cLocalBackupInstance;
						// local backup pSInst
						cLocalBackupInstance.setName( pSInst->getName() );
						cLocalBackupInstance.setPlaced(pSInst->getPlacedX(), pSInst->getPlacedY());
						removeInstOnGraph(pSInst);
						//if( swapInstance(pBGrid, pOGrid, pInst, pSInst, vTmpRip ) )
						if( swapInstance_ver2(pBGrid, pOGrid, pInst, pSInst, vTmpRip ) )
						{
							//cout << "Swap: "<<pSInst->getName() << " at " << cLocalBackupInstance.getPlacedX() <<", " << cLocalBackupInstance.getPlacedY() << endl;
							if( vTmpRip.size() != 0 )
							{
								for( int n=0; n<vTmpRip.size(); n++ )
								{
									if( sNet.count( vTmpRip[n] ) != 0 )
									{
										vTmpRip.erase( vTmpRip.begin() + n );
										n--;
									}
									else
										sNet.insert( vTmpRip[n] );	
								}
								backupNet( vTmpRip );
								for( int n=0; n<vTmpRip.size(); n++ )
								{
									ripupNet( vTmpRip[n] );	
									vRipNet.push_back( vTmpRip[n] );
								}
								cleanWire( vTmpRip );
							}

							int nTmpX, nTmpY, nTmpZ;
							pBGrid->getPosition(nTmpX, nTmpY, nTmpZ);
							putInstOnGraph(pInst, nTmpX, nTmpY, nTmpZ);
							calPseudoPinDemand( pInst, nTmpX, nTmpY, nTmpZ );
							
							pOGrid->getPosition(nTmpX, nTmpY, nTmpZ);
							putInstOnGraph(pSInst, nTmpX, nTmpY, nTmpZ);
							calPseudoPinDemand( pSInst, nTmpX, nTmpY, nTmpZ );
							
							vTmpSwap.push_back( pSInst );
							vInst.push_back( pSInst );
							m_vBackupInstance.push_back( cLocalBackupInstance );
							bPlace = true;
							nSwapCost--;
							cout << "Place at: " << nX << ", "<< nY <<", "<< nZ << endl;
							cout << "Swap from: " << pSInst->getPlacedX() << ", "<< pSInst->getPlacedY() <<", "<< nZ << endl;
							break;
						}
						else
						{
							// recover instance, put instance back on the graph
							int nTmpX, nTmpY, nTmpZ;
							pBGrid->getPosition(nTmpX, nTmpY, nTmpZ);
							putInstOnGraph(pSInst, nTmpX, nTmpY, nTmpZ);
							pOGrid->getPosition( nTmpX, nTmpY, nTmpZ );
							pInst->setPlaced(nTmpX, nTmpY);
						}
					}
					if( bPlace )
						break;
				}
			}
			if( !bPlace )
			{
				//lockInstance( pInst );
				for( int j=0; j<i; j++ )
				{
					resetPseudoPinDemand( vInst[j], vInst[j]->getPlacedX(), vInst[j]->getPlacedY(), m_nOffsetZ );
					removeInstOnGraph( vInst[j] );
				}
				for( int j=0; j<vTmpSwap.size(); j++ )
				{
					resetPseudoPinDemand( vTmpSwap[j], vTmpSwap[j]->getPlacedX(), vTmpSwap[j]->getPlacedY(), m_nOffsetZ );
					removeInstOnGraph( vTmpSwap[j] );
				}
				bFindAllPlace = false;
				break;	
			}
			else
				sUnplacedInst.erase( pInst );
		}
// update the nets networkforced model considering unplaced instance that influence by placed instance
		//cout << "Update second"<<endl;
		/*
		vector< networkForced_C* > &vTmpNF = m_vForced[ pInst->getId() ].m_vNetwork;
		for( int n=0; n<vTmpNF.size(); n++ )
			calForcedNetwork( *vTmpNF[n], sUnplacedInst );
		*/
		for( int g=1; g<=3; g++ )
		{
			gGrid_C* gG = getGrid( m_pDesign, 8, 5, g );
			cout << gG->getRemand() << " ";
			cout << gG->getRemand() - gG->getPinDemand() << " ";
		}
		cout << endl;
		if( !bFindAllPlace )
			break;
//
	}

	bool bSuccess = false;
	// route the net 
	if( bFindAllPlace )
	{
		//cout << "route net: "<< vRipNet.size() <<endl;
		gGrid_C* pTmpGrid = getGrid( m_pDesign, 8, 5, 1 );
		vector< instance_C* > vGInst = pTmpGrid->getInstance();
		cout << "Instance in 8 5 1:" << endl;
		for( int i=0; i<vGInst.size(); i++ )
		{
			cout << vGInst[i]->getName() << " ";
		}
		cout << endl;

		cout << "Place Result: "<<endl;
		for( int i=0; i<vInst.size(); i++ )
		{
			resetPseudoPinDemand( vInst[i], vInst[i]->getPlacedX(), vInst[i]->getPlacedY(), m_nOffsetZ );
		}

		for( int i=0; i<vInst.size(); i++ )
		{
			cout << vInst[i]->getName() << " " << vInst[i]->getPlacedX() << " " << vInst[i]->getPlacedY() << endl;
		}
		
		bool bRouteAllNet = true;
		bool bBetterSol = false;
		//record previous result
		int nPreviousLength = 0;
		int nNewLength = 0;
		for( int n=0; n<vRipNet.size(); n++ )
		{
			nPreviousLength = nPreviousLength + vRipNet[n]->getLength();
		}

		// route the net
		int nSuccessCount = 0;
		
		for (int i = 1; i < vRipNet.size(); i++)
		{
			for (int j = i - 1; j >= 0; j--)
			{
				int nLength_b = vRipNet[j + 1]->getLength();
				int nLength_f = vRipNet[j]->getLength();
				if (nLength_f <= nLength_b)
					break;
				else
				{
					net_C *pNet = vRipNet[j + 1];
					vRipNet[j + 1] = vRipNet[j];
					vRipNet[j] = pNet;
				}
			}
		}

		int nLengthConstraint = nPreviousLength;
		for (int r = 0; r < vRipNet.size(); r++)
		{
			//cout << "->route: " << vRipNet[r]->getName() << endl;
			vector<vector<gGrid_C *>> vResult = routeNet_length_constraint_ver2(vRipNet[r], nLengthConstraint);
			if (vResult.size() == 0)
			{
				bRouteAllNet = false;
				break;
			}
			else
			{
				
				bool bOverflow = false;
				int nOX, nOY, nOZ;
				for (int g = 0; g < vResult.size(); g++)
				{
					for (int gg = 0; gg < vResult[g].size(); gg++)
						if (vResult[g][gg]->getRemand() - 1 < 0)
						{
							vResult[g][gg]->getPosition( nOX, nOY, nOZ );
							bOverflow = true;
							break;
						}
				}
				if (bOverflow)
				{
					cout << "Net " << vRipNet[r]->getName() <<" Routing Result: " << endl;
					int nRX, nRY, nRZ;
					for (int g = 0; g < vResult.size(); g++)
					{
						for (int gg = 0; gg < vResult[g].size(); gg++)
						{
							vResult[g][gg]->getPosition( nRX, nRY, nRZ );
							cout << "(" << nRX << " " << nRY << " " << nRZ << ")  ";
						}
						cout << endl;
					}
					vector< pin_C* > vTmpPin = vRipNet[r]->getPin();
					cout << "Overflow at: " << nOX << " " << nOY << " " << nOZ << endl;
					cout << "Pin lists: " << endl;
					for( int p=0; p<vTmpPin.size(); p++ )
					{
						int nPX, nPY, nPZ;
						//vTmpPin[p]->getCell()->getPosition( nPX, nPY, nPZ );
						//cout << nPX << " " << nPY << " " << nPZ << endl;
						cout << ((instance_C*)vTmpPin[p]->getCell())->getPlacedX() << " " << ((instance_C*)vTmpPin[p]->getCell())->getPlacedY() << " " << vTmpPin[p]->getLayerId() << endl;
					}
					getchar();
					vResult.clear();
					bRouteAllNet = false;
					break;
				}
				else
				{
					cout << "Net " << vRipNet[r]->getName() <<" Routing Result: " << endl;
					int nRX, nRY, nRZ;
					for (int g = 0; g < vResult.size(); g++)
					{
						for (int gg = 0; gg < vResult[g].size(); gg++)
						{
							vResult[g][gg]->getPosition( nRX, nRY, nRZ );
							cout << "(" << nRX << " " << nRY << " " << nRZ << ")  ";
						}
						cout << endl;
					}	
				}
				
			}
			


			saveNet(vRipNet[r], vResult);
			//if( vRipNet[r]->getName() == "N4" )
			//	cout << "N4: " <<vRipNet[r]->getLength();
			addNetOnGraph(m_pDesign, vRipNet[r]);
			nLengthConstraint = nLengthConstraint - vRipNet[r]->getLength();
			nSuccessCount++;
		}
		
		// check if the new result is better
		if( bRouteAllNet )
		{
			for( int n=0; n<vRipNet.size(); n++ )
			{
				nNewLength = nNewLength + vRipNet[n]->getLength();
			}
			if( nNewLength < nPreviousLength )
			{
				bBetterSol = true;
			}
			else
			{
				bBetterSol = false;
			}
		}
		
		// deal with the routing result
		if( !bRouteAllNet || !bBetterSol )
		{
			if( !bRouteAllNet )
				cout << "Some net failed"<<endl;
			else
			{
				cout << "No better solution in routing "<<endl;
				cout << "Original Length: " << nPreviousLength << " New Length: "<< nNewLength<<endl;
			}
			for( int n=0; n<nSuccessCount; n++ )
			{
				ripupNet( vRipNet[n] );
			}
			cleanWire( vRipNet );
			bSuccess = false;
			for( int i=0; i<vInst.size(); i++ )
				removeInstOnGraph( vInst[i] );
		}
		else //( bRouteAllNet && bBetterSol )
		{	
			bSuccess = true;
		}
	}
	else
	{
		bSuccess = false;
		/*
		for( int i=0; i<vInst.size(); i++ )
			removeInstOnGraph( vInst[i] );
		*/
	}

	if( !bSuccess ) // recover all the information
	{
		//cout << "recover information" << endl;
		recoverInstance( vInst );
		for( int i=0; i<vInst.size(); i++ )
		{
			instance_C* pInst = vInst[i];
			if( i==0 )
				lockInstance( pInst );
			putInstOnGraph( pInst, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ );
			//if( pInst->getName() == "C1" )
			//	cout << "C1: "<<pInst->getPlacedX() <<", "<<pInst->getPlacedY() << endl;
			calForcedModel( m_vForced[ pInst->getId() ] );
		}
		recoverNet( vRipNet );
		for( int i=0; i<vRipNet.size(); i++ )
		{
			addNetOnGraph( m_pDesign, vRipNet[i] );	
		}
		for( int i=0; i<vInst.size(); i++ )
		{
			updateForcedModel( vInst[i] );	
		}
	}
	else
	{
		//cout << "update model" << endl;
		for( int i=0; i<vInst.size(); i++ )
		{
			instance_C* pInst = vInst[i];
			if ( !pInst->hasBeenMoved() && !( pInst->getPlacedX() == m_vBackupInstance[i].getPlacedX() && pInst->getPlacedY() == m_vBackupInstance[i].getPlacedY() ) )
			//if ( !pInst->hasBeenMoved() )
			{
				pInst->hasMoved();
				m_vMovedInstance.push_back(pInst);
			}
		
			//m_nSuccess++;
			updateForcedModel(pInst);
			freeForcedModel(pInst);
		}
		
	}

	for( int i=0; i<m_vBackupNet.size(); i++ )
	{
		m_vBackupNet[i].cleanWire();
	}
	m_vBackupNet.clear();
	m_vBackupInstance.clear();	

	cout << "return  multipleCellMovement result"<<endl;
	return bSuccess;
}

bool router_C::singleCellMovement_ver3(instance_C *pInst, boundry_C* pBound ) // add more operation in this version
{
	cout << "at single"<<endl;
	backupInstance(pInst);
	gGrid_C *pOGrid = getGrid(m_pDesign, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ);
	// remove the instance on grpah
	vector< networkForced_C* > &vTmpNF = m_vForced[ pInst->getId() ].m_vNetwork;
	bool bSame = false;
	vector< instance_C* > vSameInst;
	set< instance_C* > sSameInst;
	vector< networkForced_C* > vSameNet;
	//cout<<m_vForced[ pInst->getId() ].m_nT<<" "<<m_vForced[ pInst->getId() ].m_nD<<" "<<m_vForced[ pInst->getId() ].m_nR<<" "<<m_vForced[ pInst->getId() ].m_nL<<" "<<endl;
	
	//int nOrigX = pInst->getPlacedX();
	//int nOrigY = pInst->getPlacedY();

	char cBound = pBound->m_cType;
	int nTmpMove = 0;
	for( int i=0; i<vTmpNF.size(); i++ )
	{
		if( vTmpNF[i]->m_bSame )
		{
			vSameNet.push_back( vTmpNF[i] );
			bSame = true;
			vector< forced_C* > &vTmpF = vTmpNF[i]->m_vForced;
			for( int j=0; j<vTmpF.size(); j++ )
			{
				if( sSameInst.count( vTmpF[j]->m_pInstance ) == 0 )
				{
					vSameInst.push_back( vTmpF[j]->m_pInstance );
					sSameInst.insert( vTmpF[j]->m_pInstance );
					if( !vTmpF[j]->m_pInstance->hasBeenMoved() )
						nTmpMove++;
				}
			}
		}
	}

	bSame = false;
	//cout<<vSameInst.size()<<endl;

	if( m_vMovedInstance.size() + nTmpMove > m_pDesign->getMaxCellConstraint() )
		bSame = false;

	if( !bSame )
		removeInstOnGraph(pInst); 
	else
	{
		for( int i=0; i<vSameInst.size(); i++ )
		{
			removeInstOnGraph( vSameInst[i] );
		}
	}

	//strInfo = "Find the place for " + strInstName;
	//cout<<setw(COUTWIDTH)<<left<<setfill('.')<<strInfo;
	
	set< net_C* > sSpecialNet;
	calForcedModel_ver4( m_vForced[ pInst->getId() ], sSpecialNet, cBound );

	//vector<gGrid_C *> vBestGrid = findPlaceToMove_ver3(pInst);
	vector<gGrid_C *> vBestGrid = findPlaceToMove_ver3( pInst, pBound );
	gGrid_C *pBGrid = NULL;
	if (vBestGrid.size() == 0)
	{
		//cout<<"failed"<<endl;
		//m_nFailed++;
		if( !bSame )
		{
			lockInstance(pInst);
			//removeInstOnGraph( pInst );
			recoverInstance(pInst);
			putInstOnGraph(pInst, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ);
		}
		else
		{
			for( int i=0; i<vSameInst.size(); i++ )
			{
				lockInstance( vSameInst[i] );
				recoverInstance( vSameInst[i] );
				putInstOnGraph( vSameInst[i], vSameInst[i]->getPlacedX(), vSameInst[i]->getPlacedY(), m_nOffsetZ);
			}	
		}
		//cout<<pInst->getPlacedX()<<" "<<pInst->getPlacedY()<<endl;
		//continue;
#ifdef _DEBUG_MODE
		nTmpCantPlaced++;
#endif

		return false;
	}
	
	//if( vBestGrid.size() > 16 )
	//	vBestGrid.resize( 16 );
	//cerr << " G: " << vBestGrid.size() << " ";
	vector<net_C *> vRipNet;
	bool bFindSolution = false;
	//cout<<" R: ";
	if( !bSame )
	{
		for (int i = vBestGrid.size() - 1; i >= 0; i--)
		{
			int nX, nY, nZ;
			pBGrid = vBestGrid[i];
			pBGrid->getPosition(nX, nY, nZ);
			if (isPlaceable(pBGrid, pInst, vRipNet))
			{
				//cout<<vRipNet.size()<<" ";
				putInstOnGraph(pInst, nX, nY, nZ);
				if (reroute(pInst, vRipNet))
				{
					//cout<<" R: "<<vRipNet.size();
					bFindSolution = true;
					cout << "Place at: " << nX << " " << nY << endl;
					break;
					//return true;
				}
				else
				{
					removeInstOnGraph(pInst);
				}
			}
			else if (pBGrid != pOGrid)
			{
				// added at 0713 20:50
				// the origin place of instance
				gGrid_C *pOGrid = getGrid(m_pDesign, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ);
				// all the instance that can swap
				vector<instance_C *> vSwapInstance = findSwapInstance(pBGrid, pOGrid, pInst);
				//cerr<<"S: "<<vSwapInstance.size()<<" ";
				//cout<<endl;
				//cout<<"Number of swapable instance: "<<vSwapInstance.size()<<endl;
				// find the instance that swap without generating overlap
				for (int s = 0; s < vSwapInstance.size(); s++)
				{
					instance_C *pSInst = vSwapInstance[s];
					instance_C cLocalBackupInstance;
					// local backup pSInst
					cLocalBackupInstance.setPlaced(pSInst->getPlacedX(), pSInst->getPlacedY());
					removeInstOnGraph(pSInst);
					if (swapInstance(pBGrid, pOGrid, pInst, pSInst, vRipNet))
					{
						int nX, nY, nZ;
						pBGrid->getPosition(nX, nY, nZ);
						putInstOnGraph(pInst, nX, nY, nZ);
						pOGrid->getPosition(nX, nY, nZ);
						putInstOnGraph(pSInst, nX, nY, nZ);
						if (reroute(pInst, vRipNet))
						{
							bFindSolution = true;
							if (!pSInst->hasBeenMoved())
							{
								pSInst->hasMoved();
								m_vMovedInstance.push_back(pSInst);
								updateForcedModel_ver2(pSInst);
								freeBoundry( pInst );
								freeForcedModel(pSInst);
							}
							break;
						}
						else
						{
							removeInstOnGraph(pInst);
							removeInstOnGraph(pSInst);
							// recover pSInst
							recoverInstance(pInst);
							pSInst->setPlaced(cLocalBackupInstance.getPlacedX(), cLocalBackupInstance.getPlacedY());
							putInstOnGraph(pSInst, pSInst->getPlacedX(), pSInst->getPlacedY(), m_nOffsetZ);
						}
					}
					else
					{
						// recover instance, put instance back on the graph
						int nX, nY, nZ;
						pBGrid->getPosition(nX, nY, nZ);
						putInstOnGraph(pSInst, nX, nY, nZ);
						recoverInstance(pInst);
					}
				}
				if (bFindSolution)
					break;
				// end added at 0713 20:50

				//cout<<vRipNet.size()<<" ";
			}
			else
			{
				recoverInstance(pInst);
			}
			vRipNet.clear();
		}
	}
	else
	{	
		for (int i = vBestGrid.size() - 1; i >= 0; i--)
		{
			int nX, nY, nZ;
			pBGrid = vBestGrid[i];
			pBGrid->getPosition(nX, nY, nZ);
			if (isPlaceable(pBGrid, vSameInst, vRipNet))
			{
				//cout<<vRipNet.size()<<" ";
				for( int c=0; c<vSameInst.size(); c++ )
					putInstOnGraph(vSameInst[c], nX, nY, nZ);

				if( reroute( vSameInst, vRipNet ) )
				{
					//cout<<" R: "<<vRipNet.size();
					bFindSolution = true;
					break;
					//return true;
				}
				else
				{
					
					for( int c=0; c<vSameInst.size(); c++ )
					{
						removeInstOnGraph( vSameInst[c] );
						recoverInstance( vSameInst[c] );
					}
				}
			}
			/*
			else if (pBGrid != pOGrid)
			{
				// added at 0713 20:50
				// the origin place of instance
				gGrid_C *pOGrid = getGrid(m_pDesign, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ);
				// all the instance that can swap
				vector<instance_C *> vSwapInstance = findSwapInstance(pBGrid, pOGrid, pInst);
				cerr<<" S: "<<vSwapInstance.size()<<" ";
				//cout<<endl;
				//cout<<"Number of swapable instance: "<<vSwapInstance.size()<<endl;
				// find the instance that swap without generating overlap
				if( vSwapInstance.size() != 0 )
				{
					//cerr<<" MERGE ";
					instance_C *pSInst = vSwapInstance[ 0 ];
					instance_C cLocalBackupInstance;
					// local backup pSInst
					cLocalBackupInstance.setPlaced(pSInst->getPlacedX(), pSInst->getPlacedY());
					for( int s=0; s<vSwapInstance.size(); s++ )
					{
						removeInstOnGraph( vSwapInstance[s] );
					}
					if (swapInstance(pBGrid, pOGrid, vSameInst, vSwapInstance, vRipNet))
					{
						int nX, nY, nZ;
						pBGrid->getPosition(nX, nY, nZ);
						for( int ii=0; ii<vSameInst.size(); ii++ )
						{
							putInstOnGraph( vSameInst[ii], nX, nY, nZ);
						}
						pOGrid->getPosition(nX, nY, nZ);
						for( int s=0; s<vSwapInstance.size(); s++ )
						{
							putInstOnGraph( vSwapInstance[s], nX, nY, nZ);
						}
						if (reroute( vSameInst, vRipNet))
						{
							bFindSolution = true;
							for( int s=0; s<vSwapInstance.size(); s++ )
							{
								if (!vSwapInstance[s]->hasBeenMoved())
								{
									vSwapInstance[s]->hasMoved();
									m_vMovedInstance.push_back( vSwapInstance[s] );
									updateForcedModel( vSwapInstance[s] );
									freeForcedModel( vSwapInstance[s] );
								}
							}
							break;
						}
						else
						{
							for( int ii=0; ii<vSameInst.size(); ii++ )
							{
								removeInstOnGraph( vSameInst[ii] );
								recoverInstance( vSameInst[ii] );
							}
							for( int s=0; s<vSwapInstance.size(); s++ )
							{
								removeInstOnGraph( vSwapInstance[s] );
								// recover pSInst
								vSwapInstance[s]->setPlaced(cLocalBackupInstance.getPlacedX(), cLocalBackupInstance.getPlacedY());
								putInstOnGraph( vSwapInstance[s], vSwapInstance[s]->getPlacedX(), vSwapInstance[s]->getPlacedY(), m_nOffsetZ);
							}
						}
					}
					else
					{
						// recover instance, put instance back on the graph
						int nX, nY, nZ;
						pBGrid->getPosition(nX, nY, nZ);
						for( int s=0; s<vSwapInstance.size(); s++ )
							putInstOnGraph( vSwapInstance[s], nX, nY, nZ);
						for( int ii=0; ii<vSameInst.size(); ii++ )
							recoverInstance( vSameInst[ii] );
					}
				}
				if (bFindSolution)
					break;
				// end added at 0713 20:50

				//cout<<vRipNet.size()<<" ";
			}
			*/
			else
			{
				for( int c=0; c<vSameInst.size(); c++ )
					recoverInstance( vSameInst[c] );
			}
			vRipNet.clear();
		}
	}

	if (bFindSolution)
	{
		if( !bSame )
		{
			if ( !pInst->hasBeenMoved() && !( pInst->getPlacedX() == m_cBackupInstance.getPlacedX() && pInst->getPlacedY() == m_cBackupInstance.getPlacedY() ) )
			//if ( !pInst->hasBeenMoved() )
			{
				pInst->hasMoved();
				m_vMovedInstance.push_back(pInst);
			}
		
			//m_nSuccess++;
			updateForcedModel_ver2(pInst);
			freeBoundry( pInst );
			freeForcedModel(pInst);
			//lockInstance( pInst );
		}
		else
		{
			for( int c=0; c<vSameInst.size(); c++ )
			{
				if( !vSameInst[c]->hasBeenMoved() )
				{
					vSameInst[c]->hasMoved();
					m_vMovedInstance.push_back( vSameInst[c] );
				}
			
				//m_nSuccess++;
				updateForcedModel_ver2( vSameInst[c] );
				freeBoundry( pInst );
				freeForcedModel( vSameInst[c] );	
			}
		}
		//lockInstance( pInst );
		return true;
	}
	else
	{
		if( !bSame )
		{
			recoverInstance(pInst);
			putInstOnGraph(pInst, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ);
			lockInstance(pInst);
		}
		else
		{
			for( int c=0; c<vSameInst.size(); c++ )
			{
				recoverInstance( vSameInst[c] );
				putInstOnGraph( vSameInst[c], vSameInst[c]->getPlacedX(), vSameInst[c]->getPlacedY(), m_nOffsetZ);
				lockInstance( vSameInst[c] );	
			}
		}
		return false;
	}
}

bool router_C::singleCellMovement_ver3(instance_C *pInst, char cBound ) // add more operation in this version
{
	cout << "at single"<<endl;
	backupInstance(pInst);
	gGrid_C *pOGrid = getGrid(m_pDesign, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ);
	// remove the instance on grpah
	vector< networkForced_C* > &vTmpNF = m_vForced[ pInst->getId() ].m_vNetwork;
	bool bSame = false;
	vector< instance_C* > vSameInst;
	set< instance_C* > sSameInst;
	vector< networkForced_C* > vSameNet;
	//cout<<m_vForced[ pInst->getId() ].m_nT<<" "<<m_vForced[ pInst->getId() ].m_nD<<" "<<m_vForced[ pInst->getId() ].m_nR<<" "<<m_vForced[ pInst->getId() ].m_nL<<" "<<endl;
	
	//int nOrigX = pInst->getPlacedX();
	//int nOrigY = pInst->getPlacedY();

	int nTmpMove = 0;
	for( int i=0; i<vTmpNF.size(); i++ )
	{
		if( vTmpNF[i]->m_bSame )
		{
			vSameNet.push_back( vTmpNF[i] );
			bSame = true;
			vector< forced_C* > &vTmpF = vTmpNF[i]->m_vForced;
			for( int j=0; j<vTmpF.size(); j++ )
			{
				if( sSameInst.count( vTmpF[j]->m_pInstance ) == 0 )
				{
					vSameInst.push_back( vTmpF[j]->m_pInstance );
					sSameInst.insert( vTmpF[j]->m_pInstance );
					if( !vTmpF[j]->m_pInstance->hasBeenMoved() )
						nTmpMove++;
				}
			}
		}
	}

	bSame = false;
	//cout<<vSameInst.size()<<endl;

	if( m_vMovedInstance.size() + nTmpMove > m_pDesign->getMaxCellConstraint() )
		bSame = false;

	if( !bSame )
		removeInstOnGraph(pInst); 
	else
	{
		for( int i=0; i<vSameInst.size(); i++ )
		{
			removeInstOnGraph( vSameInst[i] );
		}
	}

	//strInfo = "Find the place for " + strInstName;
	//cout<<setw(COUTWIDTH)<<left<<setfill('.')<<strInfo;
	
	set< net_C* > sSpecialNet;
	calForcedModel_ver4( m_vForced[ pInst->getId() ], sSpecialNet, cBound );

	vector<gGrid_C *> vBestGrid = findPlaceToMove_ver3(pInst);
	gGrid_C *pBGrid = NULL;
	if (vBestGrid.size() == 0)
	{
		//cout<<"failed"<<endl;
		//m_nFailed++;
		if( !bSame )
		{
			lockInstance(pInst);
			//removeInstOnGraph( pInst );
			recoverInstance(pInst);
			putInstOnGraph(pInst, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ);
		}
		else
		{
			for( int i=0; i<vSameInst.size(); i++ )
			{
				lockInstance( vSameInst[i] );
				recoverInstance( vSameInst[i] );
				putInstOnGraph( vSameInst[i], vSameInst[i]->getPlacedX(), vSameInst[i]->getPlacedY(), m_nOffsetZ);
			}	
		}
		//cout<<pInst->getPlacedX()<<" "<<pInst->getPlacedY()<<endl;
		//continue;
#ifdef _DEBUG_MODE
		nTmpCantPlaced++;
#endif

		return false;
	}
	
	//if( vBestGrid.size() > 16 )
	//	vBestGrid.resize( 16 );
	//cerr << " G: " << vBestGrid.size() << " ";
	vector<net_C *> vRipNet;
	bool bFindSolution = false;
	//cout<<" R: ";
	if( !bSame )
	{
		for (int i = vBestGrid.size() - 1; i >= 0; i--)
		{
			int nX, nY, nZ;
			pBGrid = vBestGrid[i];
			pBGrid->getPosition(nX, nY, nZ);
			if (isPlaceable(pBGrid, pInst, vRipNet))
			{
				//cout<<vRipNet.size()<<" ";
				putInstOnGraph(pInst, nX, nY, nZ);
				if (reroute(pInst, vRipNet))
				{
					//cout<<" R: "<<vRipNet.size();
					bFindSolution = true;
					cout << "Place at: " << nX << " " << nY << endl;
					break;
					//return true;
				}
				else
				{
					removeInstOnGraph(pInst);
				}
			}
			else if (pBGrid != pOGrid)
			{
				// added at 0713 20:50
				// the origin place of instance
				gGrid_C *pOGrid = getGrid(m_pDesign, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ);
				// all the instance that can swap
				vector<instance_C *> vSwapInstance = findSwapInstance(pBGrid, pOGrid, pInst);
				//cerr<<"S: "<<vSwapInstance.size()<<" ";
				//cout<<endl;
				//cout<<"Number of swapable instance: "<<vSwapInstance.size()<<endl;
				// find the instance that swap without generating overlap
				for (int s = 0; s < vSwapInstance.size(); s++)
				{
					instance_C *pSInst = vSwapInstance[s];
					instance_C cLocalBackupInstance;
					// local backup pSInst
					cLocalBackupInstance.setPlaced(pSInst->getPlacedX(), pSInst->getPlacedY());
					removeInstOnGraph(pSInst);
					if (swapInstance(pBGrid, pOGrid, pInst, pSInst, vRipNet))
					{
						int nX, nY, nZ;
						pBGrid->getPosition(nX, nY, nZ);
						putInstOnGraph(pInst, nX, nY, nZ);
						pOGrid->getPosition(nX, nY, nZ);
						putInstOnGraph(pSInst, nX, nY, nZ);
						if (reroute(pInst, vRipNet))
						{
							bFindSolution = true;
							if (!pSInst->hasBeenMoved())
							{
								pSInst->hasMoved();
								m_vMovedInstance.push_back(pSInst);
								updateForcedModel_ver2(pSInst);
								freeBoundry( pInst );
								freeForcedModel(pSInst);
							}
							break;
						}
						else
						{
							removeInstOnGraph(pInst);
							removeInstOnGraph(pSInst);
							// recover pSInst
							recoverInstance(pInst);
							pSInst->setPlaced(cLocalBackupInstance.getPlacedX(), cLocalBackupInstance.getPlacedY());
							putInstOnGraph(pSInst, pSInst->getPlacedX(), pSInst->getPlacedY(), m_nOffsetZ);
						}
					}
					else
					{
						// recover instance, put instance back on the graph
						int nX, nY, nZ;
						pBGrid->getPosition(nX, nY, nZ);
						putInstOnGraph(pSInst, nX, nY, nZ);
						recoverInstance(pInst);
					}
				}
				if (bFindSolution)
					break;
				// end added at 0713 20:50

				//cout<<vRipNet.size()<<" ";
			}
			else
			{
				recoverInstance(pInst);
			}
			vRipNet.clear();
		}
	}
	else
	{	
		for (int i = vBestGrid.size() - 1; i >= 0; i--)
		{
			int nX, nY, nZ;
			pBGrid = vBestGrid[i];
			pBGrid->getPosition(nX, nY, nZ);
			if (isPlaceable(pBGrid, vSameInst, vRipNet))
			{
				//cout<<vRipNet.size()<<" ";
				for( int c=0; c<vSameInst.size(); c++ )
					putInstOnGraph(vSameInst[c], nX, nY, nZ);

				if( reroute( vSameInst, vRipNet ) )
				{
					//cout<<" R: "<<vRipNet.size();
					bFindSolution = true;
					break;
					//return true;
				}
				else
				{
					
					for( int c=0; c<vSameInst.size(); c++ )
					{
						removeInstOnGraph( vSameInst[c] );
						recoverInstance( vSameInst[c] );
					}
				}
			}
			/*
			else if (pBGrid != pOGrid)
			{
				// added at 0713 20:50
				// the origin place of instance
				gGrid_C *pOGrid = getGrid(m_pDesign, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ);
				// all the instance that can swap
				vector<instance_C *> vSwapInstance = findSwapInstance(pBGrid, pOGrid, pInst);
				cerr<<" S: "<<vSwapInstance.size()<<" ";
				//cout<<endl;
				//cout<<"Number of swapable instance: "<<vSwapInstance.size()<<endl;
				// find the instance that swap without generating overlap
				if( vSwapInstance.size() != 0 )
				{
					//cerr<<" MERGE ";
					instance_C *pSInst = vSwapInstance[ 0 ];
					instance_C cLocalBackupInstance;
					// local backup pSInst
					cLocalBackupInstance.setPlaced(pSInst->getPlacedX(), pSInst->getPlacedY());
					for( int s=0; s<vSwapInstance.size(); s++ )
					{
						removeInstOnGraph( vSwapInstance[s] );
					}
					if (swapInstance(pBGrid, pOGrid, vSameInst, vSwapInstance, vRipNet))
					{
						int nX, nY, nZ;
						pBGrid->getPosition(nX, nY, nZ);
						for( int ii=0; ii<vSameInst.size(); ii++ )
						{
							putInstOnGraph( vSameInst[ii], nX, nY, nZ);
						}
						pOGrid->getPosition(nX, nY, nZ);
						for( int s=0; s<vSwapInstance.size(); s++ )
						{
							putInstOnGraph( vSwapInstance[s], nX, nY, nZ);
						}
						if (reroute( vSameInst, vRipNet))
						{
							bFindSolution = true;
							for( int s=0; s<vSwapInstance.size(); s++ )
							{
								if (!vSwapInstance[s]->hasBeenMoved())
								{
									vSwapInstance[s]->hasMoved();
									m_vMovedInstance.push_back( vSwapInstance[s] );
									updateForcedModel( vSwapInstance[s] );
									freeForcedModel( vSwapInstance[s] );
								}
							}
							break;
						}
						else
						{
							for( int ii=0; ii<vSameInst.size(); ii++ )
							{
								removeInstOnGraph( vSameInst[ii] );
								recoverInstance( vSameInst[ii] );
							}
							for( int s=0; s<vSwapInstance.size(); s++ )
							{
								removeInstOnGraph( vSwapInstance[s] );
								// recover pSInst
								vSwapInstance[s]->setPlaced(cLocalBackupInstance.getPlacedX(), cLocalBackupInstance.getPlacedY());
								putInstOnGraph( vSwapInstance[s], vSwapInstance[s]->getPlacedX(), vSwapInstance[s]->getPlacedY(), m_nOffsetZ);
							}
						}
					}
					else
					{
						// recover instance, put instance back on the graph
						int nX, nY, nZ;
						pBGrid->getPosition(nX, nY, nZ);
						for( int s=0; s<vSwapInstance.size(); s++ )
							putInstOnGraph( vSwapInstance[s], nX, nY, nZ);
						for( int ii=0; ii<vSameInst.size(); ii++ )
							recoverInstance( vSameInst[ii] );
					}
				}
				if (bFindSolution)
					break;
				// end added at 0713 20:50

				//cout<<vRipNet.size()<<" ";
			}
			*/
			else
			{
				for( int c=0; c<vSameInst.size(); c++ )
					recoverInstance( vSameInst[c] );
			}
			vRipNet.clear();
		}
	}

	if (bFindSolution)
	{
		if( !bSame )
		{
			if ( !pInst->hasBeenMoved() && !( pInst->getPlacedX() == m_cBackupInstance.getPlacedX() && pInst->getPlacedY() == m_cBackupInstance.getPlacedY() ) )
			//if ( !pInst->hasBeenMoved() )
			{
				pInst->hasMoved();
				m_vMovedInstance.push_back(pInst);
			}
		
			//m_nSuccess++;
			updateForcedModel_ver2(pInst);
			freeBoundry( pInst );
			freeForcedModel(pInst);
			//lockInstance( pInst );
		}
		else
		{
			for( int c=0; c<vSameInst.size(); c++ )
			{
				if( !vSameInst[c]->hasBeenMoved() )
				{
					vSameInst[c]->hasMoved();
					m_vMovedInstance.push_back( vSameInst[c] );
				}
			
				//m_nSuccess++;
				updateForcedModel_ver2( vSameInst[c] );
				freeBoundry( pInst );
				freeForcedModel( vSameInst[c] );	
			}
		}
		//lockInstance( pInst );
		return true;
	}
	else
	{
		if( !bSame )
		{
			recoverInstance(pInst);
			putInstOnGraph(pInst, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ);
			lockInstance(pInst);
		}
		else
		{
			for( int c=0; c<vSameInst.size(); c++ )
			{
				recoverInstance( vSameInst[c] );
				putInstOnGraph( vSameInst[c], vSameInst[c]->getPlacedX(), vSameInst[c]->getPlacedY(), m_nOffsetZ);
				lockInstance( vSameInst[c] );	
			}
		}
		return false;
	}
}
// added at 0705 02:00
bool router_C::singleCellMovement_ver3(instance_C *pInst) // add more operation in this version
{
	cout << "at single"<<endl;
	backupInstance(pInst);
	gGrid_C *pOGrid = getGrid(m_pDesign, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ);
	// remove the instance on grpah
	vector< networkForced_C* > &vTmpNF = m_vForced[ pInst->getId() ].m_vNetwork;
	bool bSame = false;
	vector< instance_C* > vSameInst;
	set< instance_C* > sSameInst;
	vector< networkForced_C* > vSameNet;
	//cout<<m_vForced[ pInst->getId() ].m_nT<<" "<<m_vForced[ pInst->getId() ].m_nD<<" "<<m_vForced[ pInst->getId() ].m_nR<<" "<<m_vForced[ pInst->getId() ].m_nL<<" "<<endl;
	
	//int nOrigX = pInst->getPlacedX();
	//int nOrigY = pInst->getPlacedY();

	int nTmpMove = 0;
	for( int i=0; i<vTmpNF.size(); i++ )
	{
		if( vTmpNF[i]->m_bSame )
		{
			vSameNet.push_back( vTmpNF[i] );
			bSame = true;
			vector< forced_C* > &vTmpF = vTmpNF[i]->m_vForced;
			for( int j=0; j<vTmpF.size(); j++ )
			{
				if( sSameInst.count( vTmpF[j]->m_pInstance ) == 0 )
				{
					vSameInst.push_back( vTmpF[j]->m_pInstance );
					sSameInst.insert( vTmpF[j]->m_pInstance );
					if( !vTmpF[j]->m_pInstance->hasBeenMoved() )
						nTmpMove++;
				}
			}
		}
	}

	bSame = false;
	//cout<<vSameInst.size()<<endl;

	if( m_vMovedInstance.size() + nTmpMove > m_pDesign->getMaxCellConstraint() )
		bSame = false;

	if( !bSame )
		removeInstOnGraph(pInst); 
	else
	{
		for( int i=0; i<vSameInst.size(); i++ )
		{
			removeInstOnGraph( vSameInst[i] );
		}
	}

	//strInfo = "Find the place for " + strInstName;
	//cout<<setw(COUTWIDTH)<<left<<setfill('.')<<strInfo;
	vector<gGrid_C *> vBestGrid = findPlaceToMove_ver3(pInst);
	gGrid_C *pBGrid = NULL;
	if (vBestGrid.size() == 0)
	{
		//cout<<"failed"<<endl;
		//m_nFailed++;
		if( !bSame )
		{
			lockInstance(pInst);
			//removeInstOnGraph( pInst );
			recoverInstance(pInst);
			putInstOnGraph(pInst, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ);
		}
		else
		{
			for( int i=0; i<vSameInst.size(); i++ )
			{
				lockInstance( vSameInst[i] );
				recoverInstance( vSameInst[i] );
				putInstOnGraph( vSameInst[i], vSameInst[i]->getPlacedX(), vSameInst[i]->getPlacedY(), m_nOffsetZ);
			}	
		}
		//cout<<pInst->getPlacedX()<<" "<<pInst->getPlacedY()<<endl;
		//continue;
#ifdef _DEBUG_MODE
		nTmpCantPlaced++;
#endif

		return false;
	}
	
	//if( vBestGrid.size() > 16 )
	//	vBestGrid.resize( 16 );
	//cerr << " G: " << vBestGrid.size() << " ";
	vector<net_C *> vRipNet;
	bool bFindSolution = false;
	//cout<<" R: ";
	if( !bSame )
	{
		for (int i = vBestGrid.size() - 1; i >= 0; i--)
		{
			int nX, nY, nZ;
			pBGrid = vBestGrid[i];
			pBGrid->getPosition(nX, nY, nZ);
			if (isPlaceable(pBGrid, pInst, vRipNet))
			{
				//cout<<vRipNet.size()<<" ";
				putInstOnGraph(pInst, nX, nY, nZ);
				if (reroute(pInst, vRipNet))
				{
					//cout<<" R: "<<vRipNet.size();
					bFindSolution = true;
					cout << "Place at: " << nX << " " << nY << endl;
					break;
					//return true;
				}
				else
				{
					removeInstOnGraph(pInst);
				}
			}
			else if (pBGrid != pOGrid)
			{
				// added at 0713 20:50
				// the origin place of instance
				gGrid_C *pOGrid = getGrid(m_pDesign, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ);
				// all the instance that can swap
				vector<instance_C *> vSwapInstance = findSwapInstance(pBGrid, pOGrid, pInst);
				//cerr<<"S: "<<vSwapInstance.size()<<" ";
				//cout<<endl;
				//cout<<"Number of swapable instance: "<<vSwapInstance.size()<<endl;
				// find the instance that swap without generating overlap
				for (int s = 0; s < vSwapInstance.size(); s++)
				{
					instance_C *pSInst = vSwapInstance[s];
					instance_C cLocalBackupInstance;
					// local backup pSInst
					cLocalBackupInstance.setPlaced(pSInst->getPlacedX(), pSInst->getPlacedY());
					removeInstOnGraph(pSInst);
					if (swapInstance(pBGrid, pOGrid, pInst, pSInst, vRipNet))
					{
						int nX, nY, nZ;
						pBGrid->getPosition(nX, nY, nZ);
						putInstOnGraph(pInst, nX, nY, nZ);
						pOGrid->getPosition(nX, nY, nZ);
						putInstOnGraph(pSInst, nX, nY, nZ);
						if (reroute(pInst, vRipNet))
						{
							bFindSolution = true;
							if (!pSInst->hasBeenMoved())
							{
								pSInst->hasMoved();
								m_vMovedInstance.push_back(pSInst);
								updateForcedModel_ver2(pSInst);
								freeBoundry( pInst );
								freeForcedModel(pSInst);
							}
							break;
						}
						else
						{
							removeInstOnGraph(pInst);
							removeInstOnGraph(pSInst);
							// recover pSInst
							recoverInstance(pInst);
							pSInst->setPlaced(cLocalBackupInstance.getPlacedX(), cLocalBackupInstance.getPlacedY());
							putInstOnGraph(pSInst, pSInst->getPlacedX(), pSInst->getPlacedY(), m_nOffsetZ);
						}
					}
					else
					{
						// recover instance, put instance back on the graph
						int nX, nY, nZ;
						pBGrid->getPosition(nX, nY, nZ);
						putInstOnGraph(pSInst, nX, nY, nZ);
						recoverInstance(pInst);
					}
				}
				if (bFindSolution)
					break;
				// end added at 0713 20:50

				//cout<<vRipNet.size()<<" ";
			}
			else
			{
				recoverInstance(pInst);
			}
			vRipNet.clear();
		}
	}
	else
	{	
		for (int i = vBestGrid.size() - 1; i >= 0; i--)
		{
			int nX, nY, nZ;
			pBGrid = vBestGrid[i];
			pBGrid->getPosition(nX, nY, nZ);
			if (isPlaceable(pBGrid, vSameInst, vRipNet))
			{
				//cout<<vRipNet.size()<<" ";
				for( int c=0; c<vSameInst.size(); c++ )
					putInstOnGraph(vSameInst[c], nX, nY, nZ);

				if( reroute( vSameInst, vRipNet ) )
				{
					//cout<<" R: "<<vRipNet.size();
					bFindSolution = true;
					break;
					//return true;
				}
				else
				{
					
					for( int c=0; c<vSameInst.size(); c++ )
					{
						removeInstOnGraph( vSameInst[c] );
						recoverInstance( vSameInst[c] );
					}
				}
			}
			/*
			else if (pBGrid != pOGrid)
			{
				// added at 0713 20:50
				// the origin place of instance
				gGrid_C *pOGrid = getGrid(m_pDesign, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ);
				// all the instance that can swap
				vector<instance_C *> vSwapInstance = findSwapInstance(pBGrid, pOGrid, pInst);
				cerr<<" S: "<<vSwapInstance.size()<<" ";
				//cout<<endl;
				//cout<<"Number of swapable instance: "<<vSwapInstance.size()<<endl;
				// find the instance that swap without generating overlap
				if( vSwapInstance.size() != 0 )
				{
					//cerr<<" MERGE ";
					instance_C *pSInst = vSwapInstance[ 0 ];
					instance_C cLocalBackupInstance;
					// local backup pSInst
					cLocalBackupInstance.setPlaced(pSInst->getPlacedX(), pSInst->getPlacedY());
					for( int s=0; s<vSwapInstance.size(); s++ )
					{
						removeInstOnGraph( vSwapInstance[s] );
					}
					if (swapInstance(pBGrid, pOGrid, vSameInst, vSwapInstance, vRipNet))
					{
						int nX, nY, nZ;
						pBGrid->getPosition(nX, nY, nZ);
						for( int ii=0; ii<vSameInst.size(); ii++ )
						{
							putInstOnGraph( vSameInst[ii], nX, nY, nZ);
						}
						pOGrid->getPosition(nX, nY, nZ);
						for( int s=0; s<vSwapInstance.size(); s++ )
						{
							putInstOnGraph( vSwapInstance[s], nX, nY, nZ);
						}
						if (reroute( vSameInst, vRipNet))
						{
							bFindSolution = true;
							for( int s=0; s<vSwapInstance.size(); s++ )
							{
								if (!vSwapInstance[s]->hasBeenMoved())
								{
									vSwapInstance[s]->hasMoved();
									m_vMovedInstance.push_back( vSwapInstance[s] );
									updateForcedModel( vSwapInstance[s] );
									freeForcedModel( vSwapInstance[s] );
								}
							}
							break;
						}
						else
						{
							for( int ii=0; ii<vSameInst.size(); ii++ )
							{
								removeInstOnGraph( vSameInst[ii] );
								recoverInstance( vSameInst[ii] );
							}
							for( int s=0; s<vSwapInstance.size(); s++ )
							{
								removeInstOnGraph( vSwapInstance[s] );
								// recover pSInst
								vSwapInstance[s]->setPlaced(cLocalBackupInstance.getPlacedX(), cLocalBackupInstance.getPlacedY());
								putInstOnGraph( vSwapInstance[s], vSwapInstance[s]->getPlacedX(), vSwapInstance[s]->getPlacedY(), m_nOffsetZ);
							}
						}
					}
					else
					{
						// recover instance, put instance back on the graph
						int nX, nY, nZ;
						pBGrid->getPosition(nX, nY, nZ);
						for( int s=0; s<vSwapInstance.size(); s++ )
							putInstOnGraph( vSwapInstance[s], nX, nY, nZ);
						for( int ii=0; ii<vSameInst.size(); ii++ )
							recoverInstance( vSameInst[ii] );
					}
				}
				if (bFindSolution)
					break;
				// end added at 0713 20:50

				//cout<<vRipNet.size()<<" ";
			}
			*/
			else
			{
				for( int c=0; c<vSameInst.size(); c++ )
					recoverInstance( vSameInst[c] );
			}
			vRipNet.clear();
		}
	}

	if (bFindSolution)
	{
		if( !bSame )
		{
			if ( !pInst->hasBeenMoved() && !( pInst->getPlacedX() == m_cBackupInstance.getPlacedX() && pInst->getPlacedY() == m_cBackupInstance.getPlacedY() ) )
			//if ( !pInst->hasBeenMoved() )
			{
				pInst->hasMoved();
				m_vMovedInstance.push_back(pInst);
			}
		
			//m_nSuccess++;
			updateForcedModel_ver2(pInst);
			freeBoundry( pInst );
			freeForcedModel(pInst);
			//lockInstance( pInst );
		}
		else
		{
			for( int c=0; c<vSameInst.size(); c++ )
			{
				if( !vSameInst[c]->hasBeenMoved() )
				{
					vSameInst[c]->hasMoved();
					m_vMovedInstance.push_back( vSameInst[c] );
				}
			
				//m_nSuccess++;
				updateForcedModel_ver2( vSameInst[c] );
				freeBoundry( pInst );
				freeForcedModel( vSameInst[c] );	
			}
		}
		//lockInstance( pInst );
		return true;
	}
	else
	{
		if( !bSame )
		{
			recoverInstance(pInst);
			putInstOnGraph(pInst, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ);
			lockInstance(pInst);
		}
		else
		{
			for( int c=0; c<vSameInst.size(); c++ )
			{
				recoverInstance( vSameInst[c] );
				putInstOnGraph( vSameInst[c], vSameInst[c]->getPlacedX(), vSameInst[c]->getPlacedY(), m_nOffsetZ);
				lockInstance( vSameInst[c] );	
			}
		}
		return false;
	}
}
// end added at 0705 02:00

// added at 0704 22:00
bool router_C::singleCellMovement_ver2(instance_C *pInst) // add more operation in this version
{
	backupInstance(pInst);
	// remove the instance on grpah

	if (removeInstOnGraph(pInst))
	{
		//cout<<"complete"<<endl;
	}

	//strInfo = "Find the place for " + strInstName;
	//cout<<setw(COUTWIDTH)<<left<<setfill('.')<<strInfo;
	vector<gGrid_C *> vBestGrid = findPlaceToMove(pInst);
	gGrid_C *pBGrid = NULL;
	if (vBestGrid.size() == 0)
	{
		cout << " F: " << vBestGrid.size() << " ";
		//cout<<"failed"<<endl;
		//m_nFailed++;
		lockInstance(pInst);
		//removeInstOnGraph( pInst );
		recoverInstance(pInst);
		putInstOnGraph(pInst, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ);
		//cout<<pInst->getPlacedX()<<" "<<pInst->getPlacedY()<<endl;
		//continue;
#ifdef _DEBUG_MODE
		nTmpCantPlaced++;
#endif

		return false;
	}
	else
	{
		//cout << " F: " << vBestGrid.size() << " ";
		int nOrigX = m_cBackupInstance.getPlacedX();
		int nOrigY = m_cBackupInstance.getPlacedY();
		int nPlacedX, nPlacedY, nPlacedZ;
		// change at 0706 19:30
		//pBGrid = vBestGrid.back();
		pBGrid = vBestGrid.front();
		// end change
		pBGrid->getPosition(nPlacedX, nPlacedY, nPlacedZ);
		int nDX = nOrigX - nPlacedX;
		int nDY = nOrigY - nPlacedY;
		if (nDX != 0)
			lockInstance(pInst, 'X');
		else
			lockInstance(pInst, 'Y');
	}
	//else
	//	cout<<"complete"<<endl;
	/*
	cout<<"Check is ("<<nX<<","<<nY<<","<<nZ<<") is available to place the instance"<<endl;
	if( isPlaceable( getGrid( m_pDesign, nX, nY, nZ), pInst ) )
	{
		cout<<"Yes"<<endl;	
	}
	else
		cout<<"No"<<endl;
	*/
	//pBGrid->getPosition( nX, nY, nZ );
	//cout<<"The best place is ("<<nX<<","<<nY<<","<<nZ<<") for "<< strInstName <<endl;

	//strInstName = pInst->getName();
	//strInfo = "Put " + strInstName + " on graph";
	//cout<<setw(COUTWIDTH)<<left<<setfill('.')<<strInfo;
	//putInstOnGraph( pInst, nX, nY, nZ );

	vector<net_C *> vRipNet;
	bool bFindSolution = false;
	for (int i = vBestGrid.size() - 1; i >= 0; i--)
	{
		int nX, nY, nZ;
		pBGrid = vBestGrid[i];
		pBGrid->getPosition(nX, nY, nZ);
		putInstOnGraph(pInst, nX, nY, nZ);
		if (reroute(pInst, vRipNet))
		{
			bFindSolution = true;
			break;
			//return true;
		}
		else
		{
			removeInstOnGraph(pInst);
			//recoverInstance( pInst );
			//putInstOnGraph( pInst, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ );
			//return false;
		}
	}

	if (bFindSolution)
	{
		if (!pInst->hasBeenMoved())
		{
			pInst->hasMoved();
			m_vMovedInstance.push_back(pInst);
		}
		//m_nSuccess++;
		updateForcedModel(pInst);
		freeForcedModel(pInst);
		return true;
	}
	else
	{
		recoverInstance(pInst);
		putInstOnGraph(pInst, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ);
		return false;
	}
}
// end added at 0704 22:00
/*
bool router_C::singleCellMovement( instance_C* pInst )
{

	//cout<<endl;
	//cout<<"Check "<<m_vBackupNet.size()<<endl;
	backupInstance( pInst );
// remove the instance on grpah
		
	if( removeInstOnGraph( pInst ) )
	{
		//cout<<"complete"<<endl;
	}

	//strInfo = "Find the place for " + strInstName;
	//cout<<setw(COUTWIDTH)<<left<<setfill('.')<<strInfo;
	gGrid_C* pBGrid = findPlaceToMove( pInst );
	if( pBGrid == NULL )
	{
		//cout<<"failed"<<endl;
		//m_nFailed++;
		lockInstance( pInst );
		//removeInstOnGraph( pInst );
		recoverInstance( pInst );
		putInstOnGraph( pInst, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ );
		//cout<<pInst->getPlacedX()<<" "<<pInst->getPlacedY()<<endl;
		//continue;
#ifdef _DEBUG_MODE
		nTmpCantPlaced++;
#endif

		return false;
	}
	else
	{
		int nOrigX = m_cBackupInstance.getPlacedX();
		int nOrigY = m_cBackupInstance.getPlacedY();
		int nPlacedX, nPlacedY, nPlacedZ;
		pBGrid->getPosition( nPlacedX, nPlacedY, nPlacedZ );
		int nDX = nOrigX - nPlacedX;
		int nDY = nOrigY - nPlacedY;
		if( nDX != 0 )
			lockInstance( pInst, 'X' );
		else
			lockInstance( pInst, 'Y' );
	}
	//else
	//	cout<<"complete"<<endl;
	int nX;
	int nY;
	int nZ;
	
	//cout<<"Check is ("<<nX<<","<<nY<<","<<nZ<<") is available to place the instance"<<endl;
	//if( isPlaceable( getGrid( m_pDesign, nX, nY, nZ), pInst ) )
	//{
	//	cout<<"Yes"<<endl;	
	//}
	//else
	//	cout<<"No"<<endl;
	
	pBGrid->getPosition( nX, nY, nZ );
	//cout<<"The best place is ("<<nX<<","<<nY<<","<<nZ<<") for "<< strInstName <<endl;
	
	//strInstName = pInst->getName();
	//strInfo = "Put " + strInstName + " on graph";
	//cout<<setw(COUTWIDTH)<<left<<setfill('.')<<strInfo;
	putInstOnGraph( pInst, nX, nY, nZ );
	
	vector< net_C* > vRipNet;

	if( reroute( pInst, vRipNet ) )
	{
		if( !pInst->hasBeenMoved() )
		{
			pInst->hasMoved();
			m_vMovedInstance.push_back( pInst );
		}
		//m_nSuccess++;
		updateForcedModel( pInst );
		freeForcedModel( pInst );
		return true;
	}
	else
	{	
		removeInstOnGraph( pInst );
		recoverInstance( pInst );
		putInstOnGraph( pInst, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ );
		return false;
	}
}
*/

bool router_C::swapInstance(gGrid_C *pSGrid, gGrid_C *pTGrid, vector< instance_C * > &vInst, vector< instance_C * > &vSInst, vector<net_C *> &vRipNet)
{
	// both of the instances are not on the graph now
	int nIX, nIY, nIZ;
	int nSX, nSY, nSZ;
	pSGrid->getPosition(nSX, nSY, nSZ);
	pTGrid->getPosition(nIX, nIY, nIZ);
	// put instances on the graph and check if there exists any overflow
	int nTmpMove = 0;
	for( int i=0; i<vInst.size(); i++ )
	{
		putInstOnGraph(vInst[i], nSX, nSY, m_nOffsetZ);
		if( !vInst[i]->hasBeenMoved() )
			nTmpMove++;
	}
	for( int s=0; s<vSInst.size(); s++ )
	{
		putInstOnGraph(vSInst[s], nIX, nIY, m_nOffsetZ);
		if( !vSInst[s]->hasBeenMoved() )
			nTmpMove++;
	}
	
	bool bIsOverflow;
	if( m_vMovedInstance.size() + nTmpMove <= m_pDesign->getMaxCellConstraint() )
		bIsOverflow = false;
	else
		bIsOverflow = true;
	// check the instance is placeable
	/*
	vector< net_C* > vTmpRipNet;
	if( isPlaceable_ver3( pSGrid, pInst, vTmpRipNet ) )
	{
		if( isPlaceable_ver3( pTGrid, pSInst, vTmpRipNet ) )
		{
			// the variable record the routing nets connected to two instances
			vector< net_C* > vNet; //main net
			set< net_C* > sNet; // query; to check is the net is in the vNet

			// ripup the net 
			
			forced_C* pTmpF = &m_vForced[ pInst->getId() ];
			for( int i=0; i<pTmpF->m_vNetwork.size(); i++ )
			{
				net_C* pTmpNet = pTmpF->m_vNetwork[i]->m_pNet;
				if( sNet.count( pTmpNet ) == 0 )
				{
					vNet.push_back( pTmpNet );
					sNet.insert( pTmpNet );
				}
			}

			pTmpF = &m_vForced[ pSInst->getId() ];
			for( int i=0; i<pTmpF->m_vNetwork.size(); i++ )
			{
				net_C* pTmpNet = pTmpF->m_vNetwork[i]->m_pNet;
				if( sNet.count( pTmpNet ) == 0 )
				{
					vNet.push_back( pTmpNet );
					sNet.insert( pTmpNet );
				}
			}
		
			for( int i=0; i<vNet.size(); i++ )
			{
				vTmpRipNet.push_back( vNet[i] );
			}
			vRipNet = vTmpRipNet;
		}
		else
			bIsOverflow = true;
	}
	else
	{
		bIsOverflow = true;
	}
	*/
	// the variable record if is there exist overlap

	// the variable record the routing nets connected to two instances
	vector<net_C *> vNet; //main net
	set<net_C *> sNet;	  // query; to check is the net is in the vNet

	// ripup the net
	for( int i=0; i<vInst.size(); i++ )
	{
		instance_C* pInst = vInst[i];
		forced_C *pTmpF = &m_vForced[pInst->getId()];
		for (int i = 0; i < pTmpF->m_vNetwork.size(); i++)
		{
			net_C *pTmpNet = pTmpF->m_vNetwork[i]->m_pNet;
			if (sNet.count(pTmpNet) == 0)
			{
				vNet.push_back(pTmpNet);
				sNet.insert(pTmpNet);
			}
		}
	}

	for( int s=0; s<vSInst.size(); s++ )
	{
		instance_C* pSInst = vSInst[s];
		forced_C *pTmpF = &m_vForced[pSInst->getId()];
		for (int i = 0; i < pTmpF->m_vNetwork.size(); i++)
		{
			net_C *pTmpNet = pTmpF->m_vNetwork[i]->m_pNet;
			if (sNet.count(pTmpNet) == 0)
			{
				vNet.push_back(pTmpNet);
				sNet.insert(pTmpNet);
			}
		}
	}
	ripupNet(vNet);

	// check if is no overflow
	gGrid_C *pGrid = pSGrid;
	int nNumLayer = m_pDesign->getLayer().size();
	for (int i = 0; i < nNumLayer; i++)
	{
		if (pGrid->getRemand() < 0)
		{
			bIsOverflow = true;
			break;
		}
		pGrid = graphTravel(m_pDesign, pGrid, 0, 0, 1);
	}

	if (!bIsOverflow)
	{
		pGrid = graphTravel(m_pDesign, pSGrid, 0, 1, 0);
		if (pGrid != NULL)
		{
			for (int i = 0; i < nNumLayer; i++)
			{
				if (pGrid->getRemand() < 0)
				{
					bIsOverflow = true;
					break;
				}
				pGrid = graphTravel(m_pDesign, pGrid, 0, 0, 1);
			}
		}
	}

	if (!bIsOverflow)
	{
		pGrid = graphTravel(m_pDesign, pSGrid, 0, -1, 0);
		if (pGrid != NULL)
		{
			for (int i = 0; i < nNumLayer; i++)
			{
				if (pGrid->getRemand() < 0)
				{
					bIsOverflow = true;
					break;
				}
				pGrid = graphTravel(m_pDesign, pGrid, 0, 0, 1);
			}
		}
	}

	if (!bIsOverflow)
	{
		pGrid = pTGrid;
		for (int i = 0; i < nNumLayer; i++)
		{
			if (pGrid->getRemand() < 0)
			{
				bIsOverflow = true;
				break;
			}
			pGrid = graphTravel(m_pDesign, pGrid, 0, 0, 1);
		}
	}

	if (!bIsOverflow)
	{
		pGrid = graphTravel(m_pDesign, pTGrid, 0, 1, 0);
		if (pGrid != NULL)
		{
			for (int i = 0; i < nNumLayer; i++)
			{
				if (pGrid->getRemand() < 0)
				{
					bIsOverflow = true;
					break;
				}
				pGrid = graphTravel(m_pDesign, pGrid, 0, 0, 1);
			}
		}
	}

	if (!bIsOverflow)
	{
		pGrid = graphTravel(m_pDesign, pTGrid, 0, -1, 0);
		if (pGrid != NULL)
		{
			for (int i = 0; i < nNumLayer; i++)
			{
				if (pGrid->getRemand() < 0)
				{
					bIsOverflow = true;
					break;
				}
				pGrid = graphTravel(m_pDesign, pGrid, 0, 0, 1);
			}
		}
	}
	// recover all the data
	for( int i=0; i<vInst.size(); i++ )
		removeInstOnGraph(vInst[i]);
	for( int s=0; s<vSInst.size(); s++ )
		removeInstOnGraph(vSInst[s]);

	for (int i = 0; i < vNet.size(); i++)
	{
		addNetOnGraph(m_pDesign, vNet[i]);
	}

	// return the result; if no overlap(true) the ripup net will also be returned
	if (bIsOverflow)
		return false;
	else
	{
		vRipNet = vNet;
		return true;
	}
}

bool router_C::swapInstance_ver3(gGrid_C *pSGrid, gGrid_C *pTGrid, instance_C *pInst, instance_C *pSInst, vector<net_C *> &vRipNet, vector< net_C* > &vNotOnGraph )
{
	// both of the instances are not on the graph now
	int nIX, nIY, nIZ;
	int nSX, nSY, nSZ;
	pSGrid->getPosition(nSX, nSY, nSZ);
	pTGrid->getPosition(nIX, nIY, nIZ);
	// put instances on the graph and check if there exists any overflow
	/*
	cout << pSInst->getName() << endl;
	cout << "Before put on graph"<<endl;
	for( int i=0; i<m_vGraph.size(); i++ )
	{
		gGrid_C* pPGrid = getGrid( m_pDesign, nSX, nSY, i+1 );
		cout << pPGrid->getRemand() - pPGrid->getPinDemand() << " ";
	}
	cout << endl;
	for( int i=0; i<m_vGraph.size(); i++ )
	{
		gGrid_C* pPGrid = getGrid( m_pDesign, nIX, nIY, i+1 );
		cout << pPGrid->getRemand() - pPGrid->getPinDemand() << " ";
	}
	cout << endl;
	*/
	cout << "Swapping " << pSInst->getName() << endl;
	//cout << "Before " << getGrid( m_pDesign, 16, 3, 1)->getRemand() << " " << getGrid( m_pDesign, 16, 3, 1 )->getPinDemand() << endl;
	putInstOnGraph(pInst, nSX, nSY, m_nOffsetZ);
	putInstOnGraph(pSInst, nIX, nIY, m_nOffsetZ);
	calPseudoPinDemand( pInst, nSX, nSY, m_nOffsetZ );
	calPseudoPinDemand( pSInst, nIX, nIY, m_nOffsetZ );
	//cout << "After " << getGrid( m_pDesign, 16, 3, 1)->getRemand() << " " << getGrid( m_pDesign, 16, 3, 1 )->getPinDemand() << endl;
	bool bIsOverflow = false;
	/*
	cout << endl;
	cout << "After put on graph"<<endl;
	for( int i=0; i<m_vGraph.size(); i++ )
	{
		gGrid_C* pPGrid = getGrid( m_pDesign, nSX, nSY, i+1 );
		cout << pPGrid->getRemand() - pPGrid->getPinDemand() << " ";
	}
	cout << endl;
	for( int i=0; i<m_vGraph.size(); i++ )
	{
		gGrid_C* pPGrid = getGrid( m_pDesign, nIX, nIY, i+1 );
		cout << pPGrid->getRemand() - pPGrid->getPinDemand() << " ";
	}
	cout << endl;
	*/
	
	// check the instance is placeable
	/*
	vector< net_C* > vTmpRipNet;
	if( isPlaceable_ver3( pSGrid, pInst, vTmpRipNet ) )
	{
		if( isPlaceable_ver3( pTGrid, pSInst, vTmpRipNet ) )
		{
			// the variable record the routing nets connected to two instances
			vector< net_C* > vNet; //main net
			set< net_C* > sNet; // query; to check is the net is in the vNet

			// ripup the net 
			
			forced_C* pTmpF = &m_vForced[ pInst->getId() ];
			for( int i=0; i<pTmpF->m_vNetwork.size(); i++ )
			{
				net_C* pTmpNet = pTmpF->m_vNetwork[i]->m_pNet;
				if( sNet.count( pTmpNet ) == 0 )
				{
					vNet.push_back( pTmpNet );
					sNet.insert( pTmpNet );
				}
			}

			pTmpF = &m_vForced[ pSInst->getId() ];
			for( int i=0; i<pTmpF->m_vNetwork.size(); i++ )
			{
				net_C* pTmpNet = pTmpF->m_vNetwork[i]->m_pNet;
				if( sNet.count( pTmpNet ) == 0 )
				{
					vNet.push_back( pTmpNet );
					sNet.insert( pTmpNet );
				}
			}
		
			for( int i=0; i<vNet.size(); i++ )
			{
				vTmpRipNet.push_back( vNet[i] );
			}
			vRipNet = vTmpRipNet;
		}
		else
			bIsOverflow = true;
	}
	else
	{
		bIsOverflow = true;
	}
	*/
	// the variable record if is there exist overlap

	// the variable record the routing nets connected to two instances
	vector<net_C *> vNet; //main net
	set<net_C *> sNet;	  // query; to check is the net is in the vNet

	for( int i=0; i<vNotOnGraph.size(); i++ )
	{
		sNet.insert( vNotOnGraph[i] );
	}
	// ripup the net

	forced_C *pTmpF = &m_vForced[pInst->getId()];
	for (int i = 0; i < pTmpF->m_vNetwork.size(); i++)
	{
		net_C *pTmpNet = pTmpF->m_vNetwork[i]->m_pNet;
		if (sNet.count(pTmpNet) == 0)
		{
			vNet.push_back(pTmpNet);
			sNet.insert(pTmpNet);
		}
	}

	pTmpF = &m_vForced[pSInst->getId()];
	for (int i = 0; i < pTmpF->m_vNetwork.size(); i++)
	{
		net_C *pTmpNet = pTmpF->m_vNetwork[i]->m_pNet;
		if (sNet.count(pTmpNet) == 0)
		{
			vNet.push_back(pTmpNet);
			sNet.insert(pTmpNet);
		}
	}

	cout << "Rip up: ";
	for( int i=0; i<vNet.size(); i++ )
	{
		cout << vNet[i]->getName() << endl;
	}
	cout << endl;

	vector< gGrid_C* > vPseudoGrid;
	vector< net_C* > vPseudoNet;
	for( int i=0; i<vNet.size(); i++ )
	{
		net_C* pNet = vNet[i];
		vector< pin_C* > vPin = pNet->getPin();
		for( int j=0; j<vPin.size(); j++ )
		{
			instance_C* pTmpInst = ( instance_C* )vPin[j]->getCell();
			if( pTmpInst == pSInst || pTmpInst == pInst )
				continue;
			int nTZ = vPin[j]->getLayerId();
			int nTX = pTmpInst->getPlacedX();
			int nTY = pTmpInst->getPlacedY();
			int nLayerConstraint = max( nTZ, vNet[i]->getConstraintLayerId() );
			for( int z=nTZ; z<=nLayerConstraint; z++ )
			{
				gGrid_C* pTmpGrid = getGrid( m_pDesign, nTX, nTY, z );
				pTmpGrid->addPinDemand( vNet[i] );
				vPseudoGrid.push_back( pTmpGrid );
				vPseudoNet.push_back( vNet[i] );
			}
		}
	}
	ripupNet(vNet);
	//cout << "After Rip Net " << getGrid( m_pDesign, 16, 3, 1)->getRemand() << " " << getGrid( m_pDesign, 16, 3, 1 )->getPinDemand() << endl;
	/*
	cout << "After rip net"<<endl;
	for( int i=0; i<m_vGraph.size(); i++ )
	{
		gGrid_C* pPGrid = getGrid( m_pDesign, nSX, nSY, i+1 );
		cout << pPGrid->getRemand() - pPGrid->getPinDemand() << " ";
	}
	cout << endl;
	for( int i=0; i<m_vGraph.size(); i++ )
	{
		gGrid_C* pPGrid = getGrid( m_pDesign, nIX, nIY, i+1 );
		cout << pPGrid->getRemand() - pPGrid->getPinDemand() << " ";
	}
	cout << endl;
	*/
	// check if is no overflow
	gGrid_C *pGrid = pSGrid;
	int nNumLayer = m_pDesign->getLayer().size();
	for (int i = 0; i < nNumLayer; i++)
	{
		if (pGrid->getRemand() - pGrid->getPinDemand() < 0)
		{
			bIsOverflow = true;
			break;
		}
		pGrid = graphTravel(m_pDesign, pGrid, 0, 0, 1);
	}

	if (!bIsOverflow)
	{
		pGrid = graphTravel(m_pDesign, pSGrid, 0, 1, 0);
		if (pGrid != NULL)
		{
			for (int i = 0; i < nNumLayer; i++)
			{
				if (pGrid->getRemand() - pGrid->getPinDemand() < 0)
				{
					bIsOverflow = true;
					break;
				}
				pGrid = graphTravel(m_pDesign, pGrid, 0, 0, 1);
			}
		}
	}

	if (!bIsOverflow)
	{
		pGrid = graphTravel(m_pDesign, pSGrid, 0, -1, 0);
		if (pGrid != NULL)
		{
			for (int i = 0; i < nNumLayer; i++)
			{
				if (pGrid->getRemand() - pGrid->getPinDemand() < 0)
				{
					bIsOverflow = true;
					break;
				}
				pGrid = graphTravel(m_pDesign, pGrid, 0, 0, 1);
			}
		}
	}

	if (!bIsOverflow)
	{
		pGrid = pTGrid;
		for (int i = 0; i < nNumLayer; i++)
		{
			if (pGrid->getRemand() - pGrid->getPinDemand() < 0 )
			{
				bIsOverflow = true;
				break;
			}
			pGrid = graphTravel(m_pDesign, pGrid, 0, 0, 1);
		}
	}

	if (!bIsOverflow)
	{
		pGrid = graphTravel(m_pDesign, pTGrid, 0, 1, 0);
		if (pGrid != NULL)
		{
			for (int i = 0; i < nNumLayer; i++)
			{
				if (pGrid->getRemand() - pGrid->getPinDemand() < 0)
				{
					bIsOverflow = true;
					break;
				}
				pGrid = graphTravel(m_pDesign, pGrid, 0, 0, 1);
			}
		}
	}

	if (!bIsOverflow)
	{
		pGrid = graphTravel(m_pDesign, pTGrid, 0, -1, 0);
		if (pGrid != NULL)
		{
			for (int i = 0; i < nNumLayer; i++)
			{
				if (pGrid->getRemand() - pGrid->getPinDemand() < 0)
				{
					bIsOverflow = true;
					break;
				}
				pGrid = graphTravel(m_pDesign, pGrid, 0, 0, 1);
			}
		}
	}
	// recover all the data
	delPseudoPinDemand( pInst, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ );
	delPseudoPinDemand( pSInst, pSInst->getPlacedX(), pSInst->getPlacedY(), m_nOffsetZ );
	removeInstOnGraph(pInst);
	removeInstOnGraph(pSInst);

	for( int i=0; i<vPseudoGrid.size(); i++ )
	{
		vPseudoGrid[i]->delPinDemand( vPseudoNet[i] );
	}


	for (int i = 0; i < vNet.size(); i++)
	{
		addNetOnGraph(m_pDesign, vNet[i]);
	}

	// return the result; if no overlap(true) the ripup net will also be returned
	if (bIsOverflow)
		return false;
	else
	{
		vRipNet = vNet;
		cout << "Swap " << pInst->getName() << " at " << pSInst->getPlacedX() << " " << pSInst->getPlacedY() << " and " << pSInst->getName() << " at " << pInst->getPlacedX() << " " << pInst->getPlacedY() << endl;
		
		return true;
	}
}


bool router_C::swapInstance_ver2(gGrid_C *pSGrid, gGrid_C *pTGrid, instance_C *pInst, instance_C *pSInst, vector<net_C *> &vRipNet)
{
	// both of the instances are not on the graph now
	int nIX, nIY, nIZ;
	int nSX, nSY, nSZ;
	pSGrid->getPosition(nSX, nSY, nSZ);
	pTGrid->getPosition(nIX, nIY, nIZ);
	// put instances on the graph and check if there exists any overflow
	putInstOnGraph(pInst, nSX, nSY, m_nOffsetZ);
	calPseudoPinDemand( pInst, nSX, nSY, m_nOffsetZ );

	putInstOnGraph(pSInst, nIX, nIY, m_nOffsetZ);
	calPseudoPinDemand( pSInst, nIX, nIY, m_nOffsetZ );

	bool bIsOverflow = false;
	// check the instance is placeable
	/*
	vector< net_C* > vTmpRipNet;
	if( isPlaceable_ver3( pSGrid, pInst, vTmpRipNet ) )
	{
		if( isPlaceable_ver3( pTGrid, pSInst, vTmpRipNet ) )
		{
			// the variable record the routing nets connected to two instances
			vector< net_C* > vNet; //main net
			set< net_C* > sNet; // query; to check is the net is in the vNet

			// ripup the net 
			
			forced_C* pTmpF = &m_vForced[ pInst->getId() ];
			for( int i=0; i<pTmpF->m_vNetwork.size(); i++ )
			{
				net_C* pTmpNet = pTmpF->m_vNetwork[i]->m_pNet;
				if( sNet.count( pTmpNet ) == 0 )
				{
					vNet.push_back( pTmpNet );
					sNet.insert( pTmpNet );
				}
			}

			pTmpF = &m_vForced[ pSInst->getId() ];
			for( int i=0; i<pTmpF->m_vNetwork.size(); i++ )
			{
				net_C* pTmpNet = pTmpF->m_vNetwork[i]->m_pNet;
				if( sNet.count( pTmpNet ) == 0 )
				{
					vNet.push_back( pTmpNet );
					sNet.insert( pTmpNet );
				}
			}
		
			for( int i=0; i<vNet.size(); i++ )
			{
				vTmpRipNet.push_back( vNet[i] );
			}
			vRipNet = vTmpRipNet;
		}
		else
			bIsOverflow = true;
	}
	else
	{
		bIsOverflow = true;
	}
	*/
	// the variable record if is there exist overlap

	// the variable record the routing nets connected to two instances
	vector<net_C *> vNet; //main net
	set<net_C *> sNet;	  // query; to check is the net is in the vNet

	// ripup the net

	forced_C *pTmpF = &m_vForced[pInst->getId()];
	for (int i = 0; i < pTmpF->m_vNetwork.size(); i++)
	{
		net_C *pTmpNet = pTmpF->m_vNetwork[i]->m_pNet;
		if (sNet.count(pTmpNet) == 0)
		{
			vNet.push_back(pTmpNet);
			sNet.insert(pTmpNet);
		}
	}

	pTmpF = &m_vForced[pSInst->getId()];
	for (int i = 0; i < pTmpF->m_vNetwork.size(); i++)
	{
		net_C *pTmpNet = pTmpF->m_vNetwork[i]->m_pNet;
		if (sNet.count(pTmpNet) == 0)
		{
			vNet.push_back(pTmpNet);
			sNet.insert(pTmpNet);
		}
	}

	ripupNet(vNet);

	// check if is no overflow
	gGrid_C *pGrid = pSGrid;
	int nNumLayer = m_pDesign->getLayer().size();
	for (int i = 0; i < nNumLayer; i++)
	{
		if (pGrid->getRemand() - pGrid->getPinDemand() < 0)
		{
			bIsOverflow = true;
			break;
		}
		pGrid = graphTravel(m_pDesign, pGrid, 0, 0, 1);
	}

	if (!bIsOverflow)
	{
		pGrid = graphTravel(m_pDesign, pSGrid, 0, 1, 0);
		if (pGrid != NULL)
		{
			for (int i = 0; i < nNumLayer; i++)
			{
				if (pGrid->getRemand() - pGrid->getPinDemand() < 0)
				{
					bIsOverflow = true;
					break;
				}
				pGrid = graphTravel(m_pDesign, pGrid, 0, 0, 1);
			}
		}
	}

	if (!bIsOverflow)
	{
		pGrid = graphTravel(m_pDesign, pSGrid, 0, -1, 0);
		if (pGrid != NULL)
		{
			for (int i = 0; i < nNumLayer; i++)
			{
				if (pGrid->getRemand() - pGrid->getPinDemand() < 0)
				{
					bIsOverflow = true;
					break;
				}
				pGrid = graphTravel(m_pDesign, pGrid, 0, 0, 1);
			}
		}
	}

	if (!bIsOverflow)
	{
		pGrid = pTGrid;
		for (int i = 0; i < nNumLayer; i++)
		{
			if (pGrid->getRemand() - pGrid->getPinDemand() < 0)
			{
				bIsOverflow = true;
				break;
			}
			pGrid = graphTravel(m_pDesign, pGrid, 0, 0, 1);
		}
	}

	if (!bIsOverflow)
	{
		pGrid = graphTravel(m_pDesign, pTGrid, 0, 1, 0);
		if (pGrid != NULL)
		{
			for (int i = 0; i < nNumLayer; i++)
			{
				if (pGrid->getRemand() - pGrid->getPinDemand() < 0)
				{
					bIsOverflow = true;
					break;
				}
				pGrid = graphTravel(m_pDesign, pGrid, 0, 0, 1);
			}
		}
	}

	if (!bIsOverflow)
	{
		pGrid = graphTravel(m_pDesign, pTGrid, 0, -1, 0);
		if (pGrid != NULL)
		{
			for (int i = 0; i < nNumLayer; i++)
			{
				if (pGrid->getRemand() - pGrid->getPinDemand() < 0)
				{
					bIsOverflow = true;
					break;
				}
				pGrid = graphTravel(m_pDesign, pGrid, 0, 0, 1);
			}
		}
	}
	// recover all the data
	delPseudoPinDemand( pInst, pInst->getPlacedX(), pInst->getPlacedY(), m_nOffsetZ );
	removeInstOnGraph(pInst);
	delPseudoPinDemand( pSInst, pSInst->getPlacedX(), pSInst->getPlacedY(), m_nOffsetZ );
	removeInstOnGraph(pSInst);

	for (int i = 0; i < vNet.size(); i++)
	{
		addNetOnGraph(m_pDesign, vNet[i]);
	}

	// return the result; if no overlap(true) the ripup net will also be returned
	if (bIsOverflow)
		return false;
	else
	{
		vRipNet = vNet;
		return true;
	}
}

bool router_C::swapInstance(gGrid_C *pSGrid, gGrid_C *pTGrid, instance_C *pInst, instance_C *pSInst, vector<net_C *> &vRipNet)
{
	// both of the instances are not on the graph now
	int nIX, nIY, nIZ;
	int nSX, nSY, nSZ;
	pSGrid->getPosition(nSX, nSY, nSZ);
	pTGrid->getPosition(nIX, nIY, nIZ);
	// put instances on the graph and check if there exists any overflow
	putInstOnGraph(pInst, nSX, nSY, m_nOffsetZ);
	putInstOnGraph(pSInst, nIX, nIY, m_nOffsetZ);

	bool bIsOverflow = false;
	// check the instance is placeable
	/*
	vector< net_C* > vTmpRipNet;
	if( isPlaceable_ver3( pSGrid, pInst, vTmpRipNet ) )
	{
		if( isPlaceable_ver3( pTGrid, pSInst, vTmpRipNet ) )
		{
			// the variable record the routing nets connected to two instances
			vector< net_C* > vNet; //main net
			set< net_C* > sNet; // query; to check is the net is in the vNet

			// ripup the net 
			
			forced_C* pTmpF = &m_vForced[ pInst->getId() ];
			for( int i=0; i<pTmpF->m_vNetwork.size(); i++ )
			{
				net_C* pTmpNet = pTmpF->m_vNetwork[i]->m_pNet;
				if( sNet.count( pTmpNet ) == 0 )
				{
					vNet.push_back( pTmpNet );
					sNet.insert( pTmpNet );
				}
			}

			pTmpF = &m_vForced[ pSInst->getId() ];
			for( int i=0; i<pTmpF->m_vNetwork.size(); i++ )
			{
				net_C* pTmpNet = pTmpF->m_vNetwork[i]->m_pNet;
				if( sNet.count( pTmpNet ) == 0 )
				{
					vNet.push_back( pTmpNet );
					sNet.insert( pTmpNet );
				}
			}
		
			for( int i=0; i<vNet.size(); i++ )
			{
				vTmpRipNet.push_back( vNet[i] );
			}
			vRipNet = vTmpRipNet;
		}
		else
			bIsOverflow = true;
	}
	else
	{
		bIsOverflow = true;
	}
	*/
	// the variable record if is there exist overlap

	// the variable record the routing nets connected to two instances
	vector<net_C *> vNet; //main net
	set<net_C *> sNet;	  // query; to check is the net is in the vNet

	// ripup the net

	forced_C *pTmpF = &m_vForced[pInst->getId()];
	for (int i = 0; i < pTmpF->m_vNetwork.size(); i++)
	{
		net_C *pTmpNet = pTmpF->m_vNetwork[i]->m_pNet;
		if (sNet.count(pTmpNet) == 0)
		{
			vNet.push_back(pTmpNet);
			sNet.insert(pTmpNet);
		}
	}

	pTmpF = &m_vForced[pSInst->getId()];
	for (int i = 0; i < pTmpF->m_vNetwork.size(); i++)
	{
		net_C *pTmpNet = pTmpF->m_vNetwork[i]->m_pNet;
		if (sNet.count(pTmpNet) == 0)
		{
			vNet.push_back(pTmpNet);
			sNet.insert(pTmpNet);
		}
	}

	ripupNet(vNet);

	// check if is no overflow
	gGrid_C *pGrid = pSGrid;
	int nNumLayer = m_pDesign->getLayer().size();
	for (int i = 0; i < nNumLayer; i++)
	{
		if (pGrid->getRemand() < 0)
		{
			bIsOverflow = true;
			break;
		}
		pGrid = graphTravel(m_pDesign, pGrid, 0, 0, 1);
	}

	if (!bIsOverflow)
	{
		pGrid = graphTravel(m_pDesign, pSGrid, 0, 1, 0);
		if (pGrid != NULL)
		{
			for (int i = 0; i < nNumLayer; i++)
			{
				if (pGrid->getRemand() < 0)
				{
					bIsOverflow = true;
					break;
				}
				pGrid = graphTravel(m_pDesign, pGrid, 0, 0, 1);
			}
		}
	}

	if (!bIsOverflow)
	{
		pGrid = graphTravel(m_pDesign, pSGrid, 0, -1, 0);
		if (pGrid != NULL)
		{
			for (int i = 0; i < nNumLayer; i++)
			{
				if (pGrid->getRemand() < 0)
				{
					bIsOverflow = true;
					break;
				}
				pGrid = graphTravel(m_pDesign, pGrid, 0, 0, 1);
			}
		}
	}

	if (!bIsOverflow)
	{
		pGrid = pTGrid;
		for (int i = 0; i < nNumLayer; i++)
		{
			if (pGrid->getRemand() < 0)
			{
				bIsOverflow = true;
				break;
			}
			pGrid = graphTravel(m_pDesign, pGrid, 0, 0, 1);
		}
	}

	if (!bIsOverflow)
	{
		pGrid = graphTravel(m_pDesign, pTGrid, 0, 1, 0);
		if (pGrid != NULL)
		{
			for (int i = 0; i < nNumLayer; i++)
			{
				if (pGrid->getRemand() < 0)
				{
					bIsOverflow = true;
					break;
				}
				pGrid = graphTravel(m_pDesign, pGrid, 0, 0, 1);
			}
		}
	}

	if (!bIsOverflow)
	{
		pGrid = graphTravel(m_pDesign, pTGrid, 0, -1, 0);
		if (pGrid != NULL)
		{
			for (int i = 0; i < nNumLayer; i++)
			{
				if (pGrid->getRemand() < 0)
				{
					bIsOverflow = true;
					break;
				}
				pGrid = graphTravel(m_pDesign, pGrid, 0, 0, 1);
			}
		}
	}
	// recover all the data
	removeInstOnGraph(pInst);
	removeInstOnGraph(pSInst);

	for (int i = 0; i < vNet.size(); i++)
	{
		addNetOnGraph(m_pDesign, vNet[i]);
	}

	// return the result; if no overlap(true) the ripup net will also be returned
	if (bIsOverflow)
		return false;
	else
	{
		vRipNet = vNet;
		return true;
	}
}

vector<instance_C *> router_C::findSwapInstance_ver2(gGrid_C *pSGrid, gGrid_C *pTGrid, instance_C *pInst) // grid that instance will put, grid that instance orign place, instance
{
	// instance in the source grid;
	vector<instance_C *> vInst = pSGrid->getInstance();
	// candidate of swapping instance, will be sorted by forced;
	vector<instance_C *> vCandidate;
	vector<int> vScore;

	// tmp variable
	vector<forced_C *> vForced;
	int nTX, nTY, nTZ;
	pTGrid->getPosition(nTX, nTY, nTZ);
	

	// find all the forced model of instance in the grid
	for (int i = 0; i < vInst.size(); i++)
	{
		forced_C *pF = &m_vForced[vInst[i]->getId()];
		if (!pInst->hasBeenMoved() && m_vMovedInstance.size() < m_pDesign->getMaxCellConstraint() - 1)
			if (pF->m_pInstance->isMovable())
				vForced.push_back(pF);
			else if (m_vMovedInstance.size() < m_pDesign->getMaxCellConstraint())
				if (pF->m_pInstance->isMovable())
					vForced.push_back(pF);
				else if (pF->m_pInstance->isMovable() && pF->m_pInstance->hasBeenMoved())
					vForced.push_back(pF);
	}

	for (int i = 0; i < vForced.size(); i++)
	{
		forced_C *pF = vForced[i];
		vector< net_C* > vNet;
		vector< int > vExtraCost;
		for( int i=0; i<pF->m_vNetwork.size(); i++ )
		{
			vNet.push_back( pF->m_vNetwork[i]->m_pNet );
			vExtraCost.push_back( pF->m_vNetwork[i]->m_pNet->getConstraintLayerId() );
		}
		if (pF->m_nBoundX1 <= nTX && nTX <= pF->m_nBoundX2 && pF->m_nBoundY1 <= nTY && nTY <= pF->m_nBoundY2)
		{
			//int nCost = abs(nCenterX - nTX) + abs(nCenterY - nTY);
			int nCost = boundingBoxCost(nTX, pF->m_vMinX, pF->m_vMaxX) + boundingBoxCost(nTY, pF->m_vMinY, pF->m_vMaxY);
			//int nCost = 0;
			int nExtra = 0;
			vector< instance_C* > vTmpInst = pTGrid->getInstance();
			vector< bool > vNetInfo;
			for( int j=0; j<vNet.size(); j++ )
				vNetInfo.push_back( false );
			for( int j=0; j<vTmpInst.size(); j++ )
			{
				forced_C* pTF = &m_vForced[ vTmpInst[j]->getId() ];
				vector< networkForced_C* > &vTNF = pTF->m_vNetwork;
				for( int c=0; c<vNet.size(); c++ )
				{
					for( int n=0; n<vTNF.size(); n++ )
					{
						if( vTNF[n]->m_pNet == vNet[c] )
						{
							vNetInfo[ c ] = true;
							break;
						}
					}
				}	
			}
		
			for( int j=0; j<vNet.size(); j++ )
			{
				if( !vNetInfo[j] )
					nExtra = nExtra + vExtraCost[j];
			}
			nCost = nCost + nExtra;

			vScore.push_back(nCost);
			vCandidate.push_back(pF->m_pInstance);
		}
		else
		{
			int nCost = boundingBoxCost(nTX, pF->m_vMinX, pF->m_vMaxX) + boundingBoxCost(nTY, pF->m_vMinY, pF->m_vMaxY);

	//		
			int nExtra = 0;
			vector< instance_C* > vTmpInst = pTGrid->getInstance();
			vector< bool > vNetInfo;
			for( int j=0; j<vNet.size(); j++ )
				vNetInfo.push_back( false );
			for( int j=0; j<vTmpInst.size(); j++ )
			{
				forced_C* pTF = &m_vForced[ vTmpInst[j]->getId() ];
				vector< networkForced_C* > &vTNF = pTF->m_vNetwork;
				for( int c=0; c<vNet.size(); c++ )
				{
					for( int n=0; n<vTNF.size(); n++ )
					{
						if( vTNF[n]->m_pNet == vNet[c] )
						{
							vNetInfo[ c ] = true;
							break;
						}
					}
				}	
			}
			
			for( int j=0; j<vNet.size(); j++ )
			{
				if( !vNetInfo[j] )
					nExtra = nExtra + vExtraCost[j];
			}
			nCost = nCost + nExtra;
			
			vScore.push_back(nCost);
			vCandidate.push_back(pF->m_pInstance);

		}
	}

	// increasing order
	for (int i = 1; i < vScore.size(); i++)
	{
		for (int j = i - 1; j >= 0; j--)
		{
			int nFCost = vScore[j];
			int nBCost = vScore[j + 1];
			if (nFCost <= nBCost)
				break;
			else
			{
				instance_C *pFInst = vCandidate[j];
				instance_C *pBInst = vCandidate[j + 1];
				vCandidate[j] = pBInst;
				vCandidate[j + 1] = pFInst;
				vScore[j] = nBCost;
				vScore[j + 1] = nFCost;
			}
		}
	}

	return vCandidate;
}

vector<instance_C *> router_C::findSwapInstance(gGrid_C *pSGrid, gGrid_C *pTGrid, instance_C *pInst) // grid that instance will put, grid that instance orign place, instance
{
	// instance in the source grid;
	vector<instance_C *> vInst = pSGrid->getInstance();
	// candidate of swapping instance, will be sorted by forced;
	vector<instance_C *> vCandidate;
	vector<int> vScore;

	// tmp variable
	vector<forced_C *> vForced;
	int nTX, nTY, nTZ;
	pTGrid->getPosition(nTX, nTY, nTZ);

	// find all the forced model of instance in the grid
	for (int i = 0; i < vInst.size(); i++)
	{
		forced_C *pF = &m_vForced[vInst[i]->getId()];
		if (!pInst->hasBeenMoved() && m_vMovedInstance.size() < m_pDesign->getMaxCellConstraint() - 1)
			if (pF->m_pInstance->isMovable())
				vForced.push_back(pF);
			else if (m_vMovedInstance.size() < m_pDesign->getMaxCellConstraint())
				if (pF->m_pInstance->isMovable())
					vForced.push_back(pF);
				else if (pF->m_pInstance->isMovable() && pF->m_pInstance->hasBeenMoved())
					vForced.push_back(pF);
	}

	for (int i = 0; i < vForced.size(); i++)
	{
		forced_C *pF = vForced[i];
		if (pF->m_nBoundX1 <= nTX && nTX <= pF->m_nBoundX2 && pF->m_nBoundY1 <= nTY && nTY <= pF->m_nBoundY2)
		{
			int nCenterX = (pF->m_nBoundX1 + pF->m_nBoundX2) / 2;
			int nCenterY = (pF->m_nBoundY1 + pF->m_nBoundY2) / 2;
			int nCost = abs(nCenterX - nTX) + abs(nCenterY - nTY);
			vScore.push_back(nCost);
			vCandidate.push_back(pF->m_pInstance);
		}
		//else if( pF->m_nL == 0 && pF->m_nR == 0 && pF->m_nT == 0 && pF->m_nD == 0 )
		//{
		//	vScore.push_back( ( (pF->m_nBoundX2 - pF->m_nBoundX1) + (pF->m_nBoundY2 - pF->m_nBoundY1) )/2 );
		//	vCandidate.push_back( pF->m_pInstance );
		//}
	}

	// increasing order
	for (int i = 1; i < vScore.size(); i++)
	{
		for (int j = i - 1; j >= 0; j--)
		{
			int nFCost = vScore[j];
			int nBCost = vScore[j + 1];
			if (nFCost <= nBCost)
				break;
			else
			{
				instance_C *pFInst = vCandidate[j];
				instance_C *pBInst = vCandidate[j + 1];
				vCandidate[j] = pBInst;
				vCandidate[j + 1] = pFInst;
				vScore[j] = nBCost;
				vScore[j + 1] = nFCost;
			}
		}
	}

	return vCandidate;
}

inline bool router_C::recoverInstance( vector< instance_C* > &vInst )
{
	//cout << "Backup Info: "<<endl;
	//for( int i=0; i<vInst.size(); i++ )
	for( int i=vInst.size() - 1; i>=0; i-- )
	{
		instance_C* pInst = vInst[i];
		pInst->setPlaced( m_vBackupInstance[i].getPlacedX(), m_vBackupInstance[i].getPlacedY() );
		//cout << vInst[i]->getName() << " " << m_vBackupInstance[i].getName() << " " << m_vBackupInstance[i].getPlacedX() << " " << m_vBackupInstance[i].getPlacedY() << endl; 
	}
	return true;
}

bool router_C::backupInstance( vector< instance_C* > &vInst )
{
	for( int i=0; i<vInst.size(); i++ )
	{
		instance_C* pInst = vInst[i];
		instance_C cBackupInstance;
		cBackupInstance.setPlaced(pInst->getPlacedX(), pInst->getPlacedY());	
		cBackupInstance.setName( pInst->getName() );
		m_vBackupInstance.push_back( cBackupInstance );
	}
	return true;
}


bool router_C::multipleCellMovement( net_C *pNet )
{
	forced_net_C &cFN = m_vNetForced[ pNet->getId() ];
	vector< instance_C* > vInst = cFN.m_vCluster;
	backupInstance( vInst );

	for( int i=0; i<vInst.size(); i++ )
	{
		removeInstOnGraph( vInst[i] );
	}

	// setup the bounding box for the instance
	int nMinX = m_nTX;
	int nMaxX = m_nDX;
	int nMinY = m_nTY;
	int nMaxY = m_nDY;

	for( int i=0; i<vInst.size(); i++ )
	{
		int nX = vInst[i]->getPlacedX(); int nY = vInst[i]->getPlacedY();
		if( nMinX > nX )
		{
			nMinX = nX;
		}
		if( nMaxX < nX )
		{
			nMaxX = nX;
		}
		if( nMinY > nY )
		{
			nMinY = nY;
		}
		if( nMaxY < nY )
		{
			nMaxY = nY;
		}
	}

	int nCenterX = abs( nMaxX + nMinX )/2;
	int nCenterY = abs( nMaxY + nMinY )/2;
	int nDX = cFN.m_nTF - cFN.m_nDF;
	int nDY = cFN.m_nRF - cFN.m_nLF;
	if( nDX != 0 )
		nDX = nDX / abs( nDX );
	if( nDY != 0 )
		nDY = nDY / abs( nDY );
	//
	
	// sort the instance 
	for( int i=1; i<vInst.size(); i++ )
	{
		for( int j=i-1; j>=0; j-- )
		{
			int nFNumNet = m_vForced[ vInst[j]->getId() ].m_vNetwork.size();
			int nBNumNet = m_vForced[ vInst[j+1]->getId() ].m_vNetwork.size();
			if( nFNumNet < nBNumNet )
			{
				instance_C* pInst = vInst[j];
				vInst[j] = vInst[j+1];
				vInst[j+1] = pInst;
			}
			else
				break;
		}
	}
	//

	// move the cell iteratively
	bool bPlacedAllSuccess = true;
	for( int i=0; i<vInst.size(); i++ )
	{
		int nBX1, nBX2;
		int nBY1, nBY2;
		forced_C &cTmpF = m_vForced[ vInst[i]->getId() ];
		vector< networkForced_C* > &vTmpNF = cTmpF.m_vNetwork;
		set< int > sInterNetId;
		vector< net_C* > vInterNet = cFN.m_vInterNet;
		for( int n=0; n<vInterNet.size(); n++ )
		{
			sInterNetId.insert( vInterNet[n]->getId() );
		}

		vector< int > vMinX;
		vector< int > vMaxX;
		vector< int > vMinY;
		vector< int > vMaxY;
		
		for( int n=0; n<cTmpF.m_vNetwork.size(); n++ )
		{
			int nNetId = cTmpF.m_vNetwork[n]->m_pNet->getId();
			if( sInterNetId.count( nNetId ) == 0 )
			{
				continue;
			}

			vMinX.push_back( cTmpF.m_vMinX[n] );
			vMaxX.push_back( cTmpF.m_vMaxX[n] );
			vMinY.push_back( cTmpF.m_vMinY[n] );
			vMaxY.push_back( cTmpF.m_vMaxY[n] );
		}

		if( nDX != 0 )
		{
			int nBest;
			int nTmp;
			nTmp = cTmpF.m_pInstance->getPlacedX();
			nBest = boundingBoxCost( nTmp, vMinX, vMaxX );
			gGrid_C* pBestGrid = NULL;
			gGrid_C* pGrid = getGrid( m_pDesign, cTmpF.m_pInstance->getPlacedX(), cTmpF.m_pInstance->getPlacedY(), m_nDZ );
			while( graphTravel( m_pDesign, pGrid, nDX, 0, 0 ) != NULL )
			{
				pGrid = graphTravel(m_pDesign, pGrid, nDX, 0, 0);
				int nTmpX, nTmpY, nTmpZ;
				pGrid->getPosition(nTmpX, nTmpY, nTmpZ);
				nTmp = nTmpX;

				int nTmpBest = boundingBoxCost(nTmp, vMinX, vMaxX);

				if (nTmpBest <= nBest) // change from <
				{
					pBestGrid = pGrid;
					nBX2 = nBX2 + nDX;
				}
				else
					break;
				
			}

		}
		
		if( nDY != 0 )
		{
			int nBest;
			int nTmp;
			nTmp = cTmpF.m_pInstance->getPlacedY();
			nBest = boundingBoxCost( nTmp, vMinY, vMaxY );
			gGrid_C* pBestGrid = NULL;
			gGrid_C* pGrid = getGrid( m_pDesign, cTmpF.m_pInstance->getPlacedX(), cTmpF.m_pInstance->getPlacedY(), m_nDZ );
			while( graphTravel( m_pDesign, pGrid, 0, nDY, 0 ) != NULL )
			{
				pGrid = graphTravel(m_pDesign, pGrid, 0, nDY, 0);
				int nTmpX, nTmpY, nTmpZ;
				pGrid->getPosition(nTmpX, nTmpY, nTmpZ);
				nTmp = nTmpY;

				int nTmpBest = boundingBoxCost( nTmp, vMinY, vMaxY );

				if (nTmpBest <= nBest) // change from <
				{
					pBestGrid = pGrid;
					nBY2 = nBY2 + nDY;
				}
				else
					break;
				
			}

		}

		if( nBX1 > nBX2 )
		{
			int nTmpX = nBX1;
			nBX1 = nBX2;
			nBX2 = nTmpX;
		}
		
		if( nBY1 > nBY2 )
		{
			int nTmpY = nBY1;
			nBY1 = nBY2;
			nBY2 = nTmpY;
		}
		vector< gGrid_C* > vBestGrid;
		vector< int > vBestGridCost;
		for (int nX = nBX1; nX <= nBX2; nX++)
		{
			for (int nY = nBY1; nY <= nBY2; nY++)
			{
				gGrid_C *pPGrid = getGrid(m_pDesign, nX, nY, m_nOffsetZ);
				int nCost = boundingBoxCost(nX, vMinX, vMaxX) + boundingBoxCost(nY, vMinY, vMaxY);

				vBestGridCost.push_back(nCost);
				vBestGrid.push_back(pPGrid);
			}
		}
		
		for (int f = 1; f < vBestGrid.size(); f++)
		{
			for (int j = f - 1; j >= 0; j--)
			{
				//if( !cF.m_bLockInX )
				//	nDX = cF.m_nR - cF.m_nL;
				//if( !cF.m_bLockInY )
				//	nDY = cF.m_nT - cF.m_nD;

				//int nTmpX, nTmpY, nTmpZ, nTmpB, nTmpF;
				gGrid_C *pBG = vBestGrid[j + 1];
				//pBG->getPosition( nTmpX, nTmpY, nTmpZ );

				//int nBCost = boundingBoxCost( nTmpX, vMinX, vMaxX )
				//	   + boundingBoxCost( nTmpY, vMinY, vMaxY ) ;
				int nBCost = vBestGridCost[j + 1];
				int nFCost = vBestGridCost[j];

				gGrid_C *pFG = vBestGrid[j];
				//pFG->getPosition( nTmpX, nTmpY, nTmpZ );

				//int nFCost = boundingBoxCost( nTmpX, vMinX, vMaxX )
				//	   + boundingBoxCost( nTmpY, vMinY, vMaxY ) ;

				if (nFCost >= nBCost)
					break;
				else
				{
					vBestGrid[j + 1] = pFG;
					vBestGrid[j] = pBG;
					vBestGridCost[j + 1] = nFCost;
					vBestGridCost[j] = nBCost;
				}
			}
		}
		
		if( vBestGrid.size() == 0 )
		{
			bPlacedAllSuccess = false;
			break;
		}

	}
	//

	if( !bPlacedAllSuccess )
	{
		recoverInstance( vInst );
		for( int i=0; i<vInst.size(); i++ )
		{
			lockInstance( vInst[i] );
			putInstOnGraph( vInst[i], vInst[i]->getPlacedX(), vInst[i]->getPlacedY(), m_nOffsetZ );
		}
		m_vBackupInstance.clear();
		return false;
	}
	else
	{
		for( int i=0; i<vInst.size(); i++ )
		{
			if( !vInst[i]->hasBeenMoved() )
			{
				vInst[i]->hasMoved();
				m_vMovedInstance.push_back( vInst[i] );
			}
			updateForcedModel( vInst[i] );
			freeForcedModel( vInst[i] );
		}
		m_vBackupInstance.clear();
		return true;
	}
}

//#define _NORMAL_MODE // mark if what to test

#ifndef _NORMAL_MODE
#define _TEST_MODE
#define _TEST_LOOP 20
#define _TEST_INST "C1"
#define _TEST_CELL "MC1"
#endif

bool router_C::startOpt()
{
	//time_t start, end;
	//start = time( NULL );
	cout << "Start Routing Optimization" << endl;
	int nNumMaxCell = m_pDesign->getMaxCellConstraint();
	//int nTmpSuccess = 0;
	//vector< instance_C* > vFailedInst;
	time_t tStartRoute, tEndRoute, tRequiredTime;

	vector<net_C *> vTmpNet = m_pDesign->getNet();
	//if( !globalRoute( vTmpNet ) )
	//{
	tStartRoute = time(NULL);
	//cout<<"End sorting"<<endl;
	
	pre_route_ver3(vTmpNet);
	pre_route_ver4(vTmpNet);
	
	//pre_route_ver3(vTmpNet);
	//pre_route_ver2(vTmpNet);
	
	cout << "After pre reroute: " << calTotalWireLength() << endl;
	//rrr( vTmpNet );
	tEndRoute = time(NULL);
	tRequiredTime = tEndRoute - tStartRoute;
#ifdef _DEBUG_MODE
	vector<bool> vProcessResult;
	vector<int> vNumCellMovement;
#endif
	//}
//------------------- testing ------------------------
#ifdef _TEST_MODE
	//for( int i=0; i<_TEST_LOOP; i++ )
	//nNumMaxCell = 0;
	int nTmpInst = 0;
	int nIter = 0;
	int nFailCount = 0;
	bool bUpToLimit = false;
	while (1)
	//while(0)
	//while( m_vMovedInstance.size() < nNumMaxCell )
	//while( m_vMovedInstance.size() < 200 )
	{
		nIter++;
		//if( nIter > 1000 )
		//	break;
		// pick instance

		//if( m_nTX >= 13 && m_nTY >= 14 && getGrid( m_pDesign, 13, 14, 2)->isOverflow() )
		//	cout<<"Find (13, 14, 2) Overflow"<<endl;

		endt = time(NULL);
		int nTime = endt - start;
		if (nTime >= (time_t)TIMECONST - tRequiredTime - (time_t)10)
			break;

		instance_C *pInst;
		vector< instance_C* > vInst;
		boundry_C* pB = NULL;

		if( m_vMovedInstance.size() == nNumMaxCell && !bUpToLimit )
		{
			bUpToLimit = true;
			for( int i=0; i<m_vBoundry.size(); i++ )
			{
				m_vBoundry[i]->m_bLock = false;
			}
		}

		if( m_vMovedInstance.size() <= nNumMaxCell )
			//pInst = pickInstanceToMove_ver2();
			//pB = pickInstanceToMove_ver3();
			//pB = pickInstanceToMove_ver4();
			pB = pickInstanceToMove_ver5();
		//else
			//pInst = pickHasMovedInstanceToMove();
		else
			break;
		//if ( pInst == NULL )
		if( pB == NULL )
		{
			//if( nTmpInst == m_vMovedInstance.size() )
			//{
				break;
			//}
			//else
			//{
			//	nTmpInst = m_vMovedInstance.size();
			//	for( int i=0; i<m_vMovedInstance.size(); i++ )
			//	{
			//		freeInstance( m_vMovedInstance[i] );
			//	}
			//	continue;
			//}
		}
		else
		{
			for( int i=0; i<pB->m_vForced.size(); i++ )
			{
				vInst.push_back( pB->m_vForced[i]->m_pInstance );
			}
			cout << "Pick: "<< pB->m_pNetwork->m_pNet->getName() << " " << pB->m_cType << " ";
			if( pB->m_cType == 'T' )
				cout << pB->m_pNetwork->m_nMaxX; 
			else if( pB->m_cType == 'D' )
				cout << pB->m_pNetwork->m_nMinX; 
			else if( pB->m_cType == 'R' )
				cout << pB->m_pNetwork->m_nMaxY; 
			else if( pB->m_cType == 'L' )
				cout << pB->m_pNetwork->m_nMinY; 
			
			cout <<" F: " << pB->m_nPosF << " Instance: ";
			for( int i=0; i<vInst.size(); i++ )
			{
				cout << vInst[i]->getName() << " (" << vInst[i]->getPlacedX() << " " << vInst[i]->getPlacedY() << ") ";
			}
			cout << endl;
		}
		/*
		else
		{
			//cout << "Pick: "<<pInst->getName()<<endl;
			vInst.push_back( pInst );
			//vector< forced_C* > vTmpFI = recalForced( pInst->getId() );
			vector< forced_C* > vTmpFI = moveingCellCollection_ver2( pInst->getId() );
			for( int f=0; f<vTmpFI.size(); f++ )
			{
				cout << vTmpFI[f]->m_pInstance->getName() << " ";
				//if( m_vMovedInstance.size() + vInst.size() < m_pDesign->getMaxCellConstraint() )
					vInst.push_back( vTmpFI[f]->m_pInstance );
				//else
				//	break;
			}
			cout << endl;
			//cout << "\tadditional move " << vInst.size() - 1 << endl;
		}
		*/
		m_nIteration++;
		/*
		vector< instance_C* > vInst = m_pDesign->getInstance();
		for( int i=0; i<vInst.size(); i++ )
		{
			if( vInst[i]->getName() == _TEST_INST && vInst[i]->getType() == _TEST_CELL )
			{
				pInst = vInst[i];
				break;
			}
		}
		*/
		//cout<<setw(COUTWIDTH)<<left<<setfill('.')<<"Update forced model";
		//string strInstName = pInst->getName();
		//string strInfo = ": Moving " + strInstName;

		//string strInfo = ": Moving ";
		//strInfo.append(strInstName);
		//cerr << "Iter " << m_nIteration << setw(COUTWIDTH - 6) << left << setfill('.') << strInfo;

		//if( singleCellMovement_ver2( pInst ) ) // it can get gain 4460 for case 3 // test for more place to move
		/*		
		if (singleCellMovement_ver3(pInst))
		{
			m_nSuccess++;
			cout << "Iter " << m_nIteration << setw(COUTWIDTH - 6) << left << setfill('.') << " " <<"left: "<<m_pDesign->getMaxCellConstraint() - m_vMovedInstance.size() << endl;	
			nSglCount++;
		}
		
		else
		*/
		int nOrigin = calTotalWireLength();
		if( moveCell( vInst, pB ) )
		{
			//getchar();	
			m_nSuccess++;
			//	nTmpSuccess++;
			cout << "Iter " << m_nIteration << setw(COUTWIDTH - 6) << left << setfill('.') << " " <<"left: "<<m_pDesign->getMaxCellConstraint() - m_vMovedInstance.size() << endl;
			//nMulCount++;
		//	cerr << "complete" << endl;
#ifdef _DEBUG_MODE
			vProcessResult.push_back(true);
#endif
		}
		
		/*
		else	
		if (singleCellMovement_ver3(pInst))
		{
			m_nSuccess++;
			cout << "Iter " << m_nIteration << setw(COUTWIDTH - 6) << left << setfill('.') << " " <<"left: "<<m_pDesign->getMaxCellConstraint() - m_vMovedInstance.size() << endl;	
			nSglCount++;
		}
		*/	
		else
		{
			pB->m_bLock = true;
			cout << "lock"<<endl;
			m_nFailed++;
			//	vFailedInst.push_back( pInst );
			//nTmpFailed++;
		//	cerr << "failed" << endl;
			nFailCount++;
#ifdef _DEBUG_MODE
			vProcessResult.push_back(false);
#endif
		}

#ifdef _DEBUG_MODE
		vNumCellMovement.push_back(m_vMovedInstance.size());
#endif
		int nDiff = nOrigin - calTotalWireLength();
		cout << "Reduce: " << nDiff << endl;
		cout << endl;
		checkPseudoPin();
		//checkOverflow();
		if( !checkOverflow() )
			//cout << "Error" << endl;
			break;
		checkLength();


		/*
		if( nTmpFailed == 10 )
		{
			m_nIteration++;
			strInfo = ": Routing refinement";
			cout<<"Iter "<<m_nIteration<<setw(COUTWIDTH - 6)<<left<<setfill('.')<<strInfo;
			rrr( vTmpNet );
			cout<<"complete"<<endl;
			nTmpFailed = 0;
			m_nNumRefinement++;
		}
		*/
		/*
		if( nTmpSuccess == 20 /
		{
			rrr( vTmpNet );
			for( int i=0; i<vFailedInst.size(); i++ )
			{
				freeInstance( vFailedInst[i] );
			}
			nTmpSuccess = 0;
			vFailedInst.clear();
		}
		*/
		// setup the bounding box
		// find out the candidate grid for instance
		// find out the best position for instance
		// map the candidates and best position

#endif

//------------------- normal -------------------------
#ifdef _NORMAL_MODE
		cout << "Normal" << endl;
		while (1)
		{
			break;
#endif
		}

#ifdef _DEBUG_MODE
		//plot_process_result(vProcessResult, 20, "execute_curve.m");
		//plot_movement_process(vNumCellMovement, "cell_move_curve.m");
#endif
		cout << "Single Cell Move:   " << nSglCount << endl;
		cout << "Single Cell Failed Move:   " << nSglFCount << endl;
		cout << "Multiple Cell Move: "<< nMulCount << endl;
		cout << "Multiple Cell Failed Move: "<< nMulFCount << endl;
		cout << "Failed Move:        "<< nFailCount << endl;
		cout << endl;
		cout << "Start post-routing optimization" << endl;
		//vector< net_C* > vTmpNet = m_pDesign->getNet();

		//rrr( vTmpNet );
		//pre_route_ver2(vTmpNet);
		pre_route_ver4(vTmpNet);
		/*
	vector< net_C* > vNet = m_pDesign->getNet();
	for( int i=0; i<vNet.size(); i++ )
	{
		matlab_graph( vNet[i]->getName(), vNet[i]->getName(), m_pDesign, 0 );
	}
	*/
		checkLength();
		int nIdealLength = 0;
		for( int i=0; i<m_vNetworkForced.size(); i++ )
		{
			nIdealLength = nIdealLength + routeNet_ideal( m_vNetworkForced[i].m_pNet );
		}
		cout << "Ideal wire length is: " << nIdealLength << endl;
		cout << "End optimization" << endl;
		cout << endl;

		endt = time(NULL);
		//cout<<"Time: "<< end - start <<endl;

		return true;
	}

	bool router_C::test()
	{
		vector<net_C *> vNet = m_pDesign->getNet();
		string strCheckNet = "N2";
		for (int i = 0; i < vNet.size(); i++)
		{
			if (vNet[i]->getName() == strCheckNet)
			{

				matlab_graph("out", strCheckNet, m_pDesign, 0);
				int nLength = calWireLength(vNet[i]);
				cout << "Previous length: " << nLength << endl;
				vector<net_C *> vTmpNet;
				vector<forced_C *> vF = m_vNetworkForced[vNet[i]->getId()].m_vForced;
				unsigned int *nX = new unsigned int[vF.size()];
				unsigned int *nY = new unsigned int[vF.size()];
				for (int p = 0; p < vF.size(); p++)
				{
					nX[p] = vF[p]->m_pInstance->getPlacedX();
					nY[p] = vF[p]->m_pInstance->getPlacedY();
				}
				int nWL = flute_wl(vF.size(), nX, nY, ACCURACY);
				cout << "Estimate: " << nWL << endl;

				vTmpNet.push_back(vNet[i]);
				backupNet(vTmpNet);
				ripupNet(vTmpNet);
				cleanWire(vTmpNet);
				//vector< gGrid_C* > vResult = routeNet( vNet[i] );
				vector<gGrid_C *> vResult = routeNet_length_constraint(vNet[i], nLength);
				saveNet(vNet[i], vResult);
				addNetOnGraph(m_pDesign, vNet[i]);

				nLength = calWireLength(vNet[i]);
				cout << "After reroute: " << nLength << endl;

				matlab_graph("out", strCheckNet, m_pDesign, 1);
				ripupNet(vTmpNet);

				recoverNet(vTmpNet);

				addNetOnGraph(m_pDesign, vNet[i]);

				m_vBackupNet.clear();
			}
		}
	}
	
	bool router_C::pre_route_ver4(vector<net_C *> & vNet)
	{
		//vector< int > vLength;
		vector< int > vIdealLength;
		unordered_map< net_C*, int > mLengthTable;
		for (int i = 0; i < vNet.size(); i++)
		{
			vNet[i]->setLength(calWireLength(vNet[i]));
			mLengthTable[ vNet[i] ] = 0;
		}
		for (int i = 1; i < vNet.size(); i++)
		{
			//cout<<i;
			for (int j = i - 1; j >= 0; j--)
			{
				//int nFLength = vLength[j];
				//int nBLength = vLength[j+1];
				int nFLength = vNet[j]->getLength();
				int nBLength = vNet[j + 1]->getLength();
				;
				if (nFLength >= nBLength)
					break;
				else
				{
					net_C *pTmpNet = vNet[j];
					vNet[j] = vNet[j + 1];
					vNet[j + 1] = pTmpNet;
					//int nLength = vLength[j];
					//vLength[j] = vLength[j+1];
					//vLength[j+1] = nLength;
				}
			}
		}
		for (int i = 0; i < vNet.size(); i++)
		{
			vIdealLength.push_back( routeNet_ideal( vNet[i] ) );
		}

		for (int i = 0; i < vNet.size(); i++)
		{
			endt = time(NULL);
			int nTime = endt - start;
			if (endt - start > TIMECONST)
				break;

			//if( vNet[i]->getNumReroute() > 0  )
			//	continue;

			vector<net_C *> vRerouteNet;
			vRerouteNet.push_back(vNet[i]);

			int nPreviousLength = 0;
			for (int n = 0; n < vRerouteNet.size(); n++)
			{
				//nPreviousLength = nPreviousLength + calWireLength( vRerouteNet[n] );
				nPreviousLength = nPreviousLength + vRerouteNet[n]->getLength();
			}

			//if( vNet[i]->getName() == "N1482" )
			//	cout<<nPreviousLength<<endl;

			//cout<<"backup net"<<endl;
			backupNet(vRerouteNet);
			//cout<<"ripup net"<<endl;
			ripupNet(vRerouteNet);
			cleanWire(vRerouteNet);
			//cout<<"route the net..."<<endl;

			bool bFail = false;
			int nNetIndex = vRerouteNet.size();
			for (int r = 0; r < vRerouteNet.size(); r++)
			{
				//vector< gGrid_C* > vResult = routeNet_length_constraint( vRerouteNet[r], nPreviousLength );
				vector<vector<gGrid_C *>> vResult = routeNet_length_constraint_ver2(vRerouteNet[r], nPreviousLength);
				if (vResult.size() == 0)
				{
					//cout<<"Routing failed"<<endl;
					//recoverNet( vRerouteNet[r] );
					nNetIndex = r;
					bFail = true;
					break;
				}

				else
				{
					bool bOverflow = false;
					for (int g = 0; g < vResult.size(); g++)
					{
						for (int gg = 0; gg < vResult[g].size(); gg++)
							if (vResult[g][gg]->getRemand() - 1 < 0)
							{
								bOverflow = true;
								break;
							}
					}
					if (bOverflow)
					{
						nNetIndex = r;
						vResult.clear();
						bFail = true;
						break;
					}
				}

				saveNet(vRerouteNet[r], vResult);

				//if( vRerouteNet[r]->getName() == "N235" )
				//	cout<<"Length is: "<<calWireLength( vRerouteNet[r] )<<endl;
				addNetOnGraph(m_pDesign, vRerouteNet[r]);
			}
			//cout<<"Here"<<endl;

			int nNewLength = 0;
			if (!bFail)
			{
				for (int n = 0; n < vRerouteNet.size(); n++)
				{
					//nNewLength = nNewLength + calWireLength( vRerouteNet[n] );
					nNewLength = nNewLength + vRerouteNet[n]->getLength();
				}
				//cout<<"Previous wire length: "<<nPreviousLength<<endl;
				//cout<<"New wire length: "<<nNewLength<<endl;
			}

			if (bFail || nNewLength > nPreviousLength)
			{
				//cout<<"Failed to optimize the design, recover."<<endl;

				vector<net_C *> vRoutedNet;
				//if( bFail )
				//{
				for (int rr = 0; rr < nNetIndex; rr++)
					vRoutedNet.push_back(vRerouteNet[rr]);
				//}
				ripupNet(vRoutedNet);

				recoverNet(vRerouteNet);

				for (int r = 0; r < vRerouteNet.size(); r++)
				{
					addNetOnGraph(m_pDesign, vRerouteNet[r]);
				}
				m_vBackupNet.clear();
				/*
			for( int b=0; b<m_vBackupNet.size(); b++ )
			{
				m_vBackupNet[b].cleanWire();
			}
			m_vBackupNet.clear();
			*/
			}
			else
			{

				for (int b = 0; b < m_vBackupNet.size(); b++)
				{
					m_vBackupNet[b].cleanWire();
				}
				m_vBackupNet.clear();
			}
		}
		//cout << " reverse " << endl;
		
		for (int i = 0; i < vNet.size(); i++)
		{
			endt = time(NULL);
			int nTime = endt - start;
			if (endt - start > TIMECONST)
				break;

			//if( vNet[i]->getNumReroute() > 0  )
			//	continue;

			vector<net_C *> vRerouteNet;
			vRerouteNet.push_back(vNet[i]);

			int nPreviousLength = 0;
			for (int n = 0; n < vRerouteNet.size(); n++)
			{
				//nPreviousLength = nPreviousLength + calWireLength( vRerouteNet[n] );
				nPreviousLength = nPreviousLength + vRerouteNet[n]->getLength();
			}

			backupNet(vRerouteNet);
			ripupNet(vRerouteNet);
			cleanWire(vRerouteNet);

			vector< net_C* > vRRRNet;
			bool bFail = false;
			int nNetIndex = vRerouteNet.size();
			for (int r = 0; r < vRerouteNet.size(); r++)
			{
				vector<vector<gGrid_C *>> vResult = routeNet_length_constraint_ver3(vRerouteNet[r], nPreviousLength);
				if (vResult.size() == 0)
				{
					nNetIndex = r;
					bFail = true;
					break;
				}

				else
				{
					saveNet(vRerouteNet[r], vResult);
					addNetOnGraph(m_pDesign, vRerouteNet[r]);
					int nLengthConstraint = nPreviousLength;
					nLengthConstraint = nLengthConstraint - vRerouteNet[r]->getLength();
					vector< gGrid_C* > vOGrid = checkOverflow( vResult );
					bool bOverflow = false;
					if( vOGrid.size() > 0  )
					{
						cout << "Fix overflow"<<endl;
						bOverflow = rrr_ver2( vRerouteNet, vOGrid, mLengthTable, nLengthConstraint, vRRRNet );
					}
					if( bOverflow )
					{
						cout << "Still overflow"<<endl;
						bFail = true;
						break;
					}
				}

			}
			//cout<<"Here"<<endl;

			int nNewLength = 0;
			if (!bFail)
			{
				nPreviousLength = 0;
				for( int n=0; n < m_vBackupNet.size(); n++ )
				{
					nPreviousLength = nPreviousLength + m_vBackupNet[n].getLength();
				}
				for (int n = 0; n < vRerouteNet.size(); n++)
				{
					//nNewLength = nNewLength + calWireLength( vRerouteNet[n] );
					nNewLength = nNewLength + vRerouteNet[n]->getLength();
				}
			}

			if (bFail || nNewLength > nPreviousLength)
			{
				//cout<<"Failed to optimize the design, recover."<<endl;

				ripupNet(vRerouteNet);

				recoverNet(vRerouteNet);

				for (int r = 0; r < vRerouteNet.size(); r++)
				{
					addNetOnGraph(m_pDesign, vRerouteNet[r]);
				}
				m_vBackupNet.clear();
			}
			else
			{

				for (int b = 0; b < m_vBackupNet.size(); b++)
				{
					m_vBackupNet[b].cleanWire();
				}
				m_vBackupNet.clear();
			}
		}
		
		return true;
	}

	
	bool router_C::pre_route_ver3(vector<net_C *> & vNet)
	{
		//vector< int > vLength;
		vector< int > vIdealLength;
		unordered_map< net_C*, int > mLengthTable;
		for (int i = 0; i < vNet.size(); i++)
		{
			vNet[i]->setLength(calWireLength(vNet[i]));
			mLengthTable[ vNet[i] ] = 0;
		}
		for (int i = 1; i < vNet.size(); i++)
		{
			//cout<<i;
			for (int j = i - 1; j >= 0; j--)
			{
				//int nFLength = vLength[j];
				//int nBLength = vLength[j+1];
				int nFLength = vNet[j]->getLength();
				int nBLength = vNet[j + 1]->getLength();
				;
				if (nFLength >= nBLength)
					break;
				else
				{
					net_C *pTmpNet = vNet[j];
					vNet[j] = vNet[j + 1];
					vNet[j + 1] = pTmpNet;
					//int nLength = vLength[j];
					//vLength[j] = vLength[j+1];
					//vLength[j+1] = nLength;
				}
			}
		}
		for (int i = 0; i < vNet.size(); i++)
		{
			vIdealLength.push_back( routeNet_ideal( vNet[i] ) );
		}

		for (int i = 0; i < vNet.size(); i++)
		{
			endt = time(NULL);
			int nTime = endt - start;
			if (endt - start > TIMECONST)
				break;

			//if( vNet[i]->getNumReroute() > 0  )
			//	continue;

			vector<net_C *> vRerouteNet;
			vRerouteNet.push_back(vNet[i]);

			int nPreviousLength = 0;
			for (int n = 0; n < vRerouteNet.size(); n++)
			{
				//nPreviousLength = nPreviousLength + calWireLength( vRerouteNet[n] );
				nPreviousLength = nPreviousLength + vRerouteNet[n]->getLength();
			}

			//if( vNet[i]->getName() == "N1482" )
			//	cout<<nPreviousLength<<endl;

			//cout<<"backup net"<<endl;
			backupNet(vRerouteNet);
			//cout<<"ripup net"<<endl;
			ripupNet(vRerouteNet);
			cleanWire(vRerouteNet);
			//cout<<"route the net..."<<endl;

			bool bFail = false;
			int nNetIndex = vRerouteNet.size();
			for (int r = 0; r < vRerouteNet.size(); r++)
			{
				//vector< gGrid_C* > vResult = routeNet_length_constraint( vRerouteNet[r], nPreviousLength );
				vector<vector<gGrid_C *>> vResult = routeNet_length_constraint_ver2(vRerouteNet[r], nPreviousLength);
				if (vResult.size() == 0)
				{
					//cout<<"Routing failed"<<endl;
					//recoverNet( vRerouteNet[r] );
					nNetIndex = r;
					bFail = true;
					break;
				}

				else
				{
					bool bOverflow = false;
					for (int g = 0; g < vResult.size(); g++)
					{
						for (int gg = 0; gg < vResult[g].size(); gg++)
							if (vResult[g][gg]->getRemand() - 1 < 0)
							{
								bOverflow = true;
								break;
							}
					}
					if (bOverflow)
					{
						nNetIndex = r;
						vResult.clear();
						bFail = true;
						break;
					}
				}

				saveNet(vRerouteNet[r], vResult);

				//if( vRerouteNet[r]->getName() == "N235" )
				//	cout<<"Length is: "<<calWireLength( vRerouteNet[r] )<<endl;
				addNetOnGraph(m_pDesign, vRerouteNet[r]);
			}
			//cout<<"Here"<<endl;

			int nNewLength = 0;
			if (!bFail)
			{
				for (int n = 0; n < vRerouteNet.size(); n++)
				{
					//nNewLength = nNewLength + calWireLength( vRerouteNet[n] );
					nNewLength = nNewLength + vRerouteNet[n]->getLength();
				}
				//cout<<"Previous wire length: "<<nPreviousLength<<endl;
				//cout<<"New wire length: "<<nNewLength<<endl;
			}

			if (bFail || nNewLength > nPreviousLength)
			{
				//cout<<"Failed to optimize the design, recover."<<endl;

				vector<net_C *> vRoutedNet;
				//if( bFail )
				//{
				for (int rr = 0; rr < nNetIndex; rr++)
					vRoutedNet.push_back(vRerouteNet[rr]);
				//}
				ripupNet(vRoutedNet);

				recoverNet(vRerouteNet);

				for (int r = 0; r < vRerouteNet.size(); r++)
				{
					addNetOnGraph(m_pDesign, vRerouteNet[r]);
				}
				m_vBackupNet.clear();
				/*
			for( int b=0; b<m_vBackupNet.size(); b++ )
			{
				m_vBackupNet[b].cleanWire();
			}
			m_vBackupNet.clear();
			*/
			}
			else
			{

				for (int b = 0; b < m_vBackupNet.size(); b++)
				{
					m_vBackupNet[b].cleanWire();
				}
				m_vBackupNet.clear();
			}
		}
		//cout << " reverse " << endl;
		
		for (int i = vNet.size() - 2; i >= 0; i--)
		{
			endt = time(NULL);
			int nTime = endt - start;
			if (endt - start > TIMECONST)
				break;

			//if( vNet[i]->getNumReroute() > 0  )
			//	continue;

			vector<net_C *> vRerouteNet;
			vRerouteNet.push_back(vNet[i]);

			int nPreviousLength = 0;
			for (int n = 0; n < vRerouteNet.size(); n++)
			{
				//nPreviousLength = nPreviousLength + calWireLength( vRerouteNet[n] );
				nPreviousLength = nPreviousLength + vRerouteNet[n]->getLength();
			}

			backupNet(vRerouteNet);
			ripupNet(vRerouteNet);
			cleanWire(vRerouteNet);

			vector< net_C* > vRRRNet;
			bool bFail = false;
			int nNetIndex = vRerouteNet.size();
			for (int r = 0; r < vRerouteNet.size(); r++)
			{
				vector<vector<gGrid_C *>> vResult = routeNet_length_constraint_ver3(vRerouteNet[r], nPreviousLength);
				if (vResult.size() == 0)
				{
					nNetIndex = r;
					bFail = true;
					break;
				}

				else
				{
					saveNet(vRerouteNet[r], vResult);
					addNetOnGraph(m_pDesign, vRerouteNet[r]);
					int nLengthConstraint = nPreviousLength;
					nLengthConstraint = nLengthConstraint - vRerouteNet[r]->getLength();
					vector< gGrid_C* > vOGrid = checkOverflow( vResult );
					bool bOverflow = false;
					if( vOGrid.size() > 0  )
					{
						cout << "Fix overflow"<<endl;
						bOverflow = rrr_ver2( vRerouteNet, vOGrid, mLengthTable, nLengthConstraint, vRRRNet );
					}
					if( bOverflow )
					{
						cout << "Still overflow"<<endl;
						bFail = true;
						break;
					}
				}

			}
			//cout<<"Here"<<endl;

			int nNewLength = 0;
			if (!bFail)
			{
				nPreviousLength = 0;
				for( int n=0; n < m_vBackupNet.size(); n++ )
				{
					nPreviousLength = nPreviousLength + m_vBackupNet[n].getLength();
				}
				for (int n = 0; n < vRerouteNet.size(); n++)
				{
					//nNewLength = nNewLength + calWireLength( vRerouteNet[n] );
					nNewLength = nNewLength + vRerouteNet[n]->getLength();
				}
			}

			if (bFail || nNewLength > nPreviousLength)
			{
				//cout<<"Failed to optimize the design, recover."<<endl;

				ripupNet(vRerouteNet);

				recoverNet(vRerouteNet);

				for (int r = 0; r < vRerouteNet.size(); r++)
				{
					addNetOnGraph(m_pDesign, vRerouteNet[r]);
				}
				m_vBackupNet.clear();
			}
			else
			{

				for (int b = 0; b < m_vBackupNet.size(); b++)
				{
					m_vBackupNet[b].cleanWire();
				}
				m_vBackupNet.clear();
			}
		}
		
		return true;
	}

	
	bool router_C::pre_route_ver2(vector<net_C *> & vNet)
	{
		//vector< int > vLength;
		vector< int > vIdealLength;
		unordered_map< net_C*, int > mLengthTable;
		for (int i = 0; i < vNet.size(); i++)
		{
			vNet[i]->setLength(calWireLength(vNet[i]));
			mLengthTable[ vNet[i] ] = 0;
		}
		for (int i = 1; i < vNet.size(); i++)
		{
			//cout<<i;
			for (int j = i - 1; j >= 0; j--)
			{
				//int nFLength = vLength[j];
				//int nBLength = vLength[j+1];
				int nFLength = vNet[j]->getLength();
				int nBLength = vNet[j + 1]->getLength();
				;
				if (nFLength >= nBLength)
					break;
				else
				{
					net_C *pTmpNet = vNet[j];
					vNet[j] = vNet[j + 1];
					vNet[j + 1] = pTmpNet;
					//int nLength = vLength[j];
					//vLength[j] = vLength[j+1];
					//vLength[j+1] = nLength;
				}
			}
		}
		for (int i = 0; i < vNet.size(); i++)
		{
			vIdealLength.push_back( routeNet_ideal( vNet[i] ) );
		}

		for (int i = 0; i < vNet.size(); i++)
		{
			endt = time(NULL);
			int nTime = endt - start;
			if (endt - start > TIMECONST)
				break;

			//if( vNet[i]->getNumReroute() > 0  )
			//	continue;

			vector<net_C *> vRerouteNet;
			vRerouteNet.push_back(vNet[i]);

			int nPreviousLength = 0;
			for (int n = 0; n < vRerouteNet.size(); n++)
			{
				//nPreviousLength = nPreviousLength + calWireLength( vRerouteNet[n] );
				nPreviousLength = nPreviousLength + vRerouteNet[n]->getLength();
			}

			backupNet(vRerouteNet);
			ripupNet(vRerouteNet);
			cleanWire(vRerouteNet);

			vector< net_C* > vRRRNet;
			bool bFail = false;
			int nNetIndex = vRerouteNet.size();
			for (int r = 0; r < vRerouteNet.size(); r++)
			{
				vector<vector<gGrid_C *>> vResult = routeNet_length_constraint_ver3(vRerouteNet[r], nPreviousLength);
				if (vResult.size() == 0)
				{
					nNetIndex = r;
					bFail = true;
					break;
				}

				else
				{
					saveNet(vRerouteNet[r], vResult);
					addNetOnGraph(m_pDesign, vRerouteNet[r]);
					int nLengthConstraint = nPreviousLength;
					nLengthConstraint = nLengthConstraint - vRerouteNet[r]->getLength();
					vector< gGrid_C* > vOGrid = checkOverflow( vResult );
					bool bOverflow = false;
					if( vOGrid.size() > 0  )
					{
						cout << "Fix overflow"<<endl;
						bOverflow = rrr_ver2( vRerouteNet, vOGrid, mLengthTable, nLengthConstraint, vRRRNet );
					}
					if( bOverflow )
					{
						cout << "Still overflow"<<endl;
						bFail = true;
						break;
					}
				}

			}
			//cout<<"Here"<<endl;

			int nNewLength = 0;
			if (!bFail)
			{
				nPreviousLength = 0;
				for( int n=0; n < m_vBackupNet.size(); n++ )
				{
					nPreviousLength = nPreviousLength + m_vBackupNet[n].getLength();
				}
				for (int n = 0; n < vRerouteNet.size(); n++)
				{
					//nNewLength = nNewLength + calWireLength( vRerouteNet[n] );
					nNewLength = nNewLength + vRerouteNet[n]->getLength();
				}
			}

			if (bFail || nNewLength > nPreviousLength)
			{
				//cout<<"Failed to optimize the design, recover."<<endl;

				ripupNet(vRerouteNet);

				recoverNet(vRerouteNet);

				for (int r = 0; r < vRerouteNet.size(); r++)
				{
					addNetOnGraph(m_pDesign, vRerouteNet[r]);
				}
				m_vBackupNet.clear();
			}
			else
			{

				for (int b = 0; b < m_vBackupNet.size(); b++)
				{
					m_vBackupNet[b].cleanWire();
				}
				m_vBackupNet.clear();
			}
		}
		//cout << " reverse " << endl;
		
		for (int i = vNet.size() - 2; i >= 0; i--)
		{
			endt = time(NULL);
			int nTime = endt - start;
			if (endt - start > TIMECONST)
				break;

			//if( vNet[i]->getNumReroute() > 0  )
			//	continue;

			vector<net_C *> vRerouteNet;
			vRerouteNet.push_back(vNet[i]);

			int nPreviousLength = 0;
			for (int n = 0; n < vRerouteNet.size(); n++)
			{
				//nPreviousLength = nPreviousLength + calWireLength( vRerouteNet[n] );
				nPreviousLength = nPreviousLength + vRerouteNet[n]->getLength();
			}

			backupNet(vRerouteNet);
			ripupNet(vRerouteNet);
			cleanWire(vRerouteNet);

			vector< net_C* > vRRRNet;
			bool bFail = false;
			int nNetIndex = vRerouteNet.size();
			for (int r = 0; r < vRerouteNet.size(); r++)
			{
				vector<vector<gGrid_C *>> vResult = routeNet_length_constraint_ver3(vRerouteNet[r], nPreviousLength);
				if (vResult.size() == 0)
				{
					nNetIndex = r;
					bFail = true;
					break;
				}

				else
				{
					saveNet(vRerouteNet[r], vResult);
					addNetOnGraph(m_pDesign, vRerouteNet[r]);
					int nLengthConstraint = nPreviousLength;
					nLengthConstraint = nLengthConstraint - vRerouteNet[r]->getLength();
					vector< gGrid_C* > vOGrid = checkOverflow( vResult );
					bool bOverflow = false;
					if( vOGrid.size() > 0  )
					{
						cout << "Fix overflow"<<endl;
						bOverflow = rrr_ver2( vRerouteNet, vOGrid, mLengthTable, nLengthConstraint, vRRRNet );
					}
					if( bOverflow )
					{
						cout << "Still overflow"<<endl;
						bFail = true;
						break;
					}
				}

			}
			//cout<<"Here"<<endl;

			int nNewLength = 0;
			if (!bFail)
			{
				nPreviousLength = 0;
				for( int n=0; n < m_vBackupNet.size(); n++ )
				{
					nPreviousLength = nPreviousLength + m_vBackupNet[n].getLength();
				}
				for (int n = 0; n < vRerouteNet.size(); n++)
				{
					//nNewLength = nNewLength + calWireLength( vRerouteNet[n] );
					nNewLength = nNewLength + vRerouteNet[n]->getLength();
				}
			}

			if (bFail || nNewLength > nPreviousLength)
			{
				//cout<<"Failed to optimize the design, recover."<<endl;

				ripupNet(vRerouteNet);

				recoverNet(vRerouteNet);

				for (int r = 0; r < vRerouteNet.size(); r++)
				{
					addNetOnGraph(m_pDesign, vRerouteNet[r]);
				}
				m_vBackupNet.clear();
			}
			else
			{

				for (int b = 0; b < m_vBackupNet.size(); b++)
				{
					m_vBackupNet[b].cleanWire();
				}
				m_vBackupNet.clear();
			}
		}
		
		return true;
	}

	bool router_C::pre_route(vector<net_C *> & vNet)
	{
		//vector< int > vLength;
		vector< int > vIdealLength;
		for (int i = 0; i < vNet.size(); i++)
		{
			vNet[i]->setLength(calWireLength(vNet[i]));
		}
		for (int i = 1; i < vNet.size(); i++)
		{
			//cout<<i;
			for (int j = i - 1; j >= 0; j--)
			{
				//int nFLength = vLength[j];
				//int nBLength = vLength[j+1];
				int nFLength = vNet[j]->getLength();
				int nBLength = vNet[j + 1]->getLength();
				;
				if (nFLength >= nBLength)
					break;
				else
				{
					net_C *pTmpNet = vNet[j];
					vNet[j] = vNet[j + 1];
					vNet[j + 1] = pTmpNet;
					//int nLength = vLength[j];
					//vLength[j] = vLength[j+1];
					//vLength[j+1] = nLength;
				}
			}
		}
		for (int i = 0; i < vNet.size(); i++)
		{
			vIdealLength.push_back( routeNet_ideal( vNet[i] ) );
		}

		for (int i = 0; i < vNet.size(); i++)
		{
			endt = time(NULL);
			int nTime = endt - start;
			if (endt - start > TIMECONST)
				break;

			//if( vNet[i]->getNumReroute() > 0  )
			//	continue;

			vector<net_C *> vRerouteNet;
			vRerouteNet.push_back(vNet[i]);

			int nPreviousLength = 0;
			for (int n = 0; n < vRerouteNet.size(); n++)
			{
				//nPreviousLength = nPreviousLength + calWireLength( vRerouteNet[n] );
				nPreviousLength = nPreviousLength + vRerouteNet[n]->getLength();
			}

			//if( vNet[i]->getName() == "N1482" )
			//	cout<<nPreviousLength<<endl;

			//cout<<"backup net"<<endl;
			backupNet(vRerouteNet);
			//cout<<"ripup net"<<endl;
			ripupNet(vRerouteNet);
			cleanWire(vRerouteNet);
			//cout<<"route the net..."<<endl;

			bool bFail = false;
			int nNetIndex = vRerouteNet.size();
			for (int r = 0; r < vRerouteNet.size(); r++)
			{
				//vector< gGrid_C* > vResult = routeNet_length_constraint( vRerouteNet[r], nPreviousLength );
				vector<vector<gGrid_C *>> vResult = routeNet_length_constraint_ver2(vRerouteNet[r], nPreviousLength);
				/*
			cout<<"Route net result"<<endl;
			for( int j=0; j<vResult.size(); j++ )
			{
				int nX, nY, nZ;
				vResult[j]->getPosition( nX, nY, nZ );
				//cout<<"( "<<nX<<" , "<<nY<<" , "<<nZ<<" )"<<endl;
			}
			cout<<endl;
			*/
				/*
			if( vRerouteNet[r]->getName() == "N2202" || vRerouteNet[r]->getName() == "N2174" )
			{
				ofstream ferr;
				ferr.open("routing_info.log", ios::app );
				ferr<<"RRR: "<<endl;
				ferr<<vRerouteNet[r]->getName()<<endl;
				int nRX, nRY, nRZ;
				for( int g=0; g<vResult.size(); g++ )
				{
					vResult[g]->getPosition( nRX, nRY, nRZ );
					ferr<<nRX<<" "<<nRY<<" "<<nRZ<<endl;
				}
				ferr<<endl;
			}
			*/
				if (vResult.size() == 0)
				{
					//cout<<"Routing failed"<<endl;
					//recoverNet( vRerouteNet[r] );
					nNetIndex = r;
					bFail = true;
					break;
				}

				else
				{
					bool bOverflow = false;
					for (int g = 0; g < vResult.size(); g++)
					{
						for (int gg = 0; gg < vResult[g].size(); gg++)
							if (vResult[g][gg]->getRemand() - 1 < 0)
							{
								bOverflow = true;
								break;
							}
					}
					if (bOverflow)
					{
						nNetIndex = r;
						vResult.clear();
						bFail = true;
						break;
					}
				}

				saveNet(vRerouteNet[r], vResult);

				//if( vRerouteNet[r]->getName() == "N235" )
				//	cout<<"Length is: "<<calWireLength( vRerouteNet[r] )<<endl;
				addNetOnGraph(m_pDesign, vRerouteNet[r]);
			}
			//cout<<"Here"<<endl;

			int nNewLength = 0;
			if (!bFail)
			{
				for (int n = 0; n < vRerouteNet.size(); n++)
				{
					//nNewLength = nNewLength + calWireLength( vRerouteNet[n] );
					nNewLength = nNewLength + vRerouteNet[n]->getLength();
				}
				//cout<<"Previous wire length: "<<nPreviousLength<<endl;
				//cout<<"New wire length: "<<nNewLength<<endl;
			}

			if (bFail || nNewLength > nPreviousLength)
			{
				//cout<<"Failed to optimize the design, recover."<<endl;

				vector<net_C *> vRoutedNet;
				//if( bFail )
				//{
				for (int rr = 0; rr < nNetIndex; rr++)
					vRoutedNet.push_back(vRerouteNet[rr]);
				//}
				ripupNet(vRoutedNet);

				recoverNet(vRerouteNet);

				for (int r = 0; r < vRerouteNet.size(); r++)
				{
					addNetOnGraph(m_pDesign, vRerouteNet[r]);
				}
				m_vBackupNet.clear();
				/*
			for( int b=0; b<m_vBackupNet.size(); b++ )
			{
				m_vBackupNet[b].cleanWire();
			}
			m_vBackupNet.clear();
			*/
			}
			else
			{

				for (int b = 0; b < m_vBackupNet.size(); b++)
				{
					m_vBackupNet[b].cleanWire();
				}
				m_vBackupNet.clear();
			}
		}
		//cout << " reverse " << endl;
		for (int i = vNet.size() - 2; i >= 0; i--)
		{
			endt = time(NULL);
			int nTime = endt - start;
			if (endt - start > TIMECONST)
				break;

			//if( vNet[i]->getNumReroute() > 0  )
			//	continue;

			vector<net_C *> vRerouteNet;
			vRerouteNet.push_back(vNet[i]);

			int nPreviousLength = 0;
			for (int n = 0; n < vRerouteNet.size(); n++)
			{
				//nPreviousLength = nPreviousLength + calWireLength( vRerouteNet[n] );
				nPreviousLength = nPreviousLength + vRerouteNet[n]->getLength();
			}

			//if( vNet[i]->getName() == "N1482" )
			//	cout<<nPreviousLength<<endl;

			//cout<<"backup net"<<endl;
			backupNet(vRerouteNet);
			//cout<<"ripup net"<<endl;
			ripupNet(vRerouteNet);
			cleanWire(vRerouteNet);
			//cout<<"route the net..."<<endl;

			bool bFail = false;
			int nNetIndex = vRerouteNet.size();
			for (int r = 0; r < vRerouteNet.size(); r++)
			{
				//vector< gGrid_C* > vResult = routeNet_length_constraint( vRerouteNet[r], nPreviousLength );
				vector<vector<gGrid_C *>> vResult = routeNet_length_constraint_ver2(vRerouteNet[r], nPreviousLength);
				/*
			cout<<"Route net result"<<endl;
			for( int j=0; j<vResult.size(); j++ )
			{
				int nX, nY, nZ;
				vResult[j]->getPosition( nX, nY, nZ );
				//cout<<"( "<<nX<<" , "<<nY<<" , "<<nZ<<" )"<<endl;
			}
			cout<<endl;
			*/
				/*
			if( vRerouteNet[r]->getName() == "N2202" || vRerouteNet[r]->getName() == "N2174" )
			{
				ofstream ferr;
				ferr.open("routing_info.log", ios::app );
				ferr<<"RRR: "<<endl;
				ferr<<vRerouteNet[r]->getName()<<endl;
				int nRX, nRY, nRZ;
				for( int g=0; g<vResult.size(); g++ )
				{
					vResult[g]->getPosition( nRX, nRY, nRZ );
					ferr<<nRX<<" "<<nRY<<" "<<nRZ<<endl;
				}
				ferr<<endl;
			}
			*/
				if (vResult.size() == 0)
				{
					//cout<<"Routing failed"<<endl;
					//recoverNet( vRerouteNet[r] );
					nNetIndex = r;
					bFail = true;
					break;
				}
				else
				{
					bool bOverflow = false;
					for (int g = 0; g < vResult.size(); g++)
					{
						for (int gg = 0; gg < vResult[g].size(); gg++)
						{
							if (vResult[g][gg]->getRemand() - 1 < 0)
							{
								bOverflow = true;
								break;
							}
						}
					}
					if (bOverflow)
					{
						nNetIndex = r;
						vResult.clear();
						bFail = true;
						break;
					}
				}

				saveNet(vRerouteNet[r], vResult);

				//if( vRerouteNet[r]->getName() == "N235" )
				//	cout<<"Length is: "<<calWireLength( vRerouteNet[r] )<<endl;
				addNetOnGraph(m_pDesign, vRerouteNet[r]);
			}
			//cout<<"Here"<<endl;

			int nNewLength = 0;
			if (!bFail)
			{
				for (int n = 0; n < vRerouteNet.size(); n++)
				{
					//nNewLength = nNewLength + calWireLength( vRerouteNet[n] );
					nNewLength = nNewLength + vRerouteNet[n]->getLength();
				}
				//cout<<"Previous wire length: "<<nPreviousLength<<endl;
				//cout<<"New wire length: "<<nNewLength<<endl;
			}

			if (bFail || nNewLength > nPreviousLength)
			{
				//cout<<"Failed to optimize the design, recover."<<endl;

				vector<net_C *> vRoutedNet;
				//if( bFail )
				//{
				for (int rr = 0; rr < nNetIndex; rr++)
					vRoutedNet.push_back(vRerouteNet[rr]);
				//}
				ripupNet(vRoutedNet);

				recoverNet(vRerouteNet);
				m_vBackupNet.clear();

				for (int r = 0; r < vRerouteNet.size(); r++)
				{
					addNetOnGraph(m_pDesign, vRerouteNet[r]);
				}
				/*
			for( int b=0; b<m_vBackupNet.size(); b++ )
			{
				m_vBackupNet[b].cleanWire();
			}
			m_vBackupNet.clear();
			*/
			}
			else
			{

				for (int b = 0; b < m_vBackupNet.size(); b++)
				{
					m_vBackupNet[b].cleanWire();
				}
				m_vBackupNet.clear();
			}
		}

		// update length table
		for( int i=0; i<vNet.size(); i++ )
		{
			int nDetour = vNet[i]->getLength() - vIdealLength[i];
			if( nDetour < 0 )
				nDetour = 0;
			else if( nDetour > 0 )
				cout << vNet[i]->getName() << " " << vNet[i]->getLength() - vIdealLength[i] << endl;	
			m_mLengthTable[ vNet[i] ] = nDetour;	
		}
		return true;
	}
	bool router_C::rrr(vector<net_C *> & vNet)
	{
		for (int i = 0; i < vNet.size(); i++)
		{
			endt = time(NULL);
			int nTime = endt - start;
			if (endt - start > TIMECONST)
				break;

			//if( vNet[i]->getNumReroute() > 0  )
			//	continue;

			vector<net_C *> vRerouteNet;
			vRerouteNet.push_back(vNet[i]);

			int nPreviousLength = 0;
			for (int n = 0; n < vRerouteNet.size(); n++)
			{
				//nPreviousLength = nPreviousLength + calWireLength( vRerouteNet[n] );
				nPreviousLength = nPreviousLength + vRerouteNet[n]->getLength();
			}

			//if( vNet[i]->getName() == "N1482" )
			//	cout<<nPreviousLength<<endl;

			//cout<<"backup net"<<endl;
			backupNet(vRerouteNet);
			//cout<<"ripup net"<<endl;
			ripupNet(vRerouteNet);
			cleanWire(vRerouteNet);
			//cout<<"route the net..."<<endl;

			bool bFail = false;
			int nNetIndex = vRerouteNet.size();
			for (int r = 0; r < vRerouteNet.size(); r++)
			{
				vector<gGrid_C *> vResult = routeNet_length_constraint(vRerouteNet[r], nPreviousLength);
				/*
			cout<<"Route net result"<<endl;
			for( int j=0; j<vResult.size(); j++ )
			{
				int nX, nY, nZ;
				vResult[j]->getPosition( nX, nY, nZ );
				//cout<<"( "<<nX<<" , "<<nY<<" , "<<nZ<<" )"<<endl;
			}
			cout<<endl;
			*/
				/*
			if( vRerouteNet[r]->getName() == "N2202" || vRerouteNet[r]->getName() == "N2174" )
			{
				ofstream ferr;
				ferr.open("routing_info.log", ios::app );
				ferr<<"RRR: "<<endl;
				ferr<<vRerouteNet[r]->getName()<<endl;
				int nRX, nRY, nRZ;
				for( int g=0; g<vResult.size(); g++ )
				{
					vResult[g]->getPosition( nRX, nRY, nRZ );
					ferr<<nRX<<" "<<nRY<<" "<<nRZ<<endl;
				}
				ferr<<endl;
			}
			*/
				if (vResult.size() == 0)
				{
					//cout<<"Routing failed"<<endl;
					//recoverNet( vRerouteNet[r] );
					nNetIndex = r;
					bFail = true;
					break;
				}
				else
				{
					bool bOverflow = false;
					for (int g = 0; g < vResult.size(); g++)
					{
						if (vResult[g]->getRemand() - 1 < 0)
						{
							bOverflow = true;
							break;
						}
					}
					if (bOverflow)
					{
						nNetIndex = r;
						vResult.clear();
						bFail = true;
						break;
					}
				}

				saveNet(vRerouteNet[r], vResult);

				//if( vRerouteNet[r]->getName() == "N235" )
				//	cout<<"Length is: "<<calWireLength( vRerouteNet[r] )<<endl;
				addNetOnGraph(m_pDesign, vRerouteNet[r]);
			}
			//cout<<"Here"<<endl;

			int nNewLength = 0;
			if (!bFail)
			{
				for (int n = 0; n < vRerouteNet.size(); n++)
				{
					//nNewLength = nNewLength + calWireLength( vRerouteNet[n] );
					nNewLength = nNewLength + vRerouteNet[n]->getLength();
				}
				//cout<<"Previous wire length: "<<nPreviousLength<<endl;
				//cout<<"New wire length: "<<nNewLength<<endl;
			}

			if (bFail || nNewLength > nPreviousLength)
			{
				//cout<<"Failed to optimize the design, recover."<<endl;

				vector<net_C *> vRoutedNet;
				//if( bFail )
				//{
				for (int rr = 0; rr < nNetIndex; rr++)
					vRoutedNet.push_back(vRerouteNet[rr]);
				//}
				ripupNet(vRoutedNet);

				recoverNet(vRerouteNet);

				for (int r = 0; r < vRerouteNet.size(); r++)
				{
					addNetOnGraph(m_pDesign, vRerouteNet[r]);
				}
				/*
			for( int b=0; b<m_vBackupNet.size(); b++ )
			{
				m_vBackupNet[b].cleanWire();
			}
			m_vBackupNet.clear();
			*/
			}
			else
			{

				for (int b = 0; b < m_vBackupNet.size(); b++)
				{
					m_vBackupNet[b].cleanWire();
				}
				m_vBackupNet.clear();
			}
		}
		return true;
	}

	bool router_C::dumpResult(char *strFileName)
	{
		ofstream fout;
		fout.open(strFileName, ios::out);

		fout << "NumMovedCellInst " << m_vMovedInstance.size() << endl;
		for (int i = 0; i < m_vMovedInstance.size(); i++)
		{
			instance_C *pInst = m_vMovedInstance[i];
			int nX, nY;
			nX = pInst->getPlacedX();
			nY = pInst->getPlacedY();
			string strInstName = pInst->getName();
			fout << "CellInst " << strInstName << " " << nX << " " << nY << endl;
		}
		fout << endl;

		int nNumNet = 0;
		vector<net_C *> vNet = m_pDesign->getNet();
		vector<wire_C *> vTmpWire;
		/*
	for( int i=0; i<vNet.size(); i++ )
	{
		vector< wire_C* > vWire = vNet[i]->getWire();
		if( vWire.size() == 1 )
		{
			if( vWire[0]->getGrid1() == vWire[0]->getGrid2() )
				continue;
		}
		nNumNet = nNumNet + vNet[i]->getWire().size();
	}
	*/
		for (int i = 0; i < vNet.size(); i++)
		{
			//#ifdef _DEBUG_MODE
			//		matlab_graph(vNet[i]->getName(), vNet[i]->getName(), m_pDesign, 1);
			//#endif
			//
			vector<wire_C *> vWire = vNet[i]->getWire();
			for (int j = 0; j < vWire.size(); j++)
			{
				if (vWire[j]->getGrid1() == vWire[j]->getGrid2())
					continue;
				else
				{
					vTmpWire.push_back(vWire[j]);
					nNumNet++;
				}
			}
		}

		fout << "NumRoutes " << nNumNet << endl;
		/*
	for( int i=0; i<vNet.size(); i++ )
	{
		vector< wire_C* > vWire = vNet[i]->getWire();
		for( int j=0; j<vWire.size(); j++ )
		{
			wire_C* pWire = vWire[j];
			gGrid_C* pGrid1 = pWire->getGrid1();
			gGrid_C* pGrid2 = pWire->getGrid2();
			if( vWire.size() == 1 && pGrid1 == pGrid2 )
				continue;
			
			int nX, nY, nZ;
			pGrid1->getPosition( nX, nY, nZ );
			fout<<nX<<" "<<nY<<" "<<nZ<<" ";
			pGrid2->getPosition( nX, nY, nZ );
			fout<<nX<<" "<<nY<<" "<<nZ<<" ";
			fout<<vNet[i]->getName()<<endl;
		}
	}
	*/
		for (int i = 0; i < vTmpWire.size(); i++)
		{
			wire_C *pWire = vTmpWire[i];
			gGrid_C *pGrid1 = pWire->getGrid1();
			gGrid_C *pGrid2 = pWire->getGrid2();

			int nX, nY, nZ;
			pGrid1->getPosition(nX, nY, nZ);
			fout << nX << " " << nY << " " << nZ << " ";
			pGrid2->getPosition(nX, nY, nZ);
			fout << nX << " " << nY << " " << nZ << " ";
			fout << pWire->getNet()->getName() << endl;
		}
		fout.close();
		/*
		cout << "Check Info: "<<endl;
		for( int i=0; i<m_pDesign->getInstance().size(); i++ )
		{
			instance_C *pInst = m_pDesign->getInstance()[i];
			int nX, nY;
			nX = pInst->getPlacedX();
			nY = pInst->getPlacedY();
			string strInstName = pInst->getName();
			cout << "CellInst " << strInstName << " " << nX << " " << nY << endl;
		}
		*/
	}

	bool router_C::dumpDetailInfo()
	{
		ofstream fout;
		fout.open("detailInfo.log", ios::out);
		for (int l = m_nDZ; l <= m_nTZ; l++)
		{
			for (int x = m_nDX; x <= m_nTX; x++)
			{
				for (int y = m_nDY; y <= m_nTY; y++)
				{
					gGrid_C *pGrid = getGrid(m_pDesign, x, y, l);
					fout << x << " " << y << " " << l << " " << pGrid->getSupply();
					fout << " " << pGrid->getDemand();
					fout << " " << pGrid->getExtraDemand();
					fout << " " << pGrid->getNet().size();
					for (int i = 0; i < pGrid->getNet().size(); i++)
					{
						fout << " " << pGrid->getNet()[i]->getName() << " ";
					}
					fout << endl;
				}
			}
		}
		fout.close();
	}

	bool router_C::forcedAnalysis(net_C * pNet, vector<vector<gGrid_C *>> & vSegment)
	{
		vector<pin_C *> vPin = pNet->getPin();
		unordered_map<rGrid_C *, int> mPinIndex;
		vector<rGrid_C *> vPGrid;
		vector<bool> vTemplate;
		for (int i = 0; i < vPin.size(); i++)
		{
			instance_C *pInst = (instance_C *)vPin[i]->getCell();
			int nX = pInst->getPlacedX();
			int nY = pInst->getPlacedY();
			int nZ = vPin[i]->getLayerId();
			nZ = max(nZ, m_pDesign->getLayer_map()[pNet->getConstraint()]);
			rGrid_C *pRTGrid = getRGrid(nX, nY, nZ);
			vTemplate.push_back(false);
			if (!pInst->isMovable())
				continue;
			vPGrid.push_back(pRTGrid);
			pRTGrid->isTarget = true;
			mPinIndex[pRTGrid] = i;
		}

		vector<bool> vFL = vTemplate;
		vector<bool> vFR = vTemplate;
		vector<bool> vFT = vTemplate;
		vector<bool> vFD = vTemplate;
		pNet->m_vFD.clear();
		pNet->m_vFD.resize(vTemplate.size(), 0);
		pNet->m_vFT.clear();
		pNet->m_vFT.resize(vTemplate.size(), 0);
		pNet->m_vFR.clear();
		pNet->m_vFR.resize(vTemplate.size(), 0);
		pNet->m_vFL.clear();
		pNet->m_vFL.resize(vTemplate.size(), 0);

		for (int i = 0; i < vSegment.size(); i++)
		{
			vector<gGrid_C *> vSubPath = vSegment[i];
			int nSX, nSY, nSZ;
			int nEX, nEY, nEZ;
			gGrid_C *pSGrid = vSubPath.front();
			gGrid_C *pEGrid = vSubPath.back();
			pSGrid->getPosition(nSX, nSY, nSZ);
			pEGrid->getPosition(nEX, nEY, nEZ);
			rGrid_C *pRSGrid = getRGrid(nSX, nSY, nSZ);
			rGrid_C *pREGrid = getRGrid(nEX, nEY, nEZ);
			if (pRSGrid->isTarget)
			{
				int nIndex = mPinIndex.find(pRSGrid)->second;
				if (nEX - nSX > 0)
					vFT[nIndex] = true;
				else if (nEX - nSX < 0)
					vFD[nIndex] = true;

				if (nEY - nSY > 0)
					vFR[nIndex] = true;
				else if (nEY - nSY < 0)
					vFL[nIndex] = true;
			}

			if (pREGrid->isTarget)
			{
				int nIndex = mPinIndex.find(pREGrid)->second;
				if (nEX - nSX > 0)
					vFD[nIndex] = true;
				else if (nEX - nSX < 0)
					vFT[nIndex] = true;

				if (nEY - nSY > 0)
					vFL[nIndex] = true;
				else if (nEY - nSY < 0)
					vFR[nIndex] = true;
			}
		}

		for (int i = 0; i < vPin.size(); i++)
		{
			if (vFD[i] || !vFT[i])
				pNet->m_vFD[i]++;
			else if (!vFD[i] || vFT[i])
				pNet->m_vFT[i]++;

			if (vFR[i] || !vFL[i])
				pNet->m_vFR[i]++;
			else if (!vFR[i] || vFL[i])
				pNet->m_vFL[i]++;
		}

		for (int i = 0; i < vPGrid.size(); i++)
		{
			vPGrid[i]->isTarget = false;
		}
	}

	bool router_C::forcedAnalysis(net_C * pNet)
	{
		/*
	unordered_map< pin_C*, int > mPinIndex;
	vector< bool > vRF;
	int nPinSize = pNet->getPin().size();
	for( int i=0; i<nPinSize; i++ )
	{
		vRF.push_back( false );
	}
	vector< bool > vLF = vRF;
	vector< bool > vDF = vRF;
	vector< bool > vTF = vRF;
	string strConstraintLayer = pNet->getConstraint();
	int nMinLayer = m_pDesign->getLayer_map()[strConstraintLayer];
	// create pin map x, y, z -> vector< pin_C* >
	unordered_map< int, unordered_map< int, unordered_map< int, vector< pin_C* > > > > mPinMap;
	vector< pin_C* > vPin = pNet->getPin();
	for( int i=0; i<vPin.size(); i++ )
	{
		mPinIndex[ vPin[i] ] = i;
		int nX = vPin[i]->getCell()->getPlacedX();
		int nY = vPin[i]->getCell()->getPlacedY();
		int nZ = vPin[i]->getLayerId();
		unordered_map< int, unordered_map< int, unordered_map< int, vector< pin_C* > > > >::iterator xIter;
		if( mPinMap.find( nX ) != mPinMap.end() ) // there is a data
		{
			xIter = mPinMap.find( nX );
			unordered_map< int, unordered_map <int, vector< pin_C* > > >::iterator yIter;
			if( xIter->second.find(nY) != xIter->second.end() );
			{
				yIter = xIter->second.find( nY );
				int nLayer = max( nMinLayer, nZ );
				unordered_map <int, vector< pin_C* > >::iterator zIter;
				if( yIter->second.find( nLayer ) != yIter->second.end() )
				{
					zIter = yIter->second.find( nLayer );
					zIter->second.push_back( vPin[i] );	
				}
				else
				{	
					vector< pin_C* > vTmpPin;
					vTmpPin.push_back( vPin[i] );
					yIter->second[ nLayer ] = vTmpPin;
				}
			}
			else
			{
				unordered_map <int, vector< pin_C* > > mTmpZMap;
				vector< pin_C* > vTmpPin;
				vTmpPin.push_back( vPin[i] );
				int nLayer = max( nMinLayer, nZ );
				mTmpZMap[ nLayer ] = vTmpPin;
				xIter->second[ nY ] = mTmpZMap;	
			}
		}
		else
		{
			unordered_map< int, unordered_map <int, vector< pin_C* > > > mTmpYMap;
			unordered_map <int, vector< pin_C* > > mTmpZMap;
			vector< pin_C* > vTmpPin;
			vTmpPin.push_back( vPin[i] );
			int nLayer = max( nMinLayer, nZ );
			mTmpZMap[ nLayer ] = vTmpPin;
			mTmpYMap[ nY ] = mTmpZMap;	
			mPinMap[ nX ] = mTmpYMap;
		}
	}

	for( int i=0; i<vResult.size(); i++ )
	{
		vector< gGrid_C* > vTmpResult = vResult[i];
		for( int j=0; j<vTmpResult.size(); j++ )
		{
			gGrid_C* gStart = vTmpResult.front();
			gGrid_C* gEnd = vTmpResult.back();
			int nX, nY, nZ;
			gStart->getPosition( nX, nY, nZ );
			bool bFind = false;
			if( mPinMap.find( nX ) != mPinMap.end() )
			{
				if( mPinMap.find(nX)->second.find(nY) != mPinMap.find(nX)->second.end() )
				{
					if( mPinMap.find(nX)->second.find(nY) != mPinMap.find(nX)->second.end() )
				}
			}
		}
	}
	*/

		vector<pin_C *> vPin = pNet->getPin();
		for (int i = 0; i < vPin.size(); i++)
		{
			pin_C *pTmpPin = vPin[i];
			int nX, nY, nZ;
			instance_C *pTmpInst = (instance_C *)pTmpPin->getCell();
			nX = pTmpInst->getPlacedX();
			nY = pTmpInst->getPlacedY();
			nZ = pTmpPin->getLayerId();
			gGrid_C *pStart = getGrid(m_pDesign, nX, nY, nZ);
			bool bFind = false;
			int nL = 0;
			int nR = 0;
			int nD = 0;
			int nT = 0;
			int nFX = 0;
			int nFY = 0;
			int nDX = 0;
			int nDY = 0;
			int nDZ = 0;
			vector<gGrid_C *> vStart;
			vector<char> vDir;
			set<char> sDir;
			gGrid_C *pGrid = pStart;

			if (m_pDesign->getLayer()[nZ - m_nOffsetZ]->getDir() == 'H') // go in Y
			{
				nDX = 0;
				nDY = 1;
				nDZ = 0;
				gGrid_C *pNGrid = graphTravel(m_pDesign, pGrid, nDX, nDY, nDZ);
				if (pNGrid != NULL && pNGrid->findNet(pNet))
				{
					vStart.push_back(pNGrid);
					vDir.push_back('R');
					sDir.insert('R');
				}

				nDX = 0;
				nDY = -1;
				nDZ = 0;
				pNGrid = graphTravel(m_pDesign, pGrid, nDX, nDY, nDZ);
				if (pNGrid != NULL && pNGrid->findNet(pNet))
				{
					vStart.push_back(pNGrid);
					vDir.push_back('L');
					sDir.insert('L');
				}

				nDX = 0;
				nDY = 0;
				nDZ = 1;
				pNGrid = graphTravel(m_pDesign, pGrid, nDX, nDY, nDZ);
				while (pNGrid != NULL && pNGrid->findNet(pNet))
				{
					if (m_pDesign->getLayer()[pNGrid->m_nZ - m_nOffsetZ]->getDir() == 'H')
					{
						gGrid_C *pNNGrid = graphTravel(m_pDesign, pNGrid, 0, 1, 0);
						if (pNNGrid != NULL && pNNGrid->findNet(pNet))
						{
							vStart.push_back(pNNGrid);
							vDir.push_back('R');
							sDir.insert('R');
						}
						pNNGrid = graphTravel(m_pDesign, pNGrid, 0, -1, 0);
						if (pNNGrid != NULL && pNNGrid->findNet(pNet))
						{
							vStart.push_back(pNNGrid);
							vDir.push_back('L');
							sDir.insert('L');
						}
					}
					else
					{
						gGrid_C *pNNGrid = graphTravel(m_pDesign, pNGrid, 1, 0, 0);
						if (pNNGrid != NULL && pNNGrid->findNet(pNet))
						{
							vStart.push_back(pNNGrid);
							vDir.push_back('T');
							sDir.insert('T');
						}
						pNNGrid = graphTravel(m_pDesign, pNGrid, -1, 0, 0);
						if (pNNGrid != NULL && pNNGrid->findNet(pNet))
						{
							vStart.push_back(pNNGrid);
							vDir.push_back('D');
							sDir.insert('D');
						}
					}
					pGrid = pNGrid;
				}

				pGrid = pStart;
				nDX = 0;
				nDY = 0;
				nDZ = -1;
				pNGrid = graphTravel(m_pDesign, pGrid, nDX, nDY, nDZ);
				while (pNGrid != NULL && pNGrid->findNet(pNet))
				{
					if (m_pDesign->getLayer()[pNGrid->m_nZ - m_nOffsetZ]->getDir() == 'H')
					{
						gGrid_C *pNNGrid = graphTravel(m_pDesign, pNGrid, 0, 1, 0);
						if (pNNGrid != NULL && pNNGrid->findNet(pNet))
						{
							vStart.push_back(pNNGrid);
							vDir.push_back('R');
							sDir.insert('R');
						}
						pNNGrid = graphTravel(m_pDesign, pNGrid, 0, -1, 0);
						if (pNNGrid != NULL && pNNGrid->findNet(pNet))
						{
							vStart.push_back(pNNGrid);
							vDir.push_back('L');
							sDir.insert('L');
						}
					}
					else
					{
						gGrid_C *pNNGrid = graphTravel(m_pDesign, pNGrid, 1, 0, 0);
						if (pNNGrid != NULL && pNNGrid->findNet(pNet))
						{
							vStart.push_back(pNNGrid);
							vDir.push_back('T');
							sDir.insert('T');
						}
						pNNGrid = graphTravel(m_pDesign, pNGrid, -1, 0, 0);
						if (pNNGrid != NULL && pNNGrid->findNet(pNet))
						{
							vStart.push_back(pNNGrid);
							vDir.push_back('D');
							sDir.insert('D');
						}
					}
					pGrid = pNGrid;
				}
			}
			else
			{
				nDX = 1;
				nDY = 0;
				nDZ = 0;
				gGrid_C *pNGrid = graphTravel(m_pDesign, pGrid, nDX, nDY, nDZ);
				if (pNGrid != NULL && pNGrid->findNet(pNet))
				{
					vStart.push_back(pNGrid);
					vDir.push_back('T');
					sDir.insert('T');
				}

				nDX = -1;
				nDY = 0;
				nDZ = 0;
				pNGrid = graphTravel(m_pDesign, pGrid, nDX, nDY, nDZ);
				if (pNGrid != NULL && pNGrid->findNet(pNet))
				{
					vStart.push_back(pNGrid);
					vDir.push_back('D');
					sDir.insert('D');
				}

				nDX = 0;
				nDY = 0;
				nDZ = 1;
				pNGrid = graphTravel(m_pDesign, pGrid, nDX, nDY, nDZ);
				while (pNGrid != NULL && pNGrid->findNet(pNet))
				{
					if (m_pDesign->getLayer()[pNGrid->m_nZ - m_nOffsetZ]->getDir() == 'H')
					{
						gGrid_C *pNNGrid = graphTravel(m_pDesign, pNGrid, 0, 1, 0);
						if (pNNGrid != NULL && pNNGrid->findNet(pNet))
						{
							vStart.push_back(pNNGrid);
							vDir.push_back('R');
							sDir.insert('R');
						}
						pNNGrid = graphTravel(m_pDesign, pNGrid, 0, -1, 0);
						if (pNNGrid != NULL && pNNGrid->findNet(pNet))
						{
							vStart.push_back(pNNGrid);
							vDir.push_back('L');
							sDir.insert('L');
						}
					}
					else
					{
						gGrid_C *pNNGrid = graphTravel(m_pDesign, pNGrid, 1, 0, 0);
						if (pNNGrid != NULL && pNNGrid->findNet(pNet))
						{
							vStart.push_back(pNNGrid);
							vDir.push_back('T');
							sDir.insert('T');
						}
						pNNGrid = graphTravel(m_pDesign, pNGrid, -1, 0, 0);
						if (pNNGrid != NULL && pNNGrid->findNet(pNet))
						{
							vStart.push_back(pNNGrid);
							vDir.push_back('D');
							sDir.insert('D');
						}
					}
					pGrid = pNGrid;
				}

				pGrid = pStart;
				nDX = 0;
				nDY = 0;
				nDZ = -1;
				pNGrid = graphTravel(m_pDesign, pGrid, nDX, nDY, nDZ);
				while (pNGrid != NULL && pNGrid->findNet(pNet))
				{
					if (m_pDesign->getLayer()[pNGrid->m_nZ - m_nOffsetZ]->getDir() == 'H')
					{
						gGrid_C *pNNGrid = graphTravel(m_pDesign, pNGrid, 0, 1, 0);
						if (pNNGrid != NULL && pNNGrid->findNet(pNet))
						{
							vStart.push_back(pNNGrid);
							vDir.push_back('R');
							sDir.insert('R');
						}
						pNNGrid = graphTravel(m_pDesign, pNGrid, 0, -1, 0);
						if (pNNGrid != NULL && pNNGrid->findNet(pNet))
						{
							vStart.push_back(pNNGrid);
							vDir.push_back('L');
							sDir.insert('L');
						}
					}
					else
					{
						gGrid_C *pNNGrid = graphTravel(m_pDesign, pNGrid, 1, 0, 0);
						if (pNNGrid != NULL && pNNGrid->findNet(pNet))
						{
							vStart.push_back(pNNGrid);
							vDir.push_back('T');
							sDir.insert('T');
						}
						pNNGrid = graphTravel(m_pDesign, pNGrid, -1, 0, 0);
						if (pNNGrid != NULL && pNNGrid->findNet(pNet))
						{
							vStart.push_back(pNNGrid);
							vDir.push_back('D');
							sDir.insert('D');
						}
					}
					pGrid = pNGrid;
				}
			}

			int nLF = 0;
			int nRF = 0;
			int nTF = 0;
			int nDF = 0;
			if (sDir.size() >= 4) // the forced is zero
			{
			}
			else
			{
				for (int j = 0; j < vStart.size(); j++)
				{
					pGrid = vStart[j];
					char cDir = vDir[j];
					int nDX, nDY;
					//gGrid_C* pGrid =
					if (cDir == 'R')
					{
						nDX = 0;
						nDY = 1;
						gGrid_C *pNGrid = graphTravel(m_pDesign, pGrid, nDX, nDY, 0);
						while (pNGrid != NULL && pNGrid->findNet(pNet))
						{
						}
					}
					else if (cDir == 'L')
					{
					}
					else if (cDir == 'T')
					{
					}
					else if (cDir == 'D')
					{
					}
				}
			}

			/*
		queue< gGrid_C* > qGrid;
		qGrid.push( pStart );
		while( nFX + nFY < 2 && bFind && !qGrid.empty() )
		{
			bFind = false;
			int nQSize = qGrid.size();
			int nCount = 0;
			while( nCount < nQSize )
			{
				pGrid = qGrid.front();
				qGrid.pop();
				nCount++;
				if( m_pDesign->getLayer()][ nZ - m_nOffsetZ ]->getDir() == 'H' ) // go in Y 
				{
					nDX = 0; nDY = 1; nDZ = 0;
					gGrid_C* pNGrid = graphTravel( m_pDesign, pGrid, nDX, nDY, nDZ );
					if( pNGrid != NULL && pNGrid->findNet( pNet ) )
					{
						bFind = true;
						qGrid.push( pNGrid );
						nFY = 1;
						nR = 1;
					}

					nDX = 0; nDY = -1; nDZ = 0;
					pNGrid = graphTravel( m_pDesign, pGrid, nDX, nDY, nDZ );
					if( pNGrid != NULL && pNGrid->findNet( pNet ) )
					{
						bFind = true;
						qGrid.push( pNGrid );
						nFY = 1;
						nL = 1;
					}
					
					nDX = 0; nDY = 0; nDZ = 1;
					pNGrid = graphTravel( m_pDesign, pGrid, nDX, nDY, nDZ );
					if( pNGrid != NULL && pNGrid->findNet( pNet ) )
					{
						bFind = true;
						qGrid.push( pNGrid );
					}
					
					nDX = 0; nDY = 0; nDZ = -1;
					pNGrid = graphTravel( m_pDesign, pGrid, nDX, nDY, nDZ );
					if( pNGrid != NULL && pNGrid->findNet( pNet ) )
					{
						bFind = true;
						qGrid.push( pNGrid );
					}
				}
				else
				{
					nDX = 1; nDY = 0; nDZ = 0;
					gGrid_C* pNGrid = graphTravel( m_pDesign, pGrid, nDX, nDY, nDZ );
					if( pNGrid != NULL && pNGrid->findNet( pNet ) )
					{
						bFind = true;
						qGrid.push( pNGrid );
						nFX = 1;
						nT = 1;
					}

					nDX = -1; nDY = 0; nDZ = 0;
					pNGrid = graphTravel( m_pDesign, pGrid, nDX, nDY, nDZ );
					if( pNGrid != NULL && pNGrid->findNet( pNet ) )
					{
						bFind = true;
						qGrid.push( pNGrid );
						nFX = 1;
						nD = 1;
					}
					
					nDX = 0; nDY = 0; nDZ = 1;
					pNGrid = graphTravel( m_pDesign, pGrid, nDX, nDY, nDZ );
					if( pNGrid != NULL && pNGrid->findNet( pNet ) )
					{
						bFind = true;
						qGrid.push( pNGrid );
					}
					
					nDX = 0; nDY = 0; nDZ = -1;
					pNGrid = graphTravel( m_pDesign, pGrid, nDX, nDY, nDZ );
					if( pNGrid != NULL && pNGrid->findNet( pNet ) )
					{
						bFind = true;
						qGrid.push( pNGrid );
					}
				
				}
			}
		}
		*/
		}
	}

bool router_C::checkOverflow()
{
	cout << "Check Overflow"<<endl;
	bool bOverflow = true;
	for( int z=0; z<m_vGraph.size(); z++ )
	{
		for( int y=0; y<m_vGraph[z].size(); y++ )
		{
			for( int x=0; x<m_vGraph[z][y].size(); x++ )
			{
				if( m_vGraph[z][y][x]->isOverflow() )
				{
					bOverflow = false;
					cout << "At grid: "<< x+1 << " " << y+1 << " " << z+1 << " with Supply: " << m_vGraph[z][y][x]->getSupply() << " Demand: "<< m_vGraph[z][y][x]->getDemand() << " Extra: " << m_vGraph[z][y][x]->getExtraDemand() << " Net: " << m_vGraph[z][y][x]->getNet().size() << endl;
					cout << "\tInstance: ";
					for( int i=0; i<m_vGraph[z][y][x]->getInstance().size(); i++ )
					{
						cout<<m_vGraph[z][y][x]->getInstance()[i]->getName()<<" ";
					}
					cout << endl;
					cout << "\tNet: ";
					for( int i=0; i<m_vGraph[z][y][x]->getNet().size(); i++ )
					{
						cout<<m_vGraph[z][y][x]->getNet()[i]->getName()<<" ";
					}
					cout << endl;

				}
			}
		}
	}
	if( !bOverflow )
		return false;
	else
		return true;
}
