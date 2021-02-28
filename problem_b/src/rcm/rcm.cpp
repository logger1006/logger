#include "rcm.h"
#include <stdlib.h>
#include <iostream>
#include <set>

using namespace std;


bool design_C::checkInfo()
{
	//cout<<getGrid( this, 21, 15, 1 )->getSupply()<<endl;
	//cout<<getGrid( this, 21, 15, 1 )->getDemand()<<endl;
	//cout<<getGrid( this, 21, 15, 1 )->getExtraDemand()<<endl;
	//cout<<getGrid( this, 21, 15, 1 )->getNet().size()<<endl;
	//vector< net_C* > vNet = getGrid( this, 21, 15, 1 )->getNet();
	//for( int i=0; i<vNet.size(); i++ )
	//{
	//	cout<<vNet[i]->getName()<<endl;
	//}
	
	cout<<"Design boundary: "<<m_nGGridRowEnd<<" x "<<m_nGGridColEnd<<endl;
	cout<<"Number of layer: "<<m_vLayer.size()<<endl;
	cout<<"Number of cell: "<<m_vCell.size()<<endl;
	cout<<"Number of instance: "<<m_vInstance.size()<<endl;
	cout<<"Number of net: "<<m_vNet.size()<<endl;
	cout<<"Maximum of Cell Movement: "<<m_nMaxCellMove<<endl;
	cout<<"Total wire length: "<<calWireLength( this )<<endl;
	//cout<<"Total overflow:    "<<calOverflow( this )<<endl;
	return true;
}

int calWireLength( design_C* pDesign )
{
	int nWireLength = 0;
	/*
	vector< net_C* > vNet = pDesign->getNet();
	for( int i=0; i<vNet.size(); i++ )
	{
		set< gGrid_C* > sGrid;
		net_C* pNet = vNet[i];
		vector< wire_C* > vWire = pNet->getWire();
		for( int j=0; j<vWire.size(); j++ )
		{
			wire_C* pWire = vWire[j];
			gGrid_C* pGrid1 = pWire->getGrid1();
			gGrid_C* pGrid2 = pWire->getGrid2();
			
			int nX1, nY1, nZ1;
			int nX2, nY2, nZ2;
			pGrid1->getPosition( nX1, nY1, nZ1 );
			pGrid2->getPosition( nX2, nY2, nZ2 );
			nWireLength = nWireLength + abs( nX1 - nX2 )  + abs( nY1 - nY2 ) + abs( nZ1 - nZ2 ) + 1;
			
			if( !sGrid.empty() )
			{
				if( sGrid.count( pGrid1 ) != 0 )
				{
					nWireLength--;
				}
				else
				{
					sGrid.insert( pGrid1 );
				}
				
				if( sGrid.count( pGrid2 ) != 0 )
				{
					nWireLength--;
				}
				else
				{
					sGrid.insert( pGrid2 );
				}
			}
			else
			{
				
					sGrid.insert( pGrid1 );
					sGrid.insert( pGrid2 );
			}
		}
	}
	*/
	vector< vector< vector< gGrid_C* > > > vGraph;
	pDesign->getGraph( vGraph );
	
	int nOverflow = 0;
	int nX1, nY1, nX2, nY2;
	pDesign->getBoundary( nX1, nY1, nX2, nY2 );
	vector< layer_C* > vLayer = pDesign->getLayer();
	int nZ1 = vLayer.front()->getId();
	int nZ2 = vLayer.back()->getId();

	for( int z=nZ1; z<=nZ2; z++ )
	{
		for( int y=nY1; y<=nY2; y++ )
		{
			for( int x=nX1; x<=nX2; x++ )
			{
				gGrid_C* pGrid = getGrid( pDesign, x, y, z);
				nWireLength = nWireLength + pGrid->getNet().size();
			}
		}
	}
	return nWireLength;
}

int calOverflow( design_C* pDesign )
{
	vector< vector< vector< gGrid_C* > > > vGraph;
	pDesign->getGraph( vGraph );
	
	int nOverflow = 0;
	int nX1, nY1, nX2, nY2;
	pDesign->getBoundary( nX1, nY1, nX2, nY2 );
	vector< layer_C* > vLayer = pDesign->getLayer();
	int nZ1 = vLayer.front()->getId();
	int nZ2 = vLayer.back()->getId();

	for( int z=nZ1; z<=nZ2; z++ )
	{
		for( int y=nY1; y<=nY2; y++ )
		{
			for( int x=nX1; x<=nX2; x++ )
			{
				gGrid_C* pGrid = getGrid( pDesign, x, y, z);
				if( pGrid->isOverflow() )
					nOverflow++;
			}
		}
	}
	return nOverflow;

}

bool dumpResult( design_C* pDesign, char* pFileName )
{
	//ofstream fout;
	//fout.open( pFileName, ios::out );
	//fout << "NumMovedCellInst" << endl;
	//fout.close();
}
