#ifndef _ROUTING_H_
#define _ROUTING_H_
#include <vector>
#include <string>
#include <set>
#include "rcm.h"

using namespace std;

typedef int _history;
typedef vector< vector< vector< gGrid_C* > > > GRAPH3D;
typedef vector< vector< vector< _history > > > GRAPH3D_INT;
typedef vector< vector< bool > > GRAPH2D_BOOL;
typedef vector< vector< int > > GRAPH2D_INT;

class forced_C;
class networkForced_C;
class router_C;
class rGrid_C;
typedef vector< vector< vector< rGrid_C* > > > GRAPH3D_ROUTE;

class rGrid_C
{
	public:
	int m_nX;
	int m_nY;
	int m_nZ;
	int m_nH; // heuristic cost
	int m_nMH; // minor heuristic cost
	int m_nG; // current distance
	int m_nCost; // nH + nG
	int m_nBend; // current bend ( optional )
	bool isRouted;
	bool isPath;
	bool isTarget;
	bool isObstacle;
	// added at 0706 21:30
	gGrid_C* m_bTarget;
	// end added at 0706 21:30
	char m_cDir;
	rGrid_C* m_pFrom; // where come from
};

class forced_C
{
	public:
	instance_C* m_pInstance;
	vector< networkForced_C* > m_vNetwork;
	int m_nT, m_nD, m_nR, m_nL; // forced direction
	bool m_bLockInX;
	bool m_bLockInY;
	int m_nBoundX1, m_nBoundY1;
	int m_nBoundX2, m_nBoundY2;
	vector< int > m_vMinX;
	vector< int > m_vMaxX;
	vector< int > m_vMinY;
	vector< int > m_vMaxY;
	int m_nGain;

	vector< forced_C* > m_vMerged;
};

class networkForced_C
{
	public:
	net_C* m_pNet;
	// first version (no use now)
	int m_nCenterX, m_nCenterY;
	int m_nXHalf, m_nYHalf; // 0 for 0, 1 for 0.5
	
	// second version
	int m_nMinX, m_nMaxX;
	int m_nMinY, m_nMaxY;
	int m_nMinXCount;
	int m_nMaxXCount;
	int m_nMinYCount;
	int m_nMaxYCount;
	bool m_bSame;
// new parameter
	int m_n2MinX, m_n2MaxX;
	int m_n2MinY, m_n2MaxY;

	vector< forced_C* > m_vForced;
	unordered_map< forced_C*, int > m_mPLF;
	unordered_map< forced_C*, int > m_mPRF;
	unordered_map< forced_C*, int > m_mPTF;
	unordered_map< forced_C*, int > m_mPDF;
	
	unordered_map< forced_C*, int > m_mMinX;
	unordered_map< forced_C*, int > m_mMaxX;
	unordered_map< forced_C*, int > m_mMinY;
	unordered_map< forced_C*, int > m_mMaxY;
};

class forced_net_C
{
	public:
	net_C* m_pNet;
	vector< instance_C* > m_vCluster;
	vector< net_C* > m_vIntraNet;
	vector< net_C* > m_vInterNet;
	set< int > m_sInstId;
	bool m_bLock;
	bool m_bHasFixedInst;

	void setNet( net_C* pNet ){ m_pNet = pNet; }
	void addInstance( instance_C* pInst ){ m_vCluster.push_back( pInst ); }
	void addIntraNet( net_C* pNet ){ m_vIntraNet.push_back( pNet ); }
	void addInterNet( net_C* pNet ){ m_vInterNet.push_back( pNet ); }
	net_C* getNet(){ return m_pNet; }
	vector< instance_C* > getCluster(){ return m_vCluster; }
	vector< net_C* > getIntraNet(){ return m_vIntraNet; }
	vector< net_C* > getInterNet(){ return m_vInterNet; }

	int m_nTF;
	int m_nDF;
	int m_nRF;
	int m_nLF;

};

class router_C
{
	private:
	
	protected:
	// operaion information
	int m_nIteration;
	int m_nSuccess;
	int m_nFailed;
	int m_nNumRefinement;

	// design information
	design_C* m_pDesign;
	
	// information for router
	GRAPH3D m_vGraph;
	GRAPH3D_INT m_vHistoryGraph;
	GRAPH3D_ROUTE m_vRoutingGraph;
	
	GRAPH2D_INT m_vTargetGraph;
	GRAPH2D_BOOL m_v2DGraph;
	
	int m_nDX, m_nTX, m_nDY, m_nTY, m_nDZ, m_nTZ;
	int m_nOffsetX, m_nOffsetY, m_nOffsetZ;

	net_C* m_pTargetNet;
	int m_nConstraintLayerId;

	vector< forced_C > m_vForced;
	vector< forced_net_C > m_vNetForced;
	vector< networkForced_C > m_vNetworkForced;
	
	vector< instance_C* > m_vMovedInstance;
	
	// backup data
	instance_C m_cBackupInstance;
	vector< instance_C > m_vBackupInstance;
	vector< net_C > m_vBackupNet;

	_history getHGrid( int, int, int );
	rGrid_C* getRGrid( int, int, int );
	
	// design
	bool loadDesign();
	bool createHistoryGraph();
	bool createRoutingGraph();
	bool create2DGraph();

	// wire length calculation
	int estimateHPWL( net_C* );
	int calWireLength( net_C* );
	int estimateHPWLwithoutLayer( net_C* );

	// instance operation
// change at 0704 2230
	vector< gGrid_C* > findPlaceToMove( instance_C* );
	//gGrid_C* findPlaceToMove( instance_C* );
// end change
// change at 0705 02:00
	vector< gGrid_C* >findPlaceToMove( instance_C*, vector< net_C* > & );
	//gGrid_C* findPlaceToMove( instance_C*, vector< net_C* > & );
	vector< gGrid_C* >findPlaceToMove_ver3( instance_C* );
	vector< gGrid_C* >findPlaceToMove_ver4( instance_C* );
	vector< gGrid_C* >findPlaceToMove_ver4( instance_C*, set< net_C* > &, set< instance_C* > &);
// end change 
	bool removeInstOnGraph( instance_C* );
	bool putInstOnGraph( instance_C*, int, int, int );
	bool calPseudoPinDemand( instance_C*, int, int, int );
	bool delPseudoPinDemand( instance_C*, int, int, int );
	bool resetPseudoPinDemand( instance_C*, int, int, int );
	bool calForceDirection( vector< instance_C* > & );
	bool evaluateForceDirection( net_C* );
	instance_C* pickInstanceToMove();
	vector< instance_C* > pickInstanceToMove( int ); // rest of cost

	instance_C* pickHasMovedInstanceToMove();
	net_C* pickNetToMove();

	// net operation
	bool ripupNet( vector< net_C* > & );
	bool ripupNet( net_C* );

	// routing engine
	vector< gGrid_C* > routeNet( gGrid_C*, gGrid_C* ); // source grid, target grid
	vector< gGrid_C* > routeNet( vector< gGrid_C* > &, vector< gGrid_C* > & ); // multi-source multi-sink maze routing
	vector< gGrid_C* > routeNet( net_C* );
	vector< gGrid_C* > routeNet_length_constraint( net_C*, const int );
	vector< gGrid_C* > routeNet_neg_length_constraint( net_C*, const int, vector< int > & , vector< gGrid_C* > &);
	vector< vector< gGrid_C* > > routeNet_length_constraint_ver2( net_C*, const int );
	vector< gGrid_C* > routeNet_ver2( net_C* );
	vector< gGrid_C* > routeNet( net_C*, vector< gGrid_C* > & );
	
	bool multiNetRouting( vector< net_C* > &, const int, int & );
	
	bool rrr( vector< net_C* > & );
	bool pre_route( vector< net_C* > & );
	bool saveNet( net_C*, vector< gGrid_C* > & );
	bool saveNet( net_C*, vector< vector< gGrid_C* > > & );
	bool setRoutingConstraint( net_C* );
	bool reroute( instance_C* pInst, vector< net_C* > & );
	bool reroute( vector< instance_C* > &vInst, vector< net_C* > & );
	bool globalRoute( vector< net_C* >  &);

	// analysis
	bool createForcedModel();
	bool createForcedNetwork();
	bool createNetForcedModel();
	bool updateForcedModel( instance_C* );
	bool updateForcedNetwork( networkForced_C& );
	bool freeForcedModel( instance_C* );
	bool calForcedModel( forced_C& );
	bool calForcedModel_ver2( forced_C& );
	bool calForcedModel( forced_C&, set< net_C* > & );
	bool calForcedModel( forced_C&, vector< net_C* > &, int &, int &, int &, int & );
	bool calNetForcedModel( forced_C& );
	bool calForcedNetwork( networkForced_C& );
	bool calForcedNetwork( networkForced_C&, set< instance_C* > &);
	bool calNetForcedModel( forced_net_C& );
	bool linkForcedModel();
	vector< gGrid_C* > routingGridAnalysis( int, int, int, int ); // boundingbox; dx, tx, dy, ty;
	vector< gGrid_C* > adjBoundingBoxAnalysis( instance_C* pInst );
	vector< gGrid_C* > routingGridOrdering( vector< gGrid_C* > &, vector< gGrid_C* > & ); 	
	bool lockInstance( instance_C* );
	bool freeInstance( instance_C* );
	
	bool lockInstance( instance_C* ,char ); // X or Y
	bool freeInstance( instance_C* ,char ); // X or Y
	
	bool swapInstance( gGrid_C*, gGrid_C*, instance_C*, instance_C*, vector< net_C* > & );
	bool swapInstance_ver2( gGrid_C*, gGrid_C*, instance_C*, instance_C*, vector< net_C* > & );
	bool swapInstance( gGrid_C*, gGrid_C*, vector< instance_C* > &, vector< instance_C* > &, vector< net_C* > & );
	vector< instance_C* > findSwapInstance( gGrid_C*, gGrid_C*, instance_C* );

	// other operation
	bool isExchangable( gGrid_C*, instance_C*, instance_C* ); // target grid, to place instance, to remove instance.
	bool isPlaceable( gGrid_C*, instance_C* );
	bool isPlaceable( gGrid_C*, instance_C*, vector< net_C* > & );
	bool isPlaceable_ver4( gGrid_C*, instance_C*, vector< net_C* > & );
	bool isPlaceable_ver5( gGrid_C*, instance_C*, vector< net_C* > & );
	bool isPlaceable( gGrid_C*, vector< instance_C* > &, vector< net_C* > & );
	bool isPlaceable_ver3( gGrid_C*, instance_C*, vector< net_C* > & );
	bool checkConnectivity( net_C* );
	bool fixConnection( net_C* );
	bool setBoundingBox( net_C* );
	bool backupInstance( instance_C* );
	bool backupInstance( vector< instance_C* > &);
	bool recoverInstance( instance_C* );
	bool recoverInstance( vector< instance_C* > & );
	bool backupNet( vector< net_C* > & );
	bool recoverNet( vector< net_C* > &);
	bool cleanWire( vector< net_C* > & );
	bool forcedAnalysis( net_C* );
	bool forcedAnalysis( net_C*, vector< vector< gGrid_C* > > & );

	vector< forced_C* > recalForced( int );
	vector< forced_C* > moveingCellCollection( int );
	vector< forced_C* > moveingCellCollection_ver2( int );


	bool singleCellMovement( instance_C* );
	bool singleCellMovement_ver2( instance_C* );
// added at 0705 02:00
	bool singleCellMovement_ver3( instance_C* );
// end added
	bool multipleCellMovement( vector< instance_C* > & );
	bool multipleCellMovement_ver2( vector< instance_C* > & );
	bool multipleCellMovement_ver3( vector< instance_C* > & );
	bool multipleCellMovement_ver4( vector< instance_C* > & ); // with multiNet ripup & reroute
	bool multipleCellMovement( net_C* );
	bool moveCell( vector< instance_C* >& );
	int calDistanceFromPath( rGrid_C*, int );
	bool reduceOverflow( vector< net_C* > &, gGrid_C*, int & );

	unordered_map< instance_C*, vector< instance_C > > m_vInstHistory;

	public:
	router_C( design_C* pDesign ):m_pDesign(pDesign), m_nIteration(0), m_nFailed(0), m_nSuccess(0), m_nNumRefinement(0){};
	~router_C(){};
	bool init();
	bool startOpt(); // main function for router
	bool test();

	bool checkOverflow();
	bool dumpResult( char* );
	bool dumpDetailInfo();
	bool showSummary();
};

inline int boundingBoxCost( int, vector<int> &, vector<int> & ); // cur, min, max 
inline int nEularDistance( gGrid_C*, gGrid_C*, layer_C* );
inline int nEularDistance( vector< gGrid_C* >, gGrid_C*, layer_C* );
// added at 0704 23:30
inline int nEularDistance_sum( vector< gGrid_C* >, gGrid_C*, layer_C* );
inline int nEularDistance_min( vector< gGrid_C* >, gGrid_C*, layer_C* );
// added at 0706 21:00
inline int nEularDistance_FaS( vector< gGrid_C* >, gGrid_C*, layer_C*, rGrid_C* );
// end added at 0706 21:00
// end added at 0704 23:30

#endif