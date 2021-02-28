#include "assistant.h"

using namespace std;

bool plot_process_result( vector< bool > &vResult, int nNumTerm, string strFilename )
{	
	if( nNumTerm == 0 )
	{
		cout<<"Warning: The number of terms is zero. No process analysis will be executed"<<endl;
		return false;
	}

	int nDataCount = 0;
	int nSuccessCount = 0;
	vector< int > vSuccessNumber;
	int nDataSize = vResult.size();
	int nDataPreTerm = nDataSize / nNumTerm;
	for( int i=0; i<vResult.size(); i++ )
	{
		if( vResult[i] )
			nSuccessCount++;
		nDataCount++;
		if( nDataCount == nDataPreTerm )
		{
			vSuccessNumber.push_back( nSuccessCount );
			nSuccessCount = 0;
			nDataCount = 0;
		}
	}
	
	if( vSuccessNumber.size() != 0 )
		vSuccessNumber.back() = vSuccessNumber.back() + nSuccessCount;
	else
		vSuccessNumber.push_back( nSuccessCount );

	// writing output
	ofstream fout;
	fout.open( strFilename.c_str(), ios::out );
	fout<<"x=[";
	for( int i=0; i<vSuccessNumber.size(); i++ )
	{
		fout<<i+1;
		if( i < vSuccessNumber.size()-1 )
			fout<<",";
	}
	fout<<"]"<<endl;

	fout<<"y=[";
	for( int i=0; i<vSuccessNumber.size(); i++ )
	{
		fout<<vSuccessNumber[i];
		if( i < vSuccessNumber.size()-1 )
			fout<<",";
	}
	fout<<"]"<<endl;

	fout<<"plot(x,y)"<<endl;
	fout.close();
	return true;
}

bool plot_movement_process( vector<int> &vMoveResult, string strFilename )
{
	if( vMoveResult.size() == 0 )
	{
		cout<<"Warning: No cell movement can be plotted."<<endl;
		return false;
	}
	
	ofstream fout;
	fout.open( strFilename.c_str(), ios::out );
	fout<<"x=[";
	for( int i=0; i<vMoveResult.size(); i++ )
	{
		fout<<i+1;
		if( i<vMoveResult.size()-1 )
			fout<<",";
	}
	fout<<"]"<<endl;
	
	fout<<"y=[";
	for( int i=0; i<vMoveResult.size(); i++ )
	{
		fout<<vMoveResult[i];
		if( i<vMoveResult.size()-1 )
			fout<<",";
	}
	fout<<"]"<<endl;	
	fout<<"plot(x,y)"<<endl;
	fout.close();
	return true;

}
