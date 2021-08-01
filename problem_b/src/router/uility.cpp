#include "routing.h"
#include <iostream>
#include <algorithm>
#include <climits>
#include <stdlib.h>
#include <set>
#include <iomanip>
#include <fstream>
#include <cfloat>
#include <math.h>

using namespace std;

inline void forcedQuerier_C::update( forced_C* pF )
{
// if the forced is not in the list, which is fixed or failed to move in previous steps, it will not be in the set.

	if( m_sExistForced.count( pF ) == 0 )
	{
		m_sExistForced.insert( pF );
		bool bFind = false;
		for( list< forced_C* >::iterator it = m_lForced.begin(); it != m_lForced.end(); it++ )
		{
			forced_C* pTmpF = *it;
			if( pTmpF->m_dGain > pF->m_dGain && fabs( pTmpF->m_dGain - pF->m_dGain ) > 0.001 ) 
				continue;
			else 
			{	
				m_lForced.insert( it, pF );
				m_mIndex[ pF ] = it;
				bFind = true;
				break;
			}
		}
		if( !bFind )
		{
			m_lForced.push_back( pF );
			list< forced_C* >::iterator itEnd = m_lForced.end();
			itEnd--;
			m_mIndex[ pF ] = itEnd;
		}
	}
	else
	{
		list< forced_C* >::iterator it = m_mIndex[pF];
		list< forced_C* >::iterator itD = it;
		list< forced_C* >::iterator itI = it;
		if( it != m_lForced.begin() )
		{
			itD--;
			if( (*( itD ))->m_dGain < pF->m_dGain && fabs( (*( itD ))->m_dGain - pF->m_dGain ) > 0.001 )
			{
				for( it; it != m_lForced.begin(); it-- )
				{
					if( (*( itD ))->m_dGain < pF->m_dGain && fabs( (*( itD ))->m_dGain - pF->m_dGain ) > 0.001 )
					{
						forced_C* pTmpF = *(itD);
						*( itD ) = pF;
						*it = pTmpF;
						m_mIndex[ pTmpF ] = it;
						m_mIndex[ pF ] = itD;
						itD--;
					}
					else
						break;
				}
			}
			else 
			{
				itI++;
				if( itI++ != m_lForced.end() )
				{
					for( itI; itI != m_lForced.end(); itI++ )
					{
						if( (*( itI ))->m_dGain > pF->m_dGain && fabs( (*( itI ))->m_dGain - pF->m_dGain ) > 0.001  )
						{
							forced_C* pTmpF = *(itI);
							*( itI ) = pF;
							*it = pTmpF;
							m_mIndex[ pTmpF ] = it;
							m_mIndex[ pF ] = itI;
							it++;
						}
						else
							break;
					}
				}
			}

		}
		else
		{
			itI++;
			for( itI; itI != m_lForced.end(); itI++ )
			{
				if( (*( itI ))->m_dGain > pF->m_dGain && fabs( (*( itI ))->m_dGain - pF->m_dGain ) > 0.001  )
				{
					forced_C* pTmpF = *(itI);
					*itI = pF;
					*it = pTmpF;
					m_mIndex[ pTmpF ] = it;
					m_mIndex[ pF ] = itI;
					it++;
				}
				else
					break;
			}
		
		}
	}
};

inline void forcedQuerier_C::remove( forced_C* pF )
{
	m_sExistForced.erase( pF );
	m_lForced.remove( pF );
	m_mIndex.erase( pF );
}


inline forced_C* forcedQuerier_C::getForced()
{
	if( m_lForced.size() == 0 ) return NULL;
	else return m_lForced.front();
};

inline void forcedQuerier_C::add( forced_C* pF )
{
	m_lForced.push_back( pF );
}

inline void forcedQuerier_C::sort()
{
	m_lForced.sort( compare_nonicreasing_order );
	for( list< forced_C* >::iterator it = m_lForced.begin(); it != m_lForced.end(); it++ )
	{
		m_mIndex[ *it ] = it;
	}
}

inline bool compare_nonicreasing_order( forced_C* pFF, forced_C* pBF )
{
	return ( pFF->m_dGain > pBF->m_dGain && fabs( pFF->m_dGain - pBF->m_dGain ) > 0.001 );	
}
