/*
 
*/

#ifndef _PARSER_H_
#define _PARSER_H_

#include "rcm.h"
#include <fstream>

using namespace std;

bool parser( design_C*, char* ); 
bool parseMaxCell( design_C*, ifstream & );
bool parseBoundary( design_C*, ifstream & );
bool parseLayer( design_C*, ifstream & );
bool parseNonDefaultSupplyGGrid( design_C*, ifstream & );
bool parseCell( design_C*, ifstream & );
bool parseCellExtraDemand( design_C*, ifstream & );
bool parseInstance( design_C*, ifstream & );
bool parseNet( design_C*, ifstream & );
bool parseRoute( design_C*, ifstream & );

#endif
