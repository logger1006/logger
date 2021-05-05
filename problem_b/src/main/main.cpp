#include "main.h"
#include <iostream>
#include <omp.h>
#include "3Dresult.h"
#include <unordered_map>
using namespace std;

extern bool parser(design_C *pDesign, char *pFileName);
extern bool dumpResult(design_C *pDesign, char *pFileName);
time_t start, endt;

int main(int argc, char **argv)
{

	start = time(NULL);
	cout << "ICCAD Contest B" << endl;
	cout << "Input:  " << argv[1] << endl;
	cout << "Output: " << argv[2] << endl;
	design_C *pDesign;
	pDesign = new design_C;

	cout << endl;
	cout << "Parsing input..." << endl;
	parser(pDesign, argv[1]);
	pDesign->checkInfo();
	//dumpGraph(pDesign);
	//matlab_graph("out", "N2", pDesign, 0);

	cout << endl;
	cout << "Call RCM Router..." << endl;
	router_C *pRouter;
	pRouter = new router_C(pDesign);
	pRouter->init();
	
	//pRouter->findGroup()	
	//return 0;
	//pRouter->test();
	cout << endl;
	pRouter->startOpt();

	pRouter->showSummary();
	//dumpResult( pDesign, argv[2] );
	cout << "Writing to output..." << endl;
	pRouter->dumpResult(argv[2]);

	//dumpGraph(pDesign);
	//pRouter->dumpDetailInfo();
	//matlab_graph("out", "N2", pDesign, 1);
	endt = time(NULL);
	cout << "Spend time: " << endt - start << endl;

	return 0;
}
