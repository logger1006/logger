#include "3Dresult.h"
#include <iostream>
#include <fstream>
#include <string>
#include <unordered_map>
#include <vector>
#include <limits.h>

using namespace std;

void matlab_graph(string file, string netname, design_C *pDesign, bool main, bool op) //op = 0 ->before ;op = 1 ->after
{
    fstream f;
    char cColor;
    if( main )
    	cColor = 'r';
    else
    	cColor = 'k';

    if (op == 0)
        f.open("./graph/" + file + ".m", ios::out);
    else if (op == 1)
        f.open("./graph/" + file + ".m", ios::app);
    //if (!f)
    //    cout
    //        << "output 3D-graph error!" << endl;
    else
    {
        unordered_map<string, net_C *> vNet_map = pDesign->getNet_map();
        vector<wire_C *> vWire = vNet_map[netname]->getWire();
        vector<instance_C *> nInst = vNet_map[netname]->getInst();
        vector<pin_C *> nPin = vNet_map[netname]->getPin();
        int bx = 0, by = 0, bz = 0;
        int sx = INT_MAX, sy = INT_MAX, sz = INT_MAX;
    	/*
    	if (op == 0)
            f << "subplot(1,2,1);" << endl;
        else if (op == 1)
            f << "subplot(1,2,2);" << endl;
        */
	//f << "plot(1,2)" ;<<endl;
        f << "subplot(1,2,1);" << endl;
	for (int x = 0; x < vWire.size(); x++) // wire
        {
            gGrid_C *g1 = vWire[x]->getGrid1();
            gGrid_C *g2 = vWire[x]->getGrid2();
            int x1, y1, z1, x2, y2, z2;
            g1->getPosition(x1, y1, z1);
            g2->getPosition(x2, y2, z2);
            if (max(x1, x2) > bx)
                bx = max(x1, x2);
            if (max(y1, y2) > by)
                by = max(y1, y2);
            if (max(z1, z2) > bz)
                bz = max(z1, z2);
            if (min(x1, x2) < sx)
                sx = min(x1, x2);
            if (min(y1, y2) < sy)
                sy = min(y1, y2);
            if (min(z1, z2) < sz)
                sz = min(z1, z2);

            if (x1 != x2)
            {
                if (x1 > x2)
                    swap(x1, x2);
                f << "x = " << x1 << ":1:" << x2 << ";" << endl;
                f << "for i = 1:" << (x2 - x1) + 1 << endl
                  << "y(1,i)=" << y1 << ";" << endl
                  << "end" << endl;
                f << "for i = 1:" << (x2 - x1) + 1 << endl
                  << "z(1,i)=" << z1 << ";" << endl
                  << "end" << endl;
                f << "plot3(x,y,z,'" << cColor << "','LineWidth',2);" << endl;
                f << "x=[];" << endl;
                f << "y=[];" << endl;
                f << "z=[];" << endl;
                // f << "plot3(x," << y1 << "," << z1 << ",'k*');" << endl;
            }
            else if (y1 != y2)
            {
                if (y1 > y2)
                    swap(y1, y2);
                f << "y = " << y1 << ":1:" << y2 << ";" << endl;
                f << "for i = 1:" << (y2 - y1) + 1 << endl
                  << "x(1,i)=" << x1 << ";" << endl
                  << "end" << endl;
                f << "for i = 1:" << (y2 - y1) + 1 << endl
                  << "z(1,i)=" << z1 << ";" << endl
                  << "end" << endl;
                f << "plot3(x,y,z,'" << cColor << "','LineWidth',2);" << endl;
                f << "x=[];" << endl;
                f << "y=[];" << endl;
                f << "z=[];" << endl;
                // f << "plot3(" << x1 << ","
                //   << "y"
                //   << "," << z1 << ",'k*');" << endl;
            }
            else if (z1 != z2)
            {
                if (z1 > z2)
                    swap(z1, z2);
                f << "z = " << z1 << ":1:" << z2 << ";" << endl;
                f << "for i = 1:" << (z2 - z1) + 1 << endl
                  << "x(1,i)=" << x1 << ";" << endl
                  << "end" << endl;
                f << "for i = 1:" << (z2 - z1) + 1 << endl
                  << "y(1,i)=" << y1 << ";" << endl
                  << "end" << endl;
                f << "plot3(x,y,z,'" << cColor << "','LineWidth',2);" << endl;
                f << "x=[];" << endl;
                f << "y=[];" << endl;
                f << "z=[];" << endl;
                // f << "plot3(" << x1 << "," << y1 << ","
                //   << "z"
                //   << ",'k*');" << endl;
            }
            f << "hold on;" << endl;
        }

        for (int x = 0; x < nInst.size(); x++) //pin
        {
            f << "plot3(" << nInst[x]->getPlacedX() << "," << nInst[x]->getPlacedY() << "," << nPin[x]->getLayerId() << ",'bd','MarkerSize',10,'MarkerFaceColor','b');" << endl;
            f << "text(" << nInst[x]->getPlacedX() << "," << nInst[x]->getPlacedY() << "," << nPin[x]->getLayerId() << ","
              << "'" << nInst[x]->getName() << "');" << endl;
        }

        for (int x = sx; x <= bx; x++) //grid demand
        {
            for (int y = sy; y <= by; y++)
            {
                for (int z = sz; z <= bz; z++)
                {
                    gGrid_C *tempG = pDesign->getGrid(x, y, z);
                    if (tempG->isLimit())
                        f << "plot3(" << x << "," << y << "," << z << ",'yo','MarkerSize',20);" << endl;
                }
            }
        }

        f << "grid on;" << endl;
        f << "xlabel('x');" << endl
          << "ylabel('y');" << endl
          << "zlabel('z');" << endl;
        if (op == 0)
            f << "title('before')  ;" << endl
              << endl;
        else if (op == 1)
            f << "title('after')  ;" << endl
              << endl;
        cout << "output 3D-graph success" << endl;
    }
    f.close();
    // return true;
}
void matlab_graph(string file, string netname, design_C *pDesign, bool op) //op = 0 ->before ;op = 1 ->after
{
    fstream f;
    if (op == 0)
        f.open("./graph/" + file + ".m", ios::out);
    else if (op == 1)
        f.open("./graph/" + file + ".m", ios::out | ios::app);
    if (!f)
        cout
            << "output 3D-graph error!" << endl;
    else
    {
        unordered_map<string, net_C *> vNet_map = pDesign->getNet_map();
        vector<wire_C *> vWire = vNet_map[netname]->getWire();
        vector<instance_C *> nInst = vNet_map[netname]->getInst();
        vector<pin_C *> nPin = vNet_map[netname]->getPin();
        int bx = 0, by = 0, bz = 0;
        int sx = INT_MAX, sy = INT_MAX, sz = INT_MAX;
        if (op == 0)
            f << "subplot(1,2,1);" << endl;
        else if (op == 1)
            f << "subplot(1,2,2);" << endl;
        for (int x = 0; x < vWire.size(); x++) // wire
        {
            gGrid_C *g1 = vWire[x]->getGrid1();
            gGrid_C *g2 = vWire[x]->getGrid2();
            int x1, y1, z1, x2, y2, z2;
            g1->getPosition(x1, y1, z1);
            g2->getPosition(x2, y2, z2);
            if (max(x1, x2) > bx)
                bx = max(x1, x2);
            if (max(y1, y2) > by)
                by = max(y1, y2);
            if (max(z1, z2) > bz)
                bz = max(z1, z2);
            if (min(x1, x2) < sx)
                sx = min(x1, x2);
            if (min(y1, y2) < sy)
                sy = min(y1, y2);
            if (min(z1, z2) < sz)
                sz = min(z1, z2);

            if (x1 != x2)
            {
                if (x1 > x2)
                    swap(x1, x2);
                f << "x = " << x1 << ":1:" << x2 << ";" << endl;
                f << "for i = 1:" << (x2 - x1) + 1 << endl
                  << "y(1,i)=" << y1 << ";" << endl
                  << "end" << endl;
                f << "for i = 1:" << (x2 - x1) + 1 << endl
                  << "z(1,i)=" << z1 << ";" << endl
                  << "end" << endl;
                f << "plot3(x,y,z,'k','LineWidth',2);" << endl;
                f << "x=[];" << endl;
                f << "y=[];" << endl;
                f << "z=[];" << endl;
                // f << "plot3(x," << y1 << "," << z1 << ",'k*');" << endl;
            }
            else if (y1 != y2)
            {
                if (y1 > y2)
                    swap(y1, y2);
                f << "y = " << y1 << ":1:" << y2 << ";" << endl;
                f << "for i = 1:" << (y2 - y1) + 1 << endl
                  << "x(1,i)=" << x1 << ";" << endl
                  << "end" << endl;
                f << "for i = 1:" << (y2 - y1) + 1 << endl
                  << "z(1,i)=" << z1 << ";" << endl
                  << "end" << endl;
                f << "plot3(x,y,z,'k','LineWidth',2);" << endl;
                f << "x=[];" << endl;
                f << "y=[];" << endl;
                f << "z=[];" << endl;
                // f << "plot3(" << x1 << ","
                //   << "y"
                //   << "," << z1 << ",'k*');" << endl;
            }
            else if (z1 != z2)
            {
                if (z1 > z2)
                    swap(z1, z2);
                f << "z = " << z1 << ":1:" << z2 << ";" << endl;
                f << "for i = 1:" << (z2 - z1) + 1 << endl
                  << "x(1,i)=" << x1 << ";" << endl
                  << "end" << endl;
                f << "for i = 1:" << (z2 - z1) + 1 << endl
                  << "y(1,i)=" << y1 << ";" << endl
                  << "end" << endl;
                f << "plot3(x,y,z,'k','LineWidth',2);" << endl;
                f << "x=[];" << endl;
                f << "y=[];" << endl;
                f << "z=[];" << endl;
                // f << "plot3(" << x1 << "," << y1 << ","
                //   << "z"
                //   << ",'k*');" << endl;
            }
            f << "hold on;" << endl;
        }

        for (int x = 0; x < nInst.size(); x++) //pin
        {
            f << "plot3(" << nInst[x]->getPlacedX() << "," << nInst[x]->getPlacedY() << "," << nPin[x]->getLayerId() << ",'bd','MarkerSize',10,'MarkerFaceColor','b');" << endl;
            f << "text(" << nInst[x]->getPlacedX() << "," << nInst[x]->getPlacedY() << "," << nPin[x]->getLayerId() << ","
              << "'" << nInst[x]->getName() << "');" << endl;
        }

        for (int x = sx; x <= bx; x++) //grid demand
        {
            for (int y = sy; y <= by; y++)
            {
                for (int z = sz; z <= bz; z++)
                {
                    gGrid_C *tempG = pDesign->getGrid(x, y, z);
                    if (tempG->isLimit())
                        f << "plot3(" << x << "," << y << "," << z << ",'ro','MarkerSize',20);" << endl;
                }
            }
        }

        f << "grid on;" << endl;
        f << "xlabel('x');" << endl
          << "ylabel('y');" << endl
          << "zlabel('z');" << endl;
        if (op == 0)
            f << "title('before')  ;" << endl
              << endl;
        else if (op == 1)
            f << "title('after')  ;" << endl
              << endl;
        cout << "output 3D-graph success" << endl;
    }
    f.close();
    // return true;
}
