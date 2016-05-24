/************************************************************/
/*    NAME: Liam Paull                              		*/
/*    ORGN: UNB             					*/
/*    FILE: PDF.h                                       */
/*    DATE: November 16, 2011                                  */
/************************************************************/


#ifndef PDFH
#define PDFH

#include <vector>

using namespace std;

class PDF
{
public:
	PDF();
	PDF operator=(const PDF &PDFobj);
virtual ~PDF();

	double CalcEntropy();
	void Normalize();
	void AddPoint(double, double);

	void PrintOut();
	double GetMean();	
	double Getfx(int);
	double Getx(int);
	double Fx(double);
	void Gaussian(double, double);
	void ShiftMean(double);
	void Reset();

	int numPoints;
private:

	vector<double> x;
	vector<double> fx;
};

#endif
