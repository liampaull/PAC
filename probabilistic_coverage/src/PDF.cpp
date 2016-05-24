/************************************************************/
/*    NAME: Liam Paull                                		*/
/*    ORGN: UNB             					*/
/*    FILE: PDF.cpp                               */
/*    DATE: November 16, 2011    					*/
/*             
/*    Description:                  				*/
/************************************************************/

#include <math.h>
#include <PDF.h>
#include <stdio.h>
#ifndef PI
    #define PI 3.141592653589793
#endif

#define DEBUG 0

PDF::PDF()
{
	numPoints=0;
}

PDF PDF::operator=(const PDF &PDFobj)
{
	x = PDFobj.x;
	fx = PDFobj.fx;
	numPoints = PDFobj.numPoints;
	return *this;
}

PDF::~PDF()
{
}

void PDF::Reset()
{
	x.clear();
	fx.clear();
	numPoints=0;
}

//TODO this function should figure out the right place to put the point and use "insert" rather than push_back assuming it's supposed to go at the end
void PDF::AddPoint(double newX, double newFX)
{
	//if (newX == 1.0) newX = 0.999999; //so that we don't get a nan in the log function when calculating entropy. 0log0 = 0 Remember that the x values in the pdf are actually the confidences.
	x.push_back(newX);
	fx.push_back(newFX);
	numPoints++;
}

void PDF::PrintOut()
{
	vector<double>::iterator i,fi;
	if (DEBUG) printf("PDF: Printout: \n x: \t");
	for (i=x.begin(); i!=x.end(); i++)
	{
		printf("%f\t",*i);
	}
	printf("\nfx:\t");
	for (fi=fx.begin(); fi!=fx.end(); fi++)
	{
		printf("%f\t",*fi);
	}
	printf("\n");
}


double PDF::GetMean()
{
	double mean=0;
	vector<double>::iterator i,fi;
	for (i=x.begin(), fi=fx.begin(); i!=x.end(),fi!=fx.end(); i++,fi++)
	{
		mean += (*i)*(*fi);
	}
	if (mean == 1.0) mean = 0.9999;
	return mean;
}

double PDF::Getfx(int i)
{
	return fx.at(i);
}

double PDF::Getx(int i)
{
	return x.at(i);
}



double PDF::Fx(double xVal)
{
	double F = 0;
	vector<double>::iterator i,fi;
	for (i=x.begin(), fi=fx.begin(); i!=x.end(), fi!=fx.end(); i++, fi++)
	{
		if ((*i) <= xVal)
			F += (*fi);
		else
			break;
	}
	return F;
}

void PDF::Normalize()
{
	double total = 0;
	vector<double>::iterator fi;
	for(fi=fx.begin();fi!=fx.end();fi++)
	{
		total += (*fi);
	}
	if (total==0) 
	{
		throw "ERROR: dividing by zero in PDF:Normalize. \n";
		return;
	}
	for (fi=fx.begin();fi!=fx.end();fi++)
	{
		(*fi) = (*fi)/total;
	}
}

//Make a Gaussian
void PDF::Gaussian(double mu, double sigma)
{
	x.clear();
	fx.clear();
	numPoints = 0;

	// If sigma is 0, then build one point at the mean and we're done
	if (sigma == 0)
	{
		printf("PDF:Gaussian: Got a sigma of zero. \n");
		AddPoint(mu,1);
		return;
	}

	//let's sample the Gaussian from -3sigma to 3sigma. NOTE: If sampling frequency is 1 then discrete pdf is already normalized.
	for (int i = int(floor(mu - 3*sigma)); i <= int(ceil(mu + 3*sigma)) ; i++)
	{
		AddPoint(i, (1/sqrt(2*PI*pow(sigma,2)))*exp(-(pow(i-mu,2)/(2*pow(sigma,2) ) ) ) );
	}


}


// Shift the pdf by deltaMu
void PDF::ShiftMean(double deltaMu)
{
	if (x.empty())
	{
		printf("PDF:ShitMean: warning! Trying to shift the mean of an empty RV!\n");
		return;
	}
	vector<double>::iterator i;
	for(i=x.begin();i!=x.end();i++)
	{
		(*i)+=deltaMu;
	}


}

double PDF::CalcEntropy()
{
	double runningTotal=0;

	vector<double>::iterator i, fi;
	
	for (i=x.begin(), fi=fx.begin(); i!=x.end(), fi!=fx.end(); i++,fi++)
	{
		if ( (*i) >= 1.0) (*i) = 0.9999;
		runningTotal += (*fi)*(-(*i)*log2((*i)) - (1-(*i))*(log2(1-(*i))));
		if (isnan(runningTotal))
		{
			printf("ISNAN in CalcEntropy from the logarithm with *i=%f\n",*i);
			throw ("ISNAN");
		}
	}

	if (numPoints==0) 
	{
		printf("divide by zero in CalcEntropy\n");
		throw ("ERROR: dividing by zero in PDF:CalcEntropy -> no points in RV. \n");
	}
	return runningTotal; // Jan 31 2012 removed dividing by numPoints but it's late so I might need it
}
