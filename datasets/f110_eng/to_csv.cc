// ---------------------------------------------------------------------
// NN_FPGA Project: Neural Net for SoC/FPGA Architectures
// Copyright (C) 2020 ISAE
//
// Contacts:
// jean-baptiste.chaudron@isae-supaero.fr
// arnaud.dion@isae-supaero.fr
// ---------------------------------------------------------------------

// System includes
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <fstream> 
#include <string>
#include <algorithm> //Not part of string, use "algorithms" replace()

#include <sys/resource.h>

int main(int argc, char *argv[])
{
   std::ifstream prosis_csv;
   std::ofstream struct_csv;
   
   char delim;
   
	// copy from fpga stream
	std::string in_name;
	std::string out_name;
	if (argc > 1)
	{
		std::string file_name_tmp(argv[1]);
		in_name = file_name_tmp;
	// do stuff with arg1
	}
	else
	{
		std::string file_name_tmp = "toto.txt";
		in_name = file_name_tmp;
	}
	out_name = in_name.substr(0,in_name.find_last_of('.'))+".txt";
   prosis_csv.open(in_name.c_str());
   struct_csv.open(out_name.c_str());
   
	float mach[1000], altitude[1000], thrust[1000];
	float mach_it[7] = {0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9};
	int cnt = 0;
	std::string line;
	std::getline(prosis_csv, line);
	std::getline(prosis_csv, line);
	std::getline(prosis_csv, line);
	std::getline(prosis_csv, line);
	std::getline(prosis_csv, line);
	int i;
	for (i=0; i<105;i++)
	{
		float f1, f2, f3, f4, f5;
		prosis_csv >> f1 >> delim >> f2  >> delim >> f3 >> delim >> f4  >> delim >> f5 ;
		mach[cnt] = f1; 
		altitude[cnt] = f2; 
		thrust[cnt]  = f3; 
		std::cout << "mach = " << mach[cnt] << " altitude = " << altitude[cnt] << " thrust = " << thrust[cnt] << std::endl;
		cnt++;
	}
	struct_csv << "//-------------------------------------------------------------------\n" ;
	struct_csv << "const float table[8][16] = \n" ;
	struct_csv << "{\n" ;
	struct_csv << "\t{  " << "0.0" << ",  " << "1000.0" << ",  " << "1500.0" << ",  "  << "2000.0" << ",  " << "2500.0" << ",  " << "3000.0" << ",  " << "3500.0" << ",  " 
	<< "4000.0" << ",  " << "4500.0" << ",  " << "5000.0" << ",  " << "5500.0" << ",  " << "6000.0" << ",  " << "6500.0" <<",  " << "7000.0" << ",  " << "7500.0" << ",  " << "8000.0" << "},\n";
	
	for (int j=0; j<7;j++)
	{
		bool first_found = false;
		bool last_found = false;
		
		for (i=0; i<105;i++)
		{
			if ((abs(mach[i]-mach_it[j]) < 0.0001) && (abs(altitude[i]-1000.0) < 0.0001)) 
			{
				if (!first_found)
				{
					first_found = true;
					struct_csv << "\t{  " << mach_it[j] << ",  ";
				}
				struct_csv << thrust[i] << ",  ";
			}
			else if ((abs(mach[i]- mach_it[j]) < 0.0001) && (abs(altitude[i]-1500.0) < 0.0001)) 
			{
				struct_csv << thrust[i] << ",  ";
			}
			else if ((abs(mach[i]-mach_it[j]) < 0.0001) && (abs(altitude[i]-2000.0) < 0.0001)) 
			{
				struct_csv << thrust[i] << ",  ";
			}
			else if ((abs(mach[i]-mach_it[j]) < 0.0001) && (abs(altitude[i]-2500.0) < 0.0001)) 
			{
				struct_csv << thrust[i] << ",  ";
			}
			else if ((abs(mach[i]-mach_it[j]) < 0.0001) && (abs(altitude[i]-3000.0) < 0.0001)) 
			{
				struct_csv << thrust[i] << ",  ";
			}
			else if ((abs(mach[i]-mach_it[j]) < 0.0001) && (abs(altitude[i]-3500.0) < 0.0001)) 
			{
				struct_csv << thrust[i] << ",  ";
			}
			else if ((abs(mach[i]-mach_it[j]) < 0.0001) && (abs(altitude[i]-4000.0) < 0.0001)) 
			{
				struct_csv << thrust[i] << ",  ";
			}
			else if ((abs(mach[i]-mach_it[j]) < 0.0001) && (abs(altitude[i]-4500.0) < 0.0001)) 
			{
				struct_csv << thrust[i] << ",  ";
			}
			else if ((abs(mach[i]-mach_it[j]) < 0.0001) && (abs(altitude[i]-5000.0) < 0.0001)) 
			{
				struct_csv << thrust[i] << ",  ";
			}
			else if ((abs(mach[i]-mach_it[j]) < 0.0001) && (abs(altitude[i]-5500.0) < 0.0001)) 
			{
				struct_csv << thrust[i] << ",  ";
			}
			else if ((abs(mach[i]-mach_it[j]) < 0.0001) && (abs(altitude[i]-6000.0) < 0.0001)) 
			{
				struct_csv << thrust[i] << ",  ";
			}
			else if ((abs(mach[i]-mach_it[j]) < 0.0001) && (abs(altitude[i]-6500.0) < 0.0001)) 
			{
				struct_csv << thrust[i] << ",  ";
			}
			else if ((abs(mach[i]-mach_it[j]) < 0.0001) && (abs(altitude[i]-7000.0) < 0.0001)) 
			{
				struct_csv << thrust[i] << ",  ";
			}
			else if ((abs(mach[i]-mach_it[j]) < 0.0001) && (abs(altitude[i]-7500.0) < 0.0001)) 
			{
				struct_csv << thrust[i] << ",  ";
			}
			else if ((abs(mach[i]-mach_it[j]) < 0.0001) && (abs(altitude[i]-8000.0) < 0.0001)) 
			{
				struct_csv << thrust[i]; // << ",  ";
				if (!last_found)
				{
					last_found = true;
					struct_csv << "},\n";
				}
			}
			
		}
	}
	
	struct_csv << "}\n" ;
   prosis_csv.close();
   struct_csv.close();
   return 0;
}
