/*
 * Author :Jianchao Song
 * For parameter read
 * */
#pragma once 

#include <stdio.h>
#include <fstream>
#include <map>
using namespace std;
class ParameterReader
{
	private:
		//FILE *fp;
		string fileName;
		map<string, string> data;
	public:
		ParameterReader(string file)
		{
			fileName=file;
			ifstream fin(fileName.c_str());
			if (! fin)
			{
				//printf("%s \n",fileName);
cout<<fileName.c_str()<<endl;
				cerr<<"parameter file does not exist."<<endl;
				return;
			}
			while (	! fin.eof())
			{
				string str;
				getline(fin,str);
				if (str[0]=='#')
				{
					continue;
				}
				int pos=str.find("=");
				if (pos==-1)
				{
					continue;
				}
				string key =str.substr(0,pos);
				string value =str.substr(pos+1,str.length());
				data[key]=value;
				if (!fin.good())
				{
					break;
				}
				
			}
			
		}
		string getData( string key )
		{
			map<string, string>::iterator iter = data.find(key);
			if (iter == data.end())
			{
				cerr<<"Parameter name "<<key<<" not found!"<<endl;
				return string("NOT_FOUND");
			}
			return iter->second;
		}
};

