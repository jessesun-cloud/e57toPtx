#pragma once
#include <vector>
#include <string>
#include <stdio.h>

using namespace std;

typedef bool CB(int*) ;
class E57Reader
{
public:
  E57Reader();
  ~E57Reader();

  bool Open(const char* pFilename);
  bool GetSize(int& columns, int& rows);
  std::string GetScanName();
  bool GetHeader(double scannerPos[12], double ucs[16]);
  bool MoveNextScan();
  int ReadPoints(vector<float>& x, vector<float>& y, vector<float>& z,
                 vector<float>& rIntensity, vector<int>& rgbColor);
  bool TravelPoints(CB fun);
  int GetNumScan();
  void Reset();
  __int64 GetPointCount();
private:
  struct Impl;
  Impl* mpImpl;
};