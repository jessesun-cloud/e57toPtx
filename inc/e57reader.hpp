#pragma once
#include <vector>
#include <string>
#include <stdio.h>

using namespace std;

class E57Reader
{
public:
  E57Reader();
  ~E57Reader();

  bool Open(const char* pFilename);
  bool GetSize(int& column, int& width);
  void ReadHeader(vector<string>& rHeader);
  std::string GetScanName();
  void GetHeader(double scannerPos[3], double rotation[9], double ucs[16]);
  bool MoveNextScan();
  int ReadPoints(vector<float>& x, vector<float>& y, vector<float>& z,
                 vector<float>& rIntensity, vector<int>& rgbColor);
  int GetNumScan();
  void Reset();
  __int64 GetPointCount();
private:
  struct Impl;
  Impl* mpImpl;
};