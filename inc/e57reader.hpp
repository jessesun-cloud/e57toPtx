#pragma once
#include <vector>
#include <string>
#include <functional>

using namespace std;

typedef std::function<bool(vector<float>& x, vector<float>& y,
                           vector<float>& z,
                           vector<float>& rIntensity,
                           vector<int>& rgbColor)> CB;
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
  size_t ReadPoints(vector<float>& x, vector<float>& y,
                    vector<float>& z,
                    vector<float>& rIntensity,
                    vector<int>& rgbColor,
                    CB pFun);
  int GetNumScan();
  void Reset();
  size_t GetPointCount();
private:
  struct Impl;
  Impl* mpImpl;
};