#pragma once
#include <vector>
#include <string>
#include <functional>

using namespace std;

typedef std::function<bool(int numPoint,
                           float* x, float* y,
                           float* z,
                           float* rIntensity,
                           int* rgbColor)> PointsCB;
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
  size_t ReadPoints(PointsCB pFun);
  int GetNumScan();
  void Reset();
  size_t GetPointCount();
private:
  struct Impl;
  Impl* mpImpl;
};