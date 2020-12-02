#include <inc/e57Reader.hpp>

#ifdef _WIN32
#include <filesystem>
#else
#include <experimental/filesystem>
#endif

//#include <boost/filesystem.hpp>
//namespace fs = boost::filesystem;

//  E57Simple.h.
#if defined(WIN32)
#include <windows.h>
#endif
#include <E57Simple.h>

#include <map>
#include <math.h>
#include <float.h>
namespace fs = std::experimental::filesystem;

struct E57Reader::Impl
{
  std::shared_ptr<e57::Reader> mpReader;
  string mFilename;
  int mSubsample;
  __int64 mPointCount;
  int mNumScan;
  int mCurrentScan;

  Impl(E57Reader* pReader)
  {
    mCurrentScan = 0;
    mNumScan = 0;
    mPointCount = 0;
    mSubsample = 1;
  }
  bool Open(const char* pFile)
  {
    mFilename = pFile;
    //mpReader = std::shared_ptr<e57::Reader>(new e57::Reader);
    Init();
    return true;
  }
  void Init();
  bool MoveNextScan();

  std::string GetScanName();
  void GetHeader(double scannerPos[3], double rotation[9], double ucs[16]);

  int ReadPoints(vector<float>& x, vector<float>& y, vector<float>& z,
                 vector<float>& rIntensity, vector<int>& rgbColor);

  void Reset();
};

bool E57Reader::Open(const char* pFilename)
{
  return mpImpl->Open(pFilename);
}

E57Reader::E57Reader()
{
  mpImpl = new Impl(this);
}

E57Reader::~E57Reader()
{
  delete mpImpl;
}

bool E57Reader::GetSize(int& column, int& width)
{
  return 0;
}

void E57Reader::ReadHeader(vector<string>& rHeader) {}

bool E57Reader::MoveNextScan()
{
  return mpImpl->MoveNextScan();
}

__int64 E57Reader::GetPointCount()
{
  return mpImpl-> mPointCount;
}

int E57Reader::GetNumScan() { return mpImpl->mNumScan; }

void E57Reader::Impl::Init()
{

  mNumScan = 0;//

  mCurrentScan = -1;
}

bool  E57Reader::Impl::MoveNextScan()
{
  if (++mCurrentScan >= mNumScan)
  { return false; }

  return false;
}

std::string E57Reader::GetScanName()
{
  return mpImpl->GetScanName();
}

void E57Reader::GetHeader(double scannerPos[3], double rotation[9], double ucs[16])
{
  mpImpl->GetHeader(scannerPos, rotation, ucs);
}

int E57Reader::ReadPoints(vector<float>& x, vector<float>& y, vector<float>& z,
                          vector<float>& rIntensity, vector<int>& rColor)
{
  return mpImpl->ReadPoints(x, y, z, rIntensity, rColor);
}

void E57Reader::Reset()
{
  return mpImpl->Reset();
}

std::string E57Reader::Impl::GetScanName()
{
  return std::to_string(mCurrentScan);
}

void E57Reader::Impl::GetHeader(double scannerPos[3], double rotation[9], double ucs[16])
{
}

int E57Reader::Impl::ReadPoints(vector<float>& x, vector<float>& y, vector<float>& z,
                                vector<float>& rIntensity, vector<int>& rgbColor)
{
  return 0;
}

void E57Reader::Impl::Reset()
{
  mCurrentScan = -1;
}