#include <inc/e57reader.hpp>
#include <E57Format.h>
#include <E57Exception.h>
using namespace e57;
#include <strstream>
#ifdef _WIN32
#include <filesystem>
#else
#include <experimental/filesystem>
#endif

namespace fs = std::experimental::filesystem;

struct E57Reader::Impl
{
  E57Reader* mpReader;
  shared_ptr<e57::ImageFile> mpImf;
  string mFilename;
  int mSubsample;
  __int64 mPointCount;
  int mNumScan;
  int mCurrentScan;

  Impl(E57Reader* pReader)
  {
    mCurrentScan = 0;
    mNumScan = 0;
    mpReader = pReader;
    mPointCount = 0;
    mSubsample = 1;
  }
  bool Open(const char* pFile)
  {
    mFilename = pFile;
    mpImf = shared_ptr<e57::ImageFile>(new e57::ImageFile(pFile, "r"));
    Init();
    return true;
  }
  void Init();
  bool MoveNextScan();
};

bool E57Reader::Open(const char* pFilename)
{
  return mpImpl->Open(pFilename);
}

E57Reader::E57Reader()
{
  mpImpl = new Impl(this);
}

E57Reader::~E57Reader() {}

bool E57Reader::ReadSize(int& column, int& width)
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
  StructureNode root = mpImf->root();
  fs::path dst;

  if (root.isDefined("data3D"))
  {
    VectorNode data3D(root.get("data3D"));
    mNumScan = data3D.childCount();
  }
  mCurrentScan = -1;
}

bool  E57Reader::Impl::MoveNextScan()
{
  if (++mCurrentScan >= mNumScan)
  { return false; }
  StructureNode root = mpImf->root();
  string str;
  std::strstream s;
  root.dump(2, s);
  std::string astr = s.str();
  if (root.isDefined("data3D"))
  {
    VectorNode data3D(root.get("data3D"));

    StructureNode            scan(data3D.get(mCurrentScan));
    CompressedVectorNode     points(scan.get("points"));
    StructureNode            prototype(points.prototype());
  }
  return true;
}
