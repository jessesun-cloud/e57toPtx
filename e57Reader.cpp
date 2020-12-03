#include <inc/e57Reader.hpp>

#ifdef _WIN32
#include <filesystem>
#else
#include <experimental/filesystem>
#endif
//  E57Simple.h.
#if defined(WIN32)
#include <windows.h>
#endif
#include <E57Simple.h>
#include <Eigen/Dense>
#include <strstream>

namespace fs = std::experimental::filesystem;
using namespace e57;
using namespace Eigen;

struct E57Reader::Impl
{
  std::shared_ptr<e57::Reader> mpReader;
  string mFilename;
  int mSubsample;
  __int64 mPointCount;
  int mNumScan;
  int mCurrentScan;
  bool mbColumnIndex;
  vector<double>  mPosition;
  vector<float> mIntensity;
  vector<int >rowIndex, columnIndex;
  vector<uint8_t> mState;
  std::string mScanmColorsName;
  vector<int> mColors;
  bool mHasIntensity;
  bool mHasColor;
  bool mHasState;
  e57::Data3D mMetaData;

  Impl(E57Reader* pReader)
  {
    mCurrentScan = 0;
    mNumScan = 0;
    mPointCount = 0;
    mSubsample = 1;
  }
  bool Open(const char* pFile);
  void Init();
  bool MoveNextScan();
  bool GetSize(int& columns, int& rows);

  std::string GetScanName();
  bool GetHeader(double scannerPos[12],  double ucs[16]);

  size_t ReadPoints(vector<float>& x, vector<float>& y,
                    vector<float>& z, vector<float>& rIntensity,
                    vector<int>& rgbColor, CB pFun);
  void InitDataBuffer(int chunk, vector<SourceDestBuffer>& sdb);
  void Reset();
  void AnalysisFormat();
};

bool E57Reader::Open(const char* pFilename)
{
  return mpImpl->Open(pFilename);
}

bool E57Reader::Impl:: Open(const char* pFile)
{
  mFilename = pFile;
  fs::path fn(pFile);
  if (fs::exists(pFile) == false)
  {
    return false;
  }
  mpReader = std::shared_ptr<e57::Reader>(
               new e57::Reader(e57::ustring(pFile)));
  Init();
  return true;
}
E57Reader::E57Reader()
{
  mpImpl = new Impl(this);
}

E57Reader::~E57Reader()
{
  delete mpImpl;
}

bool E57Reader::GetSize(int& columns, int& rows)
{
  return mpImpl->GetSize(columns, rows);
}

bool E57Reader::Impl::GetSize(int& columns, int& rows)
{
  int64_t  numGroups = 0, numGroupPts = 0, nCol, nRow;
  bool ok = mpReader->GetData3DSizes(mCurrentScan, nRow,
                                     nCol, mPointCount, numGroups,
                                     numGroupPts, mbColumnIndex);
  columns = (int)nCol;
  rows = (int)nRow;
  if (mPointCount == 0)
  { mPointCount = nRow * nCol; }
  else
  {
    if (nCol == 0 && nRow == 0)
    {
      nCol = 1;
      nRow = mPointCount;
    }
  }
  return true;
}

bool E57Reader::Impl::GetHeader(double scannerPos[12], double ucs[16])
{
  e57::Data3D data3d;
  if (mpReader->ReadData3D((int32_t)mCurrentScan, data3d) == false)
  {
    return false;
  }
  memcpy(scannerPos, &data3d.pose.translation.x, sizeof(double) * 3);
  Eigen::Quaterniond quaternion = Eigen::Quaterniond(
                                    data3d.pose.rotation.w,
                                    data3d.pose.rotation.x,
                                    data3d.pose.rotation.y,
                                    data3d.pose.rotation.z);
  const Eigen::Vector3d rotX = quaternion * Eigen::Vector3d::UnitX();
  const Eigen::Vector3d rotY = quaternion * Eigen::Vector3d::UnitY();
  const Eigen::Vector3d rotZ = quaternion * Eigen::Vector3d::UnitZ();
  memcpy(scannerPos + 3, &rotX, sizeof(double) * 3);
  memcpy(scannerPos + 6, &rotY, sizeof(double) * 3);
  memcpy(scannerPos + 9, &rotZ, sizeof(double) * 3);
  Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
  memcpy(ucs, matrix.data(), sizeof(double) * 16);
  return true;
}

bool E57Reader::MoveNextScan()
{
  return mpImpl->MoveNextScan();
}

size_t E57Reader::GetPointCount()
{
  return mpImpl-> mPointCount;
}

int E57Reader::GetNumScan() { return mpImpl->mNumScan; }

void E57Reader::Impl::Init()
{
  mPointCount = 0;
  mNumScan = mpReader->GetData3DCount();
  mCurrentScan = -1;
}

bool  E57Reader::Impl::MoveNextScan()
{
  if (++mCurrentScan >= mNumScan)
  { return false; }

  if (mpReader->ReadData3D(mCurrentScan, mMetaData) == false)
  {
    return false;
  }
  AnalysisFormat();
  return true;
}

void E57Reader::Impl::AnalysisFormat()
{
  mHasIntensity = mMetaData.pointFields.intensityField;
  mHasColor = mMetaData.pointFields.colorBlueField &&
              mMetaData.pointFields.colorRedField &&
              mMetaData.pointFields.colorGreenField;
  mHasState = mMetaData.pointFields.cartesianInvalidStateField;
}

std::string E57Reader::GetScanName()
{
  return mpImpl->GetScanName();
}

bool E57Reader::GetHeader(double scannerPos[12], double ucs[16])
{
  return mpImpl->GetHeader(scannerPos, ucs);
}

size_t E57Reader::ReadPoints(vector<float>& x, vector<float>& y,
                             vector<float>& z,
                             vector<float>& rIntensity,
                             vector<int>& rColor, CB pFun)
{
  return mpImpl->ReadPoints(x, y, z, rIntensity, rColor, pFun);
}

void E57Reader::Reset()
{
  return mpImpl->Reset();
}

std::string E57Reader::Impl::GetScanName()
{
  return mMetaData.name;
}

void
E57Reader::Impl::InitDataBuffer(int chunkSize, vector<SourceDestBuffer>& sdb)
{
  using namespace e57;
  const size_t buf_size = chunkSize;
  mIntensity.reserve(buf_size);
  mPosition.reserve(buf_size * 3);
  rowIndex.reserve(buf_size);
  columnIndex.reserve(buf_size);
  mState.reserve(buf_size);
  mColors.reserve(buf_size);
  ImageFile imf = mpReader->GetRawData3D().destImageFile();
  auto fields = mMetaData.pointFields;
  if (fields.cartesianXField)
  {
    sdb.push_back(SourceDestBuffer(imf, "cartesianX",
                                   mPosition.data(), buf_size, true, true, 8));
    sdb.push_back(SourceDestBuffer(imf, "cartesianY",
                                   mPosition.data() + 1, buf_size, true, true, 8));
    sdb.push_back(SourceDestBuffer(imf, "cartesianZ",
                                   mPosition.data() + 2, buf_size, true, true, 8));
    if (fields.cartesianInvalidStateField)
    {
      sdb.push_back(SourceDestBuffer(imf, "cartesianInvalidState",
                                     mState.data(), buf_size, true, true));
    }
  }
  if (fields.intensityField)
  {
    sdb.push_back(SourceDestBuffer(imf, "intensity",
                                   mIntensity.data(), buf_size, true, true));
  }
  if (fields.colorBlueField && fields.colorGreenField && fields.colorRedField)
  {
    sdb.push_back(SourceDestBuffer(imf, "colorRed", (uint8_t*)mColors.data(),
                                   buf_size, true, true));
    sdb.push_back(SourceDestBuffer(imf, "colorGreen", ((uint8_t*)
                                   mColors.data()) + 1, buf_size, true, true, 4));
    sdb.push_back(SourceDestBuffer(imf, "colorBlue", ((uint8_t*)
                                   mColors.data()) + 2, buf_size, true, true, 4));
  }
}

//note does not handle large coordinate
size_t E57Reader::Impl::ReadPoints(vector<float>& x, vector<float>& y,
                                   vector<float>& z,
                                   vector<float>& rIntensity,
                                   vector<int>& rColor, CB pFun)
{
  e57::Data3D data3d;
  if (mpReader->ReadData3D((int32_t)mCurrentScan, data3d) == false)
  {
    return false;
  }
  const size_t chunkSize = 1024 * 1024;
  vector<SourceDestBuffer> sdb;
  InitDataBuffer(chunkSize, sdb);
  StructureNode scan(mpReader->GetRawData3D().get(mCurrentScan));
  CompressedVectorNode points(scan.get("points"));
  CompressedVectorReader vectorReader = points.reader(sdb);
  size_t total = 0;
  if (vectorReader.isOpen())
  {
    int np = 0;
    while ((np = vectorReader.read()) != 0)
    {
      x.resize(np);
      y.resize(np);
      z.resize(np);
      rIntensity.resize(np);
      if (mHasColor)
      { rColor.resize(np); }
      total += np;
      if (mHasIntensity)
      {
        mIntensity.resize(np);
        rIntensity.swap(mIntensity);
      }
      if (mHasColor)
      {
        mColors.resize(np);
        rColor.swap(mColors);
      }
      for (int i = 0; i < np; i++)
      {
        auto ax = (float)mPosition[i * 3];
        auto ay = (float)mPosition[i * 3 + 1];
        auto az = (float)mPosition[i * 3 + 2];
        bool hasdata = mHasState ? mState[i] : true;

        x[i] = (float)mPosition[i * 3];
        y[i] = (float)mPosition[i * 3 + 1];
        z[i] = (float)mPosition[i * 3 + 2];
        if (hasdata == false)
        {
          if (mHasIntensity)
          { mIntensity[i] = 0; }
        }
      }
      if (pFun)
      { pFun(x, y, z, rIntensity, rColor); }
    }
    vectorReader.close();
  }
  return total;
}

void E57Reader::Impl::Reset()
{
  mCurrentScan = -1;
}
