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
  vector<double> mIntensity;
  vector<int32_t> rowIndex, columnIndex;
  vector<int8_t> mState, mIntensityState, mColorState;
  std::string mScanmColorsName;
  vector<uint16_t> mColors;
  vector<int32_t> mIndexs;
  bool mHasIntensity;
  bool mHasColor;
  bool mHasState;
  e57::Data3D mMetaData;
  double mIntensityScale;

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

  bool GetHeader(double scannerPos[12],  double ucs[16]);
  size_t ReadPoints(PointsCB pFun);
  void AnalysisFormat();
  void InitFields(int size);
  float ConvertIntensity(double intensity);
  CompressedVectorReader InitDataReader(int);
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
  mHasIntensity = false;
  mHasColor = false;
  mIntensityScale = 1;
}

void E57Reader::Impl::InitFields(int chunkSize)
{
  mPosition.resize(chunkSize * 3);
  mIntensityState.resize(chunkSize);
  mIntensity.resize(chunkSize);

  if (mMetaData.pointFields.rowIndexField)
  {
    rowIndex.resize(chunkSize);
    columnIndex.resize(chunkSize);
  }
  else
  {
    rowIndex.clear();
    columnIndex.clear();
  }
  mState.resize(chunkSize);
  if (mMetaData.pointFields.colorBlueField)
  {
    mColors.resize(chunkSize * 3);
    mColorState.resize(chunkSize);
  }
  else
  {
    mColors.clear();
    mColorState.clear();
  }
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
  if (mHasIntensity)
  {
    double s = mMetaData.intensityLimits.intensityMaximum -
               mMetaData.intensityLimits.intensityMinimum;
    if (s != 0)
    { mIntensityScale = 1 / s; }
  }

  mHasColor = mMetaData.pointFields.colorBlueField &&
              mMetaData.pointFields.colorRedField &&
              mMetaData.pointFields.colorGreenField;
  mHasState = mMetaData.pointFields.cartesianInvalidStateField;
}

std::string E57Reader::GetScanName()
{
  return mpImpl->mMetaData.name;
}

bool E57Reader::GetHeader(double scannerPos[12], double ucs[16])
{
  return mpImpl->GetHeader(scannerPos, ucs);
}

size_t
E57Reader::ReadPoints(PointsCB pFun)
{
  return mpImpl->ReadPoints(pFun);
}

void E57Reader::Reset()
{
  mpImpl->Init();
}

float E57Reader::Impl::ConvertIntensity(double intensity)
{
  return (float)
         ((intensity - mMetaData.intensityLimits.intensityMinimum)
          * mIntensityScale);
}

CompressedVectorReader E57Reader::Impl::InitDataReader(int chunkSize)
{
  InitFields(chunkSize);
  return mpReader->SetUpData3DPointsData(
           mCurrentScan, chunkSize,
           mPosition.data(),//X
           mPosition.data() + chunkSize,//Y
           mPosition.data() + chunkSize * 2, //Z
           mState.data(),
           mIntensity.data(), mIntensityState.data(),
           mColors.data(), //color red
           (mHasColor) ? mColors.data() + chunkSize : nullptr,
           (mHasColor) ? mColors.data() + chunkSize * 2 : nullptr,
           mColorState.data(),
           nullptr, nullptr, nullptr, nullptr,
           rowIndex.data(),
           columnIndex.data());
}

//note does not handle large coordinate
size_t
E57Reader::Impl::ReadPoints(PointsCB pFun)
{
  const size_t chunkSize = 1024 * 1024;
  vector<float> x, y, z, intensity;
  vector<int> color;
  CompressedVectorReader vectorReader = InitDataReader(chunkSize);
  size_t total = 0;
  if (vectorReader.isOpen())
  {
    int np = 0;
    while ((np = vectorReader.read()) != 0)
    {
      x.resize(np);
      y.resize(np);
      z.resize(np);
      if (mHasColor)
      { color.resize(np); }
      total += np;
      if (mHasIntensity)
      {
        intensity.resize(np);
      }
      else
      {
        intensity.clear();
        intensity.resize(np, 0.5);
      }
      for (int i = 0; i < np; i++)
      {
        bool hasdata = mHasState ? mState[i] == 0 : true;
        x[i] = (float)mPosition[i];
        y[i] = (float)mPosition[chunkSize + i];
        z[i] = (float)mPosition[chunkSize * 2 + i];
        if (mHasIntensity)
        {
          intensity[i] = (float)ConvertIntensity(mIntensity[i]);
        }
        if (hasdata == false)
        {
          if (mHasIntensity)
          { mIntensity[i] = 0; }
        }
        if (mHasColor)
        {
          uint8_t r = (uint8_t)mColors[i];
          uint8_t g = (uint8_t)mColors[chunkSize + i];
          uint8_t b = (uint8_t)mColors[chunkSize * 2 + i];
          color[i] = (int)r + (g << 8) + (b << 16);
        }
      }
      if (pFun)
      {
        pFun(np, x.data(), y.data(), z.data(),
             intensity.data(),
             mHasColor ? color.data() : nullptr);
      }
    }
    vectorReader.close();
  }
  return total;
}
