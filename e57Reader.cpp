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

#include <map>
#include <float.h>
#include <Eigen/Dense>
#include <strstream>

namespace fs = std::experimental::filesystem;
using namespace e57;

struct E57Reader::Impl
{
  std::shared_ptr<e57::Reader> mpReader;
  string mFilename;
  int mSubsample;
  __int64 mPointCount;
  int mNumScan;
  int mCurrentScan;
  bool mbColumnIndex;
  vector<double> mX;
  vector<double> mY;
  vector<double> mZ;
  vector<double> mIntensity;
  vector<uint8_t> mr, mg, mb;
  vector<int >rowIndex, columnIndex;
  vector<int> mState;
  std::string mScanName;
  int mFormat = -1;
  bool mHasIntensity;
  bool mHasColor;

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
    fs::path fn(pFile);
    if (fs::exists(pFile) == false)
    { return false; }
    mpReader = std::shared_ptr<e57::Reader>(
                 new e57::Reader(e57::ustring(pFile)));
    Init();
    return true;
  }
  void Init();
  bool MoveNextScan();
  bool GetSize(int& columns, int& rows);

  std::string GetScanName();
  bool GetHeader(double scannerPos[12],  double ucs[16]);

  int ReadPoints(vector<float>& x, vector<float>& y, vector<float>& z,
                 vector<float>& rIntensity, vector<int>& rgbColor);
  CompressedVectorReader InitReader(int chunk);
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

bool E57Reader::GetSize(int& columns, int& rows)
{
  return mpImpl->GetSize(columns, rows);
}

bool E57Reader::Impl::GetSize(int& columns, int& rows)
{
  int64_t  numGroups = 0, numGroupPts = 0, nCol, nRow;
  bool ok = mpReader->GetData3DSizes(mCurrentScan, nRow,
                                     nCol, mPointCount,
                                     numGroups, numGroupPts, mbColumnIndex);
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

  return ok;
}


bool E57Reader::Impl::GetHeader(double scannerPos[12], double ucs[16])
{
  e57::Data3D data3d;
  if (mpReader->ReadData3D((int32_t)mCurrentScan, data3d) == false)
  {
    return false;
  }
  mHasIntensity = data3d.pointFields.intensityField;
  mHasColor = data3d.pointFields.colorBlueField &&
              data3d.pointFields.colorRedField &&
              data3d.pointFields.colorGreenField;
  memcpy(scannerPos, &data3d.pose.translation.x, sizeof(double) * 3);
  //Eigen::Vector3d trans = *(Eigen::Vector3d*)&data3d.pose.translation.x;
  Eigen::Quaterniond quaternion = Eigen::Quaterniond(
                                    data3d.pose.rotation.w, data3d.pose.rotation.x,
                                    data3d.pose.rotation.y, data3d.pose.rotation.z);
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

__int64 E57Reader::GetPointCount()
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
  mFormat = -1;
  return true;
}

std::string E57Reader::GetScanName()
{
  return mpImpl->GetScanName();
}

bool E57Reader::GetHeader(double scannerPos[12], double ucs[16])
{
  return mpImpl->GetHeader(scannerPos, ucs);
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
  using namespace e57;
  StructureNode scan(mpReader->GetRawData3D().get(mCurrentScan));
  return scan.elementName();
}

CompressedVectorReader
E57Reader::Impl::InitReader(int chunkSize)
{
  using namespace e57;
  StructureNode scan(mpReader->GetRawData3D().get(mCurrentScan));

  CompressedVectorNode     points(scan.get("points"));
  StructureNode            prototype(points.prototype());
  vector<SourceDestBuffer> sdb;
  const size_t buf_size = chunkSize;
  //vector<std::variant<vector<double>, vector<int64_t>, vector<ustring> > > buf;
  mX.reserve(buf_size);
  mY.reserve(buf_size);
  mZ.reserve(buf_size);
  mIntensity.reserve(buf_size);
  mr.reserve(buf_size);
  mg.reserve(buf_size);
  mb.reserve(buf_size);
  rowIndex.reserve(buf_size);
  columnIndex.reserve(buf_size);
  mState.resize(buf_size);
  ImageFile imf = mpReader->GetRawData3D().destImageFile();

  for (int i = 0; i < prototype.childCount(); ++i)
  {
    Node n(prototype.get(i));
    auto name = n.elementName();
    switch (n.type())
    {
    case e57::E57_FLOAT:
    case e57::E57_SCALED_INTEGER:
    {
      bool scaled = n.type() == e57::E57_SCALED_INTEGER;
      double* pDbl = nullptr;
      if (name == "cartesianX") { pDbl = mX.data(); }
      else if (name == "cartesianY") { pDbl = mY.data(); }
      else if (name == "cartesianZ") { pDbl = mZ.data(); }
      else if (name == "intensity") { pDbl = mIntensity.data(); }
      if (pDbl == nullptr)
      {
        break;
      }

      sdb.push_back(
        SourceDestBuffer(
          imf
          , n.elementName()
          , pDbl
          , buf_size
          , true
          , true
        )
      );
      break;
    }
    case e57::E57_INTEGER:
    {
      uint8_t* pBuffer = nullptr;
      int* pInteger = nullptr;
      if (name == "colorRed") { pBuffer = mr.data(); }
      else if (name == "colorGreen") { pBuffer = mg.data(); }
      else if (name == "colorBlue") { pBuffer = mb.data(); }
      if (pBuffer == nullptr)
      {
        if (name == "rowIndex") { pInteger = rowIndex.data(); }
        else if (name == "columnIndex") { pInteger = columnIndex.data(); }
        //else if (name == "intensity") { pInteger = (int*)mIntensity.data(); }
        else if (name == "cartesianInvalidState") { pInteger = mState.data(); }
        if (pInteger != nullptr)
        {
          sdb.push_back(
            SourceDestBuffer(
              imf
              , n.elementName()
              , pInteger
              , buf_size
              , true
              , true
            )
          );
        }
        break;
      }

      sdb.push_back(
        SourceDestBuffer(
          imf
          , n.elementName()
          , pBuffer
          , buf_size
          , true
          , true
        )
      );
      break;
    }
    case e57::E57_STRING:
      break;
    default:
      throw (runtime_error(
               "prototype contains illegal type")
            );
    }
  }

  return points.reader(sdb);
}

//note does not handle large coordinate
int E57Reader::Impl::ReadPoints(vector<float>& x, vector<float>& y,
                                vector<float>& z,
                                vector<float>& rIntensity,
                                vector<int>& rgbColor)
{
  e57::Data3D data3d;
  if (mpReader->ReadData3D((int32_t)mCurrentScan, data3d) == false)
  {
    return false;
  }
  const size_t chunkSize = 1024 * 1024;
  CompressedVectorReader vectorReader = InitReader(chunkSize);
  int total = 0;
  strstream os;
  vectorReader.dump(2, os);
  string str = os.str();
  if (vectorReader.isOpen())
  {
    int np = 0;
    while (np = vectorReader.read() != 0)
    {
      x.resize(np);
      y.resize(np);
      z.resize(np);
      rIntensity.resize(np);
      if (mHasColor)
      { rgbColor.resize(np); }
      total += np;
      for (int i = 0; i < np; i++)
      {
        x[i] = (float)mX[i];
        y[i] = (float)mY[i];
        z[i] = (float)mZ[i];
        if (mHasIntensity)
        { rIntensity[i] = (float)mIntensity[i]; }
        else
        {
          rIntensity[i] = 0.5;
        }
        /*if (mHasColor)
        { rgbColor[i] = mr[i] + (mg[i] << 8) + (mb[i] << 16); }*/
      }
    }
  }
  return total;
}

void E57Reader::Impl::Reset()
{
  mCurrentScan = -1;
}

//to do test code
bool E57Reader::TravelPoints(CB fun)
{
  int vv;
  fun(&vv);
  return 0;
}