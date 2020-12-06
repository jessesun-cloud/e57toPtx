#include <inc/e57reader.hpp>
#include <ptxwriter.hpp>
#include <iostream>

#ifdef _WIN32
#include <filesystem>
#else
#include <experimental/filesystem>
#endif

namespace fs = std::experimental::filesystem;

std::string input, output;
int posPrecision = 3, intensityPrecision = 2, subsample = 1;
bool bWriteMultiple = false;

void printusage()
{
  printf("usage: ptxzip input output -p P -i I -s S"\
         "input -input filename"\
         "output -output filename"\
         "P -position precision, range 1-6"\
         "I -intensity precision, range 1-6"\
         "S -position precision, range 1-10"
        );
}

bool parseInput(int argc, char** argv)
{
  if (argc < 3)
  {
    printusage();
    return false;
  }

  input = argv[1];
  output = argv[2];
  for (int i = 3; i < argc; i++)
  {
    std::string key = argv[i];
    if (key[0] != '-')
    { continue; }
    if (key[1] == 'p')
    { posPrecision = atoi(argv[++i]); }
    else if (key[1] == 'i')
    { intensityPrecision = atoi(argv[++i]); }
    /*else if (key[1] == 's')
    { subsample = atoi(argv[++i]); }*/
  }
  return true;
}

int ProcessConvert()
{
  fs::path filename(input.c_str());
  const bool isE57 = filename.extension() == ".e57";
  if (isE57 == false)
  { return -1; }
  PtxWriter ptxwriter;
  if (ptxwriter.Open(output.c_str()) == false)
  {
    printf("can not create file %s", output.c_str());
    return -3;
  }
  ptxwriter.Init(posPrecision, intensityPrecision);

  E57Reader reader;
  __int64 total = 0;
  if (reader.Open(input.c_str()))
    while (reader.MoveNextScan())
    {
      ptxwriter.NextScan();
      int cols, rows;
      if (reader.GetSize(cols, rows) == false)
      { break; }
      ptxwriter.WriteSize(cols, rows);
      double scannerPos[12];
      double ucs[16];
      reader.GetHeader(scannerPos, ucs);
      std::string scanName = reader.GetScanName();
      ptxwriter.WriteHeader(scannerPos, ucs);

      auto ExportLambda = [&](int np, float * x,
                              float * pIntensity,
                              int* rgbColor)->bool
      {
        ptxwriter.WritePoints(np, x, pIntensity, rgbColor);
        return true;
      };
      size_t np = reader.ReadPoints(ExportLambda);
      total += np;
    }
  printf("convert %lld points\r\n", total);
  return 0;
}

std::string FormatFloat(float x, const char* pFormat);
int main(int argc, char** argv)
{
  if (parseInput(argc, argv))
  {

    return ProcessConvert();
  }
  return -1;
}
