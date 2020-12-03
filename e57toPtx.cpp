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
int posPrecision = 3, intensityPrecision = 3, subsample = 1;

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
    else if (key[1] == 's')
    { subsample = atoi(argv[++i]); }
  }
  return true;
}


int ProcessConvert()
{
  fs::path filename(input.c_str());
  const bool isE57File = filename.extension() == ".e57";
  if (isE57File == false)
  { return -1; }

  PtxWriter ptxwriter(output.c_str());
  ptxwriter.Init(posPrecision, intensityPrecision, subsample);
  if (ptxwriter.IsOpen() == false)
  {
    printf("can not create file %s", output.c_str());
    return -3;
  }

  E57Reader reader;
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
      vector<float>x, y, z, intensity;
      vector<int> color;
      int np;
      if (np = reader.ReadPoints(x, y, z, intensity, color) != 0)
      {
        ptxwriter.WritePoints(x, y, z, intensity, color);
      }
    }
  return 0;
}

int main(int argc, char** argv)
{
  if (parseInput(argc, argv))
  {
    return ProcessConvert();
  }
  return -1;
}
