#include <inc/e57reader.hpp>
#include <ptxwriter.hpp>

#include <E57Format.h>
#include <E57Exception.h>
using namespace e57;

#ifdef _WIN32
#include <filesystem>
#else
#include <experimental/filesystem>
#endif

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
//using std::ostream;
using std::ios_base;
using std::streamsize;

#include <exception>
using std::exception;

#include <stdexcept>
using std::runtime_error;

#if defined(_MSC_VER) || defined(__APPLE__)
#   include <memory>
#else
#   include <tr1/memory>
#endif
#include <boost/config.hpp>

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
using boost::filesystem::ofstream;

#include <boost/variant.hpp>
using boost::variant;
using boost::get;
using boost::static_visitor;
using boost::apply_visitor;

#include <boost/format.hpp>
using boost::format;
using boost::io::too_many_args_bit;
using boost::io::all_error_bits;

#include <boost/lexical_cast.hpp>
using boost::lexical_cast;

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

class get_at
  : public static_visitor<variant<double, int64_t, ustring> >
{
  size_t at;
public:
  get_at(size_t at_) : at(at_) {}
  template <typename T>
  variant<double, int64_t, ustring> operator()(T& operand) const
  {
    return operand[at];
  }
};

int ProcessConvert()
{
  fs::path filename(input.c_str());
  const bool isE57File = filename.extension() == ".e57";
  if (isE57File == false)
  { return -1; }
  e57::ImageFile imf(input, "r");
  StructureNode root = imf.root();
  fs::path dst;
  std::ofstream root_inf(dst / "root.inf");
  root_inf << "formatName = " << StringNode(root.get("formatName")).value() << endl;
  root_inf << "guid = " << StringNode(root.get("guid")).value() << endl;
  root_inf << "versionMajor = " << IntegerNode(root.get("versionMajor")).value() << endl;
  root_inf << "versionMinor = " << IntegerNode(root.get("versionMinor")).value() << endl;
  if (root.isDefined("e57LibraryVersion"))
  {
    root_inf << "e57LibraryVersion = " << StringNode(root.get("e57LibraryVersion")).value() << endl;
  }
  if (root.isDefined("coordinateMetadata"))
  {
    root_inf << "coordinateMetadata = " << StringNode(root.get("coordinateMetadata")).value() << endl;
  }
  if (root.isDefined("creationDateTime"))
  {
    StructureNode t(root.get("creationDateTime"));
    root_inf << format("creationDateTime.dateTimeValue = %.15g\n") % FloatNode(t.get("dateTimeValue")).value();
    if (t.isDefined("isAtomicClockReferenced"))
    {
      root_inf << "creationDateTime.isAtomicClockReferenced = " << IntegerNode(t.get("isAtomicClockReferenced")).value() <<
               endl;
    }
  }
  root_inf.close();

  if (root.isDefined("data3D"))
  {
    VectorNode data3D(root.get("data3D"));
    for (int64_t child = 0; child < data3D.childCount(); ++child)
    {
      StructureNode            scan(data3D.get(child));
      CompressedVectorNode     points(scan.get("points"));
      StructureNode            prototype(points.prototype());
      vector<SourceDestBuffer> sdb;
      const size_t buf_size = 1024;
      vector<variant<vector<double>, vector<int64_t>, vector<ustring> > > buf;
      string pointrecord;
      string comma;
      string fmt;
      //if (!opt.count("format"))
      {
        fmt.clear();
        for (int64_t i = 0; i < prototype.childCount(); ++i)
        {
          switch (prototype.get(i).type())
          {
          case e57::E57_FLOAT:
          case e57::E57_SCALED_INTEGER:
            buf.push_back(vector<double>(buf_size));
            //if (!opt.count("format"))
            fmt += comma + "%." + lexical_cast<string>(numeric_limits<double>::digits10) + "g";
            break;
          case e57::E57_INTEGER:
            buf.push_back(vector<int64_t>(buf_size));
            //if (!opt.count("format"))
            fmt += comma + "%d";
            break;
          case e57::E57_STRING:
            buf.push_back(vector<ustring>(buf_size));
            //if (!opt.count("format"))
            fmt += comma + "%s";
          }
          if (comma.empty()) { comma = ","; }
        }
      }
      comma.clear();
      for (int i = 0; i < prototype.childCount(); ++i)
      {
        Node n(prototype.get(i));
        pointrecord += comma + n.elementName();
        if (comma.empty()) { comma = ","; }
        switch (n.type())
        {
        case e57::E57_FLOAT:
        case e57::E57_SCALED_INTEGER:
          sdb.push_back(
            SourceDestBuffer(
              imf
              , n.elementName()
              , &get<vector<double> >(buf[i])[0]
              , buf_size
              , true
              , true
            )
          );
          break;
        case e57::E57_INTEGER:
          sdb.push_back(
            SourceDestBuffer(
              imf
              , n.elementName()
              , &get<vector<int64_t> >(buf[i])[0]
              , buf_size
              , true
              , true
            )
          );
          break;

        default:
          throw (runtime_error(
                   "prototype contains illegal type")
                );
        }
      }
      std::string name = string("image3d-") + lexical_cast<string>(child) + ".inf";
      std::ofstream inf(name.c_str());
      inf << "pointrecord = " << pointrecord << endl; // can be used as a header line for the csv file
      //inf << "pointrecord.format = " << fmt << endl;
      if (scan.isDefined("name"))
      {
        inf << "name = " << StringNode(scan.get("name")).value() << endl;
      }
      inf << "guid = " << StringNode(scan.get("guid")).value() << endl;
      if (scan.isDefined("description"))
      {
        inf << "description = " << StringNode(scan.get("description")).value() << endl;
      }
      if (scan.isDefined("pose"))
      {
        StructureNode pose(scan.get("pose"));
        StructureNode translation(pose.get("translation"));
        StructureNode rotation(pose.get("rotation"));
        inf << "pose.translation.x = " << FloatNode(translation.get("x")).value() << endl;
        inf << "pose.translation.y = " << FloatNode(translation.get("y")).value() << endl;
        inf << "pose.translation.z = " << FloatNode(translation.get("z")).value() << endl;
        inf << "pose.rotation.w = " << FloatNode(rotation.get("w")).value() << endl;
        inf << "pose.rotation.x = " << FloatNode(rotation.get("x")).value() << endl;
        inf << "pose.rotation.y = " << FloatNode(rotation.get("y")).value() << endl;
        inf << "pose.rotation.z = " << FloatNode(rotation.get("z")).value() << endl;
      }

      inf.close();

      CompressedVectorReader rd(points.reader(sdb));
      //path csvname(string("image3d-") + lexical_cast<string>(child) + ".csv");
      std::string csvname = string("image3d-") + lexical_cast<string>(child) + ".csv";
      //ofstream ocsv(dst / csvname);
      std::ofstream ocsv(csvname.c_str());
      ostream& out(ocsv); // needed to fix ambiguity for << operator on msvc
      //cout << "unpacking: " << dst / csvname << " ... ";
      unsigned count;
      uint64_t total_count(0);

      format tfmt(fmt);
      tfmt.exceptions(all_error_bits ^ too_many_args_bit);
      out << pointrecord << endl; // put the header line into csv
      while (count = rd.read())
      {
        total_count += count;
        for (size_t i = 0; i < count; ++i)
        {
          for (size_t j = 0; j < buf.size(); ++j)
          {
            tfmt = tfmt % apply_visitor(get_at(i), buf.at(j));
          }
          out << tfmt << endl;
        }
        break;
      }
      cout << " total points: " << total_count << endl;
      ocsv.close();
    }
  }

  PtxWriter ptxwriter(output.c_str());
  ptxwriter.Init(posPrecision, intensityPrecision, subsample);
  if (ptxwriter.IsOpen() == false)
  {
    printf("can not create file %s", output.c_str());
    return -3;
  }
  E57Reader reader;
  if (reader.Open(input.c_str()))
    if (reader.MoveNextScan())
    {
      //reader.ProcessConvert(ptxwriter);
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