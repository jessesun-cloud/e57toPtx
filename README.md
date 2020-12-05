# e57toPtx
A tool to convert e57 to ptx format

ABOUT
================================================================================
e57toPtx is an application to extract point cloud from e57 and write to ptx format.


usage:
e57toptx inputfilename outputfilename -p PRECISION -s SAMPLERATE -i INTENSITY_PRECISION -m

PRECISION, default be 3, is the precision we want to store position.

INTENSITY_PRECISION, default be 3, is the precision we want to store intensity.

SAMPLERATE, is the sub sample rate we want to apply.
when value is 1 we don't apply subsample.
when value is N, output will keep 1 point for every N X N points of original point cloud

-m export scan to multipe ptx file. by default, export all scan to one ptx file.

for example
e57toPtx in.e57 out.ptx -p 4 -i 4

use 4 digit precision to store position and intensity, don't do subsample.

E57 FILE Format
================================================================================
E57 format is a popular point cloud format.
E57 format reference:
http://paulbourke.net/dataformats/e57

PTX FILE Format
================================================================================
Ptx format is a point cloud text format by Leica Geosystems LLC.
Ptx format reference:
http://paulbourke.net/dataformats/ptx/

