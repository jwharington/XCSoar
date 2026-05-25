rm output/include/MathTables.h
make TARGET=UNIX DEBUG=y CLANG=n output/UNIX/bin/RunMultiAircraft -j8 FMT_HEADER_ONLY=y
strip output/UNIX/bin/RunMultiAircraft
cp output/UNIX/bin/RunMultiAircraft ../cpas/distrib/patServer/proxdata/bin
# make TARGET=UNIX DEBUG=n CLANG=y output/UNIX/bin/RunTask  -j8
