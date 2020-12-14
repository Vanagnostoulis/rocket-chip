cd generated-src/
rm -rf *MyConfig*
cd ..
make CONFIG=freechips.rocketchip.system.MyConfig -j20
