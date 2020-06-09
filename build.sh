echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make

cd ../../OpenABLE

echo "Configuring and building Thirdparty/OpenABLE ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.tar.xz
cd ..
