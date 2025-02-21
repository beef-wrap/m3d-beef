copy model3d\m3d.h m3d.c
clang -c -o m3d-windows.lib -target x86_64-pc-windows -fuse-ld=llvm-lib -Wall -DM3D_IMPLEMENTATION m3d.c
mkdir libs
move m3d-windows.lib libs
del m3d.c