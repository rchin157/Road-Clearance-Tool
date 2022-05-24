% Build all of the MATLAB .mex files

% mex -v -R2018a -I.\mimalloc\include\ createfreemoct.c ".\mimalloc\out\msvc-x64\Release\mimalloc-static.lib" COMPFLAGS="$COMPFLAGS /Wall"
% mex -v -R2018a -I.\mimalloc\include\ createmoct.c .\mimalloc\out\msvc-x64\Release\mimalloc-static.lib
% mex -v -R2018a -I.\mimalloc\include\ freemoct.c .\mimalloc\out\msvc-x64\Release\mimalloc-static.lib
mex -v -R2018a createfreemoct.c COMPFLAGS="$COMPFLAGS /Wall"
mex -v -R2018a query_count_moct.c COMPFLAGS="$COMPFLAGS /Wall" 
mex -v -R2018a query_index_moct.c COMPFLAGS="$COMPFLAGS /Wall" 
mex -v -R2018a COMPFLAGS="$COMPFLAGS /openmp /Wall"  query_count_moct_par.c
mex -v -R2018a COMPFLAGS="$COMPFLAGS /openmp /Wall"  query_count_moct_par_lim.c