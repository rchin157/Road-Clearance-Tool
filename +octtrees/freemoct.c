/*
    Mex function which frees a moct tree!
    DEPRECATED - SEE CREATEFREE
*/

#include <mex.h>
#include <matrix.h>
#include "mocttree.h"


void free_octnode(octnode* node){
    // Free children
    for (int i = 0; i<8; i++){
        if(node->children[i] != NULL){
            free_octnode(node->children[i]);
        }
    }
    // Clean ourselves up
    mi_free(node);
}

/*
    This is the entry point for this file
    in matlab it must be called as
    freemoct(uint64 to a valid moct)

    If you pass an invalid moct you will cause
    the program to segfault, so be careful.
*/
void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[]){
    if (nrhs != 1){
        mexErrMsgIdAndTxt("Mocttree:freeMoct:nrhs", "Bad arguments");
    }

    // Recast the uint64
    uint64_t* tree_pointer_int = mxGetUint64s(prhs[0])[0];
    mocttree* tree = (mocttree*)tree_pointer_int;

    mi_heap_destroy(tree->heap); // Deallocates the entire thing at once!
    mi_collect(true);
    mxFree(tree);
}
