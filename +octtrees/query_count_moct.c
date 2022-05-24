/*
    Query the count of each element in a moct tree
*/

#include <mex.h>
#include <matrix.h>
#include "moctquery.h"


/*
    Returns the total number of points satisfying a constraint in an octnode (resursively)
    
    Requirements:
    All coordinates in point1 < node.midpoint < point2
*/
size_t query_count_node(constraint* cons, octnode* node, vec3 point1, vec3 point2){
    if(!cube_satisfies(cons, point1, point2)){
        return 0;
    }

    if(cube_fully_satisfies(cons, point1, point2)){
        return node->num_total_elements;
    }

    size_t count = 0;
    
    for (int i = 0; i < node->num_elements; i++){
        if (satisfies(cons, node->bucket[i].point)) count++;
    }

    // Add any in our children's bucket that satisfy
    for (int i = 0; i < 8; i++){
        if (node->children[i] != NULL){
            // X Y Z reverse indexing
            vec3 temp1;
            vec3 temp2;
            for (int j = 0; j < 3; j++){
                if (i&(1<<j)){
                    temp1.pos[j] = node->midpoint.pos[j];
                    temp2.pos[j] = point2.pos[j];
                } else {
                    temp1.pos[j] = point1.pos[j];
                    temp2.pos[j] = node->midpoint.pos[j];
                }
            }
            count += query_count_node(cons, node->children[i], temp1, temp2);
        }
    }
    return count;
}


/*
    Returns the total number of points satisfying a constrain in a tree
*/
size_t query_count_tree(constraint* cons, mocttree* tree){
    return query_count_node(cons, tree->root, tree->point1, tree->point2);
}


/*
    This is entrypoint for this file
    in matlab it must be called as 
    query_count_moct(uint64 to a moct, constraints)

    If you pass an invalid moct you will cause
    the program to segfault, so be careful.


    The constraints are a 4xN array of coefficents for planes
    [ a0 a1 a2 a3 ... ]
    [ b0 b1 b2 b3 ... ]
    [ c0 c1 c2 c3 ... ]
    [ d0 d1 d2 d3 ... ]
    The region each plane considers is 
    ax + by + cz >= d

    A point must satisfy all planes to be included
*/
void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[]){
    if (nrhs != 2){
        mexErrMsgIdAndTxt("Mocttree:query_count:nrhs", "Bad arguments");
    }

    mocttree* tree = (mocttree*)(mxGetUint64s(prhs[0])[0]);
    
    double* planearray = mxGetDoubles(prhs[1]);
    size_t num_planes = mxGetN(prhs[1]);

    constraint* cons = malloc(num_planes*sizeof(plane3) + sizeof(cons));
    cons->num_planes = num_planes;

    for (int i = 0; i < num_planes; i++){
        cons->planes[i].norm.pos[0] = planearray[4*i+0];    // a
        cons->planes[i].norm.pos[1] = planearray[4*i+1];    // b
        cons->planes[i].norm.pos[2] = planearray[4*i+2];    // c
        cons->planes[i].dval = planearray[4*i+3];           // d
    }

    size_t one = 1;
    mxArray* result = mxCreateUninitNumericArray(1, &one, mxUINT64_CLASS, mxREAL);
    mxGetUint64s(result)[0] = (uint64_t)query_count_tree(cons, tree);

    free(cons);
    plhs[0] = result;
}
