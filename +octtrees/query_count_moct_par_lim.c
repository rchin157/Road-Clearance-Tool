/*
    Query the count of each element in a moct tree
    Performs query in paralell using OpenMP

    Early dropout using a limit
*/

#include <mex.h>
#include <matrix.h>
#include <omp.h>
#include "moctquery.h"

uint64_t check_lim;
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
    
    // Add any in our bucket that satisfy
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
            // Breakout
            if(count >= check_lim) return count;
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
    query_count_moct_par_lim(uint64 to a moct, constraints, check_lim)

    If you pass an invalid moct you will cause
    the program to segfault, so be careful.

    The second argument is a cell array of pointers to constraints
    Each element in the cell array to should point to a valid constraint

    The constraints are a 4xN array of coefficents for planes
    [ a0 a1 a2 a3 ... ]
    [ b0 b1 b2 b3 ... ]
    [ c0 c1 c2 c3 ... ]
    [ d0 d1 d2 d3 ... ]
    The region each plane considers is 
    ax + by + cz >= d

    A point must satisfy all planes to be included

    There should be NO MORE THAN 32 planes in a single constraint!
    This is for better malloc behavior

    Check lim means if check lim are found the function goes into early dropout and completes
*/
void mexFunction(int nlhs, mxArray *plhs[],
                 int nrhs, const mxArray *prhs[]){
    if (nrhs != 3){
        mexErrMsgIdAndTxt("Mocttree:query_count:nrhs", "Bad arguments");
    }

    mocttree* tree = (mocttree*)(mxGetUint64s(prhs[0])[0]);
    size_t num_lookups = mxGetNumberOfElements(prhs[1]);    // How many constraints are in our cell array

    // Created in the same shape as the input constraints
    mxArray* results = mxCreateUninitNumericArray(mxGetNumberOfDimensions(prhs[1]), mxGetDimensions(prhs[1]), mxUINT64_CLASS, mxREAL);
    uint64_t* raw_results_ptr = mxGetUint64s(results);

    check_lim = mxGetUint64s(prhs[2])[0];

    // Complete the rest of the work in parallel
    #pragma omp parallel
    {
        // Instead of doing a lot of mallocs, we just have a fixed size stuck on the stack
        union {
            uint8_t block_mem[32*sizeof(plane3) + sizeof(constraint)];
            //uint8_t block_mem[64 * sizeof(plane3) + sizeof(constraint)];
            constraint c;
        } cons;

        int i = 0;

        #pragma omp for schedule(dynamic)
        for (i = 0; i < num_lookups; i++){
            mxArray* cons_matrix = mxGetCell(prhs[1], i);
            double* plane_arr = mxGetDoubles(cons_matrix);
            size_t num_planes = mxGetN(cons_matrix);

            cons.c.num_planes = num_planes;
            for (int j = 0; j < num_planes; j++){
                cons.c.planes[j].norm.pos[0] = plane_arr[4*j+0];    // a
                cons.c.planes[j].norm.pos[1] = plane_arr[4*j+1];    // b
                cons.c.planes[j].norm.pos[2] = plane_arr[4*j+2];    // c
                cons.c.planes[j].dval = plane_arr[4*j+3];           // d
            }
            raw_results_ptr[i] = (uint64_t)query_count_tree(&(cons.c), tree);
        }
    }
    plhs[0] = results;
}
