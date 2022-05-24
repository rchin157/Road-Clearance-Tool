/*
    Query for an array of indexs to points
*/

#include <mex.h>
#include <matrix.h>
#include "moctquery.h"


/*
    Bypass slow searches and add quickly if everything is confirmed to be included
    Assumes we have enough space
*/
size_t add_quickly_node(octnode* node, size_t filled, size_t* index_array){;
    int count = node->num_elements;
    for (int i = 0; i < count; i++){
        index_array[filled + i] = node->bucket[i].index;  
    }

    for (int i = 0; i < 8; i++){
        if(node->children[i] != NULL){
            count += add_quickly_node(node->children[i], filled+count, index_array);
        }
    }

    return count;
}


/*
    Returns the total number of points satisfying a constraint in an octnode (resursively)
    
    Requirements:
    All coordinates in point1 < node.midpoint < point2
*/
size_t query_count_node(constraint* cons, octnode* node, vec3 point1, vec3 point2,
                        size_t filled, size_t* space, size_t** index_array){
    if(!cube_satisfies(cons, point1, point2)){
        return 0;
    }

    if(cube_fully_satisfies(cons, point1, point2)){
        // Get more space if we need it
        if (*space < filled +  node->num_total_elements){
            while (*space < filled + node->num_total_elements){
                // Expand by 1.5*s + 4
                *space = ((*space) * 3)/2 + 4;
            }
            *index_array = mxRealloc(*index_array, (*space)*sizeof(size_t));
        }
        return add_quickly_node(node, filled, *index_array);
    }

    size_t count = 0;
    
    // Add any in our bucket that satisfy
    for (int i = 0; i < node->num_elements; i++){
        if (satisfies(cons, node->bucket[i].point)){
            if (filled+count >= *space){
                // Expand by 1.5*s + 4
                *space = ((*space) * 3)/2 + 4;
                *index_array = mxRealloc(*index_array, (*space)*sizeof(size_t));
            }
            (*index_array)[filled+count] = node->bucket[i].index;
            count++;
        } 
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
            count += query_count_node(cons, node->children[i], temp1, temp2, filled+count, space, index_array);
        }
    }
    return count;
}


/*
    Returns the total number of points satisfying a constrain in a tree
    index_array is a return parameter which gets set to a pointer to an array of the indexes
    
    cleanup of the array is the callers responsibility (or no ones if it gets returned)
*/
size_t query_index_tree(constraint* cons, mocttree* tree, size_t** index_array){
    size_t space = 4;
    *index_array = mxCalloc(space, sizeof(size_t));
    return query_count_node(cons, tree->root, tree->point1, tree->point2, 0, &space, index_array);
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

    constraint* cons = mxMalloc(num_planes*sizeof(plane3) + sizeof(cons));
    cons->num_planes = num_planes;

    for (int i = 0; i < num_planes; i++){
        cons->planes[i].norm.pos[0] = planearray[4*i+0];    // a
        cons->planes[i].norm.pos[1] = planearray[4*i+1];    // b
        cons->planes[i].norm.pos[2] = planearray[4*i+2];    // c
        cons->planes[i].dval = planearray[4*i+3];           // d
    }

    size_t* index_array;
    size_t num_points = query_index_tree(cons, tree, &index_array);
    mxFree(cons);
    
    index_array = mxRealloc(index_array, num_points*sizeof(size_t)); // Realloc it to size

    plhs[0] = mxCreateNumericMatrix(0, 0, mxUINT64_CLASS, mxREAL);
    mxSetUint64s(plhs[0], index_array);
    mxSetM(plhs[0], num_points);
    mxSetN(plhs[0], 1);
}
